import sys
sys.path.insert(0, './yolov5')

import argparse
import os
import platform
import shutil
import time
import cv2
import torch
import torch.backends.cudnn as cudnn
import random
import numpy as np
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

import paho.mqtt.client as mqtt
import pyrealsense2 as rs
from yolov5.models.experimental import attempt_load
from yolov5.utils.downloads import attempt_download
from yolov5.utils.datasets import LoadImages, LoadStreams
from yolov5.utils.general import check_img_size, non_max_suppression, scale_coords, check_imshow, xyxy2xywh
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors
from deep_sort_pytorch.utils.parser import get_config
from deep_sort_pytorch.deep_sort import DeepSort

# 쓰레드종료 알림 변수 (기본False, 종료요청True)
# stopThread_flag = False

# 테스트용 전역변수
# target_xval = 0.5 # 객체의 x좌표
# distance_val = 0.0 # 객체의 거리 
# obs_val = 0 # 가까운 장애물의 거리 
# following_pers = 0 # 추적타겟
# center_p = 0 # 중앙에 가장 가까운 객체의 id

# --> mqtt function
class mqttClass():
    def __init__(self):
        
        # generate client ID with pub prefix randomly
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        self.username = 'emqx'
        self.password = 'public'
        
        # --> MQTT value
        self.client = mqtt.Client(self.client_id)
        self.broker = 'broker.emqx.io'
        self.port = 1883
        self.topic = "python/depth"

    def cmd_argument(self):
        
        # cmd input
        parser = argparse.ArgumentParser()
        parser.add_argument('--yolo_weights', nargs='+', type=str, default='yolov5/weights/yolov5s.pt', help='model.pt path(s)')
        parser.add_argument('--deep_sort_weights', type=str, default='deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7', help='ckpt.t7 path')
        # file/folder, 0 for webcam
        parser.add_argument('--source', type=str, default='0', help='source')
        parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
        parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
        parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
        parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
        parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--show-vid', action='store_true', help='display tracking video results')
        parser.add_argument('--save-vid', action='store_true', help='save video tracking results')
        parser.add_argument('--save-txt', action='store_true', help='save MOT compliant results to *.txt')
        # class 0 is person, 1 is bycicle, 2 is car... 79 is oven
        parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 16 17')
        parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
        parser.add_argument('--augment', action='store_true', help='augmented inference')
        parser.add_argument('--evaluate', action='store_true', help='augmented inference')
        parser.add_argument("--config_deepsort", type=str, default="deep_sort_pytorch/configs/deep_sort.yaml")

        opt = parser.parse_args()

        opt.img_size = check_img_size(opt.img_size)
        opt.source = '2'
        opt.yolo_weights = 'yolov5s.pt'
        opt.show_vid = True
        opt.classes = 0

        return opt

    def connect_mqtt(self) -> mqtt:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("deepsort to MQTT on.")

            else:
                print("Failed to connect, return code %d\n", rc)

        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = on_connect
        self.client.connect(self.broker, self.port)

        return self.client

    def run(self):
        client = self.connect_mqtt()
        client.loop_start()

class deepsortClass(mqttClass):
    def __init__(self, client, topic, opt, mqtt_start, stopThread_flag):
        
        self.client = client    
        self.opt = opt
        self.topic = topic
        self.mqtt_start = mqtt_start
        
        self.stopThread_flag = stopThread_flag # 쓰레드 종료명령
        self.target_xval = 0.5 # 쓰레드 사물의x좌표 전달 변수
        self.distance_val = 0.0 # 쓰레드 거리데이터 전달 변수
        self.obs_val = 0 # 멈추기위한 장애물과의 거리 
        self.center_p = 0 # 중앙에 가장 가까운 id
        self.following_pers = 0 # 추적할 person타겟 초기화 
        self.boxCent_list = []
        
        self.keyMap = {b'A' : 'turn right:: ▶▶▶▶',
                        b'B' : 'turn right:: ▶▶▶',
                        b'C' : 'turn right:: ▶▶',
                        b'D' : 'turn right:: ▶',
                        b'E' : '◀ :: turn left',
                        b'F' : '◀◀ :: turn left',
                        b'G' : '◀◀◀ :: turn left',
                        b'H' : '◀◀◀◀ :: turn left',
                        b'i' : '▲',
                        b'k' : '▼',
                        b'j' : '■ STOP ■'}

    # 물체의 좌우 끝 좌표를 받아 그 물체 의 거리값을 가져와주는 함수 
    def location_to_depth(self, grayimg, loc1, loc2, depth_data):
        if loc1[0] < loc2[0]:
            arr = []
            start = loc1[0]
            end = loc2[0]

            for i in range(start, end+1):
                arr.append(grayimg[loc1[1], i])
            np_arr = np.array(arr)
            index_arr = np.argmax(np_arr)

            target_depth = depth_data.get_distance( loc1[0] + index_arr, loc1[1] )

        return round(100 * target_depth,2)

    def publisher(self):
        while True:
            if self.stopThread_flag == True: # 종료문
                buff_a = b'j'
                self.client.publish(self.topic, buff_a)
                print("Stop activate.")

                # os.write(fds_from_yolo, buff_a.encode())
                break

            # 장애물이 없으면.
            if self.obs_val == 0:
                # 타겟의 x좌표의 오른쪽에있고, 오른쪽으로 회전하기 위한값을 fifo로전달
                if self.target_xval > 0.9:
                    buff_a = b'A'
                    
                elif self.target_xval > 0.8:
                    buff_a = b'B'
                    
                elif self.target_xval > 0.7:
                    buff_a = b'C'
                    
                elif self.target_xval > 0.6:
                    buff_a = b'D'
                    
                # 타겟의 x좌표가 왼쪽에있고, 왼쪽으로 회전하기 위한값을 fifo로전달
                elif self.target_xval < 0.4:
                    buff_a = b'E'
                    
                elif self.target_xval < 0.3:
                    buff_a = b'F'                   
                    
                elif self.target_xval < 0.2:
                    buff_a = b'G'               
                    
                elif self.target_xval < 0.1:
                    buff_a = b'H'
                    
                # 타겟의 x좌표가 중앙 0.5에 있을때
                else:
                    # 타겟과의 거리가 멀리있을때 전진
                    if self.distance_val > 80.0:
                        buff_a = b'i'

                    # 타겟과의 거리가 가까이있을때 후진
                    # elif distance_val < 0.5:
                    #     buff_a = 'd'
                    # elif 60.0 < self.distance_val <= 80.0:
                    #     buff_a = b'k'

                    # 타겟과의 거리가 적당거리일떄 멈춤
                    else:
                        buff_a = b'j'
                        
                self.client.publish(self.topic, buff_a)
                print(f"Send payload : {buff_a} == {self.keyMap[buff_a]}")
                break
                    
            # 장애물이 있으면 obs_val == 1
            else:
                buff_a = b'j'
                self.client.publish(self.topic, buff_a)
                print(f"{buff_a} : WARNING. Obstacle come closing ")
                
                break

    def box_create(self):
        max_center = [0, 1000.0] # 임시변수 초기값 [초기id, 중앙에서 가장먼거리]                
        for name, x, y in self.boxCent_list:
            name_x= abs(320 - x)
            name_y= abs(240 - y)
            dist_fromCent = math.sqrt(name_x**2 + name_y**2) #중앙으로 부터의 거리
            if max_center[1] > dist_fromCent:
                max_center = name, dist_fromCent 
        self.center_p = max_center[0] #id와 중앙부터거리 center_p[0],[1]에 저장
        print(f"중앙에서 가장 가까운 객체 id:{max_center[0]}, 거리:{max_center[1]} ...")

    def detect(self):

        out, source, yolo_weights, deep_sort_weights, show_vid, save_vid, save_txt, imgsz, evaluate = \
            self.opt.output, self.opt.source, self.opt.yolo_weights, self.opt.deep_sort_weights, self.opt.show_vid, self.opt.save_vid, \
                self.opt.save_txt, self.opt.img_size, self.opt.evaluate
        
        webcam = source == '0' or '1' or '2' or source.startswith(
            'rtsp') or source.startswith('http') or source.endswith('.txt') 
        
        detect_start = 0.0
        
        # initialize deepsort 초기화
        cfg = get_config()
        cfg.merge_from_file(self.opt.config_deepsort)
        attempt_download(deep_sort_weights, repo='mikel-brostrom/Yolov5_DeepSort_Pytorch')
        deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                            max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                            max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                            use_cuda=True)

        # Initialize
        device = select_device(self.opt.device)

        if not evaluate:
            if os.path.exists(out):
                pass
                shutil.rmtree(out)  # delete output folder
            os.makedirs(out)  # make new output folder

        half = device.type != 'cpu'  # half precision only supported on CUDA
        # Load model
        model = attempt_load(yolo_weights, map_location=device)  # load FP32 model
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(imgsz, s=stride)  # check img_size
        names = model.module.names if hasattr(model, 'module') else model.names  # get class names
        if half:
            model.half()  # to FP16

        # Set Dataloader
        vid_path, vid_writer = None, None
        # Check if environment supports image displays
        if show_vid:
            show_vid = check_imshow()

        if webcam:
            cudnn.benchmark = True  # set True to speed up constant image size inference
            dataset = LoadStreams(source, img_size=imgsz, stride=stride)
            print(f"webcam:{webcam}, source:{source}, imgsz:{imgsz}, stride:{stride}")
        # read class object with enum ,enumerate(class)

        else:
            dataset = LoadImages(source, img_size=imgsz, stride=stride)

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        t0 = time.time()

        save_path = str(Path(out))
        # extract what is in between the last '/' and last '.'
        txt_file_name = source.split('/')[-1].split('.')[0]
        txt_path = str(Path(out)) + '/' + txt_file_name + '.txt'

        mqtt_end = time.time()
        print(f"동영상 시작까지 걸린 시간 = {mqtt_end - self.mqtt_start}")

        # datsset.py로부터 class를 통해 영상들 받아오는곳
        for frame_idx, (path, img, im0s, vid_cap, depth_img, depth_im0s, depth_data) in enumerate(dataset):
            # print(f"frame_idx:{frame_idx}, path:{path}, img:{img}, im0s:{im0s}, vid_cap:{vid_cap}, depth_img:{depth_img},\
            #   depth_im0s:{depth_im0s}, depth_data:{depth_data}." )
        
            #print("frame_idx:",frame_idx,"path:",path,"vid_cap",vid_cap)
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time.time()
            pred = model(img, augment=self.opt.augment)[0]

            # Apply NMS
            pred = non_max_suppression(
                pred, self.opt.conf_thres, self.opt.iou_thres, classes=self.opt.classes, agnostic=self.opt.agnostic_nms)
            t2 = time_sync()

            # Process detections
            for i, det in enumerate(pred):  # detections per image
                if webcam:  # batch_size >= 1
                    p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
                else:
                    p, s, im0 = path, '', im0s

                #depth
                dep_img = depth_im0s[i].copy() 
                dm = cv2.flip(dep_img, 1)
                depth_alpha = cv2.convertScaleAbs(dm, alpha=0.15)
                dm0 = depth_colorImg = cv2.applyColorMap(depth_alpha, cv2.COLORMAP_JET)

                depth_grayimg = cv2.cvtColor(dm0,cv2.COLOR_RGBA2GRAY)
                # 영상 이진화 OTSU
                _, OTSU_binary = cv2.threshold(depth_grayimg, 0, 255, cv2.THRESH_OTSU)
                # 가우시안 블러와 OTSU로 노이즈제거
                OTSU_blur = cv2.GaussianBlur(OTSU_binary, (5, 5), 0)
                cv2.imshow("OTSU_blur", OTSU_blur)
                ret, OTSU_gaubin = cv2.threshold(OTSU_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                cv2.imshow('src_gaubin', OTSU_gaubin)

                # STEP 5. 외곽선 검출
                # cv2 contours 외곽선 추출함수
                contours, _ = cv2.findContours(OTSU_gaubin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
                contours_images = np.zeros((480, 640, 3), np.uint8) # 검은판 하나 생성
                # 모든 객체 외곽선
                for Line_idx in range(len(contours)):
                    color = (0,0,255) 
                    cv2.drawContours(contours_images, contours, Line_idx, color, 1, cv2.LINE_AA)

                # 장애물 외곽선 연결선
                box_cnt = 0 # 박스번호
                close_Obs_flag = 0 # 근접한 장애물
                for contour in contours:
                    # convexHull 나머지 모든점을 포함하는 다각형을 만들어내는 알고리즘 = 장애물 외곽선
                    conhull = cv2.convexHull(contour)
                    hull = np.max(conhull, axis=1)
                    maxbox = np.max(hull, axis=1)

                    
                    # 최대값x와 최소값x을 뺀 절대값이 (노란박스크기가)작은건 안그려지게한다
                    if abs(max(maxbox) - min(maxbox)) > 90:  
                        cv2.drawContours(contours_images, [conhull], 0, (0, 255, 255), 3)  

                        # 한 박스(hull) 마다  맨 좌측값, 맨우측값 가져오기
                        max_x = np.array([24, 0]) # y축 맨오른쪽좌표가 들어갈 변수
                        min_x = np.array([639, 0])  # y축 맨왼쪽좌표가 들어갈 변수
                        for count in hull:
                            if max_x[0] < count[0]:
                                max_x = count 
                            if min_x[0] > count[0]:
                                min_x = count
                        cv2.circle(contours_images, max_x, 3, (255, 0, 0), 2, cv2.LINE_AA)  # 최우측 좌표에 파란
                        cv2.circle(contours_images, min_x, 3, (255, 255, 0), 2, cv2.LINE_AA)  # 최좌측 좌표에 청록
                        # 장애물하나당 거리값 구해오는 함수
                        obs_depth = self.location_to_depth(depth_grayimg, min_x, max_x, depth_data)
                        #print(obs_depth)
                        # 최소 장애물과의 거리. 이에 아래값에 도달할경우 거리값전달 멈추기
                        if obs_depth < 50.0:
                            close_Obs_flag = 1

                    # 박스 다음번호로
                    box_cnt = box_cnt + 1
                    # 검출된 노란박스중에 가까운박스(close_Obs_flag)가 있으면 멈추는명령 쓰레드로전달
                    if close_Obs_flag:
                        self.obs_val = 1 #멈춤
                    else:
                        self.obs_val = 0 #없음

                cv2.imshow("src", contours_images)

                s += '%gx%g ' % img.shape[2:]  # print string
                save_path = str(Path(out) / Path(p).name)

                # box
                annotator = Annotator(im0, line_width=2, pil=not ascii)
                annotator2 = Annotator(dm0, line_width=2, pil=not ascii)            

                if det is not None and len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(
                        img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    xywhs = xyxy2xywh(det[:, 0:4])
                    confs = det[:, 4]
                    clss = det[:, 5]

                    # pass detections to deepsort
                    outputs = deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
                    
                    self.boxCent_list = []
                    # 박스 하나당 draw boxes for visualization
                    if len(outputs) > 0:
                        for j, (output, conf) in enumerate(zip(outputs, confs)): 
                            
                            bboxes = output[0:4]
                            id = output[4]
                            cls = output[5]                       
                                                
                            c = int(cls)  # integer class
                            label = f'{id} {names[c]} {conf:.2f}'
                            annotator.box_label(bboxes, label, color=colors(c, True))

                            w = 640
                            h = 480
                            depth_w = (w-20) * 23 // 35
                            depth_h = h * 2 // 3
                            
                            start_left =  depth_w * (output[0] / w) + (w-20) // 7 # 왼쪽위 x좌표
                            start_top =  depth_h * (output[1] / h) + h // 6 # 왼쪽위 y좌표
                            end_left =  depth_w * (output[2] / w) + (w-20) // 7 # 오른쪽아래 x좌표
                            end_top =  depth_h * (output[3] / h) + h // 6 # 오른쪽아래 y좌표  
                
                            # 박스의 중앙값들 리스트로 전
                            x_center = (start_left + end_left) // 2
                            y_center = (start_top + end_top) // 2
                            self.boxCent_list.append([id, x_center,y_center])
                
                            # 뎁스와 다른 컬러맵의 왼쪽 끝 잘라내기
                            if output[0] >= 20:
                                bboxes2 = np.array([start_left, start_top, end_left, end_top])  
                            else:
                                bboxes2 = np.array([(w-20) // 7, start_top, end_left, end_top])
                            annotator2.box_label(bboxes2, label, color=colors(c, True))

                            if save_txt:
                                # to MOT format
                                bbox_left = output[0]
                                bbox_top = output[1]
                                bbox_w = output[2] - output[0]
                                bbox_h = output[3] - output[1]
                                # Write MOT compliant results to file
                                with open(txt_path, 'a') as f:
                                    f.write(('%g ' * 10 + '\n') % (frame_idx, id, bbox_left,
                                                                bbox_top, bbox_w, bbox_h, -1, -1, -1, -1))  # label format

                            dep_x, dep_y = int((start_left+end_left)//2),int((start_top+end_top)//2)
                            target_distacne = depth_data.get_distance(dep_x, dep_y)
                            cv2.circle(dm0,(dep_x, dep_y),2,(0,0,122),1,cv2.LINE_AA)

                            # 타겟의 거리가 적당한것만 (오류제외)
                            if 0.01 < target_distacne < 7.0:
                                # 맨처음 키자마자 추적할놈은?
                                if self.following_pers == 0:
                                    self.following_pers = self.center_p
                                # print ("현재 타겟 {}를 추적중입니다.".format(following_pers)) 
                                # 내가 원하는 타겟 person 만 쫒아가게하려면. following_pers값을 조정
                                if self.following_pers == id:
                                    # 타켓person의 x축위치를 fifo쓰레드로 전달
                                    self.target_xval = ((start_left + end_left) / 2) / w
                                    # 타켓person의 거리를 fifo쓰레드로 전달 
                                    self.distance_val = int(target_distacne * 100)
                                    print(f"{id} : 타겟 person과의 거리 = {target_distacne * 100:.2f} cm")

                                # 타겟외에 다른 person
                                #else:
                                #    print(id,":person과의 거리 = {:.2f} cm".format(target_distacne * 100))

                            # class의 target : person 0 id -> 2,3,4,5을 바꿔가며 쫒아가는달

                        # 프레임이 떨어지는 구간
                        # 중앙값리스틀 가장 중앙에서 가까운 박스찾기
                        cv2.circle(dm0, (320,240), 3, (255, 0, 255), 2, cv2.LINE_AA)
                        # max_center = [0, 1000.0] # 임시변수 초기값 [초기id, 중앙에서 가장먼거리]
                        # for name, x, y in boxCent_list:
                        #     name_x= abs(320 - x)
                        #     name_y= abs(240 - y)
                        #     dist_fromCent = math.sqrt(name_x**2 + name_y**2) #중앙으로 부터의 거리
                        #     if max_center[1] > dist_fromCent:
                        #         max_center = name, dist_fromCent 
                        # self.center_p = max_center[0] #id와 중앙부터거리 center_p[0],[1]에 저장
                        # print(f"중앙에서 가장 가까운 객체 id : {max_center[0]}, 거리 : {max_center[1]:.5f}")

                        # 멀티 프로세스 x
                        self.box_create()

                else:
                    deepsort.increment_ages()

                # Stream results
                im0 = annotator.result()
                dm0 = annotator2.result()
                if show_vid:
                    cv2.imshow("deteciton", im0)
                    cv2.imshow("dm0", dm0)

                detect_end = time.time()         
                detect_time = detect_end - detect_start
                detect_start = detect_end
                fps = 1/detect_time
                print(f'{fps:.5f} fps')
                
                self.publisher()
                
                # 영상 저장 (image with detections)
                if save_vid:
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            print("fps:",fps, "w:", w, "h:",h)
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path += '.mp4'

                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer.write(im0)

        # 로그 txt파일
        if save_txt or save_vid:
            print('Results saved to %s' % os.getcwd() + os.sep + out)
            if platform == 'darwin':  # MacOS
                os.system('open ' + save_path)

        # 전체 작동시간fps
        self.stopThread_flag = True
        #TPE.shutdown()
        self.client.publish(self.topic, b'j')
        print('Done. (%.3fs)' % (time.time() - t0))
        
if __name__ == '__main__':

    mqttDriver = mqttClass()
    
    deepSortStart = deepsortClass(mqttDriver.client, mqttDriver.topic, opt=mqttDriver.cmd_argument(), mqtt_start=time.time(), stopThread_flag=False)

    # 쓰레드 포함 -> 이 밑으로 멀티프로세스 생성 x.
    # loop_start()를 사용해야 아래 함수들 실행 가능.
    mqttDriver.run()

    with ThreadPoolExecutor(max_workers=4) as TPE:
        with torch.no_grad():
            TPE.submit(deepSortStart.detect)

    mqttDriver.client.loop_stop()
    