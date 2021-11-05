import sys
import os
import threading
import time
import serial
import signal 

exitThread = False
data = ''

keyMap = {'a' : 'turn left',
    'b' : 'turn right',
    'c' : 'forward',
    'd' : 'back',
    'i' : 'stop'}


def openSerial(port="/dev/ttyTHS2", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False):

    serial_port = serial.Serial(
        port="/dev/ttyTHS2",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )
    
    return serial_port

def readThread(ser):
    global exitThread
    global data

    while not exitThread:
        data = ser.read()
        print("output data : ", data)

def writeThread(ser, data):
    global exitThread

    while not exitThread:
        data = input("input command : \r\n")
        try:
            if keyMap[data]:
                ser.write(data.encode("utf-8"))
                print("data : ", data)
            else:
                exitThread = True
        except Exception:
            sys.exit()

def readEOF(ser):
    readed = ser.readline()
    return readed[:-1]

if __name__ == '__main__':

    try:
        ser = openSerial()

        rx = threading.Thread(target=readThread, args=(ser,))
        tx = threading.Thread(target=writeThread, args=(ser, data,))
        
        rx.start()
        time.sleep(1)
        tx.start()

        rx.join()
        tx.join()

        print(readEOF(ser))

    except KeyboardInterrupt:
        exitThread = True

# serial_port = serial.Serial(
#     port="/dev/ttyTHS1",
#     baudrate=115200,
#     bytesize=serial.EIGHTBITS,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE
# )

# try:
#     serial_port.write("UART Demonstration Program\r\n".encode())
#     serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())

#     while True:
#         if serial_port.inWaiting() > 0:
#             data = serial_port.read()
#             print(data)

#             if data == "\r".encode():
#                 serial_port.write("\n".encode())

# except KeyboardInterrupt:
#     print("Exist Program")

# except Exception as exception_error:
#     print("Error occured. Exiting Program")
#     print("Error : " + str(exception_error))

# finally:
#     serial_port.close()
#     pass
