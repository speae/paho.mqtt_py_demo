from rplidar import RPLidar

def scanData(data):
    for scan in zip(data):
        for quality, angle, distance in scan:
            print("quality : %d, angle : %f, distance : %f" % (quality, angle, distance))
        
lidar = RPLidar('/dev/ttyUSB0')
 
info = lidar.get_info()
print(info)
 
health = lidar.get_health()
print(health)

try:
    for i, scan in enumerate(lidar.iter_scans()):
        scanData(scan)
        print('%d: Got %d measurments' % (i, len(scan)))
		
except KeyboardInterrupt:  
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
