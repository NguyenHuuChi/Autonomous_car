import socket
import urllib.request
import cv2
import numpy as np


from test_controller import calculate_control_signal, send_control, find_lane_lines,find_lane_lines_canny
#from tempCodeRunnerFile import calculate_control_signal, send_control, find_lane_lines,find_lane_lines_canny
CONTROL_IP = "192.168.4.1"
CONTROL_PORT = 9999
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)

CAM_URL = "http://192.168.4.1:80"
stream = urllib.request.urlopen(CAM_URL)
bytes = bytes()



run = True
left_point_list=[0,0]
right_point_list=[0,0]
i=0
while run:
    # Get and display image
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        image = bytes[a:b+2]
        bytes = bytes[b+2:]
        try:
            image = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow("Image", image)
        except:
            continue
        
        
        
        left_point, right_point, im_center,check_sterring, draw = find_lane_lines(image,left_point_list,right_point_list, draw=True)
        # print(check_sterring)
        if i ==2:
            left_point_list.pop(0)
            left_point_list.append(left_point)
            right_point_list.pop(0)
            right_point_list.append(right_point)
            i=0

        i+=1
        cv2.imshow("Lane lines", draw)
        
        #Calculate speed and steering angle
        
        left_motor_speed, right_motor_speed = calculate_control_signal(left_point, right_point, im_center,check_sterring)
        
        print(left_motor_speed , right_motor_speed*1.05 )
        
        cv2.waitKey(1)
        
exit()
