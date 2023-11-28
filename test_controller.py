import cv2
import numpy as np
import socket
from config import CONTROL_IP, CONTROL_PORT
from statistics import mean


sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)

def set_control_ip():
    ip = socket.gethostbyname(socket.gethostname())
    control_msg = "SET_CONTROL_IP {}".format(ip).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))

def send_control(left_motor_speed, right_motor_speed):
    """Convert steering and throttle signals to a suitable format and send them to ESP32 bot"""
    control_msg = "CONTROL_WHEEL {} {}".format(
        left_motor_speed, right_motor_speed).encode('ascii')
    sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))


def calculate_control_signal(left_point, right_point, im_center,check_steering):
    """Calculate control signal"""
    if check_steering==-1:
        st=0.18
    elif check_steering==1:
        st=0.22
    if left_point == -1 or right_point == -1:
        left_motor_speed = right_motor_speed = 0
        return left_motor_speed, right_motor_speed

    # Calculate difference between car center point and image center point
    center_point = (right_point + left_point) // 2
    center_diff = center_point - im_center

    # Calculate steering angle from center point difference
    steering = float(center_diff * 0.03)
    steering = min(1, max(-1, steering))
    throttle = 0.78

    # From steering, calculate left/right motor speed
    left_motor_speed = 0
    right_motor_speed = 0

    if steering > 0:
        left_motor_speed = throttle*(1+steering*st)
        right_motor_speed = throttle 
    else:
        left_motor_speed = throttle 
        right_motor_speed = throttle* (1 - steering*st)

    left_motor_speed = int(left_motor_speed * 100)
    right_motor_speed = int(right_motor_speed * 100)
    
    return left_motor_speed, right_motor_speed



def grayscale(img):
    """Convert image to grayscale"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    """Apply Canny edge detection"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size,a):
    """Apply a Gaussian blur"""
    """
    a=1 filter 2D
    a=2 blur using kernels
    a=3 gausian blur
    a=4 median blur
    a=5 sharpen using kernel
    a=6 bilateral filtering
    """
    if a==1:
        kernel=np.array([[0,0,0],[0,1,0],[0,0,0]])
        return cv2.filter2D(src=img,ddepth=-1,kernel=kernel)
    elif a==2:
        kernel=np.ones((3,3),np.float32)/9
        return cv2.filter2D(src=img,ddepth=-1,kernel=kernel)
    elif a==3:
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    elif a==4:
        return cv2.blur(src=img,ksize=(5,5))
    elif a==5:
        return cv2.medianBlur(src=img,ksize=kernel_size)
    elif a==6:
        return cv2.bilateralFilter(src=img,d=9,sigmaColor=75,sigmaSpace=75)
def birdview_transform(img):
    """Get birdview image"""
    IMAGE_H = 296
    IMAGE_W = 400

    src = np.float32([[0, IMAGE_H*2//3-40], [320, IMAGE_H*2//3-40], [
                     0, 0], [IMAGE_W, 0]])
    dst = np.float32([[90, IMAGE_H], [265, IMAGE_H],[-90, 0], [IMAGE_W+140, 0],])
    M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
    warped_img = cv2.warpPerspective(
        img, M, (IMAGE_W, IMAGE_H))  # Image warping
    return warped_img


def preprocess(img):
    """Preprocess image to get a birdview image of lane lines"""

    img = grayscale(img)
    img = gaussian_blur(img, 3,3)
    img = canny(img, 100, 200)
    cv2.imshow("Canny", img)
    cv2.waitKey(1)
    img = birdview_transform(img)

    return img

def find_lane_lines_canny(image,draw=False):
    image=grayscale(image)
    image=gaussian_blur(image,3,3)
    image=canny(image,100,200)
    cv2.imshow("canny",image)
    im_height,im_width=image.shape[:2]
    if draw:
        vizz_img=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
    interested_line_y=int(im_height*0.7)
    interested_line_y2=int(im_height*0.6)
    interested_line=image[interested_line_y,:]
    interested_line2=image[interested_line_y2,:]
    if draw:
        cv2.line(vizz_img,(0,interested_line_y),(im_width,interested_line_y),(0,0,255),2)
        cv2.line(vizz_img,(0,interested_line_y2),(im_width,interested_line_y2),(0,0,255),2)
   
    center=im_width//2+10
    def check_left(interested_line,center):
        left_point=-1
        for i in range(center,0,-1):
            if interested_line[i]>0:
                if i-30<=0:
                    left_point=i
                else:
                    for j in range(i-6,i-30,-1):
                        if interested_line[j]>0:
                            left_point=i
                            break
                break
        return left_point
    def check_right(interested_line,center,im_width):
        right_point=-1
        for i in range(center,im_width,1):
            if interested_line[i]>0:
                if i+30>im_width:
                    right_point=i
                else:
                    for j in range(i+6,i+30,1):
                        if interested_line[j]>0:
                            right_point=i
                            break
                break
        return right_point
    right_point1=check_right(interested_line,center,im_width)
    right_point2=check_right(interested_line2,center,im_width)
    left_point1=check_left(interested_line,center,im_width)
    left_point2=check_left(interested_line2,center,im_width)
    lane_width=180
    
    
    if left_point1 != -1 and right_point1==-1:
        # right_point1= mean(list_frame_right)
        if left_point1<left_point2:
            
            right_point1 = left_point1 + lane_width
        else:
            right_point1=left_point1
            left_point1=right_point1-lane_width
    if right_point1!=1 and left_point1==-1:
        # left_point1=mean(list_frame_left)
        if right_point1<right_point2:
            left_point1 = right_point1 - lane_width
        else:
            left_point1=right_point1
            right_point1=left_point1+lane_width
    if right_point1-right_point1 >30:
        right_point1=(right_point1+right_point2)//2
    if draw:
        if left_point1 != -1:
            viz_img = cv2.circle(
                viz_img, (left_point1, interested_line_y), 7, (255, 255, 0), -1)
        # if left_point2 != -1:
        #     viz_img = cv2.circle(
        #         viz_img, (left_point2, interested_line_y2), 7, (255, 255, 0), -1)
        if right_point1 != -1:
            viz_img = cv2.circle(
                viz_img, (right_point1, interested_line_y), 7, (0, 255, 0), -1)
        # if right_point2 != -1:
        #     viz_img = cv2.circle(
        #         viz_img, (right_point2, interested_line_y2), 7, (0, 255, 0), -1)
        center1=(left_point1+right_point1)//2
        viz_img = cv2.circle(
                viz_img, (center1, interested_line_y), 7, (0, 255, 255), -1)
        # viz_img = cv2.circle(
        #         viz_img, (left_point2, interested_line_y), 7, (255, 255, 0), -1)
        viz_img = cv2.circle(
                viz_img, (center, interested_line_y), 7, (255, 0, 255), -1)
    if draw:
        return left_point1, right_point1, center, vizz_img
    else:
        return left_point1, right_point1, center
def find_lane_lines(image,left_point_list,right_point_list,draw=False):
    """Find lane lines from color image"""
    check_steering=1
    image = preprocess(image)

    im_height, im_width = image.shape[:2]
    # print(im_height,im_width,'h,w')
    if draw:
        viz_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Interested line to determine lane center
    interested_line_y = int(im_height * 0.9)
    interested_line_y2=int(im_height*0.865)
    if draw:
        cv2.line(viz_img, (0, interested_line_y),
                 (im_width, interested_line_y), (0, 0, 255), 2)
    if draw:
        cv2.line(viz_img, (0, interested_line_y2),
                 (im_width, interested_line_y2), (0, 0, 255), 2)
    interested_line = image[interested_line_y, :]
    interested_line2=image[interested_line_y2, :]
    # Determine left point and right point
   
    lane_width = 150
    
    center =   im_width // 2 -3

    # Hàm tìm left point
    def check_left(interested_line,center,im_width):
        left_point=-1
        for i in range(center,0,-1):
            if interested_line[i]>0:
                if i-30<=0:
                    left_point=i
                else:
                    for j in range(i-9,i-30,-1):
                        if interested_line[j]>0:
                            left_point=i
                            break
                break
        return left_point
    # Hàm tìm rightpoint
    def check_right(interested_line,center,im_width):
        right_point=-1
        for i in range(center,im_width,1):
            if interested_line[i]>0:
                if i+30>im_width:
                    right_point=i
                else:
                    for j in range(i+9,i+30,1):
                        if interested_line[j]>0:
                            right_point=i
                            break
                break
        return right_point
    right_point1=check_right(interested_line,center,im_width)
    right_point2=check_right(interested_line2,center,im_width)
    left_point1=check_left(interested_line,center,im_width)
    left_point2=check_left(interested_line2,center,im_width)
    # Predict occluded points
    # left_point=(left_point1+left_point2)//2
    # right_point=(right_point1+right_point2)//2
    # if left_point1 != -1 and right_point1 == -1:
    #     right_point1=mean(right_point_list)
    # if left_point1 == -1 and right_point1 != -1:
    #     left_point1=mean(left_point_list)
    # if left_point1==left_point2==right_point1==right_point2==-1:


    # if left_point2==left_point1==-1 and right_point1!=-1 and right_point2!=-1 and abs(right_point1-right_point2)<5:
    #     left_point1=left_point_list[-1]-1
    # if right_point1==right_point2==-1 and left_point1!=-1 and left_point2!=-1 and abs(left_point1-right_point2)<5:
    #     right_point1=right_point_list[-1]+1
    if left_point1!=-1 and right_point1!=-1 and left_point2!=-1 and right_point2!=-1:
        check_steering=-1
    if left_point1==-1 and right_point1==-1 :
    
        a=(left_point_list[1]-left_point_list[0])//2
        left_point1=left_point_list[-1]+a
    
        a1=(right_point_list[1]-right_point_list[0])//2
        right_point1=right_point_list[-1]+a1
    elif abs(right_point1-left_point1)< lane_width :
        if left_point1==-1:
            left_point1=right_point1-lane_width
        if right_point1==-1:
            right_point1=left_point1+lane_width
    if left_point1==-1:
        if abs(right_point1-left_point_list[-1])<20:
            left_point1=right_point1
            right_point1=left_point1+lane_width
        elif right_point1-right_point2>50 and right_point2 != -1:
            right_point1=(right_point1+right_point2)//2
            left_point1=right_point1-lane_width-(right_point1-right_point2)*2
            check_steering=1
        else:
            a=(left_point_list[1]-left_point_list[0])//2
            left_point1=left_point_list[-1]+a
            if right_point1-left_point1<lane_width:
                left_point1=right_point1-lane_width
            
    if right_point1==-1:
        if abs(left_point1-right_point_list[-1])<20:
            right_point1=left_point1
            left_point1=right_point1-lane_width
        elif left_point2-left_point1>50 and left_point2 !=-1:
            left_point1=(left_point1+left_point2)//2
            right_point1=left_point1+lane_width+(left_point1-left_point2)*2
            check_steering=1
        else:
            a1=(right_point_list[1]-right_point_list[0])//2
            right_point1=right_point_list[-1]+a1
            if right_point1-left_point1<lane_width:
                right_point1=left_point1+lane_width


    if draw:
        if left_point1 != -1:
            viz_img = cv2.circle(
                viz_img, (int(left_point1), int(interested_line_y)), 7, (255, 255, 0), -1)
        if left_point2 != -1:
            viz_img = cv2.circle(
                viz_img, (int(left_point2), int(interested_line_y2)), 7, (255, 255, 0), -1)
        if right_point1 != -1:
            viz_img = cv2.circle(
                viz_img, (int(right_point1),int( interested_line_y)), 7, (0, 255, 0), -1)
        if right_point2 != -1:
            viz_img = cv2.circle(
                viz_img, (int(right_point2), int(interested_line_y2)), 7, (0, 255, 0), -1)
        center1=(left_point1+right_point1)//2+30
        viz_img = cv2.circle(
                viz_img, (int(center1), int(interested_line_y)), 7, (0, 255, 255), -1)
 
        viz_img = cv2.circle(
                viz_img, (int(center), int(interested_line_y)), 7, (255, 0, 255), -1)
    if draw:
        return left_point1, right_point1, center,check_steering, viz_img
    else:
        return left_point1, right_point1, center,check_steering

# def calculate_control_signal(left_point, right_point, im_center):
#     """Calculate control signal"""
    
#     if left_point == -1 or right_point == -1:
#         left_motor_speed = right_motor_speed = 20
#         return left_motor_speed, right_motor_speed

#     # Calculate difference between car center point and image center point
#     center_point = (right_point + left_point) // 2
#     center_diff = center_point - im_center

#     # Calculate steering angle from center point difference
#     steering = -float(center_diff * 0.03)
#     steering = min(1, max(-1, steering))
#     throttle = 0.75

#     # From steering, calculate left/right motor speed
#     left_motor_speed = 0
#     right_motor_speed = 0

#     if steering > 0:
#         left_motor_speed = throttle * (1 - steering)
#         right_motor_speed = throttle 
#     else:
#         left_motor_speed = throttle 
#         right_motor_speed = throttle * (1 + steering)

#     left_motor_speed = int(left_motor_speed * 100)
#     right_motor_speed = int(right_motor_speed * 100)

#     diff = abs(left_motor_speed - right_motor_speed)
#     if left_motor_speed < 80: #steering > 0
#         left_motor_speed = 80
#         right_motor_speed+= diff / 5.3
#     if right_motor_speed < 80: #steerign < 0
#         right_motor_speed = 80
#         left_motor_speed+= diff / 5.3

    
#     return left_motor_speed, right_motor_speed