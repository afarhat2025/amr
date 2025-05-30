#!/usr/bin/env python3

import rclpy,os
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile
import numpy as np
import cv2 as cv
import cv_bridge
from cv_bridge import CvBridgeError
import time


"""Line Follower class for docking to charger"""
global error
global image_input
global image_input_hsv
global just_seen_line
global just_seen_right_mark
global should_move
global right_mark_count
global finalization_countdown
global image_input_shape
global last_valid_error
global latch_count
global area
global searching_for_line,search_step, search_direction, step_counter
global height_start, height_stop, width_start, width_stop
search_step = 0
search_direction = 1
step_counter = 0
searching_for_line = False
green_detected_frames = 0
green_lost_frames = 0
height_start = 375
height_stop = 480
width_start = 0
width_stop = 960
latch_count = False
image_input = 0
image_input_shape = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None
last_valid_error = 0
green_fade_frames =0 
bridge = cv_bridge.CvBridge()
MIN_AREA = 2000
MIN_AREA_TRACK = 1800
LINEAR_SPEED = 0.06
KP = 0.0009 #0.0009
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.1
FINALIZATION_PERIOD = 2
MAX_ERROR = 225
MIN_ERROR = -225
width_offset = -90
cycle_count = 0
first_time = False
lower_green = np.array([40,  17,  0])
upper_green = np.array([85, 70, 235])
prev_gray = None
shape = None
area = 0
green_history = []
request_to_check = True

# lower_green = np.array([0,  8,  120])
# upper_green = np.array([100, 255, 255]) 

#lower_blue = np.array([40,40,50])
#upper_blue = np.array([120,255,255])
#lower_red = np.array([0,40,150])
#upper_red = np.array([180,255,200])

lower_red = np.array([0, 32, 50]) 
upper_red = np.array([179, 255, 119])
lower_red_2 = np.array([0, 32, 50])
upper_red_2 = np.array([15, 255, 255])
counter = 0

robot_name = (os.getenv('ROBOT_MODEL','amr_x'))


def start_follower_callback( request, response):
    """
    Start the robot.
    """
    global should_move
    global right_mark_count
    global finalization_countdown
    global searching_for_line
    global first_time
    if area > 5000 and first_time:
        print("Line detected! Starting line following.")
        should_move = True
    elif area < 5000 and first_time:
        print("Line not detected! Initiating search.")
        #searching_for_line = True
        search_step = 0  # Start search sequence
        step_counter = 0

    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    return response

def stop_follower_callback( request, response):
    """
    Stop the robot
    """
    global should_move
    global finalization_countdown
    should_move = False
    finalization_countdown = None
    searching_for_line = False
    return response

def image_callback(data):

    global image_input
    global image_input_shape
    global image_input_hsv
    try:
        
        image_input = bridge.imgmsg_to_cv2(data, 'bgra8')
        image_input_hsv = cv.cvtColor(image_input,cv.COLOR_BGR2HSV)

    except CvBridgeError as e:
        print(f"CV Bridge Error: {e}")

def nothing(x):
    pass

def find_hsv_values():

    """
    only call this function for first time
    to find masking values in new mapped environment
    """
    cv.namedWindow('image')
    cv.createTrackbar('HMin', 'image', 0, 255, nothing)
    cv.createTrackbar('SMin', 'image', 0, 255, nothing)
    cv.createTrackbar('VMin', 'image', 0, 255, nothing)
    cv.createTrackbar('HMax', 'image', 0, 255, nothing)
    cv.createTrackbar('SMax', 'image', 0, 255, nothing)
    cv.createTrackbar('VMax', 'image', 0, 255, nothing)

    # Set default value for Max HSV trackbars
    cv.setTrackbarPos('HMax', 'image', 255)
    cv.setTrackbarPos('SMax', 'image', 255)
    cv.setTrackbarPos('VMax', 'image', 255)

    # Initialize HSV min/max values
    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0

    while(1):
        # Get current positions of all trackbars
        hMin = cv.getTrackbarPos('HMin', 'image')
        sMin = cv.getTrackbarPos('SMin', 'image')
        vMin = cv.getTrackbarPos('VMin', 'image')
        hMax = cv.getTrackbarPos('HMax', 'image')
        sMax = cv.getTrackbarPos('SMax', 'image')
        vMax = cv.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Convert to HSV format and color threshold
        hsv = cv.cvtColor(image_input, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lower, upper)
        result = cv.bitwise_and(image_input, image_input, mask=mask)

        # Print if there is a change in HSV value
        if((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
            print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax

        # Display result image
        resized_image = cv.resize(result, (800, 600))
        cv.imshow('image', resized_image)
        cv.waitKey(5)
        
        
def timer_callback():
    """
    Main Driving Function
    """
    global error
    global image_input
    global image_input_shape
    global image_input_hsv
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global green_detected_frames
    global green_lost_frames
    global right_mark_count
    global finalization_countdown
    global crop_w_start
    global crop_h_start
    global last_valid_error
    global lower_green
    global upper_green
    global latch_count
    global first_time
    global green_fade_frames
    global counter
    global searching_for_line, search_step, search_direction, step_counter, should_move,cycle_count,prev_gray,request_to_check
    global height_start, height_stop, width_start, width_stop

    if type(image_input) != np.ndarray:
        print("i failed")
        return
    else:
        first_time = True
        latch_count = True
    """
    uncomment below for first time to 
    teach mask in new environment for ease
    #"""
    #find_hsv_values()
    height, width, _ = image_input_hsv.shape
    image_hsv = image_input_hsv.copy()
    # crop_h_start = height_start
    # crop_h_stop  = height_stop
    # crop_w_start = width_start
    # crop_w_stop  = width_stop
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)
    crop = image_hsv[crop_h_start:crop_h_stop,crop_w_start:crop_w_stop]
    #crop = image_hsv[(crop_h_start):(crop_h_stop),(crop_w_start+400):(crop_w_stop-230)]
    #crop = image_hsv[(crop_h_start):(crop_h_stop),(crop_w_start+400):(crop_w_stop-230)]
    crop_red = image_hsv[(crop_h_start+125):crop_h_stop-25,crop_w_start+125:crop_w_stop]

    #mask_red = cv.inRange(crop_red, lower_red, upper_red)
    mask_red_1 = cv.inRange(crop_red, lower_red, upper_red)
    mask_red_2 = cv.inRange(crop_red, lower_red_2, upper_red_2)
    mask_red = cv.bitwise_or(mask_red_1,mask_red_2)
    h, s, v = cv.split(crop_red)
    _, sat_mask = cv.threshold(s, 30, 255, cv.THRESH_BINARY)
    _, val_mask = cv.threshold(v, 90, 255, cv.THRESH_BINARY)
    #print(f"Avg Sat: {np.mean(s):.1f} | Avg Val: {np.mean(v):.1f}")
    mask_red_filtered = cv.bitwise_and(mask_red, sat_mask)
    mask_red_filtered = cv.bitwise_and(mask_red_filtered, val_mask)
    mask = cv.inRange(crop,lower_green,upper_green)
    
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (2,2))
    combined_mask = cv.bitwise_and(mask_red, cv.bitwise_and(sat_mask, val_mask))

    
   
    noise_green = cv.morphologyEx(mask, cv.MORPH_RECT, kernel, iterations=1)
    noise_red = cv.morphologyEx(combined_mask, cv.MORPH_CLOSE, kernel, iterations=1)
    red_area = cv.countNonZero(noise_red)
    avg_green = cv.countNonZero(noise_green)
    print("Average green: {}".format(avg_green))
    print("Average red:", red_area)
    while red_area > 9200 and counter == 0:
        stop_robot()
        user_input = input("Press 'q' to continue, mimicing server")
        print("Average red:", red_area)
        if user_input.lower() == 'q':
            counter += 1
            break
    cv.imshow("noise green",noise_green
              )
    cv.imshow("noise red",noise_red)
    #cv.imshow("blurred",crop_blur)
    cv.waitKey(1)

    line, mark_side, area = get_contour_data(noise_green, image_hsv[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
    print("Area: {}".format(area))
    #shape = get_shape_data(noise_red, mask_red)

    # #print("Area: {}".format(area))
    # if smoothed_green > 20000 and not latch_count and error > -250 and error < 250:
    #     lower_green = np.array([40, 0,  0])
    #     upper_green = np.array([90, 60, 250])
    #     print("Green latched ")
    #     latch_count = True

    # elif ((smoothed_green <= 100 and red_area > 25000 and area <= 2000) or error <= -250 or error >= 250 or smoothed_green >= 41000):
    #     green_detected_frames += 1 
    #     time.sleep(0.05)
    #     print("I am in the else")
    #     if green_detected_frames > 3:
    #         latch_count = False

    # elif smoothed_green <= 16000 and error > -250 and error < 250:
    #     print("I switched the dynamic HSV values")
    #     lower_green = np.array([25, 0, 0])
    #     upper_green = np.array([100, 22, 200])



    message = Twist()
    
    if last_valid_error is None:
        last_valid_error = 0

    if line:
        x = line['x']
        error = x - width // 2 + width_offset
        message.linear.x = LINEAR_SPEED
        just_seen_line = True
        # if error < -250:  
        #     error = max(error, 5)  
        #     message.angular.z = float(error) * -KP
        # elif error > 250:  
        #     error = min(error,-5)  
        #     message.angular.z = float(error) * -KP
        # else:  
        

        # Save the current error as last valid error for future reference
            #last_valid_error = error

    else:
        if just_seen_line:
            just_seen_line = False
            error = 0
        message.linear.x = 0.0
    # if mark_side != None:
    #     if (mark_side == "right") and (finalization_countdown == None) and \
    #         (error <= MAX_ERROR) and (not just_seen_right_mark):

    #         right_mark_count += 1

    #         if right_mark_count > 1:
    #             finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
    #             print("Finalization Process has begun!")   
    #         just_seen_right_mark = True
    # else:
    #     just_seen_right_mark = False
    # if error > 250:
    #     error = min(error,5)
    #     message.angular.z = float(error) * -KP
    # elif error < -250:
    #     error = max(error,-5)
    #     message.angular.z = float(error) * -KP
    # else:
    #     message.angular.z = float(error) * -KP
    if   abs((float(error) * -KP)) >= 0.20:
        print(f"the error is: {(float(error) * -KP)}")
        message.angular.z = 0.000
        #message.linear.x = 0.000
    # elif (float(error) * -KP) <= -0.12:
    #     message.angular.z = -0.000
    #     message.linear.x = 0.000
    else:
        message.angular.z = float(error) * -KP
    # if avg_green > 22000 and not latch_count and error > -200 and error < 200:
    #     lower_green = np.array([40, 0, 0])
    #     upper_green = np.array([90, 60, 250])
    #     print("Green latched ")
    #     # height_start = 375
    #     # height_stop = 480
    #     # width_start = 0
    #     # width_stop = 960
    #     latch_count = True
    # elif avg_green <= 15000 and latch_count and error > -200 and error < 200:
    #     print("I switched the dynamic HSV values")
    #     lower_green = np.array([20, 0, 0])
    #     upper_green = np.array([130, 25, 200])
    #     # height_start = 375
    #     # height_stop = 480
    #     # width_start = 300
    #     # width_stop = 600
    # elif avg_green <= 20000 and (error >= 210 or error <= -210) or area < 2500 or abs(message.angular.z) >= 0.1:
    #     lower_green = np.array([40, 0, 0])
    #     upper_green = np.array([90, 60, 250])
    #     latch_count = False
    # if (avg_green <= 8000 or area < 250 or abs(error) > 300 or abs(message.angular.z) > 0.2):
        
    #     green_lost_frames += 1
    #     if green_lost_frames > 5:
    #         latch_count = False
    #         print("Latch reset due to out-of-zone condition")
    #         request_to_check = True
    if avg_green > 40000 or abs(message.angular.z) > 0.25:
        request_to_check = True
        latch_count = False

    if avg_green > 12000 and not latch_count and request_to_check and abs(error) < 250:
        green_lost_frames = 0
        green_detected_frames += 1
        lower_green = np.array([40, 17, 0])
        upper_green = np.array([85, 70, 235])
        if green_detected_frames > 5:
            print("Green latched ")
            green_detected_frames = 0
            latch_count = True
            request_to_check = False
        

    # elif (green_pixels <= 20000 and (error < -210 or error > 210) or area < 2500 or abs(message.angular.z) > 0.12):
    #     lower_green = np.array([37, 0, 0])
    #     upper_green = np.array([90, 60, 250])
    #     latch_count = False
    #     get_logger().info("In reset condition")

    if avg_green <= 8000 and abs(error) < 250 and latch_count:
        green_lost_frames += 1
        
        print("I switched the dynamic HSV values")
        lower_green = np.array([30, 0, 0])
        upper_green = np.array([130, 255, 200])
        if abs(message.angular.z) > 0.14 or abs(error) > 240:
            print("In reset condition")
            latch_count = False
        
        

    print("Error: {} | Mark side: {} | x: {} | z: {}, ".format(error, mark_side,message.linear.x, message.angular.z))
    #cv.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
    if finalization_countdown != None:
        if finalization_countdown > 0:
            finalization_countdown -= 1
            print("Finalization Process is ongoing! Countdown: {}".format(finalization_countdown))

        elif (finalization_countdown == 0 or finalization_countdown < 0) and red_area > 20000:
            print("Finalization Process has ended!")
            
            #should_move = False

            
    if searching_for_line:
        line, mark_side, area = get_contour_data(noise_green, image_hsv[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
        time.sleep(0.5)
        if area > 3500:
            print("Line found! Stopping search and starting line following.")
            searching_for_line = False
            stop_robot()
            should_move = True
            return

        twist = Twist()
        print("Area: {}".format(area))

            # Phase 1: Move forward in small steps
        if search_step == 0:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            publisher.publish(twist)
            time.sleep(1.0)
            step_counter += 1
            if area > 3500:
                stop_robot()
                return
            if step_counter >= 4:
                search_step = 1  # Switch to right twirl
                step_counter = 0

        # Step 2: Twirl right in steps, checking for green
        elif search_step == 1:
            twist.linear.x = 0.0
            twist.angular.z = -0.2
            publisher.publish(twist)
            time.sleep(1.0)
            step_counter += 1
            if area > 3500:
                stop_robot()
                return
            if step_counter >= 4:
                search_step = 2  # Switch to left twirl
                step_counter = 0
        
        # Step 3: Twirl center, checking for green
        elif search_step == 2:
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            publisher.publish(twist)
            time.sleep(1.0)
            step_counter += 1
            if area > 3500:
                stop_robot()
                return
            if step_counter >= 4:
                search_step = 3 
                step_counter = 0

        # Step 3: Twirl left, checking for green
        elif search_step == 3:
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            publisher.publish(twist)
            time.sleep(1.0)
            step_counter += 1
            if area > 3500:
                stop_robot()
                return
            if step_counter >= 4:
                search_step = 4 
                step_counter = 0

        # Step 3: Twirl center, checking for green
        elif search_step == 4:
            twist.linear.x = 0.0
            twist.angular.z = -0.2
            publisher.publish(twist)
            time.sleep(1.0)
            step_counter += 1
            if area > 3500:
                stop_robot()
                return
            if step_counter >= 4:
                search_step = 0 
                step_counter = 0
                cycle_count += 1
                print(f"Cycle {cycle_count} completed.")
                

        # Stop after 3 full cycles if no green detected
        if cycle_count >= 10:
            print("Search completed after 3 cycles. No green found.")
            searching_for_line = False
            stop_robot()
    
    if should_move:
        publisher.publish(message)
    else:
        empty_message = Twist()
        publisher.publish(empty_message)

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
    Width_left_boundary, Width_right_boundary)
    """
    #print(f"height: {height}")
    #print(f"width: {width}")
    #return (5*height//8, height, width//4, 3*width//4)
    return (375, height, 0, 960) #zed
    #return (0,height,0,width) #480,#360

def get_contour_data(mask, out):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line) 
    and return the side in which the smaller contour is (the track mark) 
    (If there are any of these contours),
    and draw all contours on 'out' image
    """ 
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #contours = sorted(contours, key=cv.contourArea, reverse=True)

    mark_side =  None
    mark = {}
    line = {}
    area = 0
    

    for contour in contours:
        area = cv.contourArea(contour)
        x, y, w, h = cv.boundingRect(contour)
        aspect_ratio = float(w) / h
        solidity = cv.contourArea(contour) / (w * h)
        
        
        if y + h < mask.shape[0] * 0.6:
            continue
        #print(f"Aspect ratio: {aspect_ratio} and solidity: {solidity}")
        if aspect_ratio > 4.8 or aspect_ratio < 0.5:
            
            continue
        if solidity < 0.1:
            
            continue
        #print("Area: {}".format(area))
        #if area > 2000:
        
        #print("Area: {}".format(area))
        M = cv.moments(contour)
        rect = cv.minAreaRect(contour)
        angle = rect[2]
        #if not (60 <= angle <= 95):
        #    continue
        box = cv.boxPoints(rect)
        box = np.int0(box)
        #print(f"Track center: Angle: {angle:.2f}")
        #print("m00 min area: {}".format(M['m00']))
        if M['m00'] > MIN_AREA:
            #print("m00: {}".format (M['m00']))

            if (M['m00'] > MIN_AREA_TRACK):
                # Contour is part of the track
                line['x'] = (crop_w_start+0) + int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])
                cv.circle(out, (crop_w_start + line['x'], crop_h_start + line['y'] ), 5, (0, 0, 255), -1)
            else:
                    # Contour is a track mark
                    print("i am here")
                    if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                        # if there are more than one mark, consider only 
                        # the one closest to the robot 
                        mark['y'] = int(M["m01"]/M["m00"])
                        mark['x'] = crop_w_start + int(M["m10"]/M["m00"])
                
    if 'x' in line and 'y' in line:
        if line['x'] > line['y']:
            mark_side = "right"
        else:
             mark_side = "left"
    else:
        mark_side = None
    return (line, mark_side,area)

def get_shape_data(crop_shape, output):
    global shape
    

    threshold = cv.adaptiveThreshold(crop_shape, 255, cv.ADAPTIVE_THRESH_MEAN_C, 
                                 cv.THRESH_BINARY, 11, 2)
    #_, threshold = cv.threshold(crop_shape, 200, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    i = 0
    for contour in contours:
        area = cv.contourArea(contour)
        if i == 0: 
            i = 1
            continue
        approx = cv.approxPolyDP(contour, 0.01 * cv.arcLength(contour, True), True) 
        cv.drawContours(output, [contour], 0, (0, 0, 255), 5) 

        M = cv.moments(contour)
        if M['m00'] != 0.0: 
            x = int(M['m10'] / M['m00']) 
            y = int(M['m01'] / M['m00']) 

            # Check for quadrilateral (4 vertices)
            if len(approx) == 4:
                # Compute the bounding box of the contour
                x, y, w, h = cv.boundingRect(approx)

                # Compute the aspect ratio (width/height)
                aspect_ratio = float(w) / h

                # Check if the aspect ratio is reasonable for a quadrilateral (close to 1 for square, slightly more for rectangle)
                if 0.8 <= aspect_ratio <= 1.2:
                    cv.putText(output, 'Square', (x, y), 
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    shape = 'Square'
                else:
                    cv.putText(output, 'Rectangle', (x, y), 
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    shape = 'Rectangle'
            else:
                shape = 'None'
    
    return shape



def stop_robot():
    """Stop all motion."""
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    publisher.publish(twist)



def main():
    rclpy.init()
    global node
    node = Node('LineFollower')
    global publisher
    publisher = node.create_publisher(Twist, '/'+robot_name+'/cmd_vel_tracker', 10)
    subscription = node.create_subscription(Image, '/'+robot_name+'/zed_node/rgb/image_rect_color',
                                            image_callback,
                                            QoSProfile(depth=10))

    timer = node.create_timer(0.08, timer_callback)

    start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
    stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)

    rclpy.spin(node)

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    publisher.publish(empty_message)

    node.destroy_node()
    rclpy.shutdown()
    exit()

        
if __name__ == '__main__':
    main()