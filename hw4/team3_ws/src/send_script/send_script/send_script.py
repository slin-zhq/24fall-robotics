#!/usr/bin/env python

import rclpy
import cv2
import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

#For getting image from robot
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#our orientation code
from .process_frame_unrefactored import process_frame, visualize_results
import matplotlib.pyplot as plt

import numpy as np

from math import sqrt, atan2
import math

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Replace '/camera/image_raw' with the actual camera topic
        self.subscription = self.create_subscription(
            Image,
            '/techman_image',  # Replace with your camera topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()  # Used for converting ROS Image to OpenCV format


        #[[相機參數]]
        # Camera parameters (need to find the one for the tm arm depth camera)
        horizontal_fov =   1.107#1.5009832 #5 #1.5009832 # in radians 1.089
        image_width = 1280#2592#640  # in pixels
        image_height = 960#1944#480  # in pixels

        # Calculate the focal length
        self.fx = image_width / (2 * np.tan(horizontal_fov / 2))
        self.fy = self.fx  # Assuming square pixels, so fx == fy

        # Assume cx and cy are at the center of the image
        self.cx = image_width / 2
        self.cy = image_height / 2







    def image_callback(self, msg):
        try:

            set_io(0.0)
            self.get_logger().info('Image received1')
            
            # set_io(1.0)
            # Convert ROS Image to OpenCV format (BGR)
            # cv2.waitKey(1)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #turn 8UC3 to bgr8(openCV image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv2.imwrite("src/send_script/images/cv_image.jpg", cv_image)

            height, width = cv_image.shape[:2]
            print(f"Height: {height}, Width: {width}")
            # Process the image
            objects_info, annotated_image = process_frame(cv_image, display_steps=False)
            # cv2.imwrite("annotated_image .jpg", annotated_image)
            # print(objects_info)
            self.get_logger().info('The objects_info is: ')
            self.get_logger().info(str(objects_info))

            # Initialize empty lists to store centroids and angles
            centroids = []
            angles_deg = []

            # Iterate over each object in objects_info
            for obj in objects_info:
                # Extract centroid and angle_deg
                centroids.append(obj['centroid'])
                angles_deg.append(obj['angle_deg'])

            # Print the extracted vectors
            self.get_logger().info(f'Centroids: {centroids}')
            self.get_logger().info(f'Angles (in degrees): {angles_deg}')


            #Step0. initialize the robot endeffector position
            #set parameters
            # x_endE_init=300
            # y_endE_init=100
            x_endE_init=230
            y_endE_init=230
                
            
            x_between_cam_eE=86
            y_between_cam_eE= 3.5#9.0#5.0#3.0#9.5#10.5 #off set


            # var_initial_eEpoint_base={300.00, 100, 500, -180.00, 0.0, 135.00}
            # targetP_int = "300.00, 100, 500, -180.00, 0.0, 90.00"
            targetP_int = "230.00, 230, 500, -180.00, 0.0, 90.00" #500
            self.get_logger().info("moving robot to the initial position1: ")
            self.get_logger().info(targetP_int)
            script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
            send_script(script_int2)
            # for i, (centroid, angle_deg) in enumerate(zip(centroids, angles_deg)):
            #     u, v = centroid #get the image centroid
            #     angle_deg1=angle_deg
            #     angle_deg1_pro=angle_deg1#-2#-2.00

            #     # Convert angle to radians
            #     angle_rad_pro = math.radians(angle_deg1_pro)
            #     line_length = 100
            #     end_x = int(u + line_length * math.cos(angle_rad_pro))
            #     end_y = int(v - line_length * math.sin(angle_rad_pro))  # y-coordinate is flipped for image coordinates
            #     self.get_logger().info(f'The image centroid (end_x, end_y) is: ({end_x}, {end_y})')

            #     cv2.arrowedLine(cv_image, (u, v), (end_x, end_y),  (0, 255, 0), 2)
            #     cv2.putText(cv_image, f"{angle_deg1_pro:.1f}°", (u + 25, v - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
            #     # Draw a circle at the (u, v) point in the image
            #     cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)  # Red circle with radius 5
            #     cv2.putText(cv_image, f"({u},{v})", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

            #     # cv2.waitKey(1)
            #     self.get_logger().info('Image received')

            #     # Display the image using OpenCV
            #     # cv2.imshow("Camera Image", cv_image)
            #     # cv2.imshow("Camera Image", annotated_image)
            #     # cv2.waitKey(1)
            # cv2.imshow("Camera Image", cv_image)
            # cv2.waitKey(1)


            goalx=0
            goaly=0
            for i, (centroid, angle_deg) in enumerate(zip(centroids, angles_deg)):
                PositionHight=115
                #Step1. uv-->cam x y z
                u, v = centroid #get the image centroid
                angle_deg1=-angle_deg    #<Debug> The degree should be -degree so that it will be right
                angle_deg1_pro=angle_deg1#-2#-2.00

                # Convert angle to radians
                angle_rad_pro = (angle_deg1*np.pi)/180#math.radians(angle_deg1_pro)
                line_length = 100
                end_x = int(u + line_length * math.cos(angle_rad_pro))
                end_y = int(v - line_length * math.sin(angle_rad_pro))  # y-coordinate is flipped for image coordinates
                self.get_logger().info(f'The image centroid (end_x, end_y) is: ({end_x}, {end_y})')

                cv2.arrowedLine(cv_image, (u, v), (end_x, end_y),  (0, 255, 0), 2)
                cv2.putText(cv_image, f"{angle_deg1_pro:.1f}°", (u + 25, v - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
                # Draw a circle at the (u, v) point in the image
                cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)  # Red circle with radius 5
                cv2.putText(cv_image, f"({u},{v})", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

                cv2.waitKey(1)
                self.get_logger().info('Image received')

                # Display the image using OpenCV
                cv2.imshow("Camera Image", cv_image)
                # cv2.imshow("Camera Image", annotated_image)
                cv2.waitKey(1)

                y_cam=-(u-self.cx)*450/1280 #1280pixcel at hight z_cam is 45cm (by measured)
                x_cam=-(v-self.cy)*330/960
                
                self.get_logger().info(f'The image centroid (u, v) is: ({u}, {v})')
                self.get_logger().info(f'Projected 3D camera coordinates_cam are: ({x_cam}, {y_cam}, xxx)')
            
                #Step2. cam x y z --> eE xyz #our end Effector frame are also set to be align with base frame x, y
                x_eE=x_cam+x_between_cam_eE
                y_eE=y_cam+y_between_cam_eE
                
                #Step3. eE xyz --> base xyz
                x_base=x_eE+x_endE_init
                y_base=y_eE+y_endE_init
                # z_base=z_eE+z_endE_init#+100

                rotationz=angle_deg1#+2.0#(offset)

                #[For Stacking]
                PositionHight = PositionHight + 25*i
                PositionHight_Lift=PositionHight+50
                #move to the upper position first+25 to prevent crash
                # PositionHight_Lift2 = PositionHight_Lift+25
                targetP_int = f"{x_base}, {y_base}, {PositionHight_Lift}, -180.00, 0.0, {rotationz}"
                self.get_logger().info("moving robot to the initial position1: ")
                self.get_logger().info(targetP_int)
                script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
                send_script(script_int2)



                targetP1 = f"{x_base}, {y_base}, 130, -180.00, 0.0, {rotationz}"
                # targetP1 = f"{x_base}, {y_base}, 125, -180.00, 0.0, 90.00"     
                self.get_logger().info("moving robot to targetP1: ")
                #self.get_logger().info(targetP1)
                #targetP1 = "300.00, 100, 500, -180.00, 0.0, 135.00"
                self.get_logger().info(targetP1)
                script1 = "PTP(\"CPP\","+str(targetP1)+",100,200,0,false)"
                send_script(script1)

                targetP2 = f"{x_base}, {y_base}, 120, -180.00, 0.0, {rotationz}" 
                self.get_logger().info("moving robot to targetP2: ")
                self.get_logger().info(targetP2)
                script2 = "PTP(\"CPP\","+str(targetP2)+",100,200,0,false)"
                send_script(script2)

                # rotationz=90.00-angle_deg1-1.58#(offset)
                targetP2 = f"{x_base}, {y_base}, 110, -180.00, 0.0, {rotationz}" 
                self.get_logger().info("moving robot to targetP2: ")
                self.get_logger().info(targetP2)
                script2 = "PTP(\"CPP\","+str(targetP2)+",100,200,0,false)"
                send_script(script2)

                set_io(1.0) #close
                # #[For Stacking]
                # PositionHight = PositionHight + 25*i
                # PositionHight_Lift=PositionHight+30
                #Lift
                targetP_int = f"{x_base}, {y_base}, {PositionHight_Lift}, -180.00, 0.0, {rotationz}"
                self.get_logger().info("moving robot to the initial position1: ")
                self.get_logger().info(targetP_int)
                script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
                send_script(script_int2)
                #move to goal (make the first block as the goal)
                if(i==0):
                    goalx=330.0 #x_base
                    goaly=-80.0 #y_base



                targetP_int = f"{goalx}, {goaly}, {PositionHight}, -180.00, 0.0, {rotationz}"
                self.get_logger().info("moving robot to the initial position1: ")
                self.get_logger().info(targetP_int)
                script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
                send_script(script_int2)

                set_io(0.0) #Drop it

                #Lift before going to other place (or will crack to the done one)
                targetP_int = f"{goalx}, {goaly}, {PositionHight_Lift}, -180.00, 0.0, {rotationz}"
                self.get_logger().info("moving robot to the initial position1: ")
                self.get_logger().info(targetP_int)
                script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
                send_script(script_int2)



        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    #2D投影到3D
    def project_pixel_to_point(self, u, v, z):
        if z <= 0:
            return None  # Depth must be positive

        # Reconstruct the 3D point from the 2D pixel coordinates and the depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        return x, y, z



# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

# gripper client
def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):

    print("H")
    rclpy.init(args=args)

    #Create our node
    node = CameraSubscriber()




    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, -180.00, 0.0, 135.00"
    set_io(0.0)
    targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    targetP2 = "300.00, 100, 500, -180.00, 0.0, 135.00"
    script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    # send_script(script1)
    # send_script(script2)
    targetP_int = "230.00, 230, 500, -180.00, 0.0, 90.00"
    print("moving robot to the initial position1: "+targetP_int)
    #self.get_logger().info(targetP_int)
    script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
    send_script(script_int2)
    # set_io(0.0)
    # set_io(0.0)
# What does Vision_DoJob do? Try to use it...
# -------------------------------   ------------------
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
    # send_script("Vision_DoJob(job1)") #<Debug> need t
    cv2.waitKey(1)
#--------------------------------------------------
    # set_io(0.0)
    
    

    # cv2.waitKey(1)
    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
