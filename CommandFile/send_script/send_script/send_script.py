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
from .process_frame import process_frame, visualize_results
import matplotlib.pyplot as plt

import numpy as np

from math import sqrt, atan2

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
            self.get_logger().info('Image received1')
            

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
            x_endE_init=300
            y_endE_init=100
            z_endE_init=500
            
            
            x_between_cam_eE=80
            y_between_cam_eE=0
            z_between_cam_eE=0


            var_initial_eEpoint_base={300.00, 100, 500, -180.00, 0.0, 135.00}
            targetP_int = "300.00, 100, 500, -180.00, 0.0, 90.00"
            self.get_logger().info("moving robot to the initial position1: ")
            self.get_logger().info(targetP_int)
            script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
            send_script(script_int2)

            # targetP_int = "230.00, 230, 730, -180.00, 0.0, 90.00"
            # self.get_logger().info("moving robot to the initial position0: ")
            # self.get_logger().info(targetP_int)
            # script_int = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
            # send_script(script_int)

            # targetP_int = "300.00, 100, 500, -180.00, 0.0, 90.00"
            # self.get_logger().info("moving robot to the initial position1: ")
            # self.get_logger().info(targetP_int)
            # script_int2 = "PTP(\"CPP\","+str(targetP_int)+",100,200,0,false)"
            # send_script(script_int2)

            #Step1. uv-->cam x y z
            u, v = centroids[0] #get the image centroid
            # Draw a circle at the (u, v) point in the image
            cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)  # Red circle with radius 5

            # Optionally, you can draw a line from the centroid to some other point, or add text for more information
            cv2.putText(cv_image, f"({u},{v})", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)


            z_cam=-(z_endE_init+z_between_cam_eE) #our virtual cam frame are set to be align with base frame x, y and the hight of z_endE_init+z_between_cam_eE
            #x_cam,y_cam,z_cam=self.project_pixel_to_point(u, v, z_cam)
            x_cam=(u-self.cx)*45/1280 #1280pixcel at hight z_cam is 45cm (by measured)
            y_cam=(u-self.cy)*33/960
            
            var_pBlock_CamFrame={x_cam,y_cam,z_cam, -180.00, 0.0, 135.00}
            
            self.get_logger().info(f'The image centroid (u, v) is: ({u}, {v})')
            self.get_logger().info(f'Projected 3D camera coordinates_cam are: ({x_cam}, {y_cam}, {z_cam})')
            self.get_logger().info(f'The pBlock in Camera Frame: {var_pBlock_CamFrame}')

            #Step2. cam x y z --> eE xyz #our end Effector frame are also set to be align with base frame x, y
            x_eE=x_cam+x_between_cam_eE
            y_eE=y_cam+y_between_cam_eE
            z_eE=z_cam+z_between_cam_eE
            var_pBlock_endeFrame={x_eE,y_eE,z_eE, -180.00, 0.0, 135.00}
            #float[] var_pBlock_endeFrame={x_eE,y_eE,z_eE,0,0,0}
            self.get_logger().info(f'Block coordinate refer to frame_eE: ({x_eE}, {y_eE}, {z_eE})')
            self.get_logger().info(f'The pBlock in End-Effector Frame: {var_pBlock_endeFrame}')
            
            #Step3. eE xyz --> base xyz
            x_base=x_eE+x_endE_init
            y_base=y_eE+y_endE_init
            z_base=z_eE+z_endE_init#+100

            # yaw = atan2(y_base, x_base)
            # pitch = atan2(z_base, sqrt(x_base**2 + y_base**2))
            # roll = 0.00
            
            #targetP1 = f"{x_base}, {y_base}, {z_base}, {roll}, {pitch}, {yaw}"
            # targetP1 = f"{x_base}, {y_base}, {z_base}, -180.00, 0.0, 135.00"
            # targetP1 = f"{x_base}, {y_base}, {z_base}, -180.00, 0.0, 90.00"
            targetP1 = f"{x_base}, {y_base}, 200, -180.00, 0.0, 90.00"
            
            self.get_logger().info("moving robot to targetP1: ")
            #self.get_logger().info(targetP1)



            #targetP1 = "300.00, 100, 500, -180.00, 0.0, 135.00"
            self.get_logger().info(targetP1)
            script1 = "PTP(\"CPP\","+str(targetP1)+",100,200,0,false)"
            send_script(script1)



            # result_image = visualize_results(annotated_image, objects_info, "Object Detection Results")

            # plt.imshow(result_image)

            cv2.waitKey(1)
            self.get_logger().info('Image received')

            # Display the image using OpenCV
            cv2.imshow("Camera Image", cv_image)
            # cv2.imshow("Camera Image", annotated_image)
            cv2.waitKey(1)

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

    print("HHHHHHAAA")
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
    targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    targetP2 = "300.00, 100, 500, -180.00, 0.0, 135.00"
    script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    # send_script(script1)
    # send_script(script2)

# What does Vision_DoJob do? Try to use it...
# -------------------------------   ------------------
    #send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
    #send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
#--------------------------------------------------
    
    
    set_io(0.0)

    # cv2.waitKey(1)
    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
