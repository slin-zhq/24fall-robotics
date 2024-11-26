#!/usr/bin/env python

import rclpy
import cv2
import sys
from math import sqrt, atan2
from tmrobot import robot
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

    def image_callback(self, msg):
        try:
            self.get_logger().info('Image received1')
            

            # Convert ROS Image to OpenCV format (BGR)
            # cv2.waitKey(1)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #turn 8UC3 to bgr8(openCV image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv2.imwrite("src/send_script/images/cv_image.jpg", cv_image)

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


            targetP1 = f"{x}, {y}, {730.00}, -180.00, 0.0, 135.00"
            # targetP1 = "300.00, 100, 500, -180.00, 0.0, 135.00"
            self.get_logger().info(targetP1)
            script1 = "PTP(\"CPP\","+str(targetP1)+",100,200,0,false)"
            send_script(script1)

            # define position

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

######################################################################################################################
    # align camera scale to actual robot scale -- some multiplication I guess
    safe_pos = f"300.00, 100, 500, -180.00, 0.0, 135.00" ## to be tuned later -- need to be higher than 4 stacks so it won't collapse the tower
    script1 = "PTP(\"CPP\","+safe_pos+",100,200,0,false)"
    send_script(script1)
    robot.move_linear(*safe_pos, speed=50)
        # align camera scale to actual robot scale -- some multiplication I guess

    blogs = len(centroids)
    stack = 0

    while(blogs >= 0):
        try:
            x, y = centroids[stack]# retrieve centroid from callback
            x += 300.00
            y += 100.00
            z = 25.00 + stack*50.00 # I guess number
            stack += 1

            reachable = robot.is_reachable(x, y, z)
            if not reachable:
                print("Position is not reachable!")

            # around z
            yaw = atan2(y,x) # not sure if it's what chat means by arctan2
            # around y
            pitch = atan2(z,sqrt(x**2 + y**2))
            # around x -- gripper
            roll = 0 
            ################## (x, y, z) + end effector position #####################
            targetP1 = f"{x}, {y}, {z}, {roll}, {pitch}, {yaw}"
            scriptP1 =  "PTP(\"CPP\","+targetP1+",100,200,0,false)" # What the number here do again? -- need recheck
            
            blogs -= 1
        except:
            raise("Error")

    print("Task completed!")
        



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
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)
#--------------------------------------------------
    
    
    set_io(0.0)

    # cv2.waitKey(1)
    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
