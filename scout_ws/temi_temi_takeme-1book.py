#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Shelves and books database
shelves = {
    "shelf_A": {
        "books": ["Book1", "Book2"],
        "pose": {
            "position": {"x": 2.00277423859, "y": 7.8101644516, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.662506972061, "w": 0.749055746905}
        }
    },
    "shelf_B": {
        "books": ["Book3", "Book4"],
        "pose": {
            "position": {"x": -1.50070428848, "y": -3.69934010506, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.971146142256, "w": 0.238485157569}
        }
    }
}

def find_shelf_for_book(book_title):
    """
    Given a book title, find which shelf it belongs to and return that shelf's pose.
    Returns (shelf_name, pose_dict) if found, else (None, None).
    """
    for shelf_name, data in shelves.items():
        if book_title in data["books"]:
            return shelf_name, data["pose"]
    return None, None

def send_goal(pose):
    """
    Sends a navigation goal to the move_base action server using the provided pose dictionary.
    pose should have:
    pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]
    pose["orientation"]["x"], pose["orientation"]["y"], pose["orientation"]["z"], pose["orientation"]["w"]
    """
    # Create an action client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # Create a new goal to send to move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position and orientation
    goal.target_pose.pose.position.x = pose["position"]["x"]
    goal.target_pose.pose.position.y = pose["position"]["y"]
    goal.target_pose.pose.position.z = pose["position"]["z"]

    goal.target_pose.pose.orientation.x = pose["orientation"]["x"]
    goal.target_pose.pose.orientation.y = pose["orientation"]["y"]
    goal.target_pose.pose.orientation.z = pose["orientation"]["z"]
    goal.target_pose.pose.orientation.w = pose["orientation"]["w"]

    rospy.loginfo("Sending goal: x={:.3f}, y={:.3f}, z={:.3f}, ox={:.3f}, oy={:.3f}, oz={:.3f}, ow={:.3f}".format(
        pose["position"]["x"], pose["position"]["y"], pose["position"]["z"],
        pose["orientation"]["x"], pose["orientation"]["y"], pose["orientation"]["z"], pose["orientation"]["w"]
    ))

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('library_assistant_node', anonymous=True)

    # Example: The user requests "Book2"
    requested_book = "Book2"
    rospy.loginfo("User requested: {}".format(requested_book))

    shelf_name, shelf_pose = find_shelf_for_book(requested_book)
    if shelf_name is not None:
        rospy.loginfo("Found {} at {}. Navigating there...".format(requested_book, shelf_name))
        result = send_goal(shelf_pose)
        if result:
            rospy.loginfo("Successfully reached the shelf!")
        else:
            rospy.logwarn("Failed to reach the shelf.")
    else:
        rospy.logwarn("The requested book '{}' was not found on any known shelf.".format(requested_book))
