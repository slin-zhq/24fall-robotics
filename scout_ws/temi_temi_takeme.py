#!/usr/bin/env python
import rospy
import actionlib
import math
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Shelves data structure
# Each shelf has a pose and a list of books.
shelves = {
    "shelf_A": {
        "books": ["A1", "A2"],
        "pose": {
            "position": {"x": 2.00277423859, "y": 7.8101644516, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.662506972061, "w": 0.749055746905}
        }
    },
    "shelf_B": {
        "books": ["B1"],
        "pose": {
            "position": {"x": -1.50070428848, "y": -3.69934010506, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": -0.971146142256, "w": 0.238485157569}
        }
    },
    "shelf_C": {
        "books": ["C1", "C2", "C3"],
        "pose": {
            "position": {"x": -5.1686797142, "y": 5.78412437439, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.795245920971, "w": 0.606286999019}
        }
    }
}

# Starting pose of the robot (assume known)
starting_pose = {
    "position": {"x": 0.120400190353, "y": -0.066145285964, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": -0.0733628122528, "w": 0.997305318234}
}

def find_shelf_for_book(book_title):
    """
    Given a book title, find which shelf it belongs to.
    Returns (shelf_name, pose_dict) if found, else (None, None).
    """
    for shelf_name, data in shelves.items():
        if book_title in data["books"]:
            return shelf_name, data["pose"]
    return None, None

def send_goal(pose):
    """
    Sends a navigation goal to move_base action server.
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

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

    rospy.loginfo("Sending goal to: x={:.2f}, y={:.2f}".format(
        pose["position"]["x"], pose["position"]["y"]
    ))
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def euclidean_distance(pose1, pose2):
    """
    Computes Euclidean distance between two poses.
    """
    dx = pose1["position"]["x"] - pose2["position"]["x"]
    dy = pose1["position"]["y"] - pose2["position"]["y"]
    return math.sqrt(dx*dx + dy*dy)

def compute_visit_order(start_pose, shelves_to_visit):
    """
    Given a starting pose and a list of shelves (each with a pose),
    returns an ordered list of shelves to visit based on shortest path heuristic.
    This uses a greedy approach: at each step, go to the closest shelf.
    """
    order = []
    current_pose = start_pose.copy()
    remaining = shelves_to_visit[:]

    while remaining:
        # Find closest shelf
        closest_shelf = None
        closest_dist = float('inf')
        for shelf_name, shelf_pose in remaining:
            dist = euclidean_distance(current_pose, shelf_pose)
            if dist < closest_dist:
                closest_dist = dist
                closest_shelf = (shelf_name, shelf_pose)
        
        order.append(closest_shelf)
        current_pose = closest_shelf[1]
        remaining.remove(closest_shelf)

    return order

if __name__ == '__main__':
    rospy.init_node('library_assistant_node', anonymous=True)

    # 1. Get user input for multiple books
    requested_books = []
    rospy.loginfo("Please enter the books you want (one per line). Type 'done' when finished:")

    try:
        while True:
            try:
                user_input = raw_input("Enter book name: ").strip()  # For Python 2. If Python 3, use input().
                if user_input.lower() == "done":
                    break
                if not user_input:
                    rospy.logwarn("Empty input. Please enter a valid book name.")
                    continue
                requested_books.append(user_input)
            except KeyboardInterrupt:
                rospy.loginfo("User terminated input. Exiting.")
                sys.exit(0)
    except Exception as e:
        rospy.logerr("An error occurred while reading input: {}".format(e))
        sys.exit(1)

    if not requested_books:
        rospy.loginfo("No books requested. Exiting.")
        sys.exit(0)

    rospy.loginfo("User requested books: {}".format(requested_books))

    # 2. Find which shelves need to be visited for these books
    shelves_to_visit = {}  # {shelf_name: pose}
    for book in requested_books:
        shelf_name, shelf_pose = find_shelf_for_book(book)
        if shelf_name is not None:
            shelves_to_visit[shelf_name] = shelf_pose
        else:
            rospy.logwarn("Book '{}' not found on any shelf.".format(book))

    if not shelves_to_visit:
        rospy.loginfo("No valid shelves to visit for requested books.")
        sys.exit(0)

    # 3. Determine order of shelves to visit (greedy shortest path)
    shelf_list = [(name, pose) for name, pose in shelves_to_visit.items()]
    visit_order = compute_visit_order(starting_pose, shelf_list)

    rospy.loginfo("Determined visit order: {}".format([s[0] for s in visit_order]))

    # 4. Execute navigation to each shelf in order
    for shelf_name, shelf_pose in visit_order:
        rospy.loginfo("Navigating to shelf '{}'...".format(shelf_name))
        result = send_goal(shelf_pose)
        if result:
            rospy.loginfo("Arrived at shelf '{}'. Waiting 5 seconds...".format(shelf_name))
            rospy.sleep(5.0)
        else:
            rospy.logwarn("Failed to reach shelf '{}'. Continuing to next...".format(shelf_name))

    # 5. Return to the starting pose
    rospy.loginfo("All requested shelves visited. Returning to starting pose...")
    return_result = send_goal(starting_pose)
    if return_result:
        rospy.loginfo("Successfully returned to the starting position.")
    else:
        rospy.logwarn("Failed to return to the starting position.")
