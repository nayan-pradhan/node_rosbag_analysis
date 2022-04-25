#! /usr/bin/env python3

from doctest import OutputChecker
import rospy
import rosbag
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

NAME_OF_NODE = "review_error_detection_node"
NAME_OF_BAG_FILE = rospy.get_param("selected_rosbag_file")
PATH_TO_BAG_FILE = rospy.get_param("path_to_rosbag")

class ReviewErrorDetection:

    def __init__(self):
        rospy.loginfo("Reviewing ROS BAG file: "+NAME_OF_BAG_FILE)
        bag = rosbag.Bag(PATH_TO_BAG_FILE + '/' + NAME_OF_BAG_FILE)

        latest_cmd_vel_msg = 0.0
        something_wrong = False
        
        self.min_linear_x_threshold = float(rospy.get_param("min_linear_x_threshold"))

        for topic, msg, t in bag.read_messages(topics=['/mission_active', '/state_machine_event_log', '/cmd_vel']):
            if topic == '/mission_active':
                if msg.data == 'False':
                    rospy.loginfo("MISSION INACTIVE")
                    continue 
                else:
                    pass
            
            elif topic == '/state_machine_event_log':
                try:
                    msg = msg.data
                    msg_starting_index = msg.index('with outcome:')
                except:
                    continue

                outcome_msg = msg[msg_starting_index+14:]

                if outcome_msg == "paused":
                    if latest_cmd_vel_msg < self.min_linear_x_threshold:
                        rospy.logerr("STATE MACHINE WAS PAUSED, NOW NO MOVEMENT, SOMETHING WRONG")
                        something_wrong = True 
            
            elif topic == '/cmd_vel':
                latest_cmd_vel_msg = msg.linear.x

            else:
                rospy.loginfo("Message not received for t: "+str(t))
                continue
        
        if (not something_wrong):
            rospy.loginfo("Everything looks good.")

        rospy.loginfo("End of BAG file reached!")
        bag.close()
        rospy.loginfo("BAG file closed!")


def main():
    global NAME_OF_NODE
    global NAME_OF_BAG_FILE
    global PATH_TO_BAG_FILE

    rospy.init_node(NAME_OF_NODE)
    rospy.loginfo("Starting node:   "+NAME_OF_NODE)

    ReviewErrorDetection()

if __name__ == '__main__':
    main()