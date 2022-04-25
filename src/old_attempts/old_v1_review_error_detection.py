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

        self.wait_seconds_threshold = int(rospy.get_param("wait_seconds_threshold"))
        self.min_linear_x_threshold = float(rospy.get_param("min_linear_x_threshold"))


        all_msgs = []

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

                all_msgs.append(outcome_msg)

                if outcome_msg == "paused":
                    rospy.logerr("STATE MACHINE WAS PAUSED, SOMETHING WRONG")
            
            elif topic == '/cmd_vel':
                all_msgs.append(msg.linear.x)

            else:
                rospy.loginfo("Message not received for t: "+str(t))
                continue

        # self.check_msgs(all_msgs)

        rospy.loginfo("End of BAG file reached!")
        bag.close()
        rospy.loginfo("BAG file closed!")


    def check_msgs(self, all_msgs):
        # print(all_msgs)
        flag = False 
        counter = 0

        for msg in all_msgs:

            if type(msg) == float:
                if flag:
                    if msg > self.min_linear_x_threshold:
                        flag = False 
                        counter = 0
                    else:
                        if counter > self.wait_seconds_threshold:
                            rospy.logerr("STATE MACHINE TRIED RETRY or PAUSED, NOW NO MOVEMENT, SOMETHING WRONG")
                        counter += 1
            else:
                
                if msg == 'retry' or msg == 'paused':
                    flag = True
                else: 
                    # flag = False 
                    rospy.loginfo("Robot running as expected.")


def main():
    global NAME_OF_NODE
    global NAME_OF_BAG_FILE
    global PATH_TO_BAG_FILE

    rospy.init_node(NAME_OF_NODE)
    rospy.loginfo("Starting node:   "+NAME_OF_NODE)

    ReviewErrorDetection()

if __name__ == '__main__':
    main()