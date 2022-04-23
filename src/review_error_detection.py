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

        latest_is_mission_active_msg = Bool() 
        latest_state_machine_event_log_msg = String() 
        latest_cmd_vel_msg = Twist()

        recheck = False

        counter = 0

        for topic, msg, t in bag.read_messages(topics=['/mission_active', '/state_machine_event_log', '/cmd_vel']):
            
            check_return_msg = ""
            check_return_msg = "ROBOT RUNNING AS EXPECTED"

            if topic == '/mission_active':
                latest_is_mission_active_msg = msg

                if latest_is_mission_active_msg.data == "False":
                    check_return_msg = "MISSION INACTIVE"
                    rospy.loginfo(check_return_msg)
                    continue

            elif topic == '/cmd_vel':
                latest_cmd_vel_msg = msg

            elif topic == '/state_machine_event_log':
                latest_state_machine_event_log_msg = msg
                latest_state_machine_event_log_msg = latest_state_machine_event_log_msg.data

                try:
                    outcome_msg_starting_index = latest_state_machine_event_log_msg.index('with outcome:')
                except:
                    rospy.loginfo(check_return_msg)
                    continue

                outcome_msg = latest_state_machine_event_log_msg[outcome_msg_starting_index+14:]

                # rospy.logerr(outcome_msg)
                print(counter)

                if recheck and latest_cmd_vel_msg.linear.x < 0.15:
                    counter += 1
                else:
                    counter = 0
                    recheck = False

                if counter > 5:
                    check_return_msg = "STATE MACHINE WAS PAUSED or TRIED RETRY, NOW NO MOVEMENT, SOMETHING WRONG"
                    rospy.logerr(check_return_msg)
                    

                if outcome_msg == "paused" or outcome_msg == "retry":
                    recheck = True
                else:
                    recheck = False


                # if recheck and counter > 2:
                #     if latest_cmd_vel_msg.linear.x < 0.15:
                #         check_return_msg = "STATE MACHINE WAS PAUSED or TRIED RETRY, NOW NO MOVEMENT, SOMETHING WRONG"
                #         rospy.logerr(check_return_msg)
                # else: 
                #     rospy.loginfo(check_return_msg)

                # if (outcome_msg == "paused" or outcome_msg == "retry"):
                #     recheck = True 
                #     counter += 1 
                # else:
                #     recheck = False 
                #     counter = 0 

                # if outcome_msg == "paused" or outcome_msg == "retry" or recheck:
                #     if latest_cmd_vel_msg.linear.x < 0.15:
                #         if recheck and counter > 9:
                #             # print(counter)
                #             check_return_msg = "STATE MACHINE WAS PAUSED or TRIED RETRY, NOW NO MOVEMENT, SOMETHING WRONG"
                #             rospy.logerr(check_return_msg)
                #         else:
                #             rospy.loginfo(check_return_msg)
                #         recheck = True
                #         counter+=1
                #         # continue
                # else:
                #     recheck = False
                #     counter = 0
                #     rospy.loginfo(check_return_msg)
                #     continue

            else:
                rospy.loginfo("Message not received for t: "+str(t))
                continue

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