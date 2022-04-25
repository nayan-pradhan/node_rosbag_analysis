#! /usr/bin/env python3

from doctest import OutputChecker
import rospy
import rosbag
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

NAME_OF_NODE = "real_time_error_detection_node"
NAME_OF_BAG_FILE = rospy.get_param("selected_rosbag_file")
PATH_TO_BAG_FILE = rospy.get_param("path_to_rosbag")

class RealTimeErrorDetection:

    def __init__(self):
        self.callback_latest_is_mission_active_msg = Bool() 
        self.callback_latest_state_machine_event_log_msg = String()
        self.callback_latest_cmd_vel_msg = Twist()

        ##> get parameter from launch file
        self.min_linear_x_threshold = float(rospy.get_param("min_linear_x_threshold"))

        ##> subscribers
        rospy.Subscriber("/mission_active", Bool, self.callback_set_is_mission_active_msg)
        rospy.Subscriber("/state_machine_event_log", String, self.callback_set_state_mission_event_log_msg)
        rospy.Subscriber("/cmd_vel", Twist, self.callback_set_cmd_vel_msg)
        # rospy.Subscriber("/scan", LaserScan, self.callback_laser_scan)
        
        ##> while rospy is running
        while not rospy.is_shutdown(): 
            latest_is_mission_active_msg = self.get_latest_is_mission_active_msg()
            latest_state_machine_event_log_msg = self.get_latest_state_machine_event_log_msg()
            latest_cmd_vel_msg = self.get_latest_cmd_vel_msg()

            is_everything_fine, check_return_msg = self.check_latest_msgs(latest_is_mission_active_msg, latest_state_machine_event_log_msg, latest_cmd_vel_msg)
                
            if (not is_everything_fine):
                ##> loop until linear velocity in x axis of robot is greater than threshold
                while (latest_cmd_vel_msg.linear.x < self.min_linear_x_threshold or not rospy.is_shutdown):
                    latest_cmd_vel_msg = self.get_latest_cmd_vel_msg()
                    rospy.logerr(check_return_msg)


    ##> callback for laser scan message (not used)
    def callback_laser_scan(self, msg):
        print("--------")
        print("Right:", msg.ranges[0])
        print("Forward:", msg.ranges[166])
        print("Left:", msg.ranges[332])
        print("--------")


    ##> for setting and getting is mission active msg
    def callback_set_is_mission_active_msg(self, msg):
        self.callback_latest_is_mission_active_msg = msg

    def get_latest_is_mission_active_msg(self):
        return self.callback_latest_is_mission_active_msg


    ##> for setting and getting state mission event log msg
    def callback_set_state_mission_event_log_msg(self, msg):
        self.callback_latest_state_machine_event_log_msg = msg 

    def get_latest_state_machine_event_log_msg(self):
        return self.callback_latest_state_machine_event_log_msg
    

    ##> for setting and getting cmd vel msg
    def callback_set_cmd_vel_msg(self, msg):
        self.callback_latest_cmd_vel_msg = msg

    def get_latest_cmd_vel_msg(self):
        return self.callback_latest_cmd_vel_msg


    ##> for checking latest messages
    def check_latest_msgs(self, latest_is_mission_active_msg, latest_state_machine_event_log_msg, latest_cmd_vel_msg):

        check_return_msg = ""

        ##> if mission is inactive
        if latest_is_mission_active_msg.data == "False":
            check_return_msg = "MISSION INACTIVE"
            rospy.loginfo(check_return_msg)
            return True, check_return_msg

        ##> getting outcome msg from state machine event log
        latest_state_machine_event_log_msg = latest_state_machine_event_log_msg.data

        try:
            outcome_msg_starting_index = latest_state_machine_event_log_msg.index('with outcome:')
        except:
            return True, check_return_msg

        outcome_msg = latest_state_machine_event_log_msg[outcome_msg_starting_index+14:]
        
        ##> if outcome msg is 'paused'
        if (outcome_msg == "paused"):
            check_return_msg = "STATE MACHINE WAS PAUSED, NOW NO MOVEMENT, SOMETHING WRONG"
            return False, check_return_msg

        return True, check_return_msg

def main():
    ##> global
    global NAME_OF_NODE
    global NAME_OF_BAG_FILE
    global PATH_TO_BAG_FILE

    ##> initialize ros node
    rospy.init_node(NAME_OF_NODE)
    rospy.loginfo("Starting node:   "+NAME_OF_NODE)

    RealTimeErrorDetection()

if __name__ == '__main__':
    main()