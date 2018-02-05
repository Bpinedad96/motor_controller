#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


def controller_cb(data):
    global position_controller
    position_controller = data
    
def drone_cb(data):
    global position_drone
    position_drone = data

def right_wheel1_pid_cp(data):
    global drone_cmd_vel
    drone_cmd_vel.linear.x = data.data

def left_wheel1_pid_cp(data):
    global drone_cmd_vel
    drone_cmd_vel.linear.y = data.data

# Nodo 
def pid_publisher():
    global position_controller
    global position_drone
    global drone_cmd_vel

    position_controller = PoseStamped()
    position_drone = PoseStamped()
    drone_cmd_vel = Twist()
    
    # Initialize node
    rospy.init_node('pid_publisher', anonymous=True)
    # Publishing frecuency 
    rate = rospy.Rate(50) # 50hz

    # Publishers ----------------------------------------------------------------

    # Drone X axis PID publisher
    pub_pid_state_right1 = rospy.Publisher("right_wheel1/state", Float64, queue_size=10 )
    pub_pid_setpoint_right1 = rospy.Publisher("right_wheel1/setpoint", Float64, queue_size=10 )
    
    # Drone Y axis PID publisher
    pub_pid_state_left1 = rospy.Publisher("left_wheel1/state", Float64, queue_size=10 )
    pub_pid_setpoint_left1 = rospy.Publisher("left_wheel1/setpoint", Float64, queue_size=10 )

    # Drone cmd_vel publisher
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Subscribers ----------------------------------------------------------------

    # Drone X axis PID subscriber
    rospy.Subscriber("right_wheel1/control_effort", Float64, right_wheel1_pid_cp)

    # Drone Y axis PID subscriber
    rospy.Subscriber("left_wheel1/control_effort", Float64, left_wheel1_pid_cp)

    # Drone controller subscriber
    rospy.Subscriber("vrpn_client_node/Quad1/pose", PoseStamped, controller_cb)
    
    # Controlled drone subscriber
    rospy.Subscriber("vrpn_client_node/Quad2/pose", PoseStamped, drone_cb)
    
    
    while not rospy.is_shutdown():
        
        # right_wheel1 PID
        pub_pid_state_x.publish(position_drone.pose.position.z)
        pub_pid_setpoint_x.publish(position_controller.pose.position.z)
        
        # left_wheel1 PID
        pub_pid_state_y.publish(float(position_drone.pose.position.x - 1.8))
        pub_pid_setpoint_y.publish(position_controller.pose.position.x )

        print(drone_cmd_vel)
        pub_cmd_vel.publish(drone_cmd_vel)
        rate.sleep()
        

if __name__ == '__main__':
pid_publisher()
