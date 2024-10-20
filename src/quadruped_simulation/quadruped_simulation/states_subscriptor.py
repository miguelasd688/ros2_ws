#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float64, Header
from robot_interfaces.msg import JointAngles
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import time
from sys import exit




class StatesSubscriptor(Node):

    def __init__(self):
        super().__init__('states_subscriptor')
        self.timer = self.create_timer(0.01, self.joint_angle_callback)

        self.last_time = time.time()
        self.get_logger().info('States subscriptor node has started.')

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.topic = self.create_subscription(
            JointAngles,
            'joint_angles',
            self.listener_topic_callback,
            100)

        self.subscriptions  # prevent unused variable warning

        self.msg_received = False
        self.start = False

        self.fr_foot = [0.0 , 0.0 , 0.0]
        self.fl_foot = [0.0 , 0.0 , 0.0]
        self.br_foot = [0.0 , 0.0 , 0.0]
        self.bl_foot = [0.0 , 0.0 , 0.0]


    def joint_angle_callback(self):
        time_now = time.time()
        latency = time_now - self.last_time
        self.last_time = time_now

        if (self.msg_received == False and self.start == False):
            self.get_logger().info('Waiting to receive topic...')
            self.start = True 
        if (self.msg_received):
            self.setJointAngles()
        self.setBaseTransformation()



    def setJointAngles(self):
        joint_state = JointState()

        # Establecer la cabecera con la marca de tiempo actual
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Nombres de los joints (deben coincidir con los del URDF)
        joint_state.name = ['coxaF_BL', 'coxaF_BR', 'coxaF_FL', 'coxaF_FR',
                            'femurF_BL', 'femurF_BR', 'femurF_FL', 'femurF_FR',
                            'tibiaF_BL', 'tibiaF_BR', 'tibiaF_FL', 'tibiaF_FR',]  # Nombres de tus joints

        # Posiciones de los joints en radianes
        joint_state.position = [self.fr_foot[0], self.fl_foot[0], self.br_foot[0], self.bl_foot[0],
                                self.fr_foot[1], self.fl_foot[1], self.br_foot[1], self.bl_foot[1],
                                self.fr_foot[2], self.fl_foot[2], self.br_foot[2], self.bl_foot[2]]    # Las posiciones angulares en radianes

        # Publicar el mensaje en el topic
        self.publisher_.publish(joint_state)
        #self.get_logger().info('Publicando estados de joints')



    def listener_topic_callback(self, msg):
        self.fr_foot = msg.fr_foot
        self.fl_foot = msg.fl_foot
        self.br_foot = msg.br_foot
        self.bl_foot = msg.bl_foot 

        if not (self.msg_received):
            self.get_logger().info('Hearing angles')
            self.msg_received = True

       

    # set transforms for rviz
    def setBaseTransformation(self):
        transform = TransformStamped()
        # set tranforms origin and destination
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        
        # Establecer la transformación (posición y orientación)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        # Publicar la transformación
        self.tf_broadcaster.sendTransform(transform)

        transform_base_link = TransformStamped()
        transform_base_link.header.stamp = self.get_clock().now().to_msg()
        transform_base_link.header.frame_id = 'base_link_back'
        transform_base_link.child_frame_id = 'base_link_front'
        transform_base_link.transform.translation.x = 0.0
        transform_base_link.transform.translation.y = 0.0
        transform_base_link.transform.translation.z = 0.0
        transform_base_link.transform.rotation.x = 0.0
        transform_base_link.transform.rotation.y = 0.0
        transform_base_link.transform.rotation.z = 0.0
        transform_base_link.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform_base_link)
     
def main(args=None):
    rclpy.init(args=args)
    states_subscriptor = StatesSubscriptor()
    rclpy.spin(states_subscriptor)
    states_subscriptor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()