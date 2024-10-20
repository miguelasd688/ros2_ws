#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Crear un temporizador que llame a la función de publicación cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.publish_joint_states)


    def publish_joint_states(self):
        # Crear el mensaje de JointState
        joint_state = JointState()

        # Establecer la cabecera con la marca de tiempo actual
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Nombres de los joints (deben coincidir con los del URDF)
        joint_state.name = ['coxaF_BL', 'coxaF_BR', 'coxaF_FL', 'coxaF_FR',
                            'femurF_BL', 'femurF_BR', 'femurF_FL', 'femurF_FR',
                            'tibiaF_BL', 'tibiaF_BR', 'tibiaF_FL', 'tibiaF_FR',]  # Nombres de tus joints

        # Posiciones de los joints en radianes
        joint_state.position = [0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0]    # Las posiciones angulares en radianes

        # Publicar el mensaje en el topic
        self.publisher_.publish(joint_state)
        self.get_logger().info('Publicando estados de joints')

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Finalizar el nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




