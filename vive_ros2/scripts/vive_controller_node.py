#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data

from threading import Thread, Event
from queue import Queue

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from vive_controller_client import ViveControllerClient


class ViveControllerNode(Node):

    def __init__(self):
        super().__init__('vive_controller_node')
        self.declare_parameter('host_ip', '127.0.1.1')
        self.declare_parameter('controller_port', 8000)
        self.declare_parameter('controller_name', 'controller_1')
        self.declare_parameter('controller_link', 'controller_link')
        self.declare_parameter('buttons_topic', 'joy_inputs')

        (self.host_ip, self.controller_port, self.controller_name, self.controller_link, self.buttons_topic) = self.get_parameters(
            ['host_ip', 'controller_port', 'controller_name', 'controller_link', 'buttons_topic'])

        self.joy_pub = self.create_publisher(Joy, self.buttons_topic.get_parameter_value().string_value,
            100)

        self.tf_broadcaster = TransformBroadcaster(self)

        client = ViveControllerClient(host=self.host_ip.get_parameter_value().string_value,
                                   port=self.controller_port.get_parameter_value().integer_value,
                                   controller_name=self.controller_name.get_parameter_value().string_value,
                                   should_record=False)

        self.message_queue = Queue()
        self.kill_thread = Event()
        self.client_thread = Thread(target=client.run_threaded, args=(self.message_queue, self.kill_thread,))

        try:
            self.client_thread.start()
            sum = 0 
            while rclpy.ok():
                msg = self.message_queue.get()
                
                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = ""
                
                joy_msg.axes = [msg.trig, msg.pad_x, msg.pad_y]
                joy_msg.buttons = [msg.menu_butt, msg.pad_touch, msg.pad_butt, msg.grip_butt]

                t = TransformStamped() 

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'vive_world'
                t.child_frame_id = self.controller_link.get_parameter_value().string_value

                t.transform.translation.x = msg.x
                t.transform.translation.y = msg.y
                t.transform.translation.z = msg.z
                t.transform.rotation.x = msg.qx
                t.transform.rotation.y = msg.qy
                t.transform.rotation.z = msg.qz
                t.transform.rotation.w = msg.qw

                self.tf_broadcaster.sendTransform(t)
                # if sum>10:
                #     print([msg.trig, msg.pad_x, msg.pad_y])
                #     # print([msg.menu_butt, msg.pad_touch, msg.pad_butt, msg.ul_butt, msg.ul_touch, msg.grip_butt])
                #     print([msg.menu_butt, msg.pad_touch, msg.pad_butt, msg.grip_butt])

                #     sum = 0
                # else:
                #     sum = sum + 1

                
                self.joy_pub.publish(joy_msg)

        finally:
            # cleanup
            self.kill_thread.set()
            self.client_thread.join()


def main(args=None):
    rclpy.init(args=args)
    vive_controller_node = ViveControllerNode()
    rclpy.spin(vive_controller_node)
    vive_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
