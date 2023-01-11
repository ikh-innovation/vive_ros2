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
        self.declare_parameter('host_ip', '192.168.50.171')
        self.declare_parameter('host_port', 8000)
        self.declare_parameter('controller_name', 'C_1')
        self.declare_parameter('topic', '')
        self.declare_parameter('link_name', 'odom')
        self.declare_parameter('child_link_name', 'controller_link')

        (self.host_ip, self.host_port, self.controller_name, self.link_name, self.child_link_name, self.topic) = self.get_parameters(
            ['host_ip', 'host_port', 'controller_name', 'link_name', 'child_link_name', 'topic'])

        topic = self.topic.get_parameter_value().string_value
        topic_name = self.controller_name.get_parameter_value().string_value + '/odom' if topic == "" else topic
        self.odom_pub = self.create_publisher(Odometry, topic_name,
            100)
        self.joy_pub = self.create_publisher(Joy, "joy_inputs",
            100)

        self.tf_broadcaster = TransformBroadcaster(self)

        client = ViveControllerClient(host=self.host_ip.get_parameter_value().string_value,
                                   port=self.host_port.get_parameter_value().integer_value,
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

                odom_msg = Odometry()

                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = self.link_name.get_parameter_value().string_value
                odom_msg.child_frame_id = self.child_link_name.get_parameter_value().string_value

                odom_msg.pose.pose.position.x = msg.x
                odom_msg.pose.pose.position.y = msg.y
                odom_msg.pose.pose.position.z = msg.z

                odom_msg.pose.pose.orientation.x = msg.qx
                odom_msg.pose.pose.orientation.y = msg.qy
                odom_msg.pose.pose.orientation.z = msg.qz
                odom_msg.pose.pose.orientation.w = msg.qw

                odom_msg.twist.twist.linear.x = msg.vel_x
                odom_msg.twist.twist.linear.y = msg.vel_y
                odom_msg.twist.twist.linear.z = msg.vel_z

                odom_msg.twist.twist.angular.x = msg.p
                odom_msg.twist.twist.angular.y = msg.q
                odom_msg.twist.twist.angular.z = msg.r
                
                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = ""
                
                joy_msg.axes = [msg.trig, msg.pad_x, msg.pad_y]
                joy_msg.buttons = [msg.menu_butt, msg.pad_touch, msg.pad_butt, msg.grip_butt]

                t = TransformStamped() 

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'world'
                t.child_frame_id = self.child_link_name.get_parameter_value().string_value

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

                

                self.odom_pub.publish(odom_msg)
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
