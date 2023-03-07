#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from threading import Thread, Event
from queue import Queue


from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from vive_tracker_client import ViveTrackerClient


class ViveTrackerNode(Node):

    def __init__(self):
        super().__init__('vive_tracker_node')
        self.declare_parameter('host_ip', '127.0.1.1')
        self.declare_parameter('tracker_port', 8001)
        self.declare_parameter('tracker_name', 'T_1')
        self.declare_parameter('topic', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('tracker_link', 'tracker_link')

        (self.host_ip, self.tracker_port, self.tracker_name, self.odom_frame, self.tracker_link, self.topic) = self.get_parameters(
            ['host_ip', 'tracker_port', 'tracker_name', 'odom_frame', 'tracker_link', 'topic'])

        topic = self.topic.get_parameter_value().string_value
        topic_name = self.tracker_name.get_parameter_value().string_value + '/odom' if topic == "" else topic
        self.odom_pub = self.create_publisher(Odometry, topic_name,
            qos_profile=qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self)

        client = ViveTrackerClient(host=self.host_ip.get_parameter_value().string_value,
                                   port=self.tracker_port.get_parameter_value().string_value,
                                   tracker_name=self.tracker_name.get_parameter_value().string_value,
                                   should_record=False)

        self.message_queue = Queue()
        self.kill_thread = Event()
        self.client_thread = Thread(target=client.run_threaded, args=(self.message_queue, self.kill_thread,))

        try:
            self.client_thread.start()

            while rclpy.ok():
                msg = self.message_queue.get()

                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = self.odom_frame.get_parameter_value().string_value

                odom_msg.child_frame_id = self.tracker_link.get_parameter_value().string_value

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

                self.odom_pub.publish(odom_msg)
                t = TransformStamped() 

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'world'
                t.child_frame_id = self.tracker_link.get_parameter_value().string_value

                t.transform.translation.x = msg.x
                t.transform.translation.y = msg.y
                t.transform.translation.z = msg.z
                t.transform.rotation.x = msg.qx
                t.transform.rotation.y = msg.qy
                t.transform.rotation.z = msg.qz
                t.transform.rotation.w = msg.qw

                self.tf_broadcaster.sendTransform(t)

        finally:
            # cleanup
            self.kill_thread.set()
            self.client_thread.join()


def main(args=None):
    rclpy.init(args=args)
    vive_tracker_node = ViveTrackerNode()
    rclpy.spin(vive_tracker_node)
    vive_tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
