#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from ros2_aruco_interfaces.msg import ArucoMarkers


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       self.tf_broadcaster = TransformBroadcaster(self)
       self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)

       self.aruco = ArucoMarkers()
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def aruco_callback(self, msg):
       self.get_logger().info('Tag detected "%s"' % msg.marker_ids[0])
       self.aruco = msg

   def broadcast_timer_callback(self):
       total_number_of_tag = len(self.aruco.marker_ids)
       for i in range(total_number_of_tag):
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()

            t.header.frame_id = self.aruco.header.frame_id
            t.child_frame_id = 'tag_' + str(self.aruco.marker_ids[i])
            t.transform.translation.x = self.aruco.poses[i].position.x
            t.transform.translation.y = self.aruco.poses[i].position.y
            t.transform.translation.z = self.aruco.poses[i].position.z
            t.transform.rotation = self.aruco.poses[i].orientation

            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()



if __name__ == '__main__':
    main()