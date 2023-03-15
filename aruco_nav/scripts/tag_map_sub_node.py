#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker

import math
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('tag_map_sub_node')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'map').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10)
        

        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.amcl_subscription  # prevent unused variable warning
        self.marker_publisher_ = self.create_publisher(Marker, '/tag_pose_marker', 10)
        self.aruco =ArucoMarkers()
        self.amcl = PoseWithCovarianceStamped()
        self.src_frame =""

    def aruco_callback(self, msg):
        self.get_logger().info('Tag detected "%s"' % msg.marker_ids[0])
        self.aruco = msg
        self.transform_map()

    def amcl_callback(self, msg):
        self.amcl = msg

    def transform_map(self):
        self.get_logger().info('Running')
        self.get_logger().info(self.aruco.header.frame_id)
        self.get_logger().info(self.target_frame)
        from_frame_rel = self.aruco.header.frame_id
        to_frame_rel = self.target_frame
        try:    
            if self.tf_buffer.can_transform(to_frame_rel,from_frame_rel,rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=10.0))

                r,p,y = self.euler_from_quaternion(self.amcl.pose.pose.orientation.x, self.amcl.pose.pose.orientation.y, self.amcl.pose.pose.orientation.z, self.amcl.pose.pose.orientation.w)

                tag_marker = Marker()
                tag_marker.type = Marker.SPHERE
                tag_marker.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()
                tag_marker.id = 0
                tag_marker.header.frame_id = "map"
                tag_marker.header.stamp = self.get_clock().now().to_msg()
                tag_marker.scale.x = 0.4
                tag_marker.scale.y = 0.4
                tag_marker.scale.z = 0.4
                tag_marker.color.r = 0.0
                tag_marker.color.g = 0.0
                tag_marker.color.b = 1.0
                tag_marker.color.a = 1.0
                print(-3.4 - t.transform.translation.x )
                print(t.transform.rotation)
                tag_marker.pose.position.x = t.transform.translation.x + self.aruco.poses[0].position.z * math.cos(y)
                tag_marker.pose.position.y = t.transform.translation.y + self.aruco.poses[0].position.z * math.sin(y)
                tag_marker.pose.position.z = t.transform.translation.z 
                tag_marker.pose.orientation= t.transform.rotation
                self.marker_publisher_.publish(tag_marker)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')

        

    def euler_from_quaternion(self, x, y, z, w,):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()