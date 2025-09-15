#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaRobotState

marker_pose = PoseStamped()
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisher_callback():
    marker_pose.header.stamp = rclpy.time.Time().to_msg()
    pose_pub.publish(marker_pose)


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()

def make_sphere(scale=0.3):
    """
    This function returns sphere marker for 3D translational movements.
    :param scale: scales the size of the sphere
    :return: sphere marker
    """
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker

if __name__ == "__main__":
    rclpy.init()
    node = Node("equilibrium_pose_node")
    # buffer = Buffer()
    # listener = TransformListener(buffer, node)
    link_name = 'fr3_link0' # node.get_parameter('~link_name').get_parameter_value().string_value

    print("Waiting for robot state message...")
    result, msg = wait_for_message(FrankaRobotState, node, "/franka_robot_state_broadcaster/robot_state")
    if not result:
        print("Failed to receive robot state message.")
        rclpy.shutdown()
        exit(1)
    print("Equilibrium pose node started")
    marker_pose.pose = msg.o_t_ee.pose

    pose_pub = node.create_publisher(
        PoseStamped, "equilibrium_pose", 10)
    
    server = InteractiveMarkerServer(node, "equilibrium_pose_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
                              "If you move the \nequilibrium "
                              "pose the robot will follow it\n"
                              "so be aware of potential collisions")
    int_marker.pose = marker_pose.pose
    # run pose publisher
    marker_pose.header.frame_id = link_name
    timer = node.create_timer(0.005, publisher_callback)

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 1.0
    control.orientation.z = 1.0
    control.name = "move_3d"
    control.always_visible = True
    control.markers.append(make_sphere())
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D

    int_marker.controls.append(control)
    server.insert(int_marker, feedback_callback=process_feedback)

    server.applyChanges()

    rclpy.spin(node)