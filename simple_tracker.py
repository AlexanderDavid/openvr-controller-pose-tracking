#! /usr/bin/env python

"""Simple tracking solution based off triad_openvr

Track the pose and velocity of devices connected to a steamVR system at some user-defined (default 250hz) frequency.
"""

import triad_openvr
import rospy
from geometry_msgs.msg import Pose, Twist
from pedsim_msgs.msg import AgentState, AgentStates
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf import TransformBroadcaster
import time
import sys
import numpy as np

# Set the frequency of tracking
if len(sys.argv) == 1:
    interval = 60
elif len(sys.argv) == 2:
    interval = float(sys.argv[1])
else:
    print("Invalid number of arguments", file=sys.stderr)
    exit(1)

# Connect to the VR system
v = triad_openvr.triad_openvr()

# Connect to ROS
rospy.init_node("htc_vive_tracker", anonymous=False)
tf = TransformBroadcaster()
rate = rospy.Rate(interval)

# Define all controllers/trackers that we want to track. If the device is not
# connected then it will not show up in the output
to_track = ["tracker_1", "tracker_2", "tracker_3", "controller_1", "controller_2", "controller_3"]
agent_states_pub = rospy.Publisher("/pedsim_simulator/simulated_agents", AgentStates, queue_size=10)
agent_state_pub = rospy.Publisher("/pedsim_simulator/robot_state", AgentState, queue_size=10)

# Set the minimum velocity magnitude required to publish
vel_eps = 0.005

# Loop while ROS is up
while not rospy.is_shutdown():
    # Define the header for the agent states message
    agents_msg = AgentStates()
    agents_msg.header.stamp = rospy.Time.now()
    agents_msg.header.frame_id = "odom"

    # Loop through all devices that could be trackable
    for i, device in enumerate(to_track):
        # If the device is not connected then just continue
        if device not in v.devices:
            continue

        # Try and get the pose and velocity
        pose = v.devices[device].get_pose_euler()
        vel = v.devices[device].get_velocity()

        # if the pose is None this means that the device cannot see enough
        # lighthouses to get a good fix. In this case just skip it
        if pose is None:
            continue

        # Define the pose in terms of X and Z while keeping Y set to nothing.
        # We kinda want to "project" the tracker onto the ground plane
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = 0
        pose_msg.position.z = pose[2]

        # Get the orientation from the tracker, this comes from the Vive
        # in terms of r, p, y and in degrees, We need to translate it to
        # quaternions using nice ROS functions but really we only care about
        # the pitch which translates to the rotation about ROS's Z axis
        r, p, y = [x / 180 * np.pi for x in pose[3:]]
        orn = quaternion_about_axis(p - np.pi / 2, (0, 0, 1))

        # Set the orientation within the message
        pose_msg.orientation.x = orn[0]
        pose_msg.orientation.y = orn[1]
        pose_msg.orientation.z = orn[2]
        pose_msg.orientation.w = orn[3]

        # Capture the velocity of the device, and report it as non-zero only
        # in the X and Z dimensions and only if the magnitude is bigger than
        # some epsilon. Because there is a small amount of play in the actual
        # position the velocities under this epsilon aren't actual movement
        twist_msg = Twist()
        if np.linalg.norm(np.array([vel[0], vel[1]])) > vel_eps:
            twist_msg.linear.x = vel[0]
            twist_msg.linear.z = vel[1]
            twist_msg.linear.y = 0

        # Construct the rest of the surrounding message for this agent. THe frame
        # of reference is the ground plane or "odom" and the actual movement is
        # captured above
        agent_msg = AgentState()
        agent_msg.header.stamp = rospy.Time.now()
        agent_msg.header.frame_id = "odom"
        agent_msg.id = i
        agent_msg.type = 0
        agent_msg.pose = pose_msg
        agent_msg.twist = twist_msg

        # Only the Turtlebot is tracked using the controller so if that is the current
        # device then we publish on the robot state channel and we also have to publish
        # the transform so that R2D2 shows up in RViz
        if "controller" in device:
            agent_state_pub.publish(agent_msg)
            tf.sendTransform((pose[0], pose[2], 0), orn, rospy.Time.now(), "base_link", "odom")
        # If the current device is not the controller then just append it to the "neighbors"
        # list
        else:
            agents_msg.agent_states.append(agent_msg)

    # Publish the agent list
    agent_states_pub.publish(agents_msg)

    # Sleep the rest of the loop away
    rate.sleep()
