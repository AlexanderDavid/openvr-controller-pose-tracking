#! /usr/bin/env python

"""Simple tracking solution based off triad_openvr

Track the pose and velocity of devices connected to a steamVR system at some user-defined (default 250hz) frequency.
"""

import triad_openvr
import time
import sys

# Connect to the VR system
v = triad_openvr.triad_openvr()

# Set the frequency of tracking
if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments", file=sys.stderr)
    exit(1)

# Define all controllers/trackers that we want to track. If the device
# is not connected then it will not show up in the output
to_track = ["controller_1", "tracker_1", "tracker_2", "tracker_3"]

# Print the header
print("device, x, y, z, dx, dy, dz, t")

# Collect the start time so the tracked pose has a relative timestamp
abs_start = time.time()

# Loop forever, or at least until ctrl-c
while True:
    # Calculate the loop start time so that we only sleep the remaining
    # time left in the loop after calculation
    loop_start = time.time()

    # Capture each device
    for device in to_track:
        # If the device is not connected then just continue
        if device not in v.devices:
            continue

        # Capture the position (x, y, z) and velocity (dx, dy, dz) for each
        # device, the orientation does not matter
        txt = device + ", "
        for each in v.devices[device].get_pose_euler()[:3]:
            txt += "%.4f" % each
            txt += ", "
        for each in v.devices[device].get_velocity():
            txt += "%.4f" % each
            txt += ", "

        # Add the relative timestamp
        txt += str(time.time() - abs_start)

        # Print out the data
        print(txt)

    # Calculate the remaining time to sleep this loop
    sleep_time = interval - (time.time() - loop_start)
    if sleep_time > 0:
        time.sleep(sleep_time)
