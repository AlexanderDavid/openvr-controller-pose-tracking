import triad_openvr
import time
import sys

v = triad_openvr.triad_openvr()
# v.print_discovered_objects()

if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    exit(1)

to_track = ["controller_1", "tracker_1", "tracker_2", "tracker_3"]
print("device, x, y, z, dx, dy, dz, t")
abs_start = time.time()
while(True):
    loop_start = time.time()

    for device in to_track:
        txt = device + ", "
        for each in v.devices[device].get_pose_euler()[:3]:
            txt += "%.4f" % each
            txt += ", "
        for each in v.devices[device].get_velocity():
            txt += "%.4f" % each
            txt += ", "
        txt += str(time.time() - abs_start)

        # Print out the data
        print(txt)

    # Calculate the remaining time to sleep this loop
    sleep_time = interval - (time.time() - loop_start)
    if sleep_time > 0:
        time.sleep(sleep_time)
