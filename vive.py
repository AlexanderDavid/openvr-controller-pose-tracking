import time
import pprint
import openvr
import roslibpy
from matplotlib import pyplot as plt

"""
Get the HTC Vive controllers poses and publish them through ROS Bridge

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
Author: Alex Day <adday at clemson.edu>
"""

ROS_HOST = "130.127.175.48"
ROS_PORT = 9090
USE_ROS = False

def get_controller_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
    left = None
    right = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right


def from_controller_state_to_dict(pControllerState):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting
    return d


def from_controller_state_and_pose_to_dict(pControllerState, pControllerPose):
    d = from_controller_state_to_dict(pControllerState)
    
    trans_mat = pControllerPose.mDeviceToAbsoluteTracking
    d["pos"] = {}
    d["pos"]["x"] = trans_mat[0][3]
    d["pos"]["y"] = trans_mat[1][3]
    d["pos"]["z"] = trans_mat[2][3]

    return d

def from_dict_to_nav_msg(d, frame):
    return {
        "header": {
            "frame_id": "map"
        },
        "pose": {
            "position": {
                "x": d["pos"]["x"],
                "y": d["pos"]["z"],
                "z": d["pos"]["y"]
            }
        }
    }


if __name__ == '__main__':
    max_init_retries = 4
    retries = 0
    print("===========================")
    print("Initializing OpenVR...")
    while retries < max_init_retries:
        try:
            openvr.init(openvr.VRApplication_Scene)
            break
        except openvr.OpenVRError as e:
            print("Error when initializing OpenVR (try {} / {})".format(
                  retries + 1, max_init_retries))
            print(e)
            retries += 1
            time.sleep(2.0)
    else:
        print("Could not initialize OpenVR, aborting.")
        print("Make sure the system is correctly plugged, you can also try")
        print("to do:")
        print("killall -9 vrcompositor vrmonitor vrdashboard")
        print("Before running this program again.")
        exit(0)

    print("Success!")
    print("===========================")
    vrsystem = openvr.VRSystem()

    client = None
    if USE_ROS:
        print("===========================")
        print("Connecting to ROSBridge")
        client = roslibpy.Ros(host=ROS_HOST, port=ROS_PORT)
        client.run()

        if not client.is_connected:
            print("Could not connect to ROS, aborting.")
            exit(1)

    left_pose_pub = roslibpy.Topic(client, "/htc_vive/left", "geometry_msgs/PoseStamped")
    right_pose_pub = roslibpy.Topic(client, "/htc_vive/right", "geometry_msgs/PoseStamped")

    left_id, right_id = None, None
    print("===========================")
    print("Waiting for controllers...")
    try:
        while left_id is None or right_id is None:
            left_id, right_id = get_controller_ids(vrsystem)
            if left_id and right_id:
                break
            print("Waiting for controllers...")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
        openvr.shutdown()

    print("Left controller ID: " + str(left_id))
    print("Right controller ID: " + str(right_id))
    print("===========================")

    pp = pprint.PrettyPrinter(indent=4)

    reading_rate_hz = 25
    show_only_new_events = True
    last_unPacketNum_left = 0
    last_unPacketNum_right = 0
    left_poses = []
    right_poses = []

    print("===========================")
    print("Printing controller events!")
    try:
        with open("./left.csv", "w") as lfile:
            with open("./right.csv", "w") as rfile:
                while True:
                    time.sleep(1.0 / reading_rate_hz)
                    result, pControllerState, pControllerPose = vrsystem.getControllerStateWithPose(0, left_id)
                    d = from_controller_state_and_pose_to_dict(pControllerState, pControllerPose)
                    if show_only_new_events and last_unPacketNum_left != d['unPacketNum']:
                        last_unPacketNum_left = d['unPacketNum']
                        if USE_ROS:
                            msg = from_dict_to_nav_msg(d, "left_controller")
                            left_pose_pub.publish(msg)
                        else:
                            left_poses.append(d["pos"])
                            lfile.write(f"{d['pos']['x']}, {d['pos']['y']}, {d['pos']['z']}\n")

                    result, pControllerState, pControllerPose = vrsystem.getControllerStateWithPose(0, right_id)
                    d = from_controller_state_and_pose_to_dict(pControllerState, pControllerPose)
                    if show_only_new_events and last_unPacketNum_right != d['unPacketNum']:
                        last_unPacketNum_right = d['unPacketNum']
                        if USE_ROS:
                            msg = from_dict_to_nav_msg(d, "right_controller")
                            right_pose_pub.publish(msg)
                        else:
                            right_poses.append(d['pos'])
                            rfile.write(f"{d['pos']['x']}, {d['pos']['y']}, {d['pos']['z']}\n")

                    lfile.flush()
                    rfile.flush()



    except KeyboardInterrupt:
        print("Control+C pressed, saving paths and shutting down...")
        openvr.shutdown()



