from typing import Tuple, List, Dict
import time
import openvr
from matplotlib import pyplot as plt
from dataclasses import dataclass
import numpy as np

"""
Get the HTC Vive controller poses through openvr and publish over ROSBridge
or save to a file

Author: Alex Day <adday at clemson.edu>
"""

ROS_HOST = "130.127.175.48"
ROS_PORT = 9090
USE_ROS = False

if USE_ROS:
    import roslibpy

@dataclass
class Trajectory:
    device: str
    time: float
    x: float
    y: float
    z: float

    def __str__(self):
        return f"{self.device}, {self.x}, {self.y}, {self.z}, {self.time}"

class ViveTracker:
    def __init__(self, vr_init_retries: int=4, ros_info: Tuple=(None, None),
                 poll_hz: int=25):
        self.poll_hz = poll_hz
        
        retries = 0
        print("==============")
        print("Initializing OpenVR")
        while retries < vr_init_retries:
            try:
                openvr.init(openvr.VRApplication_Scene)
                break
            except openvr.OpenVRError as e:
                print("Error while initializing... Trying again")
                print(e)
                retries += 1
                time.sleep(2.0)
        else:
            print("Couldn't initialize, aborting")
            exit(1)

        self.vrsys = openvr.VRSystem()
        print("==============")
        print("Getting all connected controller idxs")
        self.tracked_idxs = self.__get_controller_ids()
        self.last_packet_num = [None] * len(self.tracked_idxs)
        self.poses = []
        print(f"Found {len(self.tracked_idxs)} devices")

        print("==============")
        if None in ros_info:
            self.use_ros = False
        else:

            print("Connecting to ROSBridge")
            self.client = roslibpy.Ros(host=ros_info[0], port=ros_info[1])
            self.client.run()

            if not self.client.is_connected:
                print("Couldn't connect to ROS, aborting...")
                exit(1)

            self.use_ros = True

            self.publishers = []
            for idx in self.tracked_idxs:
                self.publishers.append(roslibpy.Topic(self.client,
                                                      "/htc_vive/left",
                                                      "geometry_msgs/PoseStamped"))
            print("Connected to ROS Master")
            print("==============")

    def startPolling(self):
        print("Starting to track")
        count = 0
        try:
            start_time = time.time()
            while True:
                count += 1
                time.sleep(1 / self.poll_hz)
                for i, vive_idx in enumerate(self.tracked_idxs):
                    result, controllerState, controllerPose = self.vrsys.getControllerStateWithPose(0, vive_idx)
                    print(vive_idx, result)
                    
                    state = ViveTracker.__controller_state_to_dict(controllerState)
                    pose = ViveTracker.__controller_pose_to_dict(controllerPose)

                    if self.last_packet_num[i] is None or self.last_packet_num[i] < state["unPacketNum"]:
                        self.last_packet_num[i] = state["unPacketNum"]
                        if USE_ROS:
                            msg = ViveTracker.__pose_dict_to_nav_msg(pose, "map")
                            self.publishers[i].publish(msg)
                        else:
                            self.poses.append(
                                Trajectory(
                                    vive_idx,
                                    time.time() - start_time,
                                    pose["x"],
                                    pose["y"],
                                    pose["z"]
                                )
                            )
                            
        except KeyboardInterrupt:
            print(f"Got keyboard interrupt... Plotting and/or saving {count} states")

            # Save all poses to data.csv
            with open(f"data.csv", "w") as file:
                for pose in self.poses:
                    file.write(str(pose) + "\n")

            # Do device specific output for quick validation
            colors = ["red", "blue", "green", "purple", "white", "yellow"] 
            for i, device in enumerate(set([d.device for d in self.poses])):
                print(set([p.device for p in self.poses if p.device == device]))

                # Calculate total displacement for each device
                poses = [p for p in self.poses if p.device == device]
                
                p1 = np.array([poses[0].x, poses[0].y, poses[0].z])
                p2 = np.array([poses[-1].x, poses[-1].y, poses[-1].z])
                print(f"Total Displacement for {poses[0].device} with {len(poses)} poses: {np.linalg.norm(p1 - p2)}")

                # Plot each device's path
                plt.plot(
                    [p.x for p in poses],
                    [p.z for p in poses],
                    label=i,
                    c=colors[i % len(colors)]
                )
                plt.show()

        
    def __get_controller_ids(self) -> List[int]:
        idxs = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vrsys.getTrackedDeviceClass(i)
            print(device_class)
            if device_class in [openvr.TrackedDeviceClass_Controller, openvr.TrackedDeviceClass_GenericTracker]:
                idxs.append(i) 

        return idxs


    @staticmethod
    def __controller_state_to_dict(controllerState) -> Dict:
        d = {}
        
        d["unPacketNum"] = controllerState.unPacketNum

        return d

    @staticmethod
    def __controller_pose_to_dict(controllerPose) -> Dict:
        d = {}
        trans_mat = controllerPose.mDeviceToAbsoluteTracking

        d["x"] = trans_mat[0][3]
        d["y"] = trans_mat[1][3]
        d["z"] = trans_mat[2][3]

        return d

    @staticmethod
    def __pose_dict_to_nav_msg(d: Dict, frame: str) -> Dict:
        return {
            "header": {
                "frame_id": frame
            },
            "pose": {
                "position": {
                    "x": d["x"],
                    "y": d["z"],
                    "z": d["y"]
                    }
                }
            }

if __name__ == "__main__":
    vt = ViveTracker()
    vt.startPolling()
