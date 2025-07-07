import json
from typing import List
import math

class MqttTopicNames:
    def __init__(self, robot_id):
        self.client_robot_inputs = f"robot_{robot_id}_client_robot_control_inputs"
        self.desired_leds = f"robot_{robot_id}_desired_leds"
        self.global_robot_pose = f"robot_{robot_id}_pose"
        self.pickup_pose = f"robot_{robot_id}_pickup_pose"
        self.dropoff_pose = f"robot_{robot_id}_dropoff_pose"
        self.obstacles_poses = []
        for i in range(1, 11):
            if i != robot_id:
                self.obstacles_poses.append(f"robot_{i}_pose")

class JsonMsg:
    def toJson(self):
        return json.dumps(self.__dict__, sort_keys=False)

class SetLedsMsg(JsonMsg):
    def __init__(self, r : int, g : int, b : int):
        self.r = r
        self.g = g
        self.b = b

class ClientRobotControlInputsMsg(JsonMsg):
    def __init__(self, inputs : List[float]):
        self.inputs = inputs

def get_pose_list_from_json_msg(json_msg):
    pose_list = []
    pose_list.append(json_msg["pose"]["position"]["x"])
    pose_list.append(json_msg["pose"]["position"]["y"])
    pose_list.append(2.0 * math.atan2(json_msg["pose"]["orientation"]["z"], json_msg["pose"]["orientation"]["w"]))
    return pose_list

