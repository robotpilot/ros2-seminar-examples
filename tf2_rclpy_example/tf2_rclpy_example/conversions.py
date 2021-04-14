# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ref)https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

import math


def euler_to_quaternion(roll, pitch, yaw):
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    half_roll = roll * 0.5

    cos_yaw = math.cos(half_yaw)
    sin_yaw = math.sin(half_yaw)
    cos_pitch = math.cos(half_pitch)
    sin_pitch = math.sin(half_pitch)
    cos_roll = math.cos(half_roll)
    sin_roll = math.sin(half_roll)

    return [sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,  # x
            cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,  # y
            cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,  # z
            cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw]  # w


def quaternion_to_euler(q):
    q_x = q[0]
    q_y = q[1]
    q_z = q[2]
    q_w = q[3]

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # roll (x-axis rotation)
    sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q_w * q_y - q_z * q_x)
    if math.abs(sinp) >= 1.0:
        pitch = math.copysign(math.PI / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]
