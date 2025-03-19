#!/usr/bin/env python3
# Copyright 2025 Chris Iverach-Brereton
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
import datetime

import rclpy.time

epoch_ref = datetime.datetime(
    year=1970,
    month=1,
    day=1,
    hour=0,
    minute=0,
    second=0
)

ntp_ref = datetime.datetime(
    year=1900,
    month=1,
    day=1,
    hour=0,
    minute=0,
    second=0
)


def ros_time_2_ntp_time(ros_time: rclpy.time.Time) -> float:
    """
    Convert a ROS timestamp (linux epoch time) to NTP time.

    :param ros_time: The ROS timestamp to convert
    :return:  The elapsed seconds since 1 Jan 1900 00:00:00
    """
    s, ns = ros_time.seconds_nanoseconds()
    epoch_delta = datetime.timedelta(
        seconds=(s + ns / 1000000000)
    )
    ntp_delta = (epoch_ref + epoch_delta) - ntp_ref
    return ntp_delta.total_seconds()


def ntp_time_2_ros_time(ntp_time: float) -> rclpy.time.Time:
    """
    Convert an NTP timestamp to a ROS timestamp.

    :param ntp_time:  The number of seconds elapsed since Jan 1, 1900 00:00:00
    """
    ntp_delta = datetime.timedelta(seconds=ntp_time)
    now = ntp_ref + ntp_delta
    epoch_delta = now - epoch_ref
    return rclpy.time.Time(
        nanoseconds=epoch_delta.total_seconds() * 1000000000
    )
