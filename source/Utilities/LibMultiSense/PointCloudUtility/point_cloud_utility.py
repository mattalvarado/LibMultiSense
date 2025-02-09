#!/usr/bin/env python
#
# @file point_cloud_utility.cc
#
# Copyright 2013-2025
# Carnegie Robotics, LLC
# 4501 Hatfield Street, Pittsburgh, PA 15201
# http://www.carnegierobotics.com
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Carnegie Robotics, LLC nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Significant history (date, user, job code, action):
#   2025-02-07, malvarado@carnegierobotics.com, IRAD, Created file.
#

import argparse
import time
import cv2
import numpy as np

import libmultisense as lms


color_point_dtype = np.dtype({'names': ['x', 'y', 'z', 'color'],
                              'formats': [np.float32, np.float32, np.float32, np.uint8],
                              'offsets': [0, 4, 8, 12],
                              'itemsize': 13})

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    channel = lms.Channel.create(channel_config)
    if not channel:
        print("Invalid channel")
        return

    config = channel.get_configuration()
    config.frames_per_second = 10.0
    if channel.set_configuration(config) != lms.Status.OK:
        print("Cannot set configuration")
        return

    if channel.start_streams([lms.DataSource.LEFT_RECTIFIED_RAW, lms.DataSource.LEFT_DISPARITY_RAW]) != lms.Status.OK:
        print("Unable to start streams")
        return

    while True:
        frame = channel.get_next_image_frame()
        if frame:
            point_cloud = lms.create_gray8_pointcloud(frame,
                                                     args.max_range,
                                                     lms.DataSource.LEFT_RECTIFIED_RAW,
                                                     lms.DataSource.LEFT_DISPARITY_RAW)

            # Convert to numpy array and compute the average depth
            point_cloud_array = point_cloud.as_raw_array.view(color_point_dtype)
            mean_depth = np.mean(point_cloud_array["z"])
            print("Mean depth:", mean_depth, "(m)")

            print("Saving pointcloud for frame id: ", frame.frame_id)
            lms.write_pointcloud_ply(point_cloud, str(frame.frame_id) + ".ply")

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense save image utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-r", "--max-range", type=float, default=50.0, help="The max point cloud range in meters.")
    main(parser.parse_args())


