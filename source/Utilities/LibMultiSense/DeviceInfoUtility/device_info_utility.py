#!/usr/bin/env python
#
# @file device_info_utility.cc
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
import json

import libmultisense as lms

from libmultisense import (
        DeviceInfo,
        HardwareRevision,
        ImagerType,
        LightingType,
        LensType
)

# If the enum values do not have a `.name` attribute (or if you prefer a custom mapping),
# you can define mappings between strings and enum values.
# For example:
hardware_revision_mapping = {
    "UNKNOWN": HardwareRevision.UNKNOWN,
    "S7": HardwareRevision.S7,
    "S21": HardwareRevision.S21,
    "ST21": HardwareRevision.ST21,
    "S27": HardwareRevision.S27,
    "S30": HardwareRevision.S30,
    "KS21": HardwareRevision.KS21,
    "MONOCAM": HardwareRevision.MONOCAM,
    "KS21_SILVER": HardwareRevision.KS21_SILVER,
    "ST25": HardwareRevision.ST25,
    "KS21i": HardwareRevision.KS21i
}

imager_type_mapping = {
    "UNKNOWN": ImagerType.UNKNOWN,
    "CMV2000_GREY": ImagerType.CMV2000_GREY,
    "CMV2000_COLOR": ImagerType.CMV2000_COLOR,
    "CMV4000_GREY": ImagerType.CMV4000_GREY,
    "CMV4000_COLOR": ImagerType.CMV4000_COLOR,
    "FLIR_TAU2": ImagerType.FLIR_TAU2,
    "AR0234_GREY": ImagerType.AR0234_GREY,
    "AR0239_COLOR": ImagerType.AR0239_COLOR
}

lighting_type_mapping = {
    "NONE": LightingType.NONE,
    "INTERNAL": LightingType.INTERNAL,
    "EXTERNAL": LightingType.EXTERNAL,
    "PATTERN_PROJECTOR": LightingType.PATTERN_PROJECTOR
}

lens_type_mapping = {
    "UNKNOWN": LensType.UNKNOWN
}


def device_info_to_dict(device_info):
    result = {}
    result["camera_name"] = device_info.camera_name
    result["build_date"] = device_info.build_date
    result["serial_number"] = device_info.serial_number

    result["hardware_revision"] = device_info.hardware_revision.name
    result["number_of_pcbs"] = device_info.number_of_pcbs

    pcb_list = []
    for i in range(device_info.number_of_pcbs):
        pcb = device_info.pcb_info[i]
        pcb_list.append({
            "name": pcb.name,
            "revision": pcb.revision
        })
    result["pcb_info"] = pcb_list

    result["imager_name"] = device_info.imager_name
    result["imager_type"] = device_info.imager_type.name
    result["imager_width"] = device_info.imager_width
    result["imager_height"] = device_info.imager_height
    result["lens_name"] = device_info.lens_name
    result["lens_type"] =  device_info.lens_type.name
    result["nominal_stereo_baseline"] = device_info.nominal_stereo_baseline
    result["nominal_focal_length"] = device_info.nominal_focal_length
    result["nominal_relative_aperture"] = device_info.nominal_relative_aperture
    result["lighting_type"] = device_info.lighting_type.name
    result["number_of_lights"] = device_info.number_of_lights

    return result


def device_info_from_dict(d):
    device = DeviceInfo()
    device.camera_name = d.get("camera_name", "")
    device.build_date = d.get("build_date", "")
    device.serial_number = d.get("serial_number", "")

    hw_rev_str = d.get("hardware_revision", "UNKNOWN")
    device.hardware_revision = hardware_revision_mapping.get(hw_rev_str, HardwareRevision.UNKNOWN)

    device.number_of_pcbs = d.get("number_of_pcbs", 0)
    pcb_list = d.get("pcb_info", [])
    for i, pcb in enumerate(pcb_list):
        if i >= device.number_of_pcbs:
            break
        device.pcb_info[i].name = pcb.get("name", "")
        device.pcb_info[i].revision = pcb.get("revision", 0)

    device.imager_name = d.get("imager_name", "")
    imager_type_str = d.get("imager_type", "UNKNOWN")
    device.imager_type = imager_type_mapping.get(imager_type_str, ImagerType.UNKNOWN)
    device.imager_width = d.get("imager_width", 0)
    device.imager_height = d.get("imager_height", 0)
    device.lens_name = d.get("lens_name", "")
    lens_type_str = d.get("lens_type", "UNKNOWN")
    device.lens_type = lens_type_mapping.get(lens_type_str, LensType.UNKNOWN)
    device.nominal_stereo_baseline = d.get("nominal_stereo_baseline", 0.0)
    device.nominal_focal_length = d.get("nominal_focal_length", 0.0)
    device.nominal_relative_aperture = d.get("nominal_relative_aperture", 0.0)
    lighting_type_str = d.get("lighting_type", "NONE")
    device.lighting_type = lighting_type_mapping.get(lighting_type_str, LightingType.NONE)
    device.number_of_lights = d.get("number_of_lights", 0)

    return device


def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    channel = lms.Channel.create(channel_config)
    if not channel:
        print("Invalid channel")
        exit(1)

    if args.set_file_path:
        if not args.skip_confirm:
            reply = input("Really update device information? (y/n): ")
            if reply not in ('y', 'Y'):
                print("Aborting")
                exit(1)

        with open(args.set_file_path) as f:
            info = device_info_from_dict(json.load(f))

            status = channel.set_device_info(info, "12")
            if status != lms.Status.OK:
                print("Failed to set device info:", lms.to_string(status))
                exit(1)
            print("Succesfully set device info")
    else:
        print(json.dumps(device_info_to_dict(channel.get_info().device),
                         sort_keys=True,
                         indent=4,
                         separators=(',', ': ')))

if __name__ == '__main__':
    parser = argparse.ArgumentParser("LibMultiSense save image utility")
    parser.add_argument("-a", "--ip_address", default="10.66.171.21", help="The IPv4 address of the MultiSense.")
    parser.add_argument("-m", "--mtu", type=int, default=1500, help="The MTU to use to communicate with the camera.")
    parser.add_argument("-s", "--set-file-path", help="The path to the device info to send to the camera")
    parser.add_argument("-k", "--key", help="Key to allow device info to be written to the camera")
    parser.add_argument("-y", "--skip-confirm", action="store_true", help="Disable the confirmation prompts")
    main(parser.parse_args())
