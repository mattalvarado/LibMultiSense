/**
 * @file device_info_tests.cc
 *
 * Copyright 2013-2025
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Significant history (date, user, job code, action):
 *   2025-01-22, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/device_info.hh>

using namespace multisense::legacy;

crl::multisense::details::wire::SysDeviceInfo create_wire_info(const std::string &name, const std::string &key)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo info;
    info.key = key;
    info.name = name;
    info.buildDate = "2024";
    info.serialNumber = "4501";
    info.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21;
    info.numberOfPcbs = 1;

    wire::PcbInfo pcb;
    pcb.name = name;
    pcb.revision = 12345;
    info.pcbs[0] = pcb;

    info.imagerName = name;
    info.imagerType = wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR;
    info.imagerWidth = 1000;
    info.imagerHeight = 1001;
    info.lensName = name;
    info.nominalBaseline = 0.21;
    info.nominalFocalLength = 0.024;
    info.nominalRelativeAperture = 0.1;
    info.lightingType = wire::SysDeviceInfo::LIGHTING_TYPE_S21_EXTERNAL;
    info.numberOfLights = 1;

    return info;
}

multisense::DeviceInfo create_info(const std::string &name)
{
    using namespace multisense;
    DeviceInfo info;
    info.camera_name = name;
    info.build_date = "2024";
    info.serial_number = "4501";
    info.hardware_revision = DeviceInfo::HardwareRevision::KS21;
    info.number_of_pcbs = 1;
    info.pcb_info = {DeviceInfo::PcbInfo{name, 12345}};
    info.imager_name = name;
    info.imager_type = DeviceInfo::ImagerType::AR0239_COLOR;
    info.imager_width = 1000;
    info.imager_height = 1001;
    info.lens_name = name;
    info.nominal_stereo_baseline = 0.21;
    info.nominal_focal_length = 0.024;
    info.nominal_relative_aperature = 0.1;
    info.lighting_type = DeviceInfo::LightingType::EXTERNAL;
    info.number_of_lights = 1;

    return info;
}

void check_equal(const crl::multisense::details::wire::SysDeviceInfo &wire, const multisense::DeviceInfo &info,
                 const std::string &key)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    ASSERT_EQ(wire.key, key);
    ASSERT_EQ(wire.name, info.camera_name);
    ASSERT_EQ(wire.buildDate, info.build_date);
    ASSERT_EQ(wire.serialNumber, info.serial_number);

    switch (wire.hardwareRevision)
    {
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::S7); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::S21); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::ST21); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::S27); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S30:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::S30); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::KS21); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::MONOCAM); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21_SILVER:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::KS21_SILVER); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST25:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::ST25); break;
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21i:
            ASSERT_EQ(info.hardware_revision, DeviceInfo::HardwareRevision::KS21i); break;
        default: {CRL_EXCEPTION("Unsupported hardware revision");}
    }

    ASSERT_EQ(wire.numberOfPcbs, info.number_of_pcbs);

    for (uint32_t i = 0; i < wire.numberOfPcbs ; ++i)
    {
        ASSERT_EQ(wire.pcbs[i].name, info.pcb_info[i].name);
        ASSERT_EQ(wire.pcbs[i].revision, info.pcb_info[i].revision);
    }

    ASSERT_EQ(wire.imagerName, info.imager_name);
    switch (wire.imagerType)
    {
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_GREY:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::CMV2000_GREY); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_COLOR:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::CMV2000_COLOR); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_GREY:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::CMV4000_GREY); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_COLOR:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::CMV4000_COLOR); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_FLIR_TAU2:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::FLIR_TAU2); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_AR0234_GREY:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::AR0234_GREY); break;
        case wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR:
            ASSERT_EQ(info.imager_type, DeviceInfo::ImagerType::AR0239_COLOR); break;
        default: {CRL_EXCEPTION("Unsupported imager type");}
    }
    ASSERT_EQ(wire.imagerWidth, info.imager_width);
    ASSERT_EQ(wire.imagerHeight, info.imager_height);

    ASSERT_EQ(wire.lensName, info.lens_name);
    ASSERT_EQ(wire.nominalBaseline, info.nominal_stereo_baseline);
    ASSERT_EQ(wire.nominalFocalLength, info.nominal_focal_length);
    ASSERT_EQ(wire.nominalRelativeAperture, info.nominal_relative_aperature);

    switch (wire.lightingType)
    {
        case wire::SysDeviceInfo::LIGHTING_TYPE_NONE:
             ASSERT_EQ(info.lighting_type, DeviceInfo::LightingType::NONE); break;
        case wire::SysDeviceInfo::LIGHTING_TYPE_SL_INTERNAL:
             ASSERT_EQ(info.lighting_type, DeviceInfo::LightingType::INTERNAL); break;
        case wire::SysDeviceInfo::LIGHTING_TYPE_S21_EXTERNAL:
            ASSERT_EQ(info.lighting_type, DeviceInfo::LightingType::EXTERNAL); break;
        case wire::SysDeviceInfo::LIGHTING_TYPE_S21_PATTERN_PROJECTOR:
            ASSERT_EQ(info.lighting_type, DeviceInfo::LightingType::PATTERN_PROJECTOR); break;
        default: {CRL_EXCEPTION("Unsupported lighting type");}
    }
    ASSERT_EQ(wire.numberOfLights, info.number_of_lights);
}

TEST(convert, wire_to_info)
{
    const auto info = create_wire_info("test", "key");
    check_equal(info, convert(info), "key");
}

TEST(convert, info_to_wire)
{
    const auto info = create_info("test");
    check_equal(convert(info, "key"), info, "key");
}
