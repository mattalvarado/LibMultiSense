/**
 * @file device_info.cc
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
 *   2025-01-17, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include "details/legacy/device_info.hh"

namespace multisense {
namespace legacy {

DeviceInfo convert(const crl::multisense::details::wire::SysDeviceInfo &info)
{
    using namespace crl::multisense::details;

    DeviceInfo output;

    output.camera_name = info.name;
    output.build_date = info.buildDate;
    output.serial_number = info.serialNumber;

    switch (info.hardwareRevision)
    {
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7:
            {output.hardware_revision = DeviceInfo::HardwareRevision::S7; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21:
            {output.hardware_revision = DeviceInfo::HardwareRevision::S21; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21:
            {output.hardware_revision = DeviceInfo::HardwareRevision::ST21; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27:
            {output.hardware_revision = DeviceInfo::HardwareRevision::S27; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S30:
            {output.hardware_revision = DeviceInfo::HardwareRevision::S30; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21:
            {output.hardware_revision = DeviceInfo::HardwareRevision::KS21; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM:
            {output.hardware_revision = DeviceInfo::HardwareRevision::MONOCAM; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21_SILVER:
            {output.hardware_revision = DeviceInfo::HardwareRevision::KS21_SILVER; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST25:
            {output.hardware_revision = DeviceInfo::HardwareRevision::ST25; break;}
        case wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21i:
            {output.hardware_revision = DeviceInfo::HardwareRevision::KS21i; break;}
        default: {CRL_EXCEPTION("Unsupported hardware revision");}
    }

    output.number_of_pcbs = info.numberOfPcbs;
    for (uint32_t i = 0; i < info.numberOfPcbs; ++i)
    {
        output.pcb_info[i].name = info.pcbs[i].name;
        output.pcb_info[i].revision = info.pcbs[i].revision;
    }

    output.imager_name = info.imagerName;
    switch (info.imagerType)
    {
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_GREY:
            {output.imager_type = DeviceInfo::ImagerType::CMV2000_GREY; break;}
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV2000_COLOR:
            {output.imager_type = DeviceInfo::ImagerType::CMV2000_COLOR; break;}
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_GREY:
            {output.imager_type = DeviceInfo::ImagerType::CMV4000_GREY; break;}
        case wire::SysDeviceInfo::IMAGER_TYPE_CMV4000_COLOR:
            {output.imager_type = DeviceInfo::ImagerType::CMV4000_COLOR; break;}
        case wire::SysDeviceInfo::IMAGER_TYPE_FLIR_TAU2:
            {output.imager_type = DeviceInfo::ImagerType::FLIR_TAU2; break;}
        case wire::SysDeviceInfo::IMAGER_TYPE_AR0234_GREY:
            {output.imager_type = DeviceInfo::ImagerType::AR0234_GREY; break;}
        case wire::SysDeviceInfo::IMAGER_TYPE_AR0239_COLOR:
            {output.imager_type = DeviceInfo::ImagerType::AR0239_COLOR; break;}
        default: {CRL_EXCEPTION("Unsupported imager type");}
    }

    // TOOD(malvarado): finish this migration

    return output;
}

crl::multisense::details::wire::SysDeviceInfo convert(const DeviceInfo &info, const std::string &key)
{
    using namespace crl::multisense::details;

    wire::SysDeviceInfo output;

    output.key = key;
    output.name = info.camera_name;
    output.buildDate = info.build_date;
    output.serialNumber = info.serial_number;

    switch (info.hardware_revision)
    {
        case DeviceInfo::HardwareRevision::SL:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7; break;}
        case DeviceInfo::HardwareRevision::S7:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S7S; break;}
        case DeviceInfo::HardwareRevision::S21:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S21; break;}
        case DeviceInfo::HardwareRevision::ST21:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST21; break;}
        case DeviceInfo::HardwareRevision::S27:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27; break;}
        case DeviceInfo::HardwareRevision::S30:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_S30; break;}
        case DeviceInfo::HardwareRevision::KS21:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21; break;}
        case DeviceInfo::HardwareRevision::MONOCAM:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM; break;}
        case DeviceInfo::HardwareRevision::KS21_SILVER:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21_SILVER; break;}
        case DeviceInfo::HardwareRevision::ST25:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_ST25; break;}
        case DeviceInfo::HardwareRevision::KS21i:
            {output.hardwareRevision = wire::SysDeviceInfo::HARDWARE_REV_MULTISENSE_KS21i; break;}
        default: {CRL_EXCEPTION("Unsupported hardware revision");}
    }

    output.numberOfPcbs = info.number_of_pcbs;
    for (uint32_t i = 0; i < info.number_of_pcbs; ++i)
    {
        output.pcbs[i].name = info.pcb_info[i].name;
        output.pcbs[i].revision = info.pcb_info[i].revision;
    }

    return output;
}

}
}
