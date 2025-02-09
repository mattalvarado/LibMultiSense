/**
 * @file DeviceInfoUtility.cc
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
 *   2025-02-08, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h> // htons
#endif

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

#include <nlohmann/json.hpp>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>


using json = nlohmann::json;
namespace lms = multisense;
using info = multisense::MultiSenseInfo;

namespace
{

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address> : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>             : MTU to use to communicate with the camera (default=1500)" << std::endl;
    std::cerr << "\t-s <file-path>       : The file path to the device info to set" << std::endl;
    std::cerr << "\t-k <key>             : The key required to set the device info" << std::endl;
    std::cerr << "\t-y                   : Disable the confirmation prompt" << std::endl;
    exit(1);
}

std::string to_string(info::DeviceInfo::HardwareRevision rev)
{
    switch (rev)
    {
        case info::DeviceInfo::HardwareRevision::UNKNOWN:      return "UNKNOWN";
        case info::DeviceInfo::HardwareRevision::S7:           return "S7";
        case info::DeviceInfo::HardwareRevision::S21:          return "S21";
        case info::DeviceInfo::HardwareRevision::ST21:         return "ST21";
        case info::DeviceInfo::HardwareRevision::S27:          return "S27";
        case info::DeviceInfo::HardwareRevision::S30:          return "S30";
        case info::DeviceInfo::HardwareRevision::KS21:         return "KS21";
        case info::DeviceInfo::HardwareRevision::MONOCAM:      return "MONOCAM";
        case info::DeviceInfo::HardwareRevision::KS21_SILVER:  return "KS21_SILVER";
        case info::DeviceInfo::HardwareRevision::ST25:         return "ST25";
        case info::DeviceInfo::HardwareRevision::KS21i:        return "KS21i";
    }
    return "UNKNOWN";
}

info::DeviceInfo::HardwareRevision hardware_revision_from_string(const std::string& s)
{
    if (s == "S7")           return info::DeviceInfo::HardwareRevision::S7;
    if (s == "S21")          return info::DeviceInfo::HardwareRevision::S21;
    if (s == "ST21")         return info::DeviceInfo::HardwareRevision::ST21;
    if (s == "S27")          return info::DeviceInfo::HardwareRevision::S27;
    if (s == "S30")          return info::DeviceInfo::HardwareRevision::S30;
    if (s == "KS21")         return info::DeviceInfo::HardwareRevision::KS21;
    if (s == "MONOCAM")      return info::DeviceInfo::HardwareRevision::MONOCAM;
    if (s == "KS21_SILVER")  return info::DeviceInfo::HardwareRevision::KS21_SILVER;
    if (s == "ST25")         return info::DeviceInfo::HardwareRevision::ST25;
    if (s == "KS21i")        return info::DeviceInfo::HardwareRevision::KS21i;
    return info::DeviceInfo::HardwareRevision::UNKNOWN;
}

std::string to_string(info::DeviceInfo::ImagerType type)
{
    switch (type)
    {
        case info::DeviceInfo::ImagerType::UNKNOWN:         return "UNKNOWN";
        case info::DeviceInfo::ImagerType::CMV2000_GREY:      return "CMV2000_GREY";
        case info::DeviceInfo::ImagerType::CMV2000_COLOR:     return "CMV2000_COLOR";
        case info::DeviceInfo::ImagerType::CMV4000_GREY:      return "CMV4000_GREY";
        case info::DeviceInfo::ImagerType::CMV4000_COLOR:     return "CMV4000_COLOR";
        case info::DeviceInfo::ImagerType::FLIR_TAU2:         return "FLIR_TAU2";
        case info::DeviceInfo::ImagerType::AR0234_GREY:       return "AR0234_GREY";
        case info::DeviceInfo::ImagerType::AR0239_COLOR:      return "AR0239_COLOR";
    }
    return "UNKNOWN";
}

info::DeviceInfo::ImagerType imager_from_string(const std::string& s)
{
    if (s == "CMV2000_GREY")   return info::DeviceInfo::ImagerType::CMV2000_GREY;
    if (s == "CMV2000_COLOR")  return info::DeviceInfo::ImagerType::CMV2000_COLOR;
    if (s == "CMV4000_GREY")   return info::DeviceInfo::ImagerType::CMV4000_GREY;
    if (s == "CMV4000_COLOR")  return info::DeviceInfo::ImagerType::CMV4000_COLOR;
    if (s == "FLIR_TAU2")      return info::DeviceInfo::ImagerType::FLIR_TAU2;
    if (s == "AR0234_GREY")    return info::DeviceInfo::ImagerType::AR0234_GREY;
    if (s == "AR0239_COLOR")   return info::DeviceInfo::ImagerType::AR0239_COLOR;
    return info::DeviceInfo::ImagerType::UNKNOWN;
}

std::string to_string(info::DeviceInfo::LightingType lt)
{
    switch (lt)
    {
        case info::DeviceInfo::LightingType::NONE:              return "NONE";
        case info::DeviceInfo::LightingType::INTERNAL:          return "INTERNAL";
        case info::DeviceInfo::LightingType::EXTERNAL:          return "EXTERNAL";
        case info::DeviceInfo::LightingType::PATTERN_PROJECTOR: return "PATTERN_PROJECTOR";
    }
    return "NONE";
}

info::DeviceInfo::LightingType lighting_from_string(const std::string& s)
{
    if (s == "INTERNAL")          return info::DeviceInfo::LightingType::INTERNAL;
    if (s == "EXTERNAL")          return info::DeviceInfo::LightingType::EXTERNAL;
    if (s == "PATTERN_PROJECTOR") return info::DeviceInfo::LightingType::PATTERN_PROJECTOR;
    return info::DeviceInfo::LightingType::NONE;
}

std::string to_string(info::DeviceInfo::LensType lt)
{
    switch (lt)
    {
        case info::DeviceInfo::LensType::UNKNOWN: return "UNKNOWN";
    }
    return "UNKNOWN";
}

info::DeviceInfo::LensType lens_from_string(const std::string& s)
{
    (void) s;
    return info::DeviceInfo::LensType::UNKNOWN;
}

json to_json(const info::DeviceInfo::PcbInfo& pcb)
{
    return json{{"name", pcb.name}, {"revision", pcb.revision}};
}

info::DeviceInfo::PcbInfo pcb_from_json(const json& j)
{
    return info::DeviceInfo::PcbInfo{j.at("name").get<std::string>(),
                                     j.at("revision").get<uint32_t>()};
}

json to_json(const info::DeviceInfo& d)
{
    json pcb_array = json::array();
    for (size_t i = 0; i < d.number_of_pcbs && i < d.pcb_info.size(); ++i)
    {
        pcb_array.push_back(to_json(d.pcb_info[i]));
    }

    return json{
        {"camera_name", d.camera_name},
        {"build_date", d.build_date},
        {"serial_number", d.serial_number},
        {"hardware_revision", to_string(d.hardware_revision)},
        {"number_of_pcbs", d.number_of_pcbs},
        {"pcb_info", pcb_array},
        {"imager_name", d.imager_name},
        {"imager_type", to_string(d.imager_type)},
        {"imager_width", d.imager_width},
        {"imager_height", d.imager_height},
        {"lens_name", d.lens_name},
        {"lens_type", to_string(d.lens_type)},
        {"nominal_stereo_baseline", d.nominal_stereo_baseline},
        {"nominal_focal_length", d.nominal_focal_length},
        {"nominal_relative_aperture", d.nominal_relative_aperture},
        {"lighting_type", to_string(d.lighting_type)},
        {"number_of_lights", d.number_of_lights}
    };
}

info::DeviceInfo info_from_json(const json& j)
{
    info::DeviceInfo d;

    const auto hardware_rev_string = j.at("hardware_revision").get<std::string>();

    const auto number_of_pcbs = j.at("number_of_pcbs").get<uint8_t>();

    auto pcb_array = j.at("pcb_info");
    std::array<info::DeviceInfo::PcbInfo, 8> pcb_info;
    for (size_t i = 0; i < d.number_of_pcbs && i < pcb_array.size() && i < pcb_info.size(); ++i)
    {
        pcb_info[i] = pcb_from_json(pcb_array[i]);
    }

    const auto imager_type_string = j.at("imager_type").get<std::string>();

    const auto lens_type = j.at("lens_type").get<std::string>();

    const auto lighting_type = j.at("lighting_type").get<std::string>();

    return info::DeviceInfo{
        j.at("camera_name").get<std::string>(),
        j.at("build_date").get<std::string>(),
        j.at("serial_number").get<std::string>(),
        hardware_revision_from_string(hardware_rev_string),
        number_of_pcbs,
        pcb_info,
        j.at("imager_name").get<std::string>(),
        imager_from_string(imager_type_string),
        j.at("imager_width").get<uint32_t>(),
        j.at("imager_height").get<uint32_t>(),
        j.at("lens_name").get<std::string>(),
        lens_from_string(lens_type),
        j.at("nominal_stereo_baseline").get<float>(),
        j.at("nominal_focal_length").get<float>(),
        j.at("nominal_relative_aperture").get<float>(),
        lighting_from_string(lighting_type),
        j.at("number_of_lights").get<uint32_t>()
    };
}

}

int main(int argc, char** argv)
{
    std::string ip_address = "10.66.171.21";
    int16_t mtu = 1500;
    std::optional<std::filesystem::path> device_info_path = std::nullopt;
    std::string key = "";
    bool disable_confirmation = false;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:s:k:y")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu = atoi(optarg); break;
            case 's': device_info_path = optarg; break;
            case 'k': key = optarg; break;
            case 'y': disable_confirmation = true; break;
            default: usage(*argv); break;
        }
    }

    const auto channel = lms::Channel::create(lms::Channel::ChannelConfig{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    if (device_info_path)
    {
        if (!disable_confirmation)
        {
            std::cout << "Really update device information? (y/n):" << std::endl;
            int reply = getchar();
            if ('Y' != reply && 'y' != reply)
            {
                std::cout << "Aborting" << std::endl;
                return 1;
            }
        }

        std::ifstream f(device_info_path.value());
        const auto device_info = info_from_json(json::parse(f));

        if (const auto status = channel->set_device_info(device_info, key); status != lms::Status::OK)
        {
            std::cerr << "Failed to set device info: " << lms::to_string(status) << std::endl;
            return 1;
        }

        std::cout << "Succesfully set device info" << std::endl;
        return 0;
    }
    else
    {
        auto info = to_json(channel->get_info().device);
        std::cout << info.dump(4) << std::endl;
    }

    return 0;
}
