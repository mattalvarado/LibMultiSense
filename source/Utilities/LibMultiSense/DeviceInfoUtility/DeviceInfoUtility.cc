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
#include <MultiSense/MultiSenseSerialization.hh>


using json = nlohmann::json;
namespace lms = multisense;

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
        const auto device_info = json::parse(f).template get<lms::MultiSenseInfo::DeviceInfo>();

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
        const json info = channel->get_info().device;
        std::cout << info.dump(4) << std::endl;
    }

    return 0;
}
