/**
 * @file SaveImageUtility.hh
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
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

#include <csignal>
#include <iostream>

#include <MultiSense/MultiSenseChannel.hh>


namespace lms = multisense;

namespace
{

volatile bool done = false;

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address>    : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>                : MTU to set the camera to (default=1500)" << std::endl;
    std::cerr << "\t-r <head_id>    : remote head ID (default=0)" << std::endl;

    exit(1);
}

#ifdef WIN32
BOOL WINAPI signal_handler(DWORD dwCtrlType)
{
    (void) dwCtrlType:
    done = true;
    return TRUE;
}
#else
void signal_handler(int sig)
{
    (void) sig;
    done = true;
}
#endif

}

int main(int argc, char** argv)
{

#if WIN32
    SetConsoleCtrlHandler (signal_handler, TRUE);
#else
    signal(SIGINT, signal_handler);
#endif

    std::string ip_address = "10.66.171.21";
    int16_t mtu = 1500;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu            = atoi(optarg);    break;
            default: usage(*argv);                      break;
        }
    }

    const auto channel = lms::Channel::create(lms::ChannelConfig{ip_address, mtu});

    if (!channel->start_streams({lms::DataSource::LEFT_RECTIFIED_RAW, lms::DataSource::LEFT_DISPARITY_RAW}))
    {
        std::cerr << "Cannot start streams" << std::endl;
        return 1;
    }

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            for (const auto &[source, image] : image_frame->images)
            {
                std::cout << image.width << " " << image.height << " " << image.raw_data->size() - image.image_data_offset << std::endl;
            }
        }
    }

    return 0;
}
