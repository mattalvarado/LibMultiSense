/**
 * @file PointCloudUtility.cc
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
 *   2025-02-07, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h>
#else
#include <unistd.h>
#endif

#include <chrono>
#include <csignal>
#include <iostream>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include "getopt/getopt.h"

namespace lms = multisense;

namespace
{

volatile bool done = false;

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " [<options>]" << std::endl;
    std::cerr << "Where <options> are:" << std::endl;
    std::cerr << "\t-a <current_address> : CURRENT IPV4 address (default=10.66.171.21)" << std::endl;
    std::cerr << "\t-m <mtu>             : MTU to use to communicate with the camera (default=1500)" << std::endl;
    std::cerr << "\t-r <max-range>       : Current max range from the camera for points to be included (default=50m)" << std::endl;
    std::cerr << "\t-c                   : Flag to colorize the point clouds with the aux image" << std::endl;
    exit(1);
}

#ifdef WIN32
BOOL WINAPI signal_handler(DWORD dwCtrlType)
{
    (void) dwCtrlType;
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
    double max_range = 50.0;
    bool color = false;

    int c;
    while(-1 != (c = getopt(argc, argv, "a:m:r:c")))
    {
        switch(c)
        {
            case 'a': ip_address = std::string(optarg); break;
            case 'm': mtu = static_cast<uint16_t>(atoi(optarg)); break;
            case 'r': max_range = std::stod(optarg); break;
            case 'c': color = true; break;
            default: usage(*argv); break;
        }
    }

    const auto channel = lms::Channel::create(lms::Channel::Config{ip_address, mtu});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    //
    // QuerySet dynamic config from the camera
    //
    auto config = channel->get_config();
    config.frames_per_second = 10.0;
    if (const auto status = channel->set_config(config); status != lms::Status::OK)
    {
        std::cerr << "Cannot set config" << std::endl;
        return 1;
    }

    //
    // If our camera has an aux color camera, and we want to colorize our point clouds with the aux image,
    // color
    //
    const auto info = channel->get_info();
    const auto color_stream = (color && info.device.has_aux_camera()) ?
        lms::DataSource::AUX_RECTIFIED_RAW :
        lms::DataSource::LEFT_RECTIFIED_RAW;

    //
    // Start a single image stream
    //
    if (const auto status = channel->start_streams({color_stream,
                                                    lms::DataSource::LEFT_DISPARITY_RAW}); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            if (color_stream == lms::DataSource::LEFT_RECTIFIED_RAW)
            {
                if (const auto point_cloud = lms::create_color_pointcloud<uint8_t>(image_frame.value(),
                                                                                   max_range,
                                                                                   color_stream,
                                                                                   lms::DataSource::LEFT_DISPARITY_RAW); point_cloud)
                {
                    std::cout << "Saving pointcloud for frame id: " << image_frame->frame_id << std::endl;
                    lms::write_pointcloud_ply(point_cloud.value(), std::to_string(image_frame->frame_id) + ".ply");
                }
            }
            else
            {
                using ColorT = std::array<uint8_t, 3>;
                if (const auto bgr = create_bgr_image(image_frame.value(), color_stream); bgr)
                {
                    const auto &disparity = image_frame->get_image(lms::DataSource::LEFT_DISPARITY_RAW);
                    if (const auto point_cloud = lms::create_color_pointcloud<ColorT>(disparity,
                                                                                      bgr,
                                                                                      max_range,
                                                                                      image_frame->calibration); point_cloud)
                    {
                        std::cout << "Saving aux colorized pointcloud for frame id: " << image_frame->frame_id << std::endl;
                        lms::write_pointcloud_ply(point_cloud.value(), std::to_string(image_frame->frame_id) + ".ply");
                    }
                }
            }
        }
    }

    channel->stop_streams({lms::DataSource::ALL});

    return 0;
}
