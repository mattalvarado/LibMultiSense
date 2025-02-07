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
#include <MultiSense/MultiSenseUtilities.hh>


namespace lms = multisense;

namespace
{

volatile bool done = false;

void imu_callback(const lms::ImuFrame &frame)
{
    std::cout << "imu callback " << frame.samples.size() << std::endl;
}

void image_callback(const lms::ImageFrame &frame)
{
    std::cout << "image callback " << frame.frame_id << std::endl;
}

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

    const auto channel = lms::Channel::create(lms::Channel::ChannelConfig{ip_address, mtu, std::chrono::milliseconds{500}});

    auto config = channel->get_configuration();
    config.frames_per_second = 30.0;
    if (const auto status = channel->set_configuration(config); status != lms::Status::OK)
    {
        std::cerr << "Cannot set config" << std::endl;
        return 1;
    }

    channel->add_imu_frame_callback(imu_callback);
    channel->add_image_frame_callback(image_callback);

    if (const auto status = channel->start_streams({lms::DataSource::LEFT_RECTIFIED_RAW,
                                                    lms::DataSource::LEFT_DISPARITY_RAW,
                                                    lms::DataSource::IMU}); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            const auto point_cloud = lms::create_color_pointcloud<uint8_t>(image_frame.value(), 20.0, lms::DataSource::LEFT_RECTIFIED_RAW);

            if (point_cloud)
            {
                write_pointcloud_ply(point_cloud.value(), std::to_string(image_frame->frame_id) + ".ply");
            }

            const auto depth_image = lms::create_depth_image(image_frame.value(), lms::Image::PixelFormat::FLOAT32);
            if (depth_image)
            {
                write_image(depth_image.value(), std::to_string(image_frame->frame_id) + "_depth.tiff");
            }
            //for (const auto &[source, image] : image_frame->images)
            //{
            //    std::cout << "frame " << image_frame->frame_id << " " << static_cast<int>(source) << std::endl;
            //    //write_image(image, std::to_string(image_frame->frame_id) + "_" + std::to_string(static_cast<int>(source)) + ".pgm");
            //}
        }

        if (const auto status = channel->get_system_status())
        {
            std::cout << status->system_ok << " " <<
                         status->client_network.received_messages << " " <<
                         status->client_network.dropped_messages << " " <<
                         status->client_network.invalid_packets << std::endl;
        }
    }

    return 0;
}
