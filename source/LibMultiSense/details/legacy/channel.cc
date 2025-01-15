/**
 * @file channel.cc
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

#include <cstring>
#include <iostream>

#include <utility/Exception.hh>
#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>
#include <wire/StatusRequestMessage.hh>
#include <wire/StatusResponseMessage.hh>
#include <wire/SysGetCameraCalibrationMessage.hh>
#include <wire/SysCameraCalibrationMessage.hh>

#include "details/legacy/channel.hh"
#include "details/legacy/message.hh"
#include "details/legacy/utilities.hh"

namespace multisense {
namespace legacy {

LegacyChannel::LegacyChannel(const ChannelConfig &config):
    m_config(config),
    m_buffer_pool(std::make_shared<BufferPool>(BufferPoolConfig{config.receive_buffer_configuration.num_small_buffers,
                                                                config.receive_buffer_configuration.small_buffer_size,
                                                                config.receive_buffer_configuration.num_large_buffers,
                                                                config.receive_buffer_configuration.large_buffer_size})),
    m_message_assembler(m_buffer_pool, config.receive_buffer_configuration.num_small_buffers +
                                       config.receive_buffer_configuration.num_large_buffers)
{
    connect(config);
}

LegacyChannel::~LegacyChannel()
{
    disconnect();
}


bool LegacyChannel::start_streams(const std::vector<DataSource> &sources)
{
    (void) sources;
    return true;
}

bool LegacyChannel::stop_streams(const std::vector<DataSource> &sources)
{
    (void) sources;
    return false;
}

///
/// @brief Setup user callback that will be invoked whenever a new frame is received.
///
void LegacyChannel::add_image_frame_callback(std::function<void(const ImageFrame&)> callback)
{
    (void) callback;
}

///
/// @brief Initialize the connection to the camera
///
bool LegacyChannel::connect(const ChannelConfig &config)
{
#if WIN32
    WSADATA wsaData;
    int result = WSAStartup (MAKEWORD (0x02, 0x02), &wsaData);
    if (result != 0)
    {
        CRL_EXCEPTION("WSAStartup() failed: %d", result);
    }

#endif

    m_socket.sensor_address = get_sockaddr(config.ip_address, config.command_port);

    auto [sensor_socket, server_socket_port] = bind(config.interface);
    m_socket.sensor_socket = sensor_socket;
    m_socket.server_socket_port = server_socket_port;

    m_udp_receiver = std::make_unique<UdpReceiver>(m_socket, config.mtu,
            [this](const std::vector<uint8_t>& data)
            {
                this->m_message_assembler.process_packet(data);
            });

    return true;
}

///
/// @brief Disconnect from the camera
///
void LegacyChannel::disconnect()
{
    m_socket = NetworkSocket{};

    m_udp_receiver = nullptr;
    return;
}

///
/// @brief A blocking call that waits for one frame from the camera.
///
/// If you’ve set a receive timeout (via ChannelConfig), it will block until that timeout expires;
/// otherwise, it blocks indefinitely until data arrives.
///
/// @return The newly received ImageFrame, or std::nullopt if timed out (and you used a timeout).
///
std::optional<ImageFrame> LegacyChannel::get_next_image_frame()
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    //auto waiter = m_message_assembler.register_message(wire::StatusResponse::ID);

    //publish_data(m_socket, serialize(wire::StatusRequest(), 0,  m_config.mtu));

    //if(auto status = waiter->wait_for<wire::StatusResponse>(500ms); status)
    if (const auto status = wait_for_data<wire::StatusResponse>(m_message_assembler,
                                                                m_socket,
                                                                wire::StatusRequest(),
                                                                m_transmit_id++,
                                                                m_config.mtu,
                                                                500ms); status)
    {
        std::cout << status->uptime.getNanoSeconds() << std::endl;
        std::cout << status->status << std::endl;
        std::cout << status->temperature0 << std::endl;
        std::cout << status->temperature1 << std::endl;
        std::cout << status->temperature2 << std::endl;
        std::cout << status->temperature3 << std::endl;
        std::cout << status->inputVolts << std::endl;
        std::cout << status->inputCurrent << std::endl;
        std::cout << status->fpgaPower << std::endl;
        std::cout << status->logicPower << std::endl;
        std::cout << status->imagerPower << std::endl;
    }

    if (const auto cal = wait_for_data<wire::SysCameraCalibration>(m_message_assembler,
                                                                m_socket,
                                                                wire::SysGetCameraCalibration(),
                                                                m_transmit_id++,
                                                                m_config.mtu,
                                                                500ms); cal)
    {
        for (size_t i = 0 ; i < 3 ; ++i)
        {
            for (size_t j = 0 ; j < 3 ; ++j)
            {
                std::cout << cal->left.M[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        for (size_t i = 0 ; i < 8 ; ++i)
        {
            std::cout << cal->left.D[i] << " ";
        }
        std::cout << std::endl;
    }

    return std::nullopt;
}

}
}
