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

#include <wire/ImageMessage.hh>
#include <wire/ImageMetaMessage.hh>
#include <wire/StatusRequestMessage.hh>
#include <wire/StatusResponseMessage.hh>
#include <wire/StreamControlMessage.hh>
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
    m_message_assembler(m_buffer_pool)
{
    connect(config);
}

LegacyChannel::~LegacyChannel()
{
    disconnect();
}


bool LegacyChannel::start_streams(const std::vector<DataSource> &sources)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::StreamControl cmd;

    cmd.enable(convert_sources(sources));

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      cmd,
                                      m_transmit_id++,
                                      m_config.mtu,
                                      500ms); ack)
    {
        if (ack->status != wire::Ack::Status_Ok)
        {
            CRL_DEBUG("Start streams ack invalid: %i", ack->status);
            return false;
        }

        m_active_streams.insert(std::begin(sources), std::end(sources));
        return true;
    }

    return false;
}

bool LegacyChannel::stop_streams(const std::vector<DataSource> &sources)
{
    using namespace crl::multisense::details;
    using namespace std::chrono_literals;

    wire::StreamControl cmd;

    cmd.disable(convert_sources(sources));

    if (const auto ack = wait_for_ack(m_message_assembler,
                                      m_socket,
                                      cmd,
                                      m_transmit_id++,
                                      m_config.mtu,
                                      500ms); ack)
    {
        if (ack->status != wire::Ack::Status_Ok)
        {
            CRL_DEBUG("Start streams ack invalid: %i", ack->status);
            return false;
        }

        for (const auto &source : sources)
        {
            m_active_streams.erase(source);
        }

        return true;
    }

    return false;
}

///
/// @brief Setup user callback that will be invoked whenever a new frame is received.
///
void LegacyChannel::add_image_frame_callback(std::function<void(const ImageFrame&)> callback)
{
    m_user_frame_callback = callback;
}

///
/// @brief Initialize the connection to the camera
///
bool LegacyChannel::connect(const ChannelConfig &config)
{
    using namespace crl::multisense::details;

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

    m_message_assembler.register_callback(wire::Image::ID,
                                          std::bind(&LegacyChannel::image_callback, this, std::placeholders::_1));

    m_message_assembler.register_callback(wire::ImageMeta::ID,
                                          std::bind(&LegacyChannel::image_meta_callback, this, std::placeholders::_1));

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
    using namespace crl::multisense::details;

    m_next_cv.notify_all();

    //
    // Stop all our streams before disconnecting
    //
    stop_streams({DataSource::ALL});

    m_message_assembler.remove_callback(wire::Image::ID);

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
    std::unique_lock<std::mutex> lock(m_next_mutex);

    std::optional<ImageFrame> output_frame = std::nullopt;
    if (m_config.receive_timeout)
    {
        if (std::cv_status::no_timeout == m_next_cv.wait_for(lock, m_config.receive_timeout.value()))
        {
            output_frame = std::move(m_next_frame);
        }
    }
    else
    {
        m_next_cv.wait(lock);
        output_frame = std::move(m_next_frame);
    }

    //
    // Reset our next frame
    //
    m_next_frame = std::nullopt;

    return output_frame;
}

void LegacyChannel::image_meta_callback(std::shared_ptr<const std::vector<uint8_t>> data)
{
    using namespace crl::multisense::details;

    const auto wire_meta = deserialize<wire::ImageMeta>(*data);

    m_meta_cache[wire_meta.frameId] = wire_meta;
}

void LegacyChannel::image_callback(std::shared_ptr<const std::vector<uint8_t>> data)
{
    using namespace crl::multisense::details;
    using namespace std::chrono;

    const auto wire_image = deserialize<wire::Image>(*data);

    std::cout << wire_image.frameId << std::endl;

    const auto meta = m_meta_cache.find(wire_image.frameId);
    if (meta == std::end(m_meta_cache))
    {
        CRL_DEBUG("Missing corresponding meta for frame_id %li\n", wire_image.frameId);
        return;
    }

    const system_clock::time_point capture_time{seconds{meta->second.timeSeconds} +
                                                microseconds{meta->second.timeMicroSeconds}};

    const system_clock::time_point ptp_capture_time{nanoseconds{meta->second.ptpNanoSeconds}};

    PixelFormat pixel_format = PixelFormat::UNKNOWN;
    switch (wire_image.bitsPerPixel)
    {
        case 8: {pixel_format = PixelFormat::MONO8; break;}
        case 16: {pixel_format = PixelFormat::MONO16; break;}
        default: {CRL_DEBUG("Uknown pixel format %d", wire_image.bitsPerPixel);}
    }

    const auto source = convert_sources(static_cast<uint64_t>(wire_image.sourceExtended) << 32 | wire_image.source);
    if (source.size() != 1)
    {
        CRL_DEBUG("invalid image source\n");
    }

    Image image{data,
                reinterpret_cast<const uint8_t*>(wire_image.dataP) - data->data(),
                pixel_format,
                wire_image.width,
                wire_image.height,
                capture_time,
                ptp_capture_time,
                source.front(),
                m_calibration};

    if (m_frame_buffer.count(wire_image.frameId) == 0)
    {
        ImageFrame frame{wire_image.frameId,
                         std::map<DataSource, Image>{std::make_pair(source.front(), std::move(image))},
                         capture_time,
                         ptp_capture_time};

        m_frame_buffer.emplace(wire_image.frameId, std::move(frame));
    }
    else
    {
        m_frame_buffer[wire_image.frameId].add_image(image);
    }

    //
    // Check if our frame is valid, if so dispatch to our callbacks and notify anyone who is waiting on
    // the next frame
    //
    if (const auto &frame = m_frame_buffer[wire_image.frameId]; std::all_of(std::begin(m_active_streams),
                                                                            std::end(m_active_streams),
                                                                            [&frame](const auto &e){return frame.has_image(e);}))
    {
        if (m_user_frame_callback)
        {
            m_user_frame_callback(frame);
        }

        std::lock_guard<std::mutex> lock(m_next_mutex);
        m_next_frame = frame;
        m_next_cv.notify_all();

        //
        // Remove our image frame from our frame buffer and the associated image metadata. Since frames ids will
        // only monotonically increase, it's safe to also delete all the frame_ids earlier than the current
        // frame id.
        //
        m_frame_buffer.erase(std::begin(m_frame_buffer), m_frame_buffer.upper_bound(wire_image.frameId));
        m_meta_cache.erase(std::begin(m_meta_cache), m_meta_cache.upper_bound(wire_image.frameId));
    }
}

}
}
