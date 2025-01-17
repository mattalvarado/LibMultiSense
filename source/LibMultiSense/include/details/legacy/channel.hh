/**
 * @file channel.hh
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

#pragma once

#include <set>

#include "MultiSense/MultiSenseChannel.hh"

#include <wire/Protocol.hh>
#include <utility/BufferStream.hh>
#include <wire/ImageMetaMessage.hh>

#include "details/legacy/ip.hh"
#include "details/legacy/message.hh"
#include "details/legacy/storage.hh"
#include "details/legacy/udp.hh"

namespace multisense{
namespace legacy{

class LegacyChannel : public Channel
{
public:
    explicit LegacyChannel(const ChannelConfig &config);

    virtual ~LegacyChannel();

    virtual bool start_streams(const std::vector<DataSource> &sources);

    virtual bool stop_streams(const std::vector<DataSource> &sources);

    virtual void add_image_frame_callback(std::function<void(const ImageFrame&)> callback);

    virtual bool connect(const ChannelConfig &config);

    virtual void disconnect();

    virtual std::optional<ImageFrame> get_next_image_frame();

private:
    ///
    /// @brief Image meta callback used to internally receive images sent from the MultiSense
    ///
    void image_meta_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Image callback used to internally receive images sent from the MultiSense
    ///
    void image_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Disparity callback used to internally receive images sent from the MultiSense
    ///
    void disparity_callback(std::shared_ptr<const std::vector<uint8_t>> data);

    ///
    /// @brief Handle internall process, and potentially dispatch a image
    ///
    void handle_and_dispatch(Image image,
                            int64_t frame_id,
                            const std::chrono::system_clock::time_point &capture_time,
                            const std::chrono::system_clock::time_point &ptp_capture_time);

    std::mutex m_mutex;

    ChannelConfig m_config;

    NetworkSocket m_socket;

    std::unique_ptr<UdpReceiver> m_udp_receiver = nullptr;

    std::shared_ptr<BufferPool> m_buffer_pool = nullptr;

    MessageAssembler m_message_assembler;

    std::atomic_uint16_t m_transmit_id = 0;

    CameraCalibration m_calibration;

    std::set<DataSource> m_active_streams;

    std::function<void(const ImageFrame&)> m_user_frame_callback;

    std::mutex m_next_mutex;
    std::condition_variable m_next_cv;
    std::optional<ImageFrame> m_next_frame;

    std::map<int64_t, crl::multisense::details::wire::ImageMeta> m_meta_cache;
    std::map<int64_t, ImageFrame> m_frame_buffer;
};

}
}
