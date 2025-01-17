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

    ///
    /// @brief Start a collection of image streams. Repeated calls to this function will not stop implicitly
    ///        stop the previously started streams. For example if a user started a left_raw stream in one
    ///        call and a disparity stream in a second call, both streams would be active until stop_streams
    ///        is called for either
    ///
    virtual bool start_streams(const std::vector<DataSource> &sources);

    ///
    /// @brief Stop a collection of streams
    ///
    virtual bool stop_streams(const std::vector<DataSource> &sources);

    ///
    /// @brief Add a frame callback to get serviced inline with the receipt of a new frame. Only
    ///        a single frame callback can be added to the channel
    ///        NOTE: Perform minimal work in this callback, and ideally copy the lightweight
    ///        ImageFrame object out to another processing thread
    ///
    virtual void add_image_frame_callback(std::function<void(const ImageFrame&)> callback);

    ///
    /// @brief Initialize the connection to the camera
    ///
    virtual bool connect(const ChannelConfig &config);

    ///
    /// @brief Disconnect from the camera
    ///
    virtual void disconnect();

    ///
    /// @brief A blocking call that waits for one frame from the camera.
    ///
    /// If you’ve set a receive timeout (via ChannelConfig), it will block until that timeout expires;
    /// otherwise, it blocks indefinitely until data arrives.
    ///
    /// @return The newly received ImageFrame, or std::nullopt if timed out (and you used a timeout).
    ///
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
    /// @brief Handle internal process, and potentially dispatch a image
    ///
    void handle_and_dispatch(Image image,
                            int64_t frame_id,
                            const std::chrono::system_clock::time_point &capture_time,
                            const std::chrono::system_clock::time_point &ptp_capture_time);

    ///
    /// @brief Internal mutex used to handle updates to the user supplied frame callback
    ///
    std::mutex m_mutex;

    ///
    /// @brief Channel config
    ///
    ChannelConfig m_config;

    ///
    /// @brief Active network socket for receiving and transmitting data
    ///
    NetworkSocket m_socket;

    ///
    /// @brief Helper object to receive UDP traffic. Internally manages a receive thread
    ///
    std::unique_ptr<UdpReceiver> m_udp_receiver = nullptr;

    ///
    /// @brief A collection of buffers to avoid dynamic allocation for incoming messages
    ///
    std::shared_ptr<BufferPool> m_buffer_pool = nullptr;

    ///
    /// @brief Helper object to assemble raw UDP packets into complete MultiSense wire messages
    ///
    MessageAssembler m_message_assembler;

    ///
    /// @brief Monotonically increasing internal id used to uniquely identify requests sent to the camera
    ///
    std::atomic_uint16_t m_transmit_id = 0;

    ///
    /// @brief The current cached calibration stored here for convenience
    ///
    CameraCalibration m_calibration;

    ///
    /// @brief The current set of active data streams the MultiSense is transmitting.
    ///
    std::set<DataSource> m_active_streams;

    ///
    /// @brief The currently active image frame user callback
    ///
    std::function<void(const ImageFrame&)> m_user_image_frame_callback;

    ///
    /// @brief Mutex, condition_variable, and frame used to service the get_next_image_frame member function
    ///
    std::mutex m_next_mutex;
    std::condition_variable m_next_cv;
    std::optional<ImageFrame> m_next_frame;

    ///
    /// @brief A cache of image metadata associated with a specific frame id
    ///
    std::map<int64_t, crl::multisense::details::wire::ImageMeta> m_meta_cache;

    ///
    /// @brief A cache of image frames associated with a specific frame id
    ///
    std::map<int64_t, ImageFrame> m_frame_buffer;
};

}
}
