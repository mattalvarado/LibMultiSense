/**
 * @file message.hh
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
 *   2025-01-08, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>

#include <utility/BufferStream.hh>
#include <wire/Protocol.hh>
#include <wire/AckMessage.hh>

#include "details/legacy/storage.hh"

namespace multisense{
namespace legacy{

template <typename T>
T deserialize(const std::vector<uint8_t>& data)
{
    using namespace crl::multisense::details;

    utility::BufferStreamReader stream{data.data(), data.size()};

    wire::IdType      id;
    wire::VersionType version;

    stream & id;
    stream & version;
    T m(stream, version);

    return m;
}

///
/// @brief Unwrap the wire ID into a full 16 bit sequence ID
///
int64_t unwrap_sequence_id(uint16_t current_wire_id, int32_t previous_wire_id);

///
/// @brief Get the message type of the message from the buffer
///
crl::multisense::details::wire::IdType get_message_type(const std::vector<uint8_t>& raw_buffer);

///
/// @brief Serialize a MultiSense Wire message for transmission
///
template <typename T>
std::vector<uint8_t> serialize(const T& message, uint16_t sequence_id, size_t mtu)
{
    using namespace crl::multisense::details;

    const wire::IdType      id      = T::ID;
    const wire::VersionType version = T::VERSION;

    std::vector<uint8_t> output_buffer(mtu - wire::COMBINED_HEADER_LENGTH);

    utility::BufferStreamWriter stream(output_buffer.data(), output_buffer.size());

    wire::Header& header = *(reinterpret_cast<wire::Header*>(stream.data()));
    header.magic = wire::HEADER_MAGIC;
    header.version = wire::HEADER_VERSION;
    header.group = wire::HEADER_GROUP;
    header.flags = 0;
    header.sequenceIdentifier = sequence_id;

    stream.seek(sizeof(wire::Header));
    stream & id;
    stream & version;
    const_cast<T*>(&message)->serialize(stream, version);

    header.messageLength = static_cast<uint32_t> (stream.tell() - sizeof(wire::Header));
    header.byteOffset = 0;

    output_buffer.resize(stream.tell());

    return output_buffer;
}

///
/// @brief A condition object which can be used to wait on messages from the stream
///
struct MessageCondition
{
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<uint8_t> data;
    bool notified = false;

    ///
    /// @brief convenience function equivalent to std::condition_variable::wait that performs a type
    ///        conversion to the target type
    ///
    template <typename T>
    T wait()
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this]{return this->notified;});
        return deserialize<T>(data);
    }

    ///
    /// @brief convenience function equivalent to std::condition_variable::wait_for that performs a type
    ///        conversion to the target type
    ///
    template <typename T, class Rep, class Period>
    std::optional<T> wait_for(const std::chrono::duration<Rep, Period>& abs_time)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (cv.wait_for(lock, abs_time, [this]{return this->notified;}))
        {
            return std::make_optional(deserialize<T>(data));
        }

        return std::nullopt;
    }
};

///
/// @brief Process incoming network data, and try the data into valid MultiSense Wire messages
///
class MessageAssembler
{
public:
    MessageAssembler(std::shared_ptr<BufferPool> buffer_pool, size_t max_active_messages);

    ~MessageAssembler() = default;

    void process_packet(const std::vector<uint8_t> &raw_data);

    ///
    /// @brief Register to be notified when a message of a given id arrives. Note this registration will only
    ///        receive a single message. To receive multiple messages please use the callback interface
    ///
    std::shared_ptr<MessageCondition> register_message(const crl::multisense::details::wire::IdType &message_id);

    void remove_registration(const crl::multisense::details::wire::IdType &message_id);

    ///
    /// @brief Register a callback to receive valid messages of a given id. Note currently only one callback
    ///        can be registered for a message_id
    ///
    void register_callback(const crl::multisense::details::wire::IdType& message_id,
                           std::function<void(std::shared_ptr<const std::vector<uint8_t>>)> callback);

    void remove_callback(const crl::multisense::details::wire::IdType& message_id);

private:

    ///
    /// @brief An internal message object used to track how many bytes were written to a data buffer
    ///
    struct InternalMessage
    {
        size_t bytes_written = 0;
        std::shared_ptr<std::vector<uint8_t>> data;
    };

    ///
    /// @brief Dispatch to both our registrations and callbacks
    ///
    void dispatch(const crl::multisense::details::wire::IdType& message_id,
                  std::shared_ptr<std::vector<uint8_t>> data);

    std::mutex m_mutex;

    ///
    /// @brief Buffer pool used to allocate messages into
    ///
    std::shared_ptr<BufferPool> m_buffer_pool = nullptr;
    size_t m_max_active_messages = 0;

    int32_t m_previous_wire_id = -1;

    std::deque<int64_t> m_ordered_messages;
    std::map<int64_t, InternalMessage> m_active_messages;

    std::map<crl::multisense::details::wire::IdType, std::shared_ptr<MessageCondition>> m_conditions;

    std::map<crl::multisense::details::wire::IdType,
             std::function<void(std::shared_ptr<const std::vector<uint8_t>>)>> m_callbacks;

};

}
}
