/**
 * @file message.cc
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

#include <iostream>
#include <limits>

#include <utility/BufferStream.hh>
#include <wire/Protocol.hh>

#include <wire/DisparityMessage.hh>

#include "details/legacy/message.hh"

namespace multisense{
namespace legacy{

///
/// @brief Unwrap a 16-bit wire sequence ID into a unique 64-bit local ID
///
///
int64_t unwrap_sequence_id(uint16_t current_wire_id, int32_t previous_wire_id)
{
    int64_t unwrapped_id = previous_wire_id;

    //
    // Look for a sequence change

    if (current_wire_id != previous_wire_id)
    {

        CRL_CONSTEXPR uint16_t ID_MAX  = std::numeric_limits<uint16_t>::max();
        CRL_CONSTEXPR uint16_t ID_MASK = 0xF000;
        CRL_CONSTEXPR uint16_t ID_HIGH = 0xF000;
        CRL_CONSTEXPR uint16_t ID_LOW  = 0x0000;

        if (-1 == previous_wire_id)
        {
            //
            // Seed
            //
			unwrapped_id = current_wire_id;
        }
        else if (((current_wire_id & ID_MASK) == ID_LOW) &&
                ((previous_wire_id & ID_MASK) == ID_HIGH))
        {
            //
            // Detect forward 16-bit wrap
            //
            unwrapped_id += 1 + (static_cast<int64_t>(ID_MAX) - previous_wire_id) + current_wire_id;
        }
        else
        {
            //
            // Normal case
            //
            unwrapped_id += static_cast<int64_t>(current_wire_id) - previous_wire_id;
        }
    }

    return unwrapped_id;
}

crl::multisense::details::wire::IdType get_message_type(const std::vector<uint8_t>& raw_buffer)
{
    using namespace crl::multisense::details;

    utility::BufferStreamReader stream(raw_buffer.data(), raw_buffer.size());
    stream.seek(sizeof(wire::Header));
    wire::IdType message_type;
    stream & message_type;

    return message_type;
}

MessageAssembler::MessageAssembler(std::shared_ptr<BufferPool> buffer_pool):
    m_buffer_pool(buffer_pool)
{
}

void MessageAssembler::process_packet(const std::vector<uint8_t> &raw_data)
{
    using namespace crl::multisense::details;

    if (raw_data.size() < static_cast<int>(sizeof(wire::Header)))
    {
        CRL_EXCEPTION("undersized packet: %d/%d bytes\n",
                      raw_data.size(), sizeof(wire::Header));
    }

    const wire::Header& header = *(reinterpret_cast<const wire::Header*>(raw_data.data()));

    if (wire::HEADER_MAGIC != header.magic)
    {
        CRL_EXCEPTION("bad protocol magic: 0x%x, expecting 0x%x",
                      header.magic, wire::HEADER_MAGIC);
    }
    else if (wire::HEADER_VERSION != header.version)
    {
        CRL_EXCEPTION("bad protocol version: 0x%x, expecting 0x%x",
                      header.version, wire::HEADER_VERSION);
    }
    else if (wire::HEADER_GROUP != header.group)
    {
        CRL_EXCEPTION("bad protocol group: 0x%x, expecting 0x%x",
                      header.group, wire::HEADER_GROUP);
    }

    //
    // Create a unique sequence ID based on the wire ID
    //
    const int64_t full_sequence_id = unwrap_sequence_id(header.sequenceIdentifier, m_previous_wire_id);
    m_previous_wire_id = header.sequenceIdentifier;

    auto active_message = std::end(m_active_messages);

    const auto buffer_config = m_buffer_pool->get_config();
    const bool is_large_buffer = header.messageLength > buffer_config.small_buffer_size;
    auto& ordered_messages = (is_large_buffer ? m_large_ordered_messages : m_small_ordered_messages);

    //
    // We are not currently tracking this message
    //
    if (m_active_messages.count(full_sequence_id) == 0)
    {
        if (header.messageLength > buffer_config.large_buffer_size)
        {
            CRL_DEBUG("No buffers large enough to fit a message of %d bytes\n", header.messageLength);
            return;
        }

        const auto max_active_messages = (is_large_buffer ? buffer_config.num_large_buffers :
                                                            buffer_config.num_small_buffers);

        //
        // Remove old messages that we may no longer need to make sure
        //
        while ((ordered_messages.size() + 1) > max_active_messages)
        {
            const auto old_sequence = ordered_messages.front();
            ordered_messages.pop_front();
            m_active_messages.erase(old_sequence);
        }

        //
        // We received a new header, setup a new buffer for our message
        //
        auto buffer = m_buffer_pool->get_buffer(header.messageLength);
        if (buffer == nullptr)
        {
            CRL_DEBUG("No free buffers available\n");
            return;
        }

        const auto message_type = get_message_type(raw_data);
        auto [inserted_iterator,_] = m_active_messages.emplace(full_sequence_id,
                                                               InternalMessage{message_type, 0, std::move(buffer)});
        active_message = inserted_iterator;
        ordered_messages.emplace_back(full_sequence_id);
    }

    active_message = (active_message == std::end(m_active_messages)) ? m_active_messages.find(full_sequence_id) : active_message;

    if (active_message != std::end(m_active_messages))
    {
        const size_t bytes_to_write = raw_data.size() - sizeof(wire::Header);

        auto& message = active_message->second;

        if (bytes_to_write + message.bytes_written >  message.data->size())
        {
            CRL_EXCEPTION("Error. Write Will overrun internall buffer");
        }

        //
        // Handle the special case unpacking of disparity data from 12bit to 16bit images
        //
        if(message.type == MSG_ID(wire::Disparity::ID))
        {
            utility::BufferStreamWriter stream(message.data->data(),
                                               message.data->size());

            wire::Disparity::assembler(stream,
                                       raw_data.data() + sizeof(wire::Header),
                                       header.byteOffset,
                                       bytes_to_write);

        }
        else
        {
            memcpy(message.data->data() + header.byteOffset,
                   raw_data.data() + sizeof(wire::Header),
                   bytes_to_write);
        }

        message.bytes_written += bytes_to_write;

        if (message.bytes_written == message.data->size())
        {
            //
            // Handle the special case for Ack messages. To avoid collisions for all the Ack
            // registrations, we extract the command associated with the Ack, and dispatch
            // to all the listeners for that command
            //
            if (message.type == MSG_ID(wire::Ack::ID))
            {
                const auto ack = deserialize<wire::Ack>(*message.data);
                dispatch(ack.command, message.data);
            }
            else
            {
                dispatch(message.type, message.data);
            }

            //
            // Remove processed messages
            //
            m_active_messages.erase(active_message);
            for (auto it = std::begin(ordered_messages); it != std::end(ordered_messages);)
            {
                if (*it == full_sequence_id)
                {
                    it = ordered_messages.erase(it);
                    break;
                }
                else
                {
                    ++it;
                }
            }
        }
    }
    else if (header.byteOffset != 0)
    {
        CRL_DEBUG("Missed first packet. Dropping entire message\n");
    }
}

std::shared_ptr<MessageCondition> MessageAssembler::register_message(const crl::multisense::details::wire::IdType &message_id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (auto it = m_conditions.find(message_id); it != std::end(m_conditions))
    {
        std::lock_guard<std::mutex> lock(it->second->mutex);
        it->second->notified = false;
        return it->second;
    }
    else
    {
        auto [new_it, valid] = m_conditions.emplace(message_id, std::make_shared<MessageCondition>());

        if (valid)
        {
            return new_it->second;
        }
    }

    return nullptr;
}

void MessageAssembler::remove_registration(const crl::multisense::details::wire::IdType &message_id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (auto it = m_conditions.find(message_id); it != std::end(m_conditions))
    {
        m_conditions.erase(it);
    }
}

void MessageAssembler::register_callback(const crl::multisense::details::wire::IdType& message_id,
                                         std::function<void(std::shared_ptr<const std::vector<uint8_t>>)> callback)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callbacks.emplace(message_id, callback);
}

void MessageAssembler::remove_callback(const crl::multisense::details::wire::IdType& message_id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (auto it = m_callbacks.find(message_id); it != std::end(m_callbacks))
    {
        m_callbacks.erase(it);
    }
}

void MessageAssembler::dispatch(const crl::multisense::details::wire::IdType& message_id,
                                std::shared_ptr<std::vector<uint8_t>> data)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (auto condition = m_conditions.find(message_id); condition != std::end(m_conditions))
    {
        std::lock_guard<std::mutex> lock(condition->second->mutex);
        condition->second->data = *data;
        condition->second->notified = true;
        condition->second->cv.notify_all();

        //
        // Remove our registration after we dispatch the message
        //
        m_conditions.erase(condition);
    }

    if (auto callback = m_callbacks.find(message_id); callback != std::end(m_callbacks))
    {
        callback->second(data);
    }
}

}
}