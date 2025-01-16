/**
 * @file utilities.hh
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
 *   2025-01-13, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include "details/legacy/message.hh"
#include "details/legacy/udp.hh"

#include "MultiSense/MultiSenseTypes.hh"

namespace multisense{
namespace legacy{

constexpr crl::multisense::details::wire::SourceType all_sources = {
    crl::multisense::details::wire::SOURCE_LUMA_LEFT |
    crl::multisense::details::wire::SOURCE_LUMA_RIGHT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_LEFT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RIGHT |
    crl::multisense::details::wire::SOURCE_LUMA_RECT_LEFT |
    crl::multisense::details::wire::SOURCE_LUMA_RECT_RIGHT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RECTIFIED_LEFT |
    crl::multisense::details::wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT |
    crl::multisense::details::wire::SOURCE_DISPARITY |
    crl::multisense::details::wire::SOURCE_COMPRESSED_AUX |
    crl::multisense::details::wire::SOURCE_LUMA_RECT_AUX |
    crl::multisense::details::wire::SOURCE_CHROMA_AUX |
    crl::multisense::details::wire::SOURCE_CHROMA_RECT_AUX |
    crl::multisense::details::wire::SOURCE_DISPARITY_COST
};

std::vector<DataSource> convert_sources(const crl::multisense::details::wire::SourceType &source);

crl::multisense::details::wire::SourceType convert_sources(const std::vector<DataSource> &sources);

///
/// @brief Helper to wait for ack from the camera from a given query command. Once a query
///        command is sent to the MultiSense, it Ack's the command before sending the response
///
template <typename QueryMessage, class Rep, class Period>
std::optional<crl::multisense::details::wire::Ack> wait_for_ack(MessageAssembler &assembler,
                                                               const NetworkSocket &socket,
                                                               const QueryMessage &query,
                                                               uint16_t sequence_id,
                                                               uint16_t mtu,
                                                               const std::chrono::duration<Rep, Period>& wait_time,
                                                               size_t attempts = 1)
{
    using namespace crl::multisense::details;

    auto ack_waiter = assembler.register_message(MSG_ID(QueryMessage::ID));

    for (size_t i = 0 ; i < attempts ; ++i)
    {
        if(publish_data(socket, serialize(query, sequence_id, mtu)) < 0)
        {
            continue;
        }

        if (auto ack = ack_waiter->wait_for<wire::Ack>(wait_time); ack)
        {
            return ack;
        }
    }

    return std::nullopt;
}


///
/// @brief Helper to wait for data from the camera from a given query command. Once a query
///        command is sent to the MultiSense, it Ack's the command before sending the response
///
template <typename OutputMessage, typename QueryMessage, class Rep, class Period>
std::optional<OutputMessage> wait_for_data(MessageAssembler &assembler,
                                           const NetworkSocket &socket,
                                           const QueryMessage &query,
                                           uint16_t sequence_id,
                                           uint16_t mtu,
                                           const std::chrono::duration<Rep, Period>& wait_time,
                                           size_t attempts = 1)
{
    using namespace crl::multisense::details;

    auto ack_waiter = assembler.register_message(MSG_ID(QueryMessage::ID));
    auto response_waiter = assembler.register_message(MSG_ID(OutputMessage::ID));

    for (size_t i = 0 ; i < attempts ; ++i)
    {
        if(publish_data(socket, serialize(query, sequence_id, mtu)) < 0)
        {
            continue;
        }

        if (auto ack = ack_waiter->wait_for<wire::Ack>(wait_time); ack)
        {
            if (auto response = response_waiter->wait_for<OutputMessage>(wait_time); response)
            {
                return response.value();
            }
        }
    }

    return std::nullopt;
}

}
}
