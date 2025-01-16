/**
 * @file utilities.cc
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

#include "details/legacy/utilities.hh"

namespace multisense{
namespace legacy{

std::vector<DataSource> convert_sources(const crl::multisense::details::wire::SourceType &source)
{
    using namespace crl::multisense::details;

    std::vector<DataSource> sources;
    if (source & wire::SOURCE_LUMA_LEFT) {sources.push_back(DataSource::LEFT_MONO_RAW);}
    if (source & wire::SOURCE_LUMA_RIGHT) {sources.push_back(DataSource::RIGHT_MONO_RAW);}
    if (source & wire::SOURCE_COMPRESSED_LEFT) {sources.push_back(DataSource::LEFT_MONO_COMPRESSED);}
    if (source & wire::SOURCE_COMPRESSED_RIGHT) {sources.push_back(DataSource::RIGHT_MONO_COMPRESSED);}
    if (source & wire::SOURCE_LUMA_RECT_LEFT) {sources.push_back(DataSource::LEFT_RECTIFIED_RAW);}
    if (source & wire::SOURCE_LUMA_RECT_RIGHT) {sources.push_back(DataSource::RIGHT_RECTIFIED_RAW);}
    if (source & wire::SOURCE_COMPRESSED_RECTIFIED_LEFT) {sources.push_back(DataSource::LEFT_RECTIFIED_COMPRESSED);}
    if (source & wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT) {sources.push_back(DataSource::RIGHT_RECTIFIED_COMPRESSED);}
    if (source & wire::SOURCE_DISPARITY) {sources.push_back(DataSource::LEFT_DISPARITY_RAW);}
    if (source & wire::SOURCE_COMPRESSED_AUX) {sources.push_back(DataSource::AUX_COMPRESSED);}
    if (source & wire::SOURCE_LUMA_RECT_AUX) {sources.push_back(DataSource::AUX_LUMA_RECTIFIED_RAW);}
    if (source & wire::SOURCE_CHROMA_AUX) {sources.push_back(DataSource::AUX_CHROMA_RAW);}
    if (source & wire::SOURCE_CHROMA_RECT_AUX) {sources.push_back(DataSource::AUX_CHROMA_RECTIFIED_RAW);}
    if (source & wire::SOURCE_DISPARITY_COST) {sources.push_back(DataSource::COST_RAW);}

    return sources;
}


crl::multisense::details::wire::SourceType convert_sources(const std::vector<DataSource> &sources)
{
    using namespace crl::multisense::details;

    wire::SourceType mask = 0;
    for (const auto &source : sources)
    {
        switch(source)
        {
            case DataSource::LEFT_MONO_RAW: {mask |= wire::SOURCE_LUMA_LEFT; break;}
            case DataSource::RIGHT_MONO_RAW: {mask |= wire::SOURCE_LUMA_RIGHT; break;}
            case DataSource::LEFT_MONO_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_LEFT; break;}
            case DataSource::RIGHT_MONO_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RIGHT; break;}
            case DataSource::LEFT_RECTIFIED_RAW: {mask |= wire::SOURCE_LUMA_RECT_LEFT; break;}
            case DataSource::RIGHT_RECTIFIED_RAW: {mask |= wire::SOURCE_LUMA_RECT_RIGHT; break;}
            case DataSource::LEFT_RECTIFIED_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RECTIFIED_LEFT; break;}
            case DataSource::RIGHT_RECTIFIED_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_RECTIFIED_RIGHT; break;}
            case DataSource::LEFT_DISPARITY_RAW: {mask |= wire::SOURCE_DISPARITY; break;}
            case DataSource::LEFT_DISPARITY_COMPRESSED: { CRL_DEBUG("Compressed disparity not supported"); break;}
            case DataSource::AUX_COMPRESSED: {mask |= wire::SOURCE_COMPRESSED_AUX; break;}
            case DataSource::AUX_LUMA_RECTIFIED_RAW: {mask |= wire::SOURCE_LUMA_RECT_AUX; break;}
            case DataSource::AUX_CHROMA_RAW: {mask |= wire::SOURCE_CHROMA_AUX; break;}
            case DataSource::AUX_CHROMA_RECTIFIED_RAW: {mask |= wire::SOURCE_CHROMA_RECT_AUX; break;}
            case DataSource::COST_RAW: {mask |= wire::SOURCE_DISPARITY_COST; break;}
            case DataSource::ALL: {mask |= all_sources; break;}
            default: {CRL_DEBUG("Unsupported source %d", static_cast<int32_t>(source));}
        }
    }

    return mask;
}

}
}