/**
 * @file bindings.cc
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
 *   2025-01-19, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace py = pybind11;

PYBIND11_MODULE(libmultisense, m) {
    m.doc() = "Pybind11 bindings for the LibMultiSense C++ Library";

    // DataSource
    py::enum_<multisense::DataSource>(m, "DataSource")
        .value("UNKNOWN", multisense::DataSource::UNKNOWN)
        .value("ALL", multisense::DataSource::ALL)
        .value("LEFT_MONO_RAW", multisense::DataSource::LEFT_MONO_RAW)
        .value("RIGHT_MONO_RAW", multisense::DataSource::RIGHT_MONO_RAW)
        .value("LEFT_MONO_COMPRESSED", multisense::DataSource::LEFT_MONO_COMPRESSED)
        .value("RIGHT_MONO_COMPRESSED", multisense::DataSource::RIGHT_MONO_COMPRESSED)
        .value("LEFT_RECTIFIED_RAW", multisense::DataSource::LEFT_RECTIFIED_RAW)
        .value("RIGHT_RECTIFIED_RAW", multisense::DataSource::RIGHT_RECTIFIED_RAW)
        .value("LEFT_RECTIFIED_COMPRESSED", multisense::DataSource::LEFT_RECTIFIED_COMPRESSED)
        .value("RIGHT_RECTIFIED_COMPRESSED", multisense::DataSource::RIGHT_RECTIFIED_COMPRESSED)
        .value("LEFT_DISPARITY_RAW", multisense::DataSource::LEFT_DISPARITY_RAW)
        .value("LEFT_DISPARITY_COMPRESSED", multisense::DataSource::LEFT_DISPARITY_COMPRESSED)
        .value("AUX_COMPRESSED", multisense::DataSource::AUX_COMPRESSED)
        .value("AUX_RECTIFIED_COMPRESSED", multisense::DataSource::AUX_RECTIFIED_COMPRESSED)
        .value("AUX_LUMA_RAW", multisense::DataSource::AUX_LUMA_RAW)
        .value("AUX_LUMA_RECTIFIED_RAW", multisense::DataSource::AUX_LUMA_RECTIFIED_RAW)
        .value("AUX_CHROMA_RAW", multisense::DataSource::AUX_CHROMA_RAW)
        .value("AUX_CHROMA_RECTIFIED_RAW", multisense::DataSource::AUX_CHROMA_RECTIFIED_RAW)
        .value("COST_RAW", multisense::DataSource::COST_RAW);

    // CameraCalibration::DistortionType
    py::enum_<multisense::CameraCalibration::DistortionType>(m, "DistortionType")
        .value("NONE", multisense::CameraCalibration::DistortionType::NONE)
        .value("PLUMBOB", multisense::CameraCalibration::DistortionType::PLUMBOB)
        .value("RATIONAL_POLYNOMIAL", multisense::CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL);

    // CameraCalibration
    py::class_<multisense::CameraCalibration>(m, "CameraCalibration")
        .def(py::init<>())
        .def_readwrite("K", &multisense::CameraCalibration::K)
        .def_readwrite("R", &multisense::CameraCalibration::R)
        .def_readwrite("P", &multisense::CameraCalibration::P)
        .def_readwrite("distortion_type", &multisense::CameraCalibration::distortion_type)
        .def_readwrite("D", &multisense::CameraCalibration::D);

    // StereoCalibration
    py::class_<multisense::StereoCalibration>(m, "StereoCalibration")
        .def(py::init<>())
        .def_readwrite("left", &multisense::StereoCalibration::left)
        .def_readwrite("right", &multisense::StereoCalibration::right)
        .def_readwrite("aux", &multisense::StereoCalibration::aux);

    // DeviceInfo::PcbInfo
    py::class_<multisense::DeviceInfo::PcbInfo>(m, "PcbInfo")
        .def(py::init<>())
        .def_readwrite("name", &multisense::DeviceInfo::PcbInfo::name)
        .def_readwrite("revision", &multisense::DeviceInfo::PcbInfo::revision);

    // DeviceInfo::HardwareRevision
    py::enum_<multisense::DeviceInfo::HardwareRevision>(m, "HardwareRevision")
        .value("UNKNOWN", multisense::DeviceInfo::HardwareRevision::UNKNOWN)
        .value("S7", multisense::DeviceInfo::HardwareRevision::S7)
        .value("S21", multisense::DeviceInfo::HardwareRevision::S21)
        .value("ST21", multisense::DeviceInfo::HardwareRevision::ST21)
        .value("S27", multisense::DeviceInfo::HardwareRevision::S27)
        .value("S30", multisense::DeviceInfo::HardwareRevision::S30)
        .value("KS21", multisense::DeviceInfo::HardwareRevision::KS21)
        .value("MONOCAM", multisense::DeviceInfo::HardwareRevision::MONOCAM)
        .value("KS21_SILVER", multisense::DeviceInfo::HardwareRevision::KS21_SILVER)
        .value("ST25", multisense::DeviceInfo::HardwareRevision::ST25)
        .value("KS21i", multisense::DeviceInfo::HardwareRevision::KS21i);

    // DeviceInfo::ImagerType
    py::enum_<multisense::DeviceInfo::ImagerType>(m, "ImagerType")
        .value("UNKNOWN", multisense::DeviceInfo::ImagerType::UNKNOWN)
        .value("CMV2000_GREY", multisense::DeviceInfo::ImagerType::CMV2000_GREY)
        .value("CMV2000_COLOR", multisense::DeviceInfo::ImagerType::CMV2000_COLOR)
        .value("CMV4000_GREY", multisense::DeviceInfo::ImagerType::CMV4000_GREY)
        .value("CMV4000_COLOR", multisense::DeviceInfo::ImagerType::CMV4000_COLOR)
        .value("FLIR_TAU2", multisense::DeviceInfo::ImagerType::FLIR_TAU2)
        .value("AR0234_GREY", multisense::DeviceInfo::ImagerType::AR0234_GREY)
        .value("AR0239_COLOR", multisense::DeviceInfo::ImagerType::AR0239_COLOR);

    // DeviceInfo::LightingType
    py::enum_<multisense::DeviceInfo::LightingType>(m, "LightingType")
        .value("NONE", multisense::DeviceInfo::LightingType::NONE)
        .value("INTERNAL", multisense::DeviceInfo::LightingType::INTERNAL)
        .value("EXTERNAL", multisense::DeviceInfo::LightingType::EXTERNAL)
        .value("PATTERN_PROJECTOR", multisense::DeviceInfo::LightingType::PATTERN_PROJECTOR);

    // DeviceInfo::LensType
    py::enum_<multisense::DeviceInfo::LensType>(m, "LensType")
        .value("UNKNOWN", multisense::DeviceInfo::LensType::UNKNOWN);

    // DeviceInfo
    py::class_<multisense::DeviceInfo>(m, "DeviceInfo")
        .def(py::init<>())
        .def_readwrite("camera_name", &multisense::DeviceInfo::camera_name)
        .def_readwrite("build_date", &multisense::DeviceInfo::build_date)
        .def_readwrite("serial_number", &multisense::DeviceInfo::serial_number)
        .def_readwrite("hardware_revision", &multisense::DeviceInfo::hardware_revision)
        .def_readwrite("number_of_pcbs", &multisense::DeviceInfo::number_of_pcbs)
        .def_readwrite("pcb_info", &multisense::DeviceInfo::pcb_info)
        .def_readwrite("imager_name", &multisense::DeviceInfo::imager_name)
        .def_readwrite("imager_type", &multisense::DeviceInfo::imager_type)
        .def_readwrite("imager_width", &multisense::DeviceInfo::imager_width)
        .def_readwrite("imager_height", &multisense::DeviceInfo::imager_height)
        .def_readwrite("lens_name", &multisense::DeviceInfo::lens_name)
        .def_readwrite("lens_type", &multisense::DeviceInfo::lens_type)
        .def_readwrite("nominal_stereo_baseline", &multisense::DeviceInfo::nominal_stereo_baseline)
        .def_readwrite("nominal_focal_length", &multisense::DeviceInfo::nominal_focal_length)
        .def_readwrite("nominal_relative_aperature", &multisense::DeviceInfo::nominal_relative_aperature)
        .def_readwrite("lighting_type", &multisense::DeviceInfo::lighting_type)
        .def_readwrite("number_of_lights", &multisense::DeviceInfo::number_of_lights)
        .def("has_aux_camera", &multisense::DeviceInfo::has_aux_camera);

    // Image::PixelFormat
    py::enum_<multisense::Image::PixelFormat>(m, "PixelFormat")
        .value("UNKNOWN", multisense::Image::PixelFormat::UNKNOWN)
        .value("MONO8", multisense::Image::PixelFormat::MONO8)
        .value("RGB8", multisense::Image::PixelFormat::RGB8)
        .value("MONO16", multisense::Image::PixelFormat::MONO16);

    // Image
    py::class_<multisense::Image>(m, "Image")
        .def(py::init<>())
#if HAVE_OPENCV
        .def_property_readonly("image_data", [](const multisense::Image& image)
        {
            //
            // Inspired from https://github.com/carnegierobotics/simple_sfm/blob/28dbcadb6682e002c5206a172f70dd5640ff70b5/python/bindings.cpp#L87
            //
            std::vector<size_t> shape = {static_cast<size_t>(image.height), static_cast<size_t>(image.width)};
            std::vector<size_t> strides;
            size_t element_size = 0;
            std::string format;

            switch (image.format)
            {
                case multisense::Image::PixelFormat::MONO8:
                {
                    element_size = sizeof(uint8_t);
                    format = py::format_descriptor<uint8_t>::format();
                    strides = {sizeof(uint8_t) * image.width, sizeof(uint8_t)};
                    break;
                }
                case multisense::Image::PixelFormat::MONO16:
                {
                    element_size = sizeof(uint16_t);
                    format = py::format_descriptor<uint16_t>::format();
                    strides = {sizeof(uint16_t) * image.width, sizeof(uint16_t)};
                    break;
                }
                case multisense::Image::PixelFormat::RGB8:
                {
                    element_size = sizeof(uint8_t);
                    format = py::format_descriptor<uint8_t>::format();
                    shape.push_back(3);
                    strides = {sizeof(uint8_t) * image.width, sizeof(uint8_t), sizeof(uint8_t)};
                    break;
                }
                default: {throw std::runtime_error("Unknown pixel format");}
            }

            // Map the cv::Mat to a NumPy array without copying the data
            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(image.raw_data->data() + image.image_data_offset),
                             element_size,
                             format,
                             shape.size(),
                             shape,
                             strides));
        })
#endif
        .def_readonly("format", &multisense::Image::format)
        .def_readonly("width", &multisense::Image::width)
        .def_readonly("height", &multisense::Image::height)
        .def_readonly("camera_timestamp", &multisense::Image::camera_timestamp)
        .def_readonly("ptp_timestamp", &multisense::Image::ptp_timestamp)
        .def_readonly("source", &multisense::Image::source)
        .def_readonly("calibration", &multisense::Image::calibration);

    // ImageFrame
    py::class_<multisense::ImageFrame>(m, "ImageFrame")
        .def(py::init<>())
        .def("add_image", &multisense::ImageFrame::add_image)
        .def("get_image", &multisense::ImageFrame::get_image)
        .def("has_image", &multisense::ImageFrame::has_image)
        .def_readonly("frame_id", &multisense::ImageFrame::frame_id)
        .def_readonly("images", &multisense::ImageFrame::images)
        .def_readonly("calibration", &multisense::ImageFrame::calibration)
        .def_readonly("frame_time", &multisense::ImageFrame::frame_time)
        .def_readonly("ptp_frame_time", &multisense::ImageFrame::ptp_frame_time);

    // MultiSenseConfiguration::StereoCalibration
    py::class_<multisense::MultiSenseConfiguration::StereoConfiguration>(m, "StereoConfiguration")
        .def(py::init<>())
        .def_readwrite("postfilter_strength", &multisense::MultiSenseConfiguration::StereoConfiguration::postfilter_strength);

    // MultiSenseConfiguration::ManualExposureConfiguration
    py::class_<multisense::MultiSenseConfiguration::ManualExposureConfiguration>(m, "ManualExposureConfiguration")
        .def(py::init<>())
        .def_readwrite("gain", &multisense::MultiSenseConfiguration::ManualExposureConfiguration::gain)
        .def_readwrite("exposure_time", &multisense::MultiSenseConfiguration::ManualExposureConfiguration::exposure_time);

    // MultiSenseConfiguration::AutoExposureRoiConfiguration
    py::class_<multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration>(m, "AutoExposureRoiConfiguration")
        .def(py::init<>())
        .def_readwrite("top_left_x_position", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::top_left_x_position)
        .def_readwrite("top_left_y_position", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::top_left_y_position)
        .def_readwrite("width", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::width)
        .def_readwrite("height", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::height);

    // MultiSenseConfiguration::AutoExposureConfiguration
    py::class_<multisense::MultiSenseConfiguration::AutoExposureConfiguration>(m, "AutoExposureConfiguration")
        .def(py::init<>())
        .def_readwrite("max_exposure_time", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::max_exposure_time)
        .def_readwrite("decay", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::decay)
        .def_readwrite("target_intensity", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::target_intensity)
        .def_readwrite("target_threshold", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::target_threshold)
        .def_readwrite("max_gain", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::max_gain);

    // MultiSenseConfiguration::ManualWhiteBalanceConfiguration
    py::class_<multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration>(m, "ManualWhiteBalanceConfiguration")
        .def(py::init<>())
        .def_readwrite("red", &multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration::red)
        .def_readwrite("blue", &multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration::blue);

    // MultiSenseConfiguration::AutoWhiteBalanceConfiguration
    py::class_<multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration>(m, "AutoWhiteBalanceConfiguration")
        .def(py::init<>())
        .def_readwrite("decay", &multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration::decay)
        .def_readwrite("threshold", &multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration::threshold);

    // MultiSenseConfiguration::ImageConfiguration
    py::class_<multisense::MultiSenseConfiguration::ImageConfiguration>(m, "ImageConfiguration")
        .def(py::init<>())
        .def_readwrite("gamma", &multisense::MultiSenseConfiguration::ImageConfiguration::gamma)
        .def_readwrite("hdr_enabled", &multisense::MultiSenseConfiguration::ImageConfiguration::hdr_enabled)
        .def_readwrite("auto_exposure_enabled", &multisense::MultiSenseConfiguration::ImageConfiguration::auto_exposure_enabled)
        .def_readwrite("manual_exposure", &multisense::MultiSenseConfiguration::ImageConfiguration::manual_exposure)
        .def_readwrite("auto_exposure", &multisense::MultiSenseConfiguration::ImageConfiguration::auto_exposure)
        .def_readwrite("auto_white_balance_enabled", &multisense::MultiSenseConfiguration::ImageConfiguration::auto_white_balance_enabled)
        .def_readwrite("manual_white_balance", &multisense::MultiSenseConfiguration::ImageConfiguration::manual_white_balance)
        .def_readwrite("auto_white_balance", &multisense::MultiSenseConfiguration::ImageConfiguration::auto_white_balance);

    // MultiSenseConfiguration::AuxConfiguration
    py::class_<multisense::MultiSenseConfiguration::AuxConfiguration>(m, "AuxConfiguration")
        .def(py::init<>())
        .def_readwrite("image_config", &multisense::MultiSenseConfiguration::AuxConfiguration::image_config)
        .def_readwrite("sharpening_enabled", &multisense::MultiSenseConfiguration::AuxConfiguration::sharpening_enabled)
        .def_readwrite("sharpening_percentage", &multisense::MultiSenseConfiguration::AuxConfiguration::sharpening_percentage)
        .def_readwrite("sharpening_limit", &multisense::MultiSenseConfiguration::AuxConfiguration::sharpening_limit);

    // MultiSenseConfiguration::MaxDisparities
    py::enum_<multisense::MultiSenseConfiguration::MaxDisparities>(m, "MaxDisparities")
        .value("D64", multisense::MultiSenseConfiguration::MaxDisparities::D64)
        .value("D128", multisense::MultiSenseConfiguration::MaxDisparities::D128)
        .value("D256", multisense::MultiSenseConfiguration::MaxDisparities::D256);

    // MultiSenseConfiguration::TimeConfig
    py::class_<multisense::MultiSenseConfiguration::TimeConfiguration>(m, "TimeConfiguration")
        .def(py::init<>())
        .def_readwrite("ptp_enabled", &multisense::MultiSenseConfiguration::TimeConfiguration::ptp_enabled);

    // MultiSenseConfiguration
    py::class_<multisense::MultiSenseConfiguration>(m, "MultiSenseConfiguration")
        .def(py::init<>())
        .def_readwrite("width", &multisense::MultiSenseConfiguration::width)
        .def_readwrite("height", &multisense::MultiSenseConfiguration::height)
        .def_readwrite("disparities", &multisense::MultiSenseConfiguration::disparities)
        .def_readwrite("frames_per_second", &multisense::MultiSenseConfiguration::frames_per_second)
        .def_readwrite("stereo_config", &multisense::MultiSenseConfiguration::stereo_config)
        .def_readwrite("image_config", &multisense::MultiSenseConfiguration::image_config)
        .def_readwrite("aux_config", &multisense::MultiSenseConfiguration::aux_config)
        .def_readwrite("time_config", &multisense::MultiSenseConfiguration::time_config);

    // ChannelImplementation
    py::enum_<multisense::Channel::ChannelImplementation>(m, "ChannelImplementation")
        .value("LEGACY", multisense::Channel::ChannelImplementation::LEGACY);

    // Channel::ReceiveBufferConfiguration
    py::class_<multisense::Channel::ReceiveBufferConfiguration>(m, "ReceiveBufferConfiguration")
        .def(py::init<>())
        .def_readwrite("num_small_buffers", &multisense::Channel::ReceiveBufferConfiguration::num_small_buffers)
        .def_readwrite("small_buffer_size", &multisense::Channel::ReceiveBufferConfiguration::small_buffer_size)
        .def_readwrite("num_large_buffers", &multisense::Channel::ReceiveBufferConfiguration::num_large_buffers)
        .def_readwrite("large_buffer_size", &multisense::Channel::ReceiveBufferConfiguration::large_buffer_size);

    // Channel::ChannelConfig
    py::class_<multisense::Channel::ChannelConfig>(m, "ChannelConfig")
        .def(py::init<>())
        .def_readwrite("ip_address", &multisense::Channel::ChannelConfig::ip_address)
        .def_readwrite("mtu", &multisense::Channel::ChannelConfig::mtu)
        .def_readwrite("receive_timeout", &multisense::Channel::ChannelConfig::receive_timeout)
        .def_readwrite("command_port", &multisense::Channel::ChannelConfig::command_port)
        .def_readwrite("interface", &multisense::Channel::ChannelConfig::interface)
        .def_readwrite("receive_buffer_configuration", &multisense::Channel::ChannelConfig::receive_buffer_configuration);

    // Channel
    py::class_<multisense::Channel, std::unique_ptr<multisense::Channel>>(m, "Channel")
        .def_static("create", &multisense::Channel::create)
        .def("start_streams", &multisense::Channel::start_streams)
        .def("stop_streams", &multisense::Channel::stop_streams)
        .def("add_image_frame_callback", &multisense::Channel::add_image_frame_callback)
        .def("connect", &multisense::Channel::connect)
        .def("disconnect", &multisense::Channel::disconnect)
        .def("get_next_image_frame", &multisense::Channel::get_next_image_frame)
        .def("get_configuration", &multisense::Channel::get_configuration)
        .def("set_configuration", &multisense::Channel::set_configuration)
        .def("get_calibration", &multisense::Channel::get_calibration)
        .def("set_calibration", &multisense::Channel::set_calibration)
        .def("get_device_info", &multisense::Channel::get_device_info)
        .def("set_device_info", &multisense::Channel::set_device_info);

    // Utilities
    m.def("write_image", [](const multisense::Image &image, const std::string &path)
                         {
                             return multisense::write_image(image, std::filesystem::path{path});
                         }
         );
}
