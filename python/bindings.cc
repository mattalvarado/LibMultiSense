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

#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#ifdef BUILD_JSON
#include <MultiSense/MultiSenseSerialization.hh>
#endif

#ifdef BUILD_JSON
#define PYBIND11_JSON_SUPPORT(Type)                                                     \
    .def(py::init([](const py::dict &d) {                                               \
        py::module json = py::module::import("json");                                   \
        py::object json_str = json.attr("dumps")(d);                                    \
        const nlohmann::json j = nlohmann::json::parse(json_str.cast<std::string>());   \
        return j.template get<Type>();                                                  \
    }))                                                                                 \
    .def_property_readonly("json", [](const Type &obj) {                                \
        const nlohmann::json j = obj;                                                   \
        py::module json = py::module::import("json");                                   \
        py::object result = json.attr("loads")(j.dump());                               \
        return result.cast<py::dict>();                                                 \
    })                                                                                  \
    .def("__repr__", [](const Type &obj) {                                              \
        const nlohmann::json j = obj;                                                   \
        return j.dump();                                                                \
    })
#else
#define PYBIND11_JSON_SUPPORT(Type)
#endif

namespace py = pybind11;

PYBIND11_MODULE(libmultisense, m) {
    m.doc() = "Pybind11 bindings for the LibMultiSense C++ Library";

    // Status
    py::enum_<multisense::Status>(m, "Status")
        .value("OK", multisense::Status::OK)
        .value("TIMEOUT", multisense::Status::TIMEOUT)
        .value("ERROR", multisense::Status::ERROR)
        .value("FAILED", multisense::Status::FAILED)
        .value("UNSUPPORTED", multisense::Status::UNSUPPORTED)
        .value("UNKNOWN", multisense::Status::UNKNOWN)
        .value("EXCEPTION", multisense::Status::EXCEPTION)
        .value("UNINITIALIZED", multisense::Status::UNINITIALIZED);

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
        PYBIND11_JSON_SUPPORT(multisense::CameraCalibration)
        .def_readwrite("K", &multisense::CameraCalibration::K)
        .def_readwrite("R", &multisense::CameraCalibration::R)
        .def_readwrite("P", &multisense::CameraCalibration::P)
        .def_readwrite("distortion_type", &multisense::CameraCalibration::distortion_type)
        .def_readwrite("D", &multisense::CameraCalibration::D);

    // StereoCalibration
    py::class_<multisense::StereoCalibration>(m, "StereoCalibration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::StereoCalibration)
        .def_readwrite("left", &multisense::StereoCalibration::left)
        .def_readwrite("right", &multisense::StereoCalibration::right)
        .def_readwrite("aux", &multisense::StereoCalibration::aux);

    // Image::PixelFormat
    py::enum_<multisense::Image::PixelFormat>(m, "PixelFormat")
        .value("UNKNOWN", multisense::Image::PixelFormat::UNKNOWN)
        .value("MONO8", multisense::Image::PixelFormat::MONO8)
        .value("RGB8", multisense::Image::PixelFormat::RGB8)
        .value("MONO16", multisense::Image::PixelFormat::MONO16)
        .value("FLOAT32", multisense::Image::PixelFormat::FLOAT32);

    // Image
    py::class_<multisense::Image>(m, "Image")
        .def(py::init<>())
        .def_property_readonly("as_array", [](const multisense::Image& image)
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
                case multisense::Image::PixelFormat::FLOAT32:
                {
                    element_size = sizeof(float);
                    format = py::format_descriptor<float>::format();
                    strides = {sizeof(float) * image.width, sizeof(float)};
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
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::StereoConfiguration)
        .def_readwrite("postfilter_strength", &multisense::MultiSenseConfiguration::StereoConfiguration::postfilter_strength);

    // MultiSenseConfiguration::ManualExposureConfiguration
    py::class_<multisense::MultiSenseConfiguration::ManualExposureConfiguration>(m, "ManualExposureConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::ManualExposureConfiguration)
        .def_readwrite("gain", &multisense::MultiSenseConfiguration::ManualExposureConfiguration::gain)
        .def_readwrite("exposure_time", &multisense::MultiSenseConfiguration::ManualExposureConfiguration::exposure_time);

    // MultiSenseConfiguration::AutoExposureRoiConfiguration
    py::class_<multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration>(m, "AutoExposureRoiConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration)
        .def_readwrite("top_left_x_position", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::top_left_x_position)
        .def_readwrite("top_left_y_position", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::top_left_y_position)
        .def_readwrite("width", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::width)
        .def_readwrite("height", &multisense::MultiSenseConfiguration::AutoExposureRoiConfiguration::height);

    // MultiSenseConfiguration::AutoExposureConfiguration
    py::class_<multisense::MultiSenseConfiguration::AutoExposureConfiguration>(m, "AutoExposureConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::AutoExposureConfiguration)
        .def_readwrite("max_exposure_time", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::max_exposure_time)
        .def_readwrite("decay", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::decay)
        .def_readwrite("target_intensity", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::target_intensity)
        .def_readwrite("target_threshold", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::target_threshold)
        .def_readwrite("max_gain", &multisense::MultiSenseConfiguration::AutoExposureConfiguration::max_gain);

    // MultiSenseConfiguration::ManualWhiteBalanceConfiguration
    py::class_<multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration>(m, "ManualWhiteBalanceConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration)
        .def_readwrite("red", &multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration::red)
        .def_readwrite("blue", &multisense::MultiSenseConfiguration::ManualWhiteBalanceConfiguration::blue);

    // MultiSenseConfiguration::AutoWhiteBalanceConfiguration
    py::class_<multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration>(m, "AutoWhiteBalanceConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration)
        .def_readwrite("decay", &multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration::decay)
        .def_readwrite("threshold", &multisense::MultiSenseConfiguration::AutoWhiteBalanceConfiguration::threshold);

    // MultiSenseConfiguration::ImageConfiguration
    py::class_<multisense::MultiSenseConfiguration::ImageConfiguration>(m, "ImageConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::ImageConfiguration)
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
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::AuxConfiguration)
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
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::TimeConfiguration)
        .def_readwrite("ptp_enabled", &multisense::MultiSenseConfiguration::TimeConfiguration::ptp_enabled);

    // MultiSenseConfiguration::NetworkTransmissionConfiguration
    py::class_<multisense::MultiSenseConfiguration::NetworkTransmissionConfiguration>(m, "NetworkTransmissionConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::NetworkTransmissionConfiguration)
        .def_readwrite("transmit_delay", &multisense::MultiSenseConfiguration::NetworkTransmissionConfiguration::transmit_delay)
        .def_readwrite("packet_delay_enabled", &multisense::MultiSenseConfiguration::NetworkTransmissionConfiguration::packet_delay_enabled);

    // MultiSenseConfiguration::ImuConfiguration::OperatingMode
    py::class_<multisense::MultiSenseConfiguration::ImuConfiguration::OperatingMode>(m, "ImuOperatingMode")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::ImuConfiguration::OperatingMode)
        .def_readwrite("name", &multisense::MultiSenseConfiguration::ImuConfiguration::OperatingMode::name)
        .def_readwrite("enabled", &multisense::MultiSenseConfiguration::ImuConfiguration::OperatingMode::enabled)
        .def_readwrite("rate_index", &multisense::MultiSenseConfiguration::ImuConfiguration::OperatingMode::rate_index)
        .def_readwrite("range_index", &multisense::MultiSenseConfiguration::ImuConfiguration::OperatingMode::range_index);

    // MultiSenseConfiguration::ImuConfiguration
    py::class_<multisense::MultiSenseConfiguration::ImuConfiguration>(m, "ImuConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::ImuConfiguration)
        .def_readwrite("samples_per_frame", &multisense::MultiSenseConfiguration::ImuConfiguration::samples_per_frame)
        .def_readwrite("modes", &multisense::MultiSenseConfiguration::ImuConfiguration::modes);

    // MultiSenseConfiguration::LightingConfiguration::FlashMode
    py::enum_<multisense::MultiSenseConfiguration::LightingConfiguration::FlashMode>(m, "FlashMode")
        .value("NONE", multisense::MultiSenseConfiguration::LightingConfiguration::FlashMode::NONE)
        .value("SYNC_WITH_MAIN_STEREO", multisense::MultiSenseConfiguration::LightingConfiguration::FlashMode::SYNC_WITH_MAIN_STEREO)
        .value("SYNC_WITH_AUX", multisense::MultiSenseConfiguration::LightingConfiguration::FlashMode::SYNC_WITH_AUX);

    // MultiSenseConfiguration::LightingConfiguration
    py::class_<multisense::MultiSenseConfiguration::LightingConfiguration>(m, "LightingConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration::LightingConfiguration)
        .def_readwrite("intensity", &multisense::MultiSenseConfiguration::LightingConfiguration::intensity)
        .def_readwrite("flash", &multisense::MultiSenseConfiguration::LightingConfiguration::flash);

    // MultiSenseConfiguration
    py::class_<multisense::MultiSenseConfiguration>(m, "MultiSenseConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseConfiguration)
        .def_readwrite("width", &multisense::MultiSenseConfiguration::width)
        .def_readwrite("height", &multisense::MultiSenseConfiguration::height)
        .def_readwrite("disparities", &multisense::MultiSenseConfiguration::disparities)
        .def_readwrite("frames_per_second", &multisense::MultiSenseConfiguration::frames_per_second)
        .def_readwrite("stereo_config", &multisense::MultiSenseConfiguration::stereo_config)
        .def_readwrite("image_config", &multisense::MultiSenseConfiguration::image_config)
        .def_readwrite("aux_config", &multisense::MultiSenseConfiguration::aux_config)
        .def_readwrite("time_config", &multisense::MultiSenseConfiguration::time_config)
        .def_readwrite("network_config", &multisense::MultiSenseConfiguration::network_config)
        .def_readwrite("imu_config", &multisense::MultiSenseConfiguration::imu_config)
        .def_readwrite("lighting_config", &multisense::MultiSenseConfiguration::lighting_config);

    // MultiSenseStatus::PtpStatus
    py::class_<multisense::MultiSenseStatus::PtpStatus>(m, "PtpStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::PtpStatus)
        .def_readwrite("grandmaster_present", &multisense::MultiSenseStatus::PtpStatus::grandmaster_present)
        .def_readwrite("grandmaster_id", &multisense::MultiSenseStatus::PtpStatus::grandmaster_id)
        .def_readwrite("grandmaster_offset", &multisense::MultiSenseStatus::PtpStatus::grandmaster_offset)
        .def_readwrite("path_delay", &multisense::MultiSenseStatus::PtpStatus::path_delay)
        .def_readwrite("steps_from_local_to_grandmaster", &multisense::MultiSenseStatus::PtpStatus::steps_from_local_to_grandmaster);

    // MultiSenseStatus::CameraStatus
    py::class_<multisense::MultiSenseStatus::CameraStatus>(m, "CameraStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::CameraStatus)
        .def_readwrite("cameras_ok", &multisense::MultiSenseStatus::CameraStatus::cameras_ok)
        .def_readwrite("processing_pipeline_ok", &multisense::MultiSenseStatus::CameraStatus::processing_pipeline_ok);

    // MultiSenseStatus::TemperatureStatus
    py::class_<multisense::MultiSenseStatus::TemperatureStatus>(m, "TemperatureStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::TemperatureStatus)
        .def_readwrite("fpga_temperature_C", &multisense::MultiSenseStatus::TemperatureStatus::fpga_temperature_C)
        .def_readwrite("left_imager_temperature_C", &multisense::MultiSenseStatus::TemperatureStatus::left_imager_temperature_C)
        .def_readwrite("right_imager_temperature_C", &multisense::MultiSenseStatus::TemperatureStatus::right_imager_temperature_C)
        .def_readwrite("power_supply_temperature_C", &multisense::MultiSenseStatus::TemperatureStatus::power_supply_temperature_C);

    // MultiSenseStatus::PowerStatus
    py::class_<multisense::MultiSenseStatus::PowerStatus>(m, "PowerStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::PowerStatus)
        .def_readwrite("input_voltage", &multisense::MultiSenseStatus::PowerStatus::input_voltage)
        .def_readwrite("input_current", &multisense::MultiSenseStatus::PowerStatus::input_current)
        .def_readwrite("fpga_power", &multisense::MultiSenseStatus::PowerStatus::fpga_power);

    // MultiSenseStatus::ClientNetworkStatus
    py::class_<multisense::MultiSenseStatus::ClientNetworkStatus>(m, "ClientNetworkStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::ClientNetworkStatus)
        .def_readwrite("received_messages", &multisense::MultiSenseStatus::ClientNetworkStatus::received_messages)
        .def_readwrite("dropped_messages", &multisense::MultiSenseStatus::ClientNetworkStatus::dropped_messages)
        .def_readwrite("invalid_packets", &multisense::MultiSenseStatus::ClientNetworkStatus::invalid_packets);

    // MultiSenseStatus::TimeStatus
    py::class_<multisense::MultiSenseStatus::TimeStatus>(m, "TimeStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus::TimeStatus)
        .def_readwrite("camera_time", &multisense::MultiSenseStatus::TimeStatus::camera_time)
        .def_readwrite("client_host_time", &multisense::MultiSenseStatus::TimeStatus::client_host_time)
        .def_readwrite("network_delay", &multisense::MultiSenseStatus::TimeStatus::network_delay);

    // MultiSenseStatus
    py::class_<multisense::MultiSenseStatus>(m, "MultiSenseStatus")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseStatus)
        .def_readwrite("system_ok", &multisense::MultiSenseStatus::system_ok)
        .def_readwrite("ptp", &multisense::MultiSenseStatus::ptp)
        .def_readwrite("camera", &multisense::MultiSenseStatus::camera)
        .def_readwrite("temperature", &multisense::MultiSenseStatus::temperature)
        .def_readwrite("power", &multisense::MultiSenseStatus::power)
        .def_readwrite("client_network", &multisense::MultiSenseStatus::client_network)
        .def_readwrite("time", &multisense::MultiSenseStatus::time);

    // MultiSenseInfo::DeviceInfo::PcbInfo
    py::class_<multisense::MultiSenseInfo::DeviceInfo::PcbInfo>(m, "PcbInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::DeviceInfo::PcbInfo)
        .def_readwrite("name", &multisense::MultiSenseInfo::DeviceInfo::PcbInfo::name)
        .def_readwrite("revision", &multisense::MultiSenseInfo::DeviceInfo::PcbInfo::revision);

    // MultiSenseInfo::DeviceInfo::HardwareRevision
    py::enum_<multisense::MultiSenseInfo::DeviceInfo::HardwareRevision>(m, "HardwareRevision")
        .value("UNKNOWN", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::UNKNOWN)
        .value("S7", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S7)
        .value("S21", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S21)
        .value("ST21", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::ST21)
        .value("S27", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S27)
        .value("S30", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::S30)
        .value("KS21", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21)
        .value("MONOCAM", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::MONOCAM)
        .value("KS21_SILVER", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21_SILVER)
        .value("ST25", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::ST25)
        .value("KS21i", multisense::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21i);

    // MultiSenseInfo::DeviceInfo::ImagerType
    py::enum_<multisense::MultiSenseInfo::DeviceInfo::ImagerType>(m, "ImagerType")
        .value("UNKNOWN", multisense::MultiSenseInfo::DeviceInfo::ImagerType::UNKNOWN)
        .value("CMV2000_GREY", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_GREY)
        .value("CMV2000_COLOR", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV2000_COLOR)
        .value("CMV4000_GREY", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_GREY)
        .value("CMV4000_COLOR", multisense::MultiSenseInfo::DeviceInfo::ImagerType::CMV4000_COLOR)
        .value("FLIR_TAU2", multisense::MultiSenseInfo::DeviceInfo::ImagerType::FLIR_TAU2)
        .value("AR0234_GREY", multisense::MultiSenseInfo::DeviceInfo::ImagerType::AR0234_GREY)
        .value("AR0239_COLOR", multisense::MultiSenseInfo::DeviceInfo::ImagerType::AR0239_COLOR);

    // MultiSenseInfo::DeviceInfo::LightingType
    py::enum_<multisense::MultiSenseInfo::DeviceInfo::LightingType>(m, "LightingType")
        .value("NONE", multisense::MultiSenseInfo::DeviceInfo::LightingType::NONE)
        .value("INTERNAL", multisense::MultiSenseInfo::DeviceInfo::LightingType::INTERNAL)
        .value("EXTERNAL", multisense::MultiSenseInfo::DeviceInfo::LightingType::EXTERNAL)
        .value("PATTERN_PROJECTOR", multisense::MultiSenseInfo::DeviceInfo::LightingType::PATTERN_PROJECTOR);

    // MultiSenseInfo::DeviceInfo::LensType
    py::enum_<multisense::MultiSenseInfo::MultiSenseInfo::DeviceInfo::LensType>(m, "LensType")
        .value("UNKNOWN", multisense::MultiSenseInfo::DeviceInfo::LensType::UNKNOWN);

    // MultiSenseInfo::NetworkInfo
    py::class_<multisense::MultiSenseInfo::MultiSenseInfo::NetworkInfo>(m, "NetworkInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::NetworkInfo)
        .def_readwrite("ipv4_address", &multisense::MultiSenseInfo::NetworkInfo::ipv4_address)
        .def_readwrite("ipv4_gateway", &multisense::MultiSenseInfo::NetworkInfo::ipv4_gateway)
        .def_readwrite("ipv4_netmask", &multisense::MultiSenseInfo::NetworkInfo::ipv4_netmask);


    // MultiSenseInfo::DeviceInfo
    py::class_<multisense::MultiSenseInfo::DeviceInfo>(m, "DeviceInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::DeviceInfo)
        .def_readwrite("camera_name", &multisense::MultiSenseInfo::DeviceInfo::camera_name)
        .def_readwrite("build_date", &multisense::MultiSenseInfo::DeviceInfo::build_date)
        .def_readwrite("serial_number", &multisense::MultiSenseInfo::DeviceInfo::serial_number)
        .def_readwrite("hardware_revision", &multisense::MultiSenseInfo::DeviceInfo::hardware_revision)
        .def_readwrite("pcb_info", &multisense::MultiSenseInfo::DeviceInfo::pcb_info)
        .def_readwrite("imager_name", &multisense::MultiSenseInfo::DeviceInfo::imager_name)
        .def_readwrite("imager_type", &multisense::MultiSenseInfo::DeviceInfo::imager_type)
        .def_readwrite("imager_width", &multisense::MultiSenseInfo::DeviceInfo::imager_width)
        .def_readwrite("imager_height", &multisense::MultiSenseInfo::DeviceInfo::imager_height)
        .def_readwrite("lens_name", &multisense::MultiSenseInfo::DeviceInfo::lens_name)
        .def_readwrite("lens_type", &multisense::MultiSenseInfo::DeviceInfo::lens_type)
        .def_readwrite("nominal_stereo_baseline", &multisense::MultiSenseInfo::DeviceInfo::nominal_stereo_baseline)
        .def_readwrite("nominal_focal_length", &multisense::MultiSenseInfo::DeviceInfo::nominal_focal_length)
        .def_readwrite("nominal_relative_aperture", &multisense::MultiSenseInfo::DeviceInfo::nominal_relative_aperture)
        .def_readwrite("lighting_type", &multisense::MultiSenseInfo::DeviceInfo::lighting_type)
        .def_readwrite("number_of_lights", &multisense::MultiSenseInfo::DeviceInfo::number_of_lights)
        .def("has_aux_camera", &multisense::MultiSenseInfo::DeviceInfo::has_aux_camera);

    // MultiSenseInfo::Version
    py::class_<multisense::MultiSenseInfo::Version>(m, "Version")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::Version)
        .def("__lt__", &multisense::MultiSenseInfo::Version::operator<)
        .def_readwrite("major", &multisense::MultiSenseInfo::Version::major)
        .def_readwrite("minor", &multisense::MultiSenseInfo::Version::minor)
        .def_readwrite("patch", &multisense::MultiSenseInfo::Version::patch)
        .def("to_string", &multisense::MultiSenseInfo::Version::to_string);

    // MultiSenseInfo::SensorVersion
    py::class_<multisense::MultiSenseInfo::MultiSenseInfo::SensorVersion>(m, "SensorVersion")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::SensorVersion)
        .def_readwrite("firmware_build_date", &multisense::MultiSenseInfo::SensorVersion::firmware_build_date)
        .def_readwrite("firmware_version", &multisense::MultiSenseInfo::SensorVersion::firmware_version)
        .def_readwrite("hardware_version", &multisense::MultiSenseInfo::SensorVersion::hardware_version)
        .def_readwrite("hardware_magic", &multisense::MultiSenseInfo::SensorVersion::hardware_magic)
        .def_readwrite("fpga_dna", &multisense::MultiSenseInfo::SensorVersion::fpga_dna);

    // MultiSenseInfo::SupportedOperatingMode
    py::class_<multisense::MultiSenseInfo::SupportedOperatingMode>(m, "SupportedOperatingMode")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::SupportedOperatingMode)
        .def_readwrite("width", &multisense::MultiSenseInfo::SupportedOperatingMode::width)
        .def_readwrite("height", &multisense::MultiSenseInfo::SupportedOperatingMode::height)
        .def_readwrite("disparities", &multisense::MultiSenseInfo::SupportedOperatingMode::disparities)
        .def_readwrite("supported_sources", &multisense::MultiSenseInfo::SupportedOperatingMode::supported_sources);

    // MultiSenseInfo::ImuSource::Rate
    py::class_<multisense::MultiSenseInfo::ImuSource::Rate>(m, "ImuRate")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::ImuSource::Rate)
        .def_readwrite("sample_rate", &multisense::MultiSenseInfo::ImuSource::Rate::sample_rate)
        .def_readwrite("bandwith_cutoff", &multisense::MultiSenseInfo::ImuSource::Rate::bandwith_cutoff);

    // MultiSenseInfo::ImuSource::Range
    py::class_<multisense::MultiSenseInfo::ImuSource::Range>(m, "ImuRange")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::ImuSource::Range)
        .def_readwrite("range", &multisense::MultiSenseInfo::ImuSource::Range::range)
        .def_readwrite("resolution", &multisense::MultiSenseInfo::ImuSource::Range::resolution);

    // MultiSenseInfo::ImuSource
    py::class_<multisense::MultiSenseInfo::ImuSource>(m, "ImuSource")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo::ImuSource)
        .def_readwrite("name", &multisense::MultiSenseInfo::ImuSource::name)
        .def_readwrite("device", &multisense::MultiSenseInfo::ImuSource::device)
        .def_readwrite("units", &multisense::MultiSenseInfo::ImuSource::units)
        .def_readwrite("rates", &multisense::MultiSenseInfo::ImuSource::rates)
        .def_readwrite("ranges", &multisense::MultiSenseInfo::ImuSource::ranges);

    // MultiSenseInfo
    py::class_<multisense::MultiSenseInfo>(m, "MultiSenseInfo")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::MultiSenseInfo)
        .def_readwrite("device", &multisense::MultiSenseInfo::device)
        .def_readwrite("version", &multisense::MultiSenseInfo::version)
        .def_readwrite("operating_modes", &multisense::MultiSenseInfo::operating_modes)
        .def_readwrite("imu", &multisense::MultiSenseInfo::imu)
        .def_readwrite("network", &multisense::MultiSenseInfo::network);

    // ChannelImplementation
    py::enum_<multisense::Channel::ChannelImplementation>(m, "ChannelImplementation")
        .value("LEGACY", multisense::Channel::ChannelImplementation::LEGACY);

    // Channel::ReceiveBufferConfiguration
    py::class_<multisense::Channel::ReceiveBufferConfiguration>(m, "ReceiveBufferConfiguration")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::Channel::ReceiveBufferConfiguration)
        .def_readwrite("num_small_buffers", &multisense::Channel::ReceiveBufferConfiguration::num_small_buffers)
        .def_readwrite("small_buffer_size", &multisense::Channel::ReceiveBufferConfiguration::small_buffer_size)
        .def_readwrite("num_large_buffers", &multisense::Channel::ReceiveBufferConfiguration::num_large_buffers)
        .def_readwrite("large_buffer_size", &multisense::Channel::ReceiveBufferConfiguration::large_buffer_size);

    // Channel::ChannelConfig
    py::class_<multisense::Channel::ChannelConfig>(m, "ChannelConfig")
        .def(py::init<>())
        PYBIND11_JSON_SUPPORT(multisense::Channel::ChannelConfig)
        .def_readwrite("ip_address", &multisense::Channel::ChannelConfig::ip_address)
        .def_readwrite("mtu", &multisense::Channel::ChannelConfig::mtu)
        .def_readwrite("receive_timeout", &multisense::Channel::ChannelConfig::receive_timeout)
        .def_readwrite("command_port", &multisense::Channel::ChannelConfig::command_port)
        .def_readwrite("interface", &multisense::Channel::ChannelConfig::interface)
        .def_readwrite("receive_buffer_configuration", &multisense::Channel::ChannelConfig::receive_buffer_configuration);

    // Channel
    py::class_<multisense::Channel, std::unique_ptr<multisense::Channel>>(m, "Channel")
        .def_static("create", &multisense::Channel::create,
                py::arg("config"),
                py::arg("impl") = multisense::Channel::ChannelImplementation::LEGACY)
        .def("start_streams", &multisense::Channel::start_streams, py::call_guard<py::gil_scoped_release>())
        .def("stop_streams", &multisense::Channel::stop_streams, py::call_guard<py::gil_scoped_release>())
        .def("add_image_frame_callback", &multisense::Channel::add_image_frame_callback, py::call_guard<py::gil_scoped_acquire>())
        .def("add_imu_frame_callback", &multisense::Channel::add_imu_frame_callback, py::call_guard<py::gil_scoped_acquire>())
        .def("connect", &multisense::Channel::connect)
        .def("disconnect", &multisense::Channel::disconnect)
        .def("get_next_image_frame", &multisense::Channel::get_next_image_frame, py::call_guard<py::gil_scoped_release>())
        .def("get_configuration", &multisense::Channel::get_configuration, py::call_guard<py::gil_scoped_release>())
        .def("set_configuration", &multisense::Channel::set_configuration, py::call_guard<py::gil_scoped_release>())
        .def("get_calibration", &multisense::Channel::get_calibration, py::call_guard<py::gil_scoped_release>())
        .def("set_calibration", &multisense::Channel::set_calibration, py::call_guard<py::gil_scoped_release>())
        .def("get_info", &multisense::Channel::get_info, py::call_guard<py::gil_scoped_release>())
        .def("set_device_info", &multisense::Channel::set_device_info, py::call_guard<py::gil_scoped_release>())
        .def("get_system_status", &multisense::Channel::get_system_status, py::call_guard<py::gil_scoped_release>());

    // Utilities
    py::class_<multisense::Point<void>>(m, "Point")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<void>::x)
        .def_readwrite("y", &multisense::Point<void>::y)
        .def_readwrite("z", &multisense::Point<void>::z);

    py::class_<multisense::Point<uint8_t>>(m, "PointLuma8")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<uint8_t>::x)
        .def_readwrite("y", &multisense::Point<uint8_t>::y)
        .def_readwrite("z", &multisense::Point<uint8_t>::z)
        .def_readwrite("color", &multisense::Point<uint8_t>::color);

    py::class_<multisense::Point<uint16_t>>(m, "PointLuma16")
        .def(py::init<>())
        .def_readwrite("x", &multisense::Point<uint16_t>::x)
        .def_readwrite("y", &multisense::Point<uint16_t>::y)
        .def_readwrite("z", &multisense::Point<uint16_t>::z)
        .def_readwrite("color", &multisense::Point<uint16_t>::color);

    py::class_<multisense::PointCloud<void>>(m, "PointCloud")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<void>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<void> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            const std::vector<size_t> strides = {sizeof(multisense::Point<void>), sizeof(float)};
            const size_t element_size = sizeof(float);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             shape.size(),
                             shape,
                             strides));
        });
    py::class_<multisense::PointCloud<uint8_t>>(m, "PointCloudLuma8")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<uint8_t>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<uint8_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            //
            // Make sure we skip over the color
            //
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint8_t>), sizeof(float)};
            const size_t element_size = sizeof(multisense::Point<uint8_t>);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             2,
                             shape,
                             strides));
        })
        .def_property_readonly("as_raw_array", [](const multisense::PointCloud<uint8_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size())};
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint8_t>)};
            const size_t element_size = sizeof(multisense::Point<uint8_t>);

            return py::array(py::buffer_info(
                             const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cloud.cloud.data())),
                             element_size,
                             "13B",
                             1,
                             shape,
                             strides));
        });


    py::class_<multisense::PointCloud<uint16_t>>(m, "PointCloudLuma16")
        .def(py::init<>())
        .def_readwrite("cloud", &multisense::PointCloud<uint16_t>::cloud)
        .def_property_readonly("as_array", [](const multisense::PointCloud<uint16_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size()), 3};
            //
            // Make sure we skip over the color
            //
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint16_t>), sizeof(float)};
            const size_t element_size = sizeof(multisense::Point<uint16_t>);
            const std::string format = py::format_descriptor<float>::format();;

            return py::array(py::buffer_info(
                             const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(cloud.cloud.data())),
                             element_size,
                             format,
                             2,
                             shape,
                             strides));
        })
        .def_property_readonly("as_raw_array", [](const multisense::PointCloud<uint16_t> &cloud)
        {
            const std::vector<size_t> shape = {static_cast<size_t>(cloud.cloud.size())};
            const std::vector<size_t> strides = {sizeof(multisense::Point<uint16_t>)};
            const size_t element_size = sizeof(multisense::Point<uint16_t>);

            return py::array(py::buffer_info(
                             const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(cloud.cloud.data())),
                             element_size,
                             "14B",
                             1,
                             shape,
                             strides));
        });

    m.def("write_image",
          [](const multisense::Image &image, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_image(image, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<void> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<uint8_t> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("write_pointcloud_ply",
          [](const multisense::PointCloud<uint16_t> &pointcloud, const std::string &path)
          {
              py::gil_scoped_release release;
              return multisense::write_pointcloud_ply(pointcloud, std::filesystem::path{path});
          }
    );

    m.def("create_pointcloud", &multisense::create_pointcloud, py::call_guard<py::gil_scoped_release>());

    m.def("create_gray8_pointcloud", &multisense::create_color_pointcloud<uint8_t>, py::call_guard<py::gil_scoped_release>());

    m.def("create_gray16_pointcloud", &multisense::create_color_pointcloud<uint16_t>, py::call_guard<py::gil_scoped_release>());

    m.def("create_depth_image", &multisense::create_depth_image, py::call_guard<py::gil_scoped_release>());
}
