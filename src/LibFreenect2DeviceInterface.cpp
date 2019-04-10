/*
This file is part of the OpenIMPRESS project.

OpenIMPRESS is free software: you can redistribute it and/or modify
it under the terms of the Lesser GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

OpenIMPRESS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with OpenIMPRESS. If not, see <https://www.gnu.org/licenses/>.
*/

#include "LibFreenect2DeviceInterface.hpp"

using namespace oi::client::libfreenect2;
using namespace oi::core;

bool freenect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s) {
    freenect_shutdown = true;
}

bool freenect_paused = false;
libfreenect2::Freenect2Device *devtopause;
void sigusr1_handler(int s) {
    if (devtopause == 0)
        return;
    if (freenect_paused)
        devtopause->start();
    else
        devtopause->stop();
    freenect_paused = !freenect_paused;
}


LibFreenect2DeviceInterface::LibFreenect2DeviceInterface(std::string serial, std::string pipeline_name, float minDepth, float maxDepth) {
    this->serial = serial;
    this->pipeline_name = pipeline_name;
    this->maxDepth = maxDepth;
    this->minDepth = minDepth;
    freenect2 = new ::libfreenect2::Freenect2();
    ::libfreenect2::setGlobalLogger(::libfreenect2::createConsoleLogger(::libfreenect2::Logger::Warning));
    OpenDevice();
}

int LibFreenect2DeviceInterface::Cycle(oi::core::rgbd::RGBDDevice * streamer) {
    if (!listener->waitForNewFrame(*frames, 5 * 1000)) {
        std::cerr << "\nERROR: Libfreenect2 did not provide a new frame." << std::endl;
        return -1;
    }
    std::chrono::milliseconds timestamp = NOW();
    ::libfreenect2::Frame *rgb = (*frames)[::libfreenect2::Frame::Color];
    ::libfreenect2::Frame *depth = (*frames)[::libfreenect2::Frame::Depth];
    registration->apply(rgb, depth, undistorted, registered);
    int res = streamer->QueueRGBDFrame(rgb->sequence, registered->data, undistorted->data, timestamp);
    
    listener->release(*frames);
    return res;
}


int LibFreenect2DeviceInterface::OpenDevice() {
    ::libfreenect2::PacketPipeline *pipeline = 0;
    if (freenect2->enumerateDevices() == 0) {
        std::cerr << "ERROR: No device connected." << std::endl;
        return -1;
    }
    
    if (pipeline_name.compare("cpu") == 0) {
        pipeline = new ::libfreenect2::CpuPacketPipeline();
    } else if (pipeline_name.compare("opengl") == 0) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        pipeline = new ::libfreenect2::OpenGLPacketPipeline();
#else
        std::cerr << "OpenGL pipeline is not supported!" << std::endl;
        return  -1;
#endif
    } else if (pipeline_name.compare("cuda") == 0) {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        pipeline = new ::libfreenect2::CudaPacketPipeline(-1);
#else
        std::cerr << "CUDA pipeline is not supported!" << std::endl;
        return  -1;
#endif
    } else if (pipeline_name.compare("opencl") == 0) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        pipeline = new ::libfreenect2::OpenCLPacketPipeline(-1);
#else
        std::cerr << "OpenCL pipeline is not supported!" << std::endl;
        return  -1;
#endif
    } else {
        std::cerr << "Unknown pipeline: " << pipeline_name << std::endl;
        return  -1;
    }
    
    if (serial == "") serial = freenect2->getDefaultDeviceSerialNumber();
    
    // Listeners
    listener = new ::libfreenect2::SyncMultiFrameListener(::libfreenect2::Frame::Color | ::libfreenect2::Frame::Depth);
    frames = new ::libfreenect2::FrameMap();
    // Open Device & Register listenres
    dev = freenect2->openDevice(serial, pipeline);
    if (!dev->start()) {
        std::cerr << "ERROR: Failed to start device." << std::endl;
        return -1;
    }
    
    i_d = dev->getIrCameraParams();
    i_c = dev->getColorCameraParams();
    ::libfreenect2::Freenect2Device::Config config;
    config.MinDepth = minDepth;    ///< Clip at this minimum distance (meter).
    config.MaxDepth = maxDepth;    ///< Clip at this maximum distance (meter).
    config.EnableBilateralFilter = true;        ///< Remove some "flying pixels".
    config.EnableEdgeAwareFilter = true;        ///< Remove pixels on edges because ToF cameras produce noisy edges.
    dev->setConfiguration(config);
    
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    
    devtopause = dev;
    signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
    signal(SIGUSR1, sigusr1_handler);
#endif
    
    // Setup Registration
    registration = new ::libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    undistorted = new ::libfreenect2::Frame(this->frame_width(), this->frame_height(), this->raw_depth_stride());
    registered = new ::libfreenect2::Frame(this->frame_width(), this->frame_height(), this->raw_depth_stride());
    
    printf("Freenect2 Device started.\nSerial:\t%s\nFirmware:\t%s\n", dev->getSerialNumber().c_str(), dev->getFirmwareVersion().c_str());
    return 1;
}

int LibFreenect2DeviceInterface::CloseDevice() {
    dev->close();
    delete listener;
    delete registration;
    delete registered;
    delete undistorted;
    delete frames;
    return 1;
}

int LibFreenect2DeviceInterface::frame_width() {
    return 512;
}

int LibFreenect2DeviceInterface::frame_height() {
    return 424;
}

int LibFreenect2DeviceInterface::send_depth_stride() {
    return 2;
}

int LibFreenect2DeviceInterface::raw_depth_stride() {
    return 4;
}

int LibFreenect2DeviceInterface::raw_color_stride() {
    return 4;
}

float LibFreenect2DeviceInterface::device_cx() {
    if (this->dev == 0) return 0.0f;
    return i_d.cx;
}

float LibFreenect2DeviceInterface::device_cy() {
    if (this->dev == 0) return 0.0f;
    return i_d.cy;
}

float LibFreenect2DeviceInterface::device_fx() {
    if (this->dev == 0) return 0.0f;
    return i_d.fx;
}

float LibFreenect2DeviceInterface::device_fy() {
    if (this->dev == 0) return 0.0f;
    return i_d.fy;
}

float LibFreenect2DeviceInterface::device_depth_scale() {
    if (this->dev == 0) return 0.0f;
    return 0.001f; // TODO: is this correct for kinect?!
}

std::string LibFreenect2DeviceInterface::device_guid() {
    if (this->dev == 0) return "";
    return serial;
}

TJPF LibFreenect2DeviceInterface::color_pixel_format() {
    return TJPF_BGRX;
}

bool LibFreenect2DeviceInterface::supports_depth() {
    return false;
}

bool LibFreenect2DeviceInterface::supports_audio() {
    return false;
}

bool LibFreenect2DeviceInterface::supports_body() {
    return false;
}

bool LibFreenect2DeviceInterface::supports_bidx() {
    return false;
}

bool LibFreenect2DeviceInterface::supports_hd() {
    return false;
}
