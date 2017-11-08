#include "LibFreenect2Streamer.hpp"

bool _shutdown;
bool _paused;
libfreenect2::Freenect2Device * devtopause;
void sigint_handler(int s) {
	_shutdown = true;
}

void sigusr1_handler(int s) {
	if (devtopause == 0)
		return;
	if (_paused)
		devtopause->start();
	else
		devtopause->stop();
	_paused = !_paused;
}

namespace oi { namespace core { namespace rgbd {

	LibFreenect2Streamer::LibFreenect2Streamer(StreamerConfig cfg, oi::core::network::UDPBase * c) : RGBDStreamer(cfg, c) {
		_AllocateBuffers();
	}



	bool LibFreenect2Streamer::OpenDevice() {

		freenect2 = new libfreenect2::Freenect2();
		libfreenect2::PacketPipeline *pipeline = 0;
		if (freenect2->enumerateDevices() == 0) {
			std::cout << "E: No device connected." << std::endl;
			return false;
		}

		libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));

		if (this->config.pipeline.compare("cpu") == 0) {
			pipeline = new libfreenect2::CpuPacketPipeline();
		} else if (this->config.pipeline.compare("opengl") == 0) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
			pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
			std::cout << "OpenGL pipeline is not supported!" << std::endl;
			return false;
#endif
		} else if (this->config.pipeline.compare("cuda") == 0) {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
			pipeline = new libfreenect2::CudaPacketPipeline(-1);
#else
			std::cout << "CUDA pipeline is not supported!" << std::endl;
			return false;
#endif
		} else if (this->config.pipeline.compare("opencl") == 0) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
			pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
#else
			std::cout << "OpenCL pipeline is not supported!" << std::endl;
			return false;
#endif
		} else {
			std::cout << "Unknown pipeline: " << this->config.pipeline << std::endl;
			return false;
		}

		// Default device serial (if not specified)
		serial = this->config.deviceSerial;
		if (serial == "") serial = freenect2->getDefaultDeviceSerialNumber();

		// Listeners
		listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
		frames = new libfreenect2::FrameMap();
		// Open Device & Register listenres
		dev = freenect2->openDevice(serial, pipeline);
		if (!dev->start()) {
			std::cout << "E: Failed to start device." << std::endl;
			return false;
		}

		i_d = dev->getIrCameraParams();

		// config
		libfreenect2::Freenect2Device::Config config;
		config.MinDepth = 0.1f;						///< Clip at this minimum distance (meter).      
		config.MaxDepth = this->config.maxDepth;	///< Clip at this maximum distance (meter).
		config.EnableBilateralFilter = true;		///< Remove some "flying pixels".
		config.EnableEdgeAwareFilter = true;		///< Remove pixels on edges because ToF cameras produce noisy edges.
		dev->setConfiguration(config);

		dev->setColorFrameListener(listener);
		dev->setIrAndDepthFrameListener(listener);

		devtopause = dev;
		signal(SIGINT, sigint_handler);
		#ifdef SIGUSR1
		signal(SIGUSR1, sigusr1_handler);
		#endif

		// Setup Registration
		registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
		undistorted = new libfreenect2::Frame(this->frame_width(), this->frame_height(), this->raw_depth_stride());
		registered = new libfreenect2::Frame(this->frame_width(), this->frame_height(), this->raw_depth_stride());

		std::cout << "Device started." << std::endl;
		std::cout << "\tSerial:\t " << dev->getSerialNumber() << std::endl;
		std::cout << "\tFirmware:\t " << dev->getFirmwareVersion() << std::endl << std::endl;

		return true;
	}

	int LibFreenect2Streamer::SendFrame() {
		if (!listener->waitForNewFrame(*frames, 5 * 1000)) return -1;
		libfreenect2::Frame *rgb = (*frames)[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = (*frames)[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, undistorted, registered);
		int res = _SendFrame(rgb->sequence, registered->data, undistorted->data);
		listener->release(*frames);
		return res;
	}

	bool LibFreenect2Streamer::CloseDevice() {
		dev->close();
		delete listener;
		delete registration;
		delete registered;
		delete undistorted;
		delete frames;
		return true;
	}

	bool LibFreenect2Streamer::HandleData(oi::core::network::DataContainer * dc) {
		return false;
	}

	int LibFreenect2Streamer::frame_width() {
		return 512;
	}

	int LibFreenect2Streamer::frame_height() {
		return 424;
	}

	int LibFreenect2Streamer::send_depth_stride() {
		return 2;
	}

	int LibFreenect2Streamer::raw_depth_stride() {
		return 4;
	}

	int LibFreenect2Streamer::raw_color_stride() {
		return 4;
	}

	float LibFreenect2Streamer::device_cx() {
		if (this->dev == 0) return 0.0f;
		return i_d.cx;
	}

	float LibFreenect2Streamer::device_cy() {
		if (this->dev == 0) return 0.0f;
		return i_d.cy;
	}

	float LibFreenect2Streamer::device_fx() {
		if (this->dev == 0) return 0.0f;
		return i_d.fx;
	}

	float LibFreenect2Streamer::device_fy() {
		if (this->dev == 0) return 0.0f;
		return i_d.fy;
	}

	float LibFreenect2Streamer::device_depth_scale() {
		if (this->dev == 0) return 0.0f;
		return 0.001f; // TODO: is this correct for kinect?!
	}

	std::string LibFreenect2Streamer::device_guid() {
		if (this->dev == 0) return "";
		return serial;
	}

	TJPF LibFreenect2Streamer::color_pixel_format() {
		return TJPF_BGRX;
	}



} } }