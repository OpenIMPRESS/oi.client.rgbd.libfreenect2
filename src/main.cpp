#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <fstream>
#include <cstdlib>
#include <chrono>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <asio.hpp>
#include <turbojpeg.h>
#include "UDPConnector.cpp"

using namespace std;
using namespace chrono;
using asio::ip::udp;
using namespace oi::core::network;

bool stream_shutdown = false;
void sigint_handler(int s) {
	stream_shutdown = true;
}

asio::io_service io_service_;
UDPConnector client(io_service_);

bool kinect_paused = false;
libfreenect2::Freenect2Device *devtopause;
void sigusr1_handler(int s) {
	if (devtopause == 0)  return;
	if (kinect_paused)    devtopause->start();
	else                  devtopause->stop();
	kinect_paused = !kinect_paused;
}

ofstream dumpStream;
int dumpFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence);

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence);
int openAndStream(string serial, string pipelineId);
int sendConfig();

struct IMAGE_DATA_HEADER {
    unsigned char msgType = 0x03;   // 00 // 0x03 = Depth; 0x04 = Color
    unsigned char deviceID = 0x00;  // 01
    unsigned char unused1 = 0x00;   // 02 => for consistent alignment
    unsigned char unused2 = 0x00;   // 03 => for consistent alignment
    uint32_t sequence = 0;          // 04-07
    unsigned short startRow = 0;    // 08-09
    unsigned short endRow = 0;      // 10-11
};

struct CONFIG_MSG {
    unsigned char msgType = 0x01;    // 00
    unsigned char deviceID = 0x00;   // 01
    unsigned char deviceType = 0x02; // 02
    unsigned char unused1 = 0x00;    // 03 => for consistent alignment
    unsigned short frameWidth = 0;   // 04-05
    unsigned short frameHeight = 0;  // 06-07
    unsigned short maxLines = 0;     // 08-09
    unsigned short unused2 = 0;      // 10-11 => for consistent alignment
    float Cx = 0.0f;                 // 12-15
    float Cy = 0.0f;                 // 16-19
    float Fx = 0.0f;                 // 20-23
    float Fy = 0.0f;                 // 24-27
    float DepthScale = 0.0f;         // 28-31
    char guid[33] = "00000000000000000000000000000000"; // 32-...
};

IMAGE_DATA_HEADER img_header;
CONFIG_MSG stream_config;

unsigned int headerSize = sizeof(img_header);
unsigned int linesPerMessage;
unsigned int sendThrottle;
unsigned char * frameStreamBufferDepth;
unsigned char * frameStreamBufferColor;
int frameStreamBufferDepthSize;
int frameStreamBufferColorSize;
char config_msg_buf[sizeof(stream_config)];

// JPEG COMPRESSION START ==================================
const int JPEG_QUALITY = 50;
// =========================================================

string socketIDValue = "kinect1";
string ipValue = "mm.openimpress.org";
string portValue = "6312";
string serialValue = "";
string linesValue = "-1"; // 24
string sendThrottleValue = "10";
string pipelineValue = "opencl"; // opengl
string maxDepthValue = "10";
string fileDumpValue = ""; // dump.bin

int main(int argc, char *argv[]) {
	string socketIDParam("-i");
	string portParam("-p");
	string ipParam("-d");
	string serialParam("-s");
	string linesParam("-l");
	string sendThrottleParam("-t");
	string pipelineParam("-q");
	string maxDepthParam("-r");
    string fileDumpParam("-f");

	if (argc % 2 != 1) {
		cout << "Usage:\n  freenectStreamer "
			<< socketIDParam << " [socketID] "
			<< portParam << " [port] "
			<< ipParam << " [ip] "
			<< serialParam << " [serial] "
			<< linesParam << " [maxLines] "
			<< sendThrottleParam << " [sendThrottle] "
			<< pipelineParam << " [pipeline] "
            << fileDumpParam << " [fileDump] "
			<< "\nDefaults"
			<< "\n  socketID: " << socketIDValue
			<< "\n  port:     " << portValue
			<< "\n  ip:       " << ipValue
            << "\n  serial:   " << "\"\" (serial # of kinect to open, empty = first available)"
			<< "\n  maxLines:      " << linesValue << " (max. block size in lines; -1 = auto)"
			<< "\n  sendThrottle:  " << sendThrottleValue << " (us delay between sending line blocks; 0 = no limit)"
			<< "\n  pipeline: " << pipelineValue << " (one of cpu,opengl,cuda,opencl)"
			<< "\n  maxDepth: " << maxDepthValue << " (Clip at this maximum distance (meter))"
            << "\n  fileDump: " << fileDumpValue << " (file to dump raw data to, empty = disabled)"
			<< "\n";
		return -1;
	}

	for (int count = 1; count < argc; count += 2) {
		if (socketIDParam.compare(argv[count]) == 0) {
			socketIDValue = argv[count + 1];
		}
		else if (ipParam.compare(argv[count]) == 0) {
			ipValue = argv[count + 1];
		}
		else if (portParam.compare(argv[count]) == 0) {
			portValue = argv[count + 1];
		}
		else if (serialParam.compare(argv[count]) == 0) {
			serialValue = argv[count + 1];
		}
		else if (linesParam.compare(argv[count]) == 0) {
			linesValue = argv[count + 1];
		}
		else if (sendThrottleParam.compare(argv[count]) == 0) {
			sendThrottleValue = argv[count + 1];
		}
		else if (pipelineParam.compare(argv[count]) == 0) {
			pipelineValue = argv[count + 1];
		}
		else if (maxDepthParam.compare(argv[count]) == 0) {
			maxDepthValue = argv[count + 1];
		}
        else if (fileDumpParam.compare(argv[count]) == 0) {
            fileDumpValue = argv[count + 1];
        }
		else {
			cout << "Unknown Parameter: " << argv[count] << endl;
		}
	}
    
    sendThrottle = stoi(sendThrottleValue);
    int parsedLines = stoi(linesValue);
    
    int bytesPerDepthLine = 512*2;
    int maximumLinesPerDepthPacket = (65506-headerSize)/ bytesPerDepthLine;
    
    if (parsedLines > maximumLinesPerDepthPacket) {
        cout << "Invalid value for -p. Try >= 4 and < " << maximumLinesPerDepthPacket << endl;
        return -1;
    }
    
    if (parsedLines == -1) {
        linesPerMessage = maximumLinesPerDepthPacket;
    } else if (parsedLines < 4) {
        linesPerMessage = 4; 
    } else {
        linesPerMessage = parsedLines;
    }
    
    cout << "Sending " << linesPerMessage << " scanlines per message." << endl;
    
    // Round to multiple of 4 for DTX compression
    //int remainderM4 = linesPerMessage % 4;
    //linesPerMessage = linesPerMessage - remainderM4;

	cout << linesPerMessage << "\n";
    
    frameStreamBufferDepthSize = bytesPerDepthLine * linesPerMessage + headerSize;
    frameStreamBufferDepth = new unsigned char[frameStreamBufferDepthSize];
	frameStreamBufferColorSize = tjBufSize(512, 424, JPEG_QUALITY)+headerSize;
	frameStreamBufferColor = new unsigned char[frameStreamBufferDepthSize]; // TODO: what is a good estimate maximum?


    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));

	string program_path(argv[0]);
	size_t executable_name_idx = program_path.rfind("freenectStreamer");

	string binpath = "/";
	if (executable_name_idx != string::npos) {
		binpath = program_path.substr(0, executable_name_idx);
	}
	cout << "Running in: " << binpath << endl;

	return openAndStream(serialValue, pipelineValue);
}
int openAndStream(string serial, string pipelineId) {
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	if (freenect2.enumerateDevices() == 0) {
		cout << "E: No device connected." << endl;
		return -1;
	}

	unsigned int fpsCounter = 0;
	milliseconds lastFpsAverage = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	milliseconds interval = milliseconds(2000);
    

	//s.native_non_blocking(true);

//#if !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
//	asio::socket_base::send_low_watermark slwoption(frameStreamaze);
//	s.set_option(slwoption);
//#endif

	if (pipelineId.compare("cpu") == 0) {
		pipeline = new libfreenect2::CpuPacketPipeline();
	} else if (pipelineId.compare("opengl") == 0) {
		pipeline = new libfreenect2::OpenGLPacketPipeline();
	} else if (pipelineId.compare("cuda") == 0) {
		//pipeline = new libfreenect2::CudaPacketPipeline(-1);
		// THIS WILL NOT COMPILE IF LIBFREENECT IS NOT COMPILED WITH CUDA!!!
		// set flags...
	} else if (pipelineId.compare("opencl") == 0) {
		pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
	} else {
		cout << "Unknown pipeline: " << pipelineId << endl;
		return -1;
	}

	//	cout << freenect2.getDefaultDeviceSerialNumber() << endl;


	// Default device serial (if not specified)
	if (serial == "") serial = freenect2.getDefaultDeviceSerialNumber();

	// Listeners
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth); // | libfreenect2::Frame::Ir
	libfreenect2::FrameMap frames;
	

	// Open Device & Register listenres
	dev = freenect2.openDevice(serial, pipeline);
	if (!dev->start()) {
		cout << "E: Failed to start device." << endl;
		return -1;
	}

	// config
	libfreenect2::Freenect2Device::Config config;
	config.MinDepth = 0.1;						///< Clip at this minimum distance (meter).      
	config.MaxDepth = stof(maxDepthValue);	///< Clip at this maximum distance (meter).
	config.EnableBilateralFilter = true;		///< Remove some "flying pixels".
	config.EnableEdgeAwareFilter = true;		///< Remove pixels on edges because ToF cameras produce noisy edges.
	dev->setConfiguration(config);

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	devtopause = dev;

	cout << "Device started." << endl;
	cout << "\tSerial:\t " << dev->getSerialNumber() << endl;
	cout << "\tFirmware:\t " << dev->getFirmwareVersion() << endl << endl;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler);
#endif
	stream_shutdown = false;

	// Setup Registration
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    
    stream_config.frameWidth = undistorted.width;
    stream_config.frameHeight = undistorted.height;
    stream_config.maxLines = (unsigned short) linesPerMessage;
    libfreenect2::Freenect2Device::IrCameraParams i_d = dev->getIrCameraParams();
    float depthScale = 0.001f; // TODO: is this correct for kinect?!
    stream_config.Cx = i_d.cx;
    stream_config.Cy = i_d.cy;
    stream_config.Fx = i_d.fx;
    stream_config.Fy = i_d.fy;
    stream_config.DepthScale = depthScale;
    strcpy(stream_config.guid, dev->getSerialNumber().c_str());

	client.init(socketIDValue, dev->getSerialNumber(), true, ipValue, portValue);
    
    if (fileDumpValue.length() > 0) {
        dumpStream.open(fileDumpValue, ios::binary | ios::out | ios::trunc);
    }
    
	while (!stream_shutdown) {

		milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

		if (!listener.waitForNewFrame(frames, 5 * 1000)) {
			return -1;
		}

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered);
        
		if (streamFrame(&undistorted, &registered, rgb->sequence) == -1) {
			stream_shutdown = true;
		}

		fpsCounter++;
		ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

		if (lastFpsAverage + interval <= ms) {
			lastFpsAverage = ms;
			cout << "Average FPS: " << fpsCounter / 2.0 << endl;
			fpsCounter = 0;
            sendConfig();
		}

		listener.release(frames);
	}

    if (fileDumpValue.length() > 0) dumpStream.close();
	dev->stop();
	dev->close();
	delete registration;
    client.exit();
    //delete client;
    
    cout << "Ended gracefully." << endl;
    
	return 0;
}

int sendConfig() {
    memcpy(config_msg_buf, &stream_config, sizeof(stream_config));
    
    try {
		client.SendData(config_msg_buf, sizeof(stream_config));
    } catch (exception& e) {
        cerr << "Exception send config: " << e.what() << "\n";
        return -1;
    }
    
    return 0;
}

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence) {
    img_header.sequence = sequence;

    // Send color data first...
	img_header.msgType = 0x04;
	img_header.startRow = (unsigned short)0;               // ... we can fit the whole...
	img_header.endRow = (unsigned short)regdepth->height; //...RGB data in one packet
	memcpy(&frameStreamBufferColor[0], &img_header, headerSize);

	// COMPRESS COLOR
	long unsigned int _jpegSize = frameStreamBufferColorSize-headerSize;
	unsigned char* _compressedImage = &frameStreamBufferColor[headerSize];

	// replace  _complressedImage with &frameStreamBufferColor[headerSize]
	tjhandle _jpegCompressor = tjInitCompress();
	tjCompress2(_jpegCompressor, regrgb->data, regdepth->width, 0, regdepth->height, TJPF_BGRX,
		&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
		TJFLAG_FASTDCT);

	//cout << "JPEG Size: " << _jpegSize << "\n";
	//memcpy(&frameStreamBufferColor[headerSize], _compressedImage, _jpegSize);

    if (fileDumpValue.length() > 0) dumpStream.write((const char*) &_jpegSize, sizeof(_jpegSize));
    if (fileDumpValue.length() > 0) dumpStream.write((const char*) _compressedImage, _jpegSize);
    
	try {
		client.SendData(frameStreamBufferColor, headerSize + _jpegSize);
	} catch (exception& e) {
		cerr << "Exception stream RGB Frame: " << e.what() << "\n";
	}

	//tjFree(_compressedImage);
	tjDestroy(_jpegCompressor);
	

	// Send Depth
	img_header.msgType = 0x03;
	for (unsigned int startRow = 0; startRow < regdepth->height; startRow += linesPerMessage) {

		size_t endRow = startRow + linesPerMessage;
		if (endRow >= regdepth->height) endRow = regdepth->height;
		if (startRow >= endRow) break;

		//size_t totalLines = endRow - startRow;

        img_header.startRow = (unsigned short) startRow;
        img_header.endRow = (unsigned short) endRow;
        memcpy(&frameStreamBufferDepth[0], &img_header, headerSize);
        size_t writeOffset = headerSize;

		size_t depthLineSizeR = regdepth->width * regdepth->bytes_per_pixel;
		size_t depthLineSizeW = regdepth->width * 2;
		size_t readOffset = startRow*depthLineSizeR;
		for (int line = startRow; line < endRow; line++) {
			for (int i = 0; i < regdepth->width; i++) {
				float depthValue = 0;
				memcpy(&depthValue, &regdepth->data[readOffset + i * 4], sizeof(depthValue));
				unsigned short depthValueShort = (unsigned short)(depthValue);
				memcpy(&frameStreamBufferDepth[writeOffset + i * 2], &depthValueShort, sizeof(depthValueShort));
                
			}

			writeOffset += depthLineSizeW;
			readOffset += depthLineSizeR;
		}

		//size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
		//readOffset = startRow*colorLineSizeR;

		//stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, (int)totalLines, 0);

		try {
			client.SendData(frameStreamBufferDepth, writeOffset);
		} catch (exception& e) {
			cerr << "Exception stream Depth frame: " << e.what() << "\n";
        }
        
        if (fileDumpValue.length() > 0) dumpStream.write((const char*) frameStreamBufferDepth, writeOffset);
		//this_thread::sleep_for(chrono::microseconds(sendThrottle));
	}

	return 0;
}


/*

# Protocol

 OUTDATED!!!!!!!!!!!

## Registered Frame Data (0x03):

,----------------------------- Msg Type
|    ,------------------------ Device ID
|    |    ,------------------- Start Row (0 = first row)
|    |    |    ,--------------
|    |    |    |    ,--------- End Row
|    |    |    |    |    ,----
|    |    |    |    |    |        ,---- Frame Data. 2 * (EndRow-StartRow) * 4 bytes. First half is depth, second is rgb
|    |    |    |    |    |        |         ....
V    V    V    V    V    V    <=========>
0x03 0x01 0x00 0x00 0x00 0x05 0xab ... 0xef


## Config Frame (0x04):

,---------------------------- Msg Type
|    ,----------------------- Parameter 1 (0x01 = device id)
|    |    ,------------------ Parameter 2 (...)
|    |    |    ,------------- Parameter 3 (...)
|    |    |    |
|    |    |    |        ,---- Payload (could be guid)
|    |    |    |        |           ....
V    V    V    V    <=========>
0x04 0x01 0x0b 0x0c 0xab ... 0xef

TODO: other frame data we have available are: timestamp, sequenceNr, exposure, gain, gamma, status

*/


