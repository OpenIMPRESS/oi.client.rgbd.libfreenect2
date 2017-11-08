#include "RGBDStreamer.hpp"
#include <algorithm> 

namespace oi { namespace core { namespace rgbd {
	using string = std::string;

	RGBDStreamer::RGBDStreamer(StreamerConfig cfg, oi::core::network::UDPBase * c) {
		fpsCounter = 0;
		bytesCounter = 0;
		lastFpsAverage = NOW();
		this->config = cfg;
		client = c;
	}

	void RGBDStreamer::Run() {
		OpenDevice();
		ConfigStream();

		stream_shutdown = false;
		while (!stream_shutdown) {
			int sentBytes = SendFrame();
			if (sentBytes >= 0) {
				bytesCounter += sentBytes;
				fpsCounter++;
			} else {
				stream_shutdown = true;
				continue;
			}

			HandleData();

			if (lastFpsAverage + interval <= NOW()) {
				lastFpsAverage = NOW();
				double timeDelta = (interval.count() / 1000.0);

				printf("\r >> Streaming at %5.2f Fps - Data: %5.2f Mbps", fpsCounter / timeDelta, ((bytesCounter / 1024.0) / 1024) / timeDelta);
				std::cout << std::flush;

				fpsCounter = 0;
				bytesCounter = 0;
				if (SendConfig() <= 0) stream_shutdown = true;
			}
		}

	}

	void RGBDStreamer::Exit() {
		stream_shutdown = true;
		CloseDevice();
		client->Close();
	}

	int RGBDStreamer::SendConfig() {
		oi::core::network::DataContainer * dc;
		if (!client->GetFreeWriteContainer(&dc)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}

		int data_len = sizeof(stream_config);
		memcpy(dc->dataBuffer, &stream_config, data_len);
		dc->_data_end = data_len;
		client->QueueForSending(&dc);
		return data_len;
	}

	int RGBDStreamer::_SendFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata) {
		int res = 0;
		std::chrono::milliseconds now = NOW();
		std::chrono::milliseconds delta = now - prevFrame;
		prevFrame = now;

		rgbd_header.sequence = sequence;

		// Send color data first...
		rgbd_header.msgType = 0x04;
		rgbd_header.startRow = (unsigned short) 0;             // ... we can fit the whole...
		rgbd_header.endRow = (unsigned short) frame_height(); //...RGB data in one packet

		oi::core::network::DataContainer * dc_rgb;
		if (!client->GetFreeWriteContainer(&dc_rgb)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}

		memcpy(dc_rgb->dataBuffer, &rgbd_header, headerSize);

		// COMPRESS COLOR
		long unsigned int _jpegSize = dc_rgb->bufferSize - headerSize;
		unsigned char* _compressedImage = &(dc_rgb->dataBuffer[headerSize]);

		tjhandle _jpegCompressor = tjInitCompress();
		tjCompress2(_jpegCompressor, rgbdata, (int)frame_width(), 0, (int)frame_height(), color_pixel_format(),
			&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
			TJFLAG_FASTDCT);

		/*
		FILE * file = fopen("img.jpg", "wb");
		fwrite(_compressedImage, 1, _jpegSize, file);
		fclose(file);
		*/

		int c_data_len = headerSize + _jpegSize;
		dc_rgb->_data_end = c_data_len;
		client->QueueForSending(&dc_rgb);
		res += c_data_len;

		/*
		if (fileDumpValue.length() > 0 && !replay) {
			long unsigned int deltaValue = (long unsigned int) delta.count();
			//cout << "frame" << currentFrame << " delta:" << deltaValue;
			//cout << " rgbLength:" << colorDataSize;

			dumpStream.write((const char*)&deltaValue, sizeof(deltaValue));
			dumpStream.write((const char*)&colorDataSize, sizeof(colorDataSize));
			dumpStream.write((const char*)frameStreamBufferColor, colorDataSize);
		}*/

		tjDestroy(_jpegCompressor);

		// Send Depth
		rgbd_header.msgType = 0x03;


		/*
		unsigned long int npackets = (unsigned long int) (frame_height() + linesPerMessage - 1) / linesPerMessage; // round up division
		if (fileDumpValue.length() > 0 && !replay) {
			dumpStream.write((const char*)&npackets, sizeof(npackets));
		}*/

		for (unsigned int startRow = 0; startRow < frame_height(); startRow += linesPerMessage) {

			size_t endRow = startRow + linesPerMessage;
			if (endRow >= frame_height()) endRow = frame_height();
			if (startRow >= endRow) break;

			rgbd_header.startRow = (unsigned short)startRow;
			rgbd_header.endRow = (unsigned short)endRow;

			oi::core::network::DataContainer * dc_depth;
			if (!client->GetFreeWriteContainer(&dc_depth)) {
				std::cout << "\nERROR: No free buffers available" << std::endl;
				return -1;
			}

			memcpy(dc_depth->dataBuffer, &rgbd_header, headerSize);
			size_t writeOffset = headerSize;

			size_t depthLineSizeR = frame_width() * raw_depth_stride();
			size_t depthLineSizeW = frame_width() * 2;
			size_t readOffset = startRow*depthLineSizeR;
			for (int line = startRow; line < endRow; line++) {
				for (int i = 0; i < frame_width(); i++) {
					float depthValue = 0;
					memcpy(&depthValue, &depthdata[readOffset + i * 4], sizeof(depthValue));
					unsigned short depthValueShort = (unsigned short)(depthValue);
					memcpy(&(dc_depth->dataBuffer[writeOffset + i * 2]), &depthValueShort, sizeof(depthValueShort));
				}
				writeOffset += depthLineSizeW;
				readOffset += depthLineSizeR;
			}

			//size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
			//readOffset = startRow*colorLineSizeR;
			//stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, (int)totalLines, 0);
			

			int d_data_len = writeOffset;
			dc_depth->_data_end = d_data_len;
			client->QueueForSending(&dc_depth);
			res += d_data_len;

			/*
			long unsigned int depthDataSize = (unsigned long int) writeOffset;
			if (fileDumpValue.length() > 0 && !replay) {
				dumpStream.write((const char*)&depthDataSize, sizeof(depthDataSize));
				dumpStream.write((const char*)frameStreamBufferDepth, depthDataSize);
			}*/
		}

		return res;
	}

	void RGBDStreamer::HandleData() {
		oi::core::network::DataContainer * dc;
		if (!client->DequeueForReading(&dc)) return;

		std::string raw((char *) &(dc->dataBuffer[dc->data_start()]), dc->data_end() - dc->data_start());
		std::cout << "\n|" << raw << "|" << std::endl;

		if (!HandleData(dc)) {
			nlohmann::json msg = nlohmann::json::parse(
				&(dc->dataBuffer[dc->data_start()]),
				&(dc->dataBuffer[dc->data_end()]));
			
			if (msg["cmd"] == "application") {
				if (msg["val"] == "stop") {
					stream_shutdown = true;
				}
			}

			if (msg["cmd"] == "recording") {
				if (msg["val"] == "startrec") {
				}

				if (msg["val"] == "stoprec") {
				}

				if (msg["val"] == "startplay") {
				}

				if (msg["val"] == "stopplay") {
				}
			}
		}

		client->ReleaseForReceiving(&dc);
	}


	void RGBDStreamer::ConfigStream() {
		linesPerMessage = (unsigned int)(MAX_UDP_PACKET_SIZE - headerSize) / (2 * frame_width());

		stream_config.frameWidth = (unsigned short) frame_width();
		stream_config.frameHeight = (unsigned short) frame_height();
		stream_config.maxLines = (unsigned short) linesPerMessage;
		stream_config.Cx = device_cx();
		stream_config.Cy = device_cy();
		stream_config.Fx = device_fx();
		stream_config.Fy = device_fy();
		stream_config.DepthScale = device_depth_scale();
		std::string guid = device_guid();
		strcpy_s(stream_config.guid, 33, guid.c_str());

		config_msg_buf = new unsigned char[sizeof(stream_config)];
	}


	std::chrono::milliseconds RGBDStreamer::NOW() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
			);
	}

	void StreamerConfig::Parse(int argc, char *argv[]) {

		string useMMParam("-mm");
		string socketIDParam("-id");
		string listenPortParam("-lp");
		string remotePortParam("-rp");
		string remoteHostParam("-rh");
		string serialParam("-sn");
		
		string maxDepthParam("-d");
		string pipelineParam("-p");
		string fileDumpParam("-o");

		for (int count = 1; count < argc; count += 2) {
			if (socketIDParam.compare(argv[count]) == 0) {
				this->socketID = argv[count + 1];
			} else if (remoteHostParam.compare(argv[count]) == 0) {
				this->remoteHost = argv[count + 1];
			} else if (remotePortParam.compare(argv[count]) == 0) {
				this->remotePort = std::stoi(argv[count + 1]);
			} else if (listenPortParam.compare(argv[count]) == 0) {
				this->listenPort = std::stoi(argv[count + 1]);
			} else if (serialParam.compare(argv[count]) == 0) {
				this->deviceSerial = argv[count + 1];
			} else if (pipelineParam.compare(argv[count]) == 0) {
				this->pipeline = argv[count + 1];
			} else if (maxDepthParam.compare(argv[count]) == 0) {
				this->maxDepth = std::stof(argv[count + 1]);
			} else if (fileDumpParam.compare(argv[count]) == 0) {
				this->fileDump = argv[count + 1];
			} else if (useMMParam.compare(argv[count]) == 0) {
				this->useMatchMaking = std::stoi(argv[count + 1])==1;
			} else {
				std::cout << "Unknown Parameter: " << argv[count] << std::endl;
			}
		}

	}
	
} } }