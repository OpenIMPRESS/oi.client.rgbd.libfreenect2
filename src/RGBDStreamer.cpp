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

	void RGBDStreamer::_AllocateBuffers() {
		float maximumLinesPerDepthPacket = (MAX_UDP_PACKET_SIZE - headerSize) / (2 * frame_width());
		linesPerMessage = (unsigned int) maximumLinesPerDepthPacket;
		// Round to multiple of 4 for DTX compression
		//int remainderM4 = linesPerMessage % 4;
		//linesPerMessage = linesPerMessage - remainderM4;

		// ALLOCATE BUFFERS
		frameStreamBufferDepthSize = 2 * frame_width() * linesPerMessage + headerSize;
		frameStreamBufferDepth = new unsigned char[frameStreamBufferDepthSize];
		// TODO: for some reason tjBufSize now returns ulong.maxValue...
		// tjBufSize(frame_width(), frame_height(), JPEG_QUALITY) + headerSize
		frameStreamBufferColorSize = std::min((int)(frame_width() * frame_height() * 3 + headerSize), (int)MAX_UDP_PACKET_SIZE);
		std::cout << frameStreamBufferColorSize << std::endl;
		frameStreamBufferColor = new unsigned char[frameStreamBufferColorSize];
		// TODO: what is a good estimate maximum?
	}

	void RGBDStreamer::Run() {
		OpenDevice();
		PopulateDeviceConfigMessage();

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
				std::cout << fpsCounter / timeDelta << " f/s, "
						  << ((bytesCounter/1024.0)/1024) / timeDelta << " MB/s\n";
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
		memcpy(config_msg_buf, &stream_config, sizeof(stream_config));
		return client->SendData((unsigned char*)config_msg_buf, sizeof(stream_config));
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
		memcpy(&frameStreamBufferColor[0], &rgbd_header, headerSize);

		// COMPRESS COLOR
		long unsigned int _jpegSize = frameStreamBufferColorSize - headerSize;
		unsigned char* _compressedImage = &frameStreamBufferColor[headerSize];

		// replace  _complressedImage with &frameStreamBufferColor[headerSize]
		tjhandle _jpegCompressor = tjInitCompress();
		tjCompress2(_jpegCompressor, rgbdata, (int)frame_width(), 0, (int)frame_height(), color_pixel_format(),
			&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
			TJFLAG_FASTDCT);

		/*
		FILE * file = fopen("img.jpg", "wb");
		fwrite(_compressedImage, 1, _jpegSize, file);
		fclose(file);
		*/

		long unsigned int colorDataSize = headerSize + _jpegSize;
		int cres = client->SendData(frameStreamBufferColor, colorDataSize);
		if (cres <= 0) return -1;
		res += cres;

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
		//tjFree(_compressedImage);

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

			memcpy(&frameStreamBufferDepth[0], &rgbd_header, headerSize);
			size_t writeOffset = headerSize;

			size_t depthLineSizeR = frame_width() * raw_depth_stride();
			size_t depthLineSizeW = frame_width() * 2;
			size_t readOffset = startRow*depthLineSizeR;
			for (int line = startRow; line < endRow; line++) {
				for (int i = 0; i < frame_width(); i++) {
					float depthValue = 0;
					memcpy(&depthValue, &depthdata[readOffset + i * 4], sizeof(depthValue));
					unsigned short depthValueShort = (unsigned short)(depthValue);
					memcpy(&frameStreamBufferDepth[writeOffset + i * 2], &depthValueShort, sizeof(depthValueShort));
				}
				writeOffset += depthLineSizeW;
				readOffset += depthLineSizeR;
			}

			//size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
			//readOffset = startRow*colorLineSizeR;
			//stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, (int)totalLines, 0);
			
			int dres = client->SendData(frameStreamBufferDepth, writeOffset);
			if (dres <= 0) return -1;
			res += dres;

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
		std::cout << "|" << raw << "|" << std::endl;

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


	void RGBDStreamer::PopulateDeviceConfigMessage() {
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
		// TODO: check if this works if guid is smaller;

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