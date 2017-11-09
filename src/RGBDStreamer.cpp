#include "RGBDStreamer.hpp"
#include <algorithm> 

namespace oi { namespace core { namespace rgbd {

	static std::chrono::milliseconds NOW() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		);
	}

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
			int sentBytes = 0;


			// TODO:
			//		if we both record and REPLAY, can't we replay in a thread while waiting for frames?
			//  std::thread replayFrameThread(ReplayNextFrame, this);
			// ...and join it at the end?
			//   replayFrameThread.join()?
			if (record_state.recording() || (!record_state.recording() && !record_state.replaying())) {
				sentBytes = SendFrame();
			}

			if (record_state.replaying() && NOW() >= record_state._replay_next_frame) {
				sentBytes = ReplayNextFrame();
			}

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
		if (record_state.recording()) {
			record_state.StopRecording();
		}
		if (record_state.replaying()) {
			record_state.StopRecording();
		}
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
		std::ostream * data_writer = NULL;
		std::ostream * meta_writer = NULL;

		if (record_state.recording()) {
			data_writer = record_state.data_writer();
			meta_writer = record_state.meta_writer();
		}

		int res = 0;
		std::chrono::milliseconds now = NOW();
		std::chrono::milliseconds delta = now - _prev_frame;
		_prev_frame = now;

		if (meta_writer != NULL && data_writer != NULL) {
			META_STRUCT frame_meta;
			frame_meta.timestamp = now;
			frame_meta.memory_pos = data_writer->tellp();
			frame_meta.frame_nr = record_state.next_rec_frame_nr();
			frame_meta.payload_size = 0;
			meta_writer->write((const char*)&frame_meta, sizeof(frame_meta)); // [memory_pos (unsigned long=8bytes)]

		}

		unsigned short deltaValue = (unsigned short) delta.count();
		if (delta.count() >= 60000) deltaValue = 0; // Just to be sure that we don't overflow...

		rgbd_header.sequence = sequence;
		if (sequence > highest_sequence) highest_sequence = highest_sequence;
		else rgbd_header.sequence = ++highest_sequence;
		rgbd_header.delta_t = deltaValue;

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

		int c_data_len = headerSize + _jpegSize;
		dc_rgb->_data_end = c_data_len;

		if (data_writer != NULL) {
			unsigned int jpeg_size = (unsigned int)_jpegSize;
			data_writer->write((const char*)&deltaValue, sizeof(rgbd_header.delta_t)); // [delta_t (unsigned short)]
			data_writer->write((const char*)&rgbd_header.startRow, sizeof(rgbd_header.startRow)); // [color_start_l (unsigned short)]
			data_writer->write((const char*)&rgbd_header.endRow, sizeof(rgbd_header.endRow)); // [color_end_l (unsigned short)]
			data_writer->write((const char*)&jpeg_size, sizeof(jpeg_size)); // [color_size  (unsigned int)]
			data_writer->write((const char*)_compressedImage, jpeg_size); // [color_data ...]
		}

		if (!record_state.replaying()) {
			client->QueueForSending(&dc_rgb);
		} else {
			client->ReleaseForWriting(&dc_rgb);
		}

		res += c_data_len;
		tjDestroy(_jpegCompressor);

		// Send Depth
		rgbd_header.msgType = 0x03;

		if (data_writer != NULL) {
			unsigned short npackets = (unsigned short)(frame_height() + linesPerMessage - 1) / linesPerMessage; // round up division
			data_writer->write((const char*)&npackets, sizeof(npackets)); // [n_depth_packets(unsigned short)]
		}

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

			int d_data_len = writeOffset;
			dc_depth->_data_end = d_data_len;

			if (data_writer != NULL) {
				unsigned char* _depthBlock = &(dc_depth->dataBuffer[headerSize]);
				unsigned int depth_block_size = (unsigned int) (writeOffset-headerSize);
				data_writer->write((const char*)&rgbd_header.startRow, sizeof(rgbd_header.startRow)); // [depth_start_l (unsigned short)]
				data_writer->write((const char*)&rgbd_header.endRow, sizeof(rgbd_header.endRow)); // [depth_end_l (unsigned short)]
				data_writer->write((const char*)&depth_block_size, sizeof(depth_block_size)); // [depth_size  (unsigned int)]
				data_writer->write((const char*)_depthBlock, depth_block_size); // [depth_data ...]
			}

			if (!record_state.replaying()) {
				client->QueueForSending(&dc_depth);
			} else {
				client->ReleaseForWriting(&dc_depth);
			}
			res += d_data_len;
		}

		return res;
	}

	int RGBDStreamer::ReplayNextFrame() {
		std::istream * data_reader = record_state.data_reader();
		std::chrono::milliseconds now = NOW();
		int res = 0;

		unsigned short delta = 0;
		unsigned short start_l = 0;
		unsigned short end_l = 0;
		unsigned short npackets = 0;
		unsigned int rgbLength = 0;
		unsigned int depthLength = 0;

		// RGB
		data_reader->read(reinterpret_cast<char *>(&start_l), sizeof(start_l));
		data_reader->read(reinterpret_cast<char *>(&end_l), sizeof(end_l));
		data_reader->read(reinterpret_cast<char *>(&rgbLength), sizeof(rgbLength));
		// HEADER
		record_state.replay_header.sequence = ++highest_sequence;
		record_state.replay_header.delta_t = delta;
		record_state.replay_header.startRow = start_l;
		record_state.replay_header.endRow = end_l;
		record_state.replay_header.deviceID = record_state.current_replay_file->config().deviceID;
		record_state.replay_header.msgType = 0x04;

		oi::core::network::DataContainer * dc_rgb;
		if (!client->GetFreeWriteContainer(&dc_rgb)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		
		memcpy(dc_rgb->dataBuffer, &record_state.replay_header, headerSize);
		data_reader->read(reinterpret_cast<char *>(&(dc_rgb->dataBuffer[headerSize])), rgbLength);
		dc_rgb->_data_end = headerSize + rgbLength;
		res += dc_rgb->_data_end;
		client->QueueForSending(&dc_rgb);

		// DEPTH
		data_reader->read(reinterpret_cast<char *>(&npackets), sizeof(npackets));
		record_state.replay_header.msgType = 0x03;

		for (unsigned int i = 0; i < npackets; i++) {
			data_reader->read(reinterpret_cast<char *>(&start_l), sizeof(start_l));
			data_reader->read(reinterpret_cast<char *>(&end_l), sizeof(end_l));
			data_reader->read(reinterpret_cast<char *>(&depthLength), sizeof(depthLength));
			record_state.replay_header.startRow = start_l;
			record_state.replay_header.endRow = end_l;

			oi::core::network::DataContainer * dc_depth;
			if (!client->GetFreeWriteContainer(&dc_depth)) {
				std::cout << "\nERROR: No free buffers available" << std::endl;
				return -1;
			}

			memcpy(dc_depth->dataBuffer, &record_state.replay_header, headerSize);
			data_reader->read(reinterpret_cast<char *>(&(dc_depth->dataBuffer[headerSize])), depthLength);
			dc_depth->_data_end = headerSize + depthLength;
			res += dc_depth->_data_end;
			client->QueueForSending(&dc_depth);
		}

		data_reader->read(reinterpret_cast<char *>(&delta), sizeof(delta));

		// denotes end of file
		if (delta >= 60000) {
			std::cout << "\n CLEAN END " << std::endl;
			if (record_state.loop) {
				data_reader->clear();
				data_reader->seekg(0, std::ios::beg);
				unsigned short first_delta = 0;
				data_reader->read(reinterpret_cast<char *>(&delta), sizeof(delta));
			} else {
				record_state.StopReplaying();
			}
		}

		record_state._replay_next_frame = now + std::chrono::milliseconds(delta);

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

			if (msg["cmd"] == "record") {
				if (msg["val"] == "startrec") {
					record_state.StartRecording(msg["file"], stream_config);
				}

				if (msg["val"] == "stoprec") {
					record_state.StopRecording();
				}

				if (msg["val"] == "startplay") {
					record_state.StartReplaying(msg["file"]);
				}

				if (msg["val"] == "stopplay") {
					record_state.StopReplaying();
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

	void StreamerConfig::Parse(int argc, char *argv[]) {

		std::string useMMParam("-mm");
		std::string socketIDParam("-id");
		std::string listenPortParam("-lp");
		std::string remotePortParam("-rp");
		std::string remoteHostParam("-rh");
		std::string serialParam("-sn");
		
		std::string maxDepthParam("-d");
		std::string pipelineParam("-p");
		std::string fileDumpParam("-o");

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


	RecordState::RecordState() {
		std::cout << "init record state" << std::endl;
		_recording = false;
		_replaying = false;
		loop = true;
	}

	std::ostream * RecordState::data_writer() {
		return &_out_data;
	}

	std::ostream * RecordState::meta_writer() {
		return &_out_meta;
	}

	std::ifstream * RecordState::data_reader() {
		return &_in_data;
	}

	bool RecordState::recording() {
		return _recording;
	}

	bool RecordState::replaying() {
		return _replaying;
	}

	unsigned int RecordState::next_rec_frame_nr() {
		unsigned int res = _current_frame;
		_current_frame++;
		return res;
	}

	CONFIG_STRUCT * RecordState::replay_config() {
		return &_replay_config;
	}

	bool RecordState::LoadMeta(std::string name) {
		std::string filename_meta = name + META_SUFFIX;
		std::ifstream in_meta;
		in_meta.open(filename_meta, std::ios::binary | std::ios::in);
		if (in_meta.fail()) return false;
		FileMeta fm;
		fm.Load(name, &in_meta);
		files_meta[name] = fm;
		in_meta.close();
		in_meta.clear();
		return true;
	}

	bool RecordState::StartRecording(std::string name, CONFIG_STRUCT config) {
		if (_recording) {
			std::cerr << "\nCannot start recording with name " << name << ". Already recording." << std::endl;
			return false;
		}
		if (files_meta.find(name) != files_meta.end()) {
			std::cerr << "\nCannot start recording. Name " << name << " already exists." << std::endl;
			return false;
		}

		std::string filename_data = name + DATA_SUFFIX;
		std::string filename_meta = name + META_SUFFIX;
		_next_frame = name;
		_current_frame = 0;
		_out_data.open(filename_data, std::ios::binary | std::ios::out | std::ios::trunc);
		_out_meta.open(filename_meta, std::ios::binary | std::ios::out | std::ios::trunc);
		unsigned short version = 1; // TODO: make this global
		_out_meta.write((const char*) &version, sizeof(version));
		_out_meta.write((const char*) &config, sizeof(config));
		_current_recording_name = name;
		_recording = true;
		return true;
	}

	bool RecordState::StopRecording() {
		if (!_recording) return false;

		unsigned short last_delta = 60000;
		_out_data.write((const char*)&last_delta, sizeof(last_delta)); // [memory_pos (unsigned long=8bytes)]

		_out_data.close();
		_out_data.clear();
		_out_meta.close();
		_out_meta.clear();
		_recording = false;
		_current_recording_name = "";
		if (!LoadMeta(_next_frame)) {
			std::cerr << "Stopped recording but failed to load Meta" << std::endl;
			return false;
		}
		return true;
	}

	bool RecordState::StartReplaying(std::string name) {
		if (_recording && name == _current_recording_name) {
			std::cerr << "\nERROR: cannot replay file while recording to it." << std::endl;
			return false;
		}

		if (_replaying) {
			StopReplaying();
		}

		std::string filename_data = name + DATA_SUFFIX;
		std::string filename_meta = name + META_SUFFIX;
		_in_data.open(filename_data, std::ios::binary | std::ios::in);
		if (files_meta.find(name) == files_meta.end() && (!LoadMeta(name) || _in_data.fail())) {
			std::cout << "\nERROR: Failed to open \"" << name << "\" for replaying" << std::endl;
		}

		unsigned short first_delta = 0;
		_in_data.read(reinterpret_cast<char *>(&first_delta), sizeof(first_delta));
		_replay_next_frame = oi::core::rgbd::NOW() + std::chrono::milliseconds(first_delta);

		_replay_config = files_meta[name].config();
		current_replay_file = &files_meta[name];
		_replaying = true;
		return true;
	}

	bool RecordState::StopReplaying() {
		if (!_replaying) return false;
		_in_data.close();
		_in_data.clear();
		current_replay_file = NULL;
		_replaying = false;
	}

	CONFIG_STRUCT FileMeta::config() {
		return _config;
	}

	void rgbd::FileMeta::Load(std::string name, std::ifstream * in_meta) {
		this->_name = name;
		in_meta->read(reinterpret_cast<char *>(&_recording_version), sizeof(_recording_version));
		in_meta->read(reinterpret_cast<char *>(&_config), sizeof(_config));
		while (true) {
			META_STRUCT frame_meta;
			in_meta->read(reinterpret_cast<char *>(&frame_meta), sizeof(frame_meta));
			in_meta->ignore(frame_meta.payload_size);
			if (in_meta->eof()) break;
			_frames.push_back(frame_meta);
			std::cout << std::endl;
		}
	}

	std::string FileMeta::name() {
		return _name;
	}


} } }