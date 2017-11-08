#include "UDPConnector.hpp"

namespace oi { namespace core { namespace network {
    using asio::ip::udp;
	using asio::ip::tcp;
	using json = nlohmann::json;
	using namespace std;
    using namespace chrono;


	UDPBase::UDPBase(int listenPort, int sendPort, std::string sendHost, asio::io_service & io_service) 
		: io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), listenPort)), resolver_(io_service) {
		this->listenPort = listenPort;
		this->sendPort = sendPort;
		this->sendHost = sendHost;
		asio::socket_base::send_buffer_size option_set(65507);
		this->socket_.set_option(option_set);
		this->endpoint_ = * resolver_.resolve(udp::v4(), this->sendHost, to_string(this->sendPort));
	}

	bool UDPBase::Init(size_t receive_buffer_size, int receive_containers, size_t send_buffer_size, int send_containers) {
		for (int i = 0; i < receive_containers; i++) {
			DataContainer * dc = new DataContainer(i, receive_buffer_size);
			unused_receive.push(dc);
		}

		for (int i = 0; i < send_containers; i++) {
			DataContainer * dc = new DataContainer(i, send_buffer_size);
			unused_send.push(dc);
		}

		listen_thread = new std::thread(&UDPBase::DataListener, this);
		return true;
	}




	int UDPBase::SendData(unsigned char* data, size_t size) {
		return SendData(data, size, endpoint_);
	}

	int UDPBase::SendData(unsigned char* data, size_t size, udp::endpoint ep) {
		try {
			return socket_.send_to(asio::buffer(data, size), ep);
		} catch (std::exception& e) {
			std::cerr << "Send Exception: " << e.what() << "\n";
			return -1;
		}
	}



	void UDPBase::Close() {
		running = false;
		socket_.close();
		listen_thread->join();

		while (!queued_receive.empty()) {
			DataContainer * dc = queued_receive.front();
			queued_receive.pop();
			delete dc;
		}

		while (!unused_receive.empty()) {
			DataContainer * dc = unused_receive.front();
			unused_receive.pop();
			delete dc;
		}

		while (!queued_receive.empty()) {
			DataContainer * dc = queued_send.front();
			queued_send.pop();
			delete dc;
		}

		while (!unused_receive.empty()) {
			DataContainer * dc = unused_send.front();
			unused_send.pop();
			delete dc;
		}

	}


	bool UDPBase::GetFreeReceiveContainer(DataContainer ** container) {
		std::unique_lock<std::mutex> lk(listen_mutex);
		if (unused_receive.empty()) return false;
		*container = unused_receive.front();
		unused_receive.pop();
		return true;
	}

	void UDPBase::QueueForReading(DataContainer ** container) {
		std::unique_lock<std::mutex> lk(listen_mutex);
		queued_receive.push(*container);
		*container = NULL;
		// ... could notify a processer thread here?
	}

	bool UDPBase::DequeueForReading(DataContainer ** container) {
		std::unique_lock<std::mutex> lk(listen_mutex);
		if (queued_receive.empty()) return false;
		*container = queued_receive.front();
		queued_receive.pop();
		return true;
	}

	void UDPBase::ReleaseForReceiving(DataContainer ** container) {
		if (*container == NULL) return;
		std::unique_lock<std::mutex> lk(listen_mutex);
		unused_receive.push(*container);
		*container = NULL;
	}


	bool UDPBase::GetFreeWriteContainer(DataContainer ** container) {
		std::unique_lock<std::mutex> lk(send_mutex);
		if (unused_send.empty()) return false;
		*container = unused_send.front();
		unused_send.pop();
		return true;
	}

	void UDPBase::QueueForSending(DataContainer ** container, udp::endpoint ep) {
		std::unique_lock<std::mutex> lk(send_mutex);
		(*container)->_endpoint = ep;
		queued_send.push(*container);
		*container = NULL;
		send_cv.notify_one();
	}

	void UDPBase::QueueForSending(DataContainer ** container) {
		QueueForSending(container, endpoint_);
	}

	bool UDPBase::DequeueForSending(DataContainer ** container) {
		std::unique_lock<std::mutex> lk(send_mutex);
		if (queued_send.empty()) return false;
		*container = queued_send.front();
		queued_send.pop();
		return true;
	}

	void UDPBase::ReleaseForWriting(DataContainer ** container) {
		if (*container == NULL) return;
		std::unique_lock<std::mutex> lk(send_mutex);
		unused_send.push(*container);
		*container = NULL;
	}


	unsigned int DataContainer::header_start() {
		return _header_start;
	}

	unsigned int DataContainer::data_start() {
		return _data_start;
	}

	unsigned int DataContainer::data_end() {
		return _data_end;
	}

	asio::ip::udp::endpoint DataContainer::endpoint() {
		return _endpoint;
	}

	char DataContainer::packet_id() {
		return dataBuffer[_header_start];
	}

	void UDPBase::DataSender() {
		running = true;
		DataContainer * send_data_container;
		while (running) {
			std::unique_lock<std::mutex> lk(send_mutex);
			//while (queued_send.empty())
			std::cout << "sender waiting" << std::endl;
			send_cv.wait(lk);

			if (!GetSendData(&send_data_container)) continue;

			SendData(send_data_container->dataBuffer, send_data_container->data_end(), send_data_container->endpoint());

			ReleaseForWriting(&send_data_container);
		}
	}

	bool UDPBase::GetSendData(DataContainer ** container) {
		std::unique_lock<std::mutex> lk(send_mutex);
		if (queued_send.empty()) return false;
		*container = queued_send.front();
		queued_send.pop();
		return true;
	}

	void UDPBase::DataListener() {
		running = true;
		DataContainer * work_container;
		while (running) {
			if (!GetFreeReceiveContainer(&work_container)) {
				cout << "ERROR: no unused DataContainer to load data into." << endl;
				this_thread::sleep_for(1s);
				continue;
			}

			asio::error_code ec;
			try {
				asio::socket_base::message_flags mf = 0;

				size_t len = socket_.receive(asio::buffer(&work_container->dataBuffer[0], work_container->bufferSize), mf, ec);

				if (len > 0) {
					work_container->_header_start = 0;
					work_container->_data_end = len;
					if (work_container->packet_id() == 100) {
						work_container->_data_start = 1;
					} else if (work_container->packet_id() == 20) {
						work_container->_data_start = 13; // 1+4*sizeof(uint32)
					}
					QueueForReading(&work_container);
				} else {
				}
			} catch (exception& e) {
				cerr << "Exception while receiving (Code " << ec << "): " << e.what() << endl;
				this_thread::sleep_for(1s);
			}

			ReleaseForReceiving(&work_container);
		}
	}























































	OldUDPConnector::OldUDPConnector(asio::io_service & io_service) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)) {
		asio::socket_base::send_buffer_size option_set(65507);
		socket_.set_option(option_set);

		lastRegister = (milliseconds)0;
		registerInterval = (milliseconds)2000;
		lastSentHB = (milliseconds)0;
		HBInterval = (milliseconds)2000;
		lastReceivedHB = (milliseconds)0;
		connectionTimeout = (milliseconds)5000;
	}

	void OldUDPConnector::Init(string _socketID, string _UID, bool _isSender, const string& host, const string& port, bool _useMatchmaking) {
		socketID = _socketID;
		isSender = _isSender;
		UID = _UID;
		useMatchmaking = _useMatchmaking;

		udp::resolver resolver(io_service_);
		udp::resolver::query query(udp::v4(), host, port);
		udp::resolver::iterator iter = resolver.resolve(query);
		serverEndpoint_ = *iter;

		_listenThread = std::thread(&OldUDPConnector::DataListener, this);

		if (useMatchmaking) {
			localIP = getLocalIP();
			_updateThread = std::thread(&OldUDPConnector::Update, this);
		} else {
			connected = true;
		}
	}

	// TODO: Not threadsafe!
	json OldUDPConnector::ReadData() {
		if (msgQueue.size() < 1) {
			return NULL;
		} else {
			json result = msgQueue.front();
			msgQueue.pop();
			return result;
		}
	}

	void OldUDPConnector::SendData(unsigned char* data, size_t size) {
		if (!useMatchmaking) {
			socket_.send_to(asio::buffer(data, size), serverEndpoint_);
		} else if (connected) {
			socket_.send_to(asio::buffer(data, size), clientEndpoint_);
		}
	}

	void OldUDPConnector::SendData(char* data, size_t size) {
		if (!useMatchmaking) {
			socket_.send_to(asio::buffer(data, size), serverEndpoint_);
		} else if (connected) {
			socket_.send_to(asio::buffer(data, size), clientEndpoint_);
		}
	}

	void OldUDPConnector::Close() {
		_listenRunning = false;
		_updateRunning = false;

		socket_.close(); // stop current transmissions (otherwise thread doesn't want to end).
		if (useMatchmaking) _updateThread.join();
		_listenThread.join();
	}

	bool OldUDPConnector::isConnected() {
		return connected;
	}


	// PRIVATE

	void OldUDPConnector::_SendData(const std::string& msg, udp::endpoint endpoint) {
		try {
			socket_.send_to(asio::buffer(msg, msg.size()), endpoint);
		} catch (const exception& e) {
			cout << "exception while sending: " << e.what() << endl;
		}
	}

	void OldUDPConnector::DataListener() {
		const size_t size = 65507;
		char data[size];
		_listenRunning = true;
		while (_listenRunning) {
			asio::error_code ec;
			try {
				asio::socket_base::message_flags mf = 0;
				size_t len = socket_.receive(asio::buffer(data, size), mf, ec);
				if (len > 0) {
					cout << len << endl;
					HandleReceivedData(data, len);
				}
			} catch (exception& e) {
				cerr << "Exception while receiving (Code " << ec << "): " << e.what() << endl;
			}
		}
	}

	void OldUDPConnector::HandleReceivedData(char* data, int len) {
		char magicByte = data[0];

		if (magicByte == 100) {
			json j = json::parse(&data[1], &data[len]);
			cout << j << endl;
			if (j["type"] == "answer") {
				string host = j.at("address").get<string>();
				string port = to_string(j.at("port").get<int>());

				udp::resolver resolver(io_service_);
				udp::resolver::query query(udp::v4(), host, port);
				udp::resolver::iterator iter = resolver.resolve(query);
				clientEndpoint_ = *iter;

				Punch();
				Punch();
			} else if (j["type"] == "punch") {
				lastReceivedHB = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
				connected = true;
			}
		} else if (magicByte == 20) {
			uint32_t packageSequenceID = 0;
			uint32_t partsAm = 0;
			uint32_t currentPart = 0;
			unsigned int bytePos = 1;
			memcpy(&packageSequenceID, &data[bytePos], sizeof(packageSequenceID));
			bytePos += sizeof(packageSequenceID);
			memcpy(&partsAm, &data[bytePos], sizeof(partsAm));
			bytePos += sizeof(partsAm);
			memcpy(&currentPart, &data[bytePos], sizeof(currentPart));
			bytePos += sizeof(currentPart);
			cout << packageSequenceID << " " << partsAm << " " << currentPart << endl;
			json j = json::parse(&data[bytePos], &data[len]);
			cout << j << endl;
			msgQueue.push(j);
			while (msgQueue.size() > 10) {
				msgQueue.pop();
			}
		} else {
			cout << "Unknown msg type: " << magicByte << endl;
		}
	}

	string OldUDPConnector::getLocalIP() {
		string _localIP = "noLocalIP";
		try {
			asio::io_service netService;
			tcp::resolver resolver(netService);
			tcp::resolver::query query(tcp::v4(), "google.com", "80");
			tcp::resolver::iterator endpoints = resolver.resolve(query);
			tcp::endpoint ep = *endpoints;
			tcp::socket socket(netService);
			socket.connect(ep);
			asio::ip::address addr = socket.local_endpoint().address();
			_localIP = addr.to_string();
		} catch (exception& e) {
			cerr << "Could not deal with socket. Exception: " << e.what() << endl;
		}
		return _localIP;
	}

	// Interact with MM server, perform UDP Hole Punches
	void OldUDPConnector::Update() {
		_updateRunning = true;
		while (_updateRunning) {
			milliseconds currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

			if (connected && currentTime > lastReceivedHB + connectionTimeout) {
				connected = false;
			}

			if (!connected && currentTime>lastRegister + registerInterval) {
				Register();
				lastRegister = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
			}

			if (connected) {
				if (currentTime > lastSentHB + HBInterval) {
					lastSentHB = currentTime;
					Punch();
				}
			}
			this_thread::sleep_for(chrono::milliseconds(50));
		}
	}

	void OldUDPConnector::Register() {
		stringstream ss;
		ss << "d{\"packageType\":\"register\",\"socketID\":\"" << socketID << "\",\"isSender\":" << (string)(isSender ? "true" : "false") << ",\"localIP\":\"" << localIP << "\",\"UID\":\"" << UID << "\"}";
		string json = ss.str();
		cout << "REGISTER" << endl;
		_SendData(json, serverEndpoint_);
	}

	void OldUDPConnector::Punch() {
		string data = "d{\"type\":\"punch\"}";
		_SendData(data, clientEndpoint_);
	}



} } } // oi::core::network
