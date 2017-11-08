#include "UDPConnector.hpp"

namespace oi { namespace core { namespace network {
    using asio::ip::udp;
	using asio::ip::tcp;
	using json = nlohmann::json;
	using namespace std;
    using namespace chrono;

	UDPConnector::UDPConnector(int listenPort, int sendPort, std::string sendHost, asio::io_service& io_service) : UDPBase(listenPort, sendPort, sendHost, io_service) {
	}

	bool UDPConnector::Init(std::string sid, std::string guid, bool is_sender,
		size_t receive_buffer_size, int receive_containers, size_t send_buffer_size, int send_containers) {
		UDPBase::Init(receive_buffer_size, receive_containers, send_buffer_size, send_containers);
		remote_endpoint_ = endpoint_;
		endpoint_ = asio::ip::udp::endpoint();
		registerInterval = (milliseconds)2000;
		HBInterval = (milliseconds)2000;
		connectionTimeout = (milliseconds)5000;
		lastReceivedHB = (milliseconds)0;
		lastRegister = (milliseconds)0;
		lastSentHB = (milliseconds)0;
		_connected = false;

		this->socketID = sid;
		this->guid = guid;
		this->is_sender = is_sender;

		localIP = get_local_ip();
		update_thread = new std::thread(&UDPConnector::Update, this);

		return true;
	}

	int UDPConnector::_SendDataBlocking(unsigned char* data, size_t size, udp::endpoint ep) {
		if (!_connected && ep != remote_endpoint_) return 0;
		try {
			return socket_.send_to(asio::buffer(data, size), ep);
		} catch (std::exception& e) {
			std::cerr << "\nERROR: Send Exception: " << e.what() << "\n";
			return -1;
		}
	}

	void UDPConnector::Update() {
		running = true;
		while (running) {
			milliseconds currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

			if (_connected && currentTime > lastReceivedHB + connectionTimeout) {
				_connected = false;
			}

			if (!_connected && currentTime>lastRegister + registerInterval) {
				Register();
				lastRegister = currentTime;
			}

			if (_connected) {
				if (currentTime > lastSentHB + HBInterval) {
					lastSentHB = currentTime;
					Punch();
				}
			}

			this_thread::sleep_for(chrono::milliseconds(50));
		}
	}

	void UDPConnector::Close() {
		UDPBase::Close();
		update_thread->join();
	}

	void UDPConnector::Register() {
		stringstream ss;
		ss << "d{\"packageType\":\"register\",\"socketID\":\"" << socketID << "\",\"isSender\":" << (string)(is_sender ? "true" : "false") << ",\"localIP\":\"" << localIP << "\",\"UID\":\"" << guid << "\"}";
		std::string json = ss.str();
		SendData((unsigned char *) json.c_str(), json.length(), remote_endpoint_);
	}

	void UDPConnector::Punch() {
		std::string data = "d{\"type\":\"punch\"}";
		SendData((unsigned char *) data.c_str(), data.length(), endpoint_);
	}

	std::string UDPConnector::get_local_ip() {
		std::string _localIP = "noLocalIP";
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

	void UDPConnector::HandleReceivedData(DataContainer * container) {
		unsigned char * data = &(container->dataBuffer[0]);
		size_t len = container->data_end();

		char magicByte = data[0];

		if (magicByte == 100) {
			json j = json::parse(&data[1], &data[len]);
			cout << endl << j << endl;
			if (j["type"] == "answer") {
				string host = j.at("address").get<string>();
				string port = to_string(j.at("port").get<int>());

				udp::resolver resolver(io_service_);
				udp::resolver::query query(udp::v4(), host, port);
				udp::resolver::iterator iter = resolver.resolve(query);
				endpoint_ = *iter;

				Punch();
				Punch();
			} else if (j["type"] == "punch") {
				lastReceivedHB = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
				_connected = true;
			}

			return ReleaseForReceiving(&container);
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

			QueueForReading(&container);
		} else {
			std::cerr << "\nERROR: Unknown msg type: " << magicByte << endl;
		}

		ReleaseForReceiving(&container);
	}




} } } // oi::core::network
