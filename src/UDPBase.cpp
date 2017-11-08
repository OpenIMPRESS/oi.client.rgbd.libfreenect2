#include "UDPBase.hpp"

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
		this->_connected = true;
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
		send_thread = new std::thread(&UDPBase::DataSender, this);
		return true;
	}


	int UDPBase::SendData(unsigned char* data, size_t size) {
		return SendData(data, size, endpoint_);
	}

	int UDPBase::SendData(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint) {
		DataContainer * dc;
		if (!GetFreeWriteContainer(&dc)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		memcpy(&(dc->dataBuffer[0]), data, size);
		dc->_data_end = size;
		QueueForSending(&dc, endpoint);
		return size;
	}

	int UDPBase::SendDataBlocking(unsigned char* data, size_t size) {
		return SendDataBlocking(data, size, endpoint_);
	}

	int UDPBase::SendDataBlocking(unsigned char* data, size_t size, udp::endpoint ep) {
		std::unique_lock<std::mutex> lk(send_mutex);
		return _SendDataBlocking(data, size, ep);
	}

	int UDPBase::_SendDataBlocking(unsigned char* data, size_t size, udp::endpoint ep) {
		try {
			return socket_.send_to(asio::buffer(data, size), ep);
		} catch (std::exception& e) {
			std::cerr << "\nERROR: Send Exception: " << e.what() << "\n";
			return -1;
		}
	}

	void UDPBase::Close() {
		running = false;
		socket_.close();
		listen_thread->join();
		std::unique_lock<std::mutex> lk(send_mutex);
		send_cv.notify_all();
		send_cv.notify_one();
		lk.unlock();
		send_thread->join();

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
		//std::unique_lock<std::mutex> lk(send_mutex);
		if (queued_send.empty()) return false;
		*container = queued_send.front();
		queued_send.pop();
		return true;
	}

	void UDPBase::ReleaseForWriting(DataContainer ** container) {
		if (*container == NULL) return;
		//std::unique_lock<std::mutex> lk(send_mutex);
		unused_send.push(*container);
		*container = NULL;
	}

	bool UDPBase::connected() {
		return _connected;
	}

	void UDPBase::DataSender() {
		running = true;
		DataContainer * send_data_container;
		while (running) {
			std::unique_lock<std::mutex> lk(send_mutex);
			while (running && queued_send.empty()) send_cv.wait(lk);

			if (!running || !DequeueForSending(&send_data_container)) continue;

			_SendDataBlocking(send_data_container->dataBuffer, send_data_container->data_end(), send_data_container->endpoint());

			ReleaseForWriting(&send_data_container);
		}
	}


	// Method for abstractions to overwrite
	void UDPBase::HandleReceivedData(DataContainer * container) {
		QueueForReading(&container);
	}

	void UDPBase::DataListener() {
		running = true;
		DataContainer * work_container;
		while (running) {
			if (!GetFreeReceiveContainer(&work_container)) {
				std::cerr << "\nERROR: no unused DataContainer to load data into." << std::endl;
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
					HandleReceivedData(work_container);
				} else {
				}
			} catch (exception& e) {
				cerr << "Exception while receiving (Code " << ec << "): " << e.what() << endl;
				this_thread::sleep_for(1s);
			}

			ReleaseForReceiving(&work_container);
		}
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

} } } // oi::core::network
