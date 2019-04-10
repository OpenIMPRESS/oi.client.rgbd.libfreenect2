#pragma once
#include <asio.hpp>
#include <chrono>
#include <thread>
#include <queue>
#include <string>
#include <iostream>
#include <cstdlib>

#include "json.hpp"
#include "UDPBase.hpp"

namespace oi { namespace core { namespace network {

	class UDPConnector : public UDPBase {
	public:
		UDPConnector(int listenPort, int sendPort, std::string sendHost, asio::io_service& io_service);
		bool Init(std::string sid, std::string guid, bool issender, size_t receive_buffer_size, int receive_containers, size_t send_buffer_size, int send_containers);

	private:
		void Update();
		void Register();
		void Punch();
		void Close();

		void HandleReceivedData(DataContainer * container);
		int _SendDataBlocking(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint);


		std::string get_local_ip();

		std::chrono::milliseconds lastRegister;
		std::chrono::milliseconds registerInterval;
		std::chrono::milliseconds lastSentHB;
		std::chrono::milliseconds HBInterval;
		std::chrono::milliseconds lastReceivedHB;
		std::chrono::milliseconds connectionTimeout;

		std::string localIP;
		std::string socketID;
		std::string guid;
		bool is_sender;
		asio::ip::udp::endpoint remote_endpoint_;

		std::thread * update_thread;
	};

} } } // oi::core::network
