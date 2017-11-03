#include <iostream>
#include <asio.hpp>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <queue>
#include <string> 
#include "json.hpp"

namespace oi { namespace core { namespace network {
    using asio::ip::udp;
    using asio::ip::tcp;
    using namespace std::chrono;
    using json = nlohmann::json;
    
    class UDPConnector {
        
    public:
        std::queue<json> msgQueue;
        
        UDPConnector(asio::io_service& io_service) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)) {
            asio::socket_base::send_buffer_size option_set(65507);
            socket_.set_option(option_set);
            
        }

        void init(std::string _socketID, std::string _UID, bool _isSender, const std::string& host, const std::string& port) {
            socketID = _socketID;
            isSender = _isSender;
            UID = _UID;

            udp::resolver resolver(io_service_);
            udp::resolver::query query(udp::v4(), host, port);
            udp::resolver::iterator iter = resolver.resolve(query);
            serverEndpoint_ = *iter;

            localIP = getLocalIP();
            _listenThread = std::thread(&UDPConnector::DataListener, this);
            _updateThread = std::thread(&UDPConnector::update, this);
        }
        
        // TODO: Not threadsafe!
        json GetNewData() {
            if (msgQueue.size() < 1) {
                return NULL;
            } else {
                json result = msgQueue.front();
                msgQueue.pop();
                return result;
            }
        }

        void update() {
            _updateRunning = true;
            while (_updateRunning) {
                milliseconds currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

                if (connected && currentTime > lastReceivedHB + connectionTimeout) {
                    connected = false;
                }

                if (!connected && currentTime>lastRegister+registerInterval) {
                    Register();
                    lastRegister = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                }

                if (connected) {
                    if (currentTime > lastSentHB + HBInterval) {
                        lastSentHB = currentTime;
                        Punch();
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        void SendData(unsigned char* data, size_t size) {
            if (connected) {
                socket_.send_to(asio::buffer(data, size), clientEndpoint_);
            }
        }

        void SendData(char* data, size_t size) {
            if (connected) {
                socket_.send_to(asio::buffer(data, size), clientEndpoint_);
            }
        }

        void exit() {
            _listenRunning = false;
            _updateRunning = false;
            
            socket_.close(); // stop current transmissions (otherwise thread doesn't want to end).
            _updateThread.join();
            _listenThread.join();
        }

        std::string getLocalIP() {
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
            } catch (std::exception& e) {
                std::cerr << "Could not deal with socket. Exception: " << e.what() << std::endl;
            }
            return _localIP;
        }

        void Register() {
            std::stringstream ss;
            ss << "d{\"packageType\":\"register\",\"socketID\":\"" << socketID << "\",\"isSender\":" << (std::string)(isSender ? "true" : "false") << ",\"localIP\":\"" << localIP << "\",\"UID\":\"" << UID << "\"}";
            std::string json = ss.str();
            std::cout << "REGISTER" << std::endl;
            _sendData(json, serverEndpoint_);
        }

        void Punch() {
            std::string data = "d{\"type\":\"punch\"}";
            _sendData(data, clientEndpoint_);
        }

    private:
        asio::io_service& io_service_;
        udp::socket socket_;
        udp::endpoint serverEndpoint_;
        udp::endpoint clientEndpoint_;

        std::string localIP;
        std::string socketID;
        std::string UID;
        bool isSender;
        bool connected = false;

        std::thread _listenThread;
        bool _listenRunning = false;
        std::thread _updateThread;
        bool _updateRunning = false;

        milliseconds lastRegister = (milliseconds)0;
        milliseconds registerInterval = (milliseconds)2000;
        milliseconds lastSentHB = (milliseconds)0;
        milliseconds HBInterval = (milliseconds)2000;
        milliseconds lastReceivedHB = (milliseconds)0;
        milliseconds connectionTimeout = (milliseconds)5000;

        void _sendData(const std::string& msg, udp::endpoint endpoint) {
            try {
                socket_.send_to(asio::buffer(msg, msg.size()), endpoint);
            }catch(const std::exception& e){
                std::cout << "exception while sending" << std::endl;
            }
        }

        void DataListener() {
            const std::size_t size = 65507;
            char data[size];
            _listenRunning = true;
            while (_listenRunning) {
                try {
                    std::cout << "receive..." << std::endl;
                    int len = socket_.receive(asio::buffer(data, size));
                    HandleReceivedData(data, len);
                } catch (std::exception& e) {
                    std::cerr << "Exception while receiving: " << e.what() << std::endl;
                }
            }
        }

        void HandleReceivedData(char* data, int len) {
            char magicByte = data[0];

            if (magicByte == 100) {
                json j = json::parse(&data[1], &data[len]);
                std::cout << j << std::endl;
                if (j["type"] == "answer") {
                    std::string host = j.at("address").get<std::string>();
                    std::string port = std::to_string(j.at("port").get<int>());

                    udp::resolver resolver(io_service_);
                    udp::resolver::query query(udp::v4(), host, port);
                    udp::resolver::iterator iter = resolver.resolve(query);
                    clientEndpoint_ = *iter;

                    Punch();
                    Punch();
                }
                else if (j["type"] == "punch") {
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
                std::cout << packageSequenceID << " " << partsAm << " " << currentPart << std::endl;
                json j = json::parse(&data[bytePos], &data[len]);
                std::cout << j << std::endl;
                msgQueue.push(j);
                while (msgQueue.size() > 10) {
                    msgQueue.pop();
                }
            } else {
                std::cout << "Unknown msg type: " << magicByte << std::endl;
            }
        }
    };
} } } // oi::core::network
