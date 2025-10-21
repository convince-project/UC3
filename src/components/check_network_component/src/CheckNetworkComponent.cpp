/******************************************************************************
 *                                                                            *
 * Copyright (C) 2024 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "CheckNetworkComponent.h"


#include <thread>

// Includes to perform ping
#include <iostream>
#include <regex>
#include <chrono>

using namespace std::chrono_literals;


void CheckNetworkComponent::set_name(std::string name)
{
    m_name=name;
}

bool CheckNetworkComponent::setup(int argc, char* argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(argc , argv);
    }

    m_node = rclcpp::Node::make_shared(m_name);
    m_address_name = "192.168.100.103";
    m_publisherStatus = m_node->create_publisher<std_msgs::msg::Bool>("/CheckNetworkComponent/NetworkStatus", 10);
    // timer_ = m_node->create_wall_timer(1000ms, std::bind(&CheckNetworkComponent::topic_callback, this));
    m_threadStatus = std::make_shared<std::thread>(&CheckNetworkComponent::threadConnected, this);
    m_publisherNetworkChanged = m_node->create_publisher<std_msgs::msg::Bool>("/CheckNetworkComponent/NetworkChanged", 10);
    timer_2 = m_node->create_wall_timer(1000ms, std::bind(&CheckNetworkComponent::StatusChangedPublisher, this));

    RCLCPP_DEBUG(m_node->get_logger(), "CheckNetworkComponent::start");
    // m_thread = std::make_shared<std::thread>([this]{ start();});
    return true;
}


void CheckNetworkComponent::threadConnected() {
    while(m_threadActive) {
        bool networkIsCurrentlyUp=true;
        m_is_connected = isNetworkConnected(m_address_name);
	    RCLCPP_DEBUG(m_node->get_logger(), "m_is_connected %d m_lastStatus.size() %d", m_is_connected, m_lastStatus.size());
        if (m_lastStatus.size() < 5) {
            m_changed = false;
            m_lastStatus.push_back(m_is_connected);
        } else {
            int countFalse = 0;
            for (auto status : m_lastStatus) {
                RCLCPP_DEBUG(m_node->get_logger(), "status %d", status);
                if (status == false) {
                    countFalse++;
                }
            }
            if (countFalse > 3) {
                networkIsCurrentlyUp = false;
            }
            m_lastStatus.clear();
            RCLCPP_DEBUG(m_node->get_logger(), "m_previousStatusConnected %d networkIsCurrentlyUp %d", m_previousStatusConnected, networkIsCurrentlyUp);
            if (m_previousStatusConnected != networkIsCurrentlyUp) {
                m_changed = true;
            } else {
                m_changed = false;
	    }
            m_previousStatusConnected = networkIsCurrentlyUp;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

bool CheckNetworkComponent::close()
{
    rclcpp::shutdown();
    m_threadActive = false;
    if (m_threadStatus != nullptr && m_threadStatus->joinable()) {
        m_threadStatus->join();
    }
    return true;
}

void CheckNetworkComponent::spin()
{
    rclcpp::spin(m_node);
}

bool CheckNetworkComponent::start() {
    RCLCPP_DEBUG(m_node->get_logger(), "Before spinning");
    spin();
    RCLCPP_DEBUG(m_node->get_logger(), "Finished spinning");
    close();
    return true;
}

void CheckNetworkComponent::StatusChangedPublisher() {
    auto msg = std_msgs::msg::Bool();
    msg.data = m_changed;
    m_publisherNetworkChanged->publish(msg);
    auto msgConnected  = std_msgs::msg::Bool();
    msgConnected.data = m_is_connected;
    m_publisherStatus->publish(msgConnected);
}



bool CheckNetworkComponent::isNetworkConnected(const std::string& host) {

    double threshold = 3000.;
    bool is_connected = false;
    const std::string ping_command = "ping -c 4 -w 1 -q " + host + " 2>&1";  // Ping 4 times
    std::string ping_output = exec(ping_command.c_str());
    //std::cout << "ping_output\n" << ping_output << std::endl;

    if(ping_output.find("Temporary failure in name resolution") != std::string::npos  ||
        ping_output.find("Name or service not known") != std::string::npos ){
        RCLCPP_DEBUG(m_node->get_logger(), "Internet connection absent");
        return false;
    }

    auto ping_output_stream = std::stringstream{ping_output};
    std::string line;

    std::string packets_summary_line;
    std::string rtt_summary_line;

    std::vector<std::string> ping_output_lines;
    while(std::getline(ping_output_stream,line,'\n'))
    {
        ping_output_lines.push_back(line);
    }

    if(ping_output_lines.size() >= 3){
        packets_summary_line = ping_output_lines[3];

    }
    if(ping_output_lines.size() >= 4){
        rtt_summary_line = ping_output_lines[4];
    }

    std::cout << packets_summary_line << std::endl;
    std::cout << rtt_summary_line << std::endl;

    std::regex rgx_packets(" (\\d*)\\% packet loss");
    std::regex rgx_ttl("= \\d*\\.\\d*\\/(\\d*[.]\\d*)\\/");
    std::smatch match_packets;
    std::smatch match_ttl;

    if(std::regex_search(packets_summary_line, match_packets, rgx_packets))
        RCLCPP_DEBUG(m_node->get_logger(), "match match_packets: %d", match_packets[1]);

    double rtt = 0.0;
    bool match_ttl_found = std::regex_search(rtt_summary_line,match_ttl,rgx_ttl);
    if (match_ttl_found){

	rtt = stod(match_ttl[1]);
        RCLCPP_DEBUG(m_node->get_logger(), "match match_ttl: %d", match_ttl[1]);
    }
    RCLCPP_DEBUG(m_node->get_logger(), "found %d match: %d", match_ttl_found, match_ttl[1]);
    double packet_loss = stod(match_packets[1]);

    if(packet_loss < 100.){
	if (!match_ttl_found) {
	    is_connected = false;
	} else {
        if(rtt < threshold)
            is_connected = true;
        else
            is_connected = false;
	}
    }

    if(is_connected){
        std::cout << "is connected!" << std::endl;
    }

    return is_connected;

}

std::string CheckNetworkComponent::exec(const char* cmd) {

    std::array<char, 128> buffer;
    std::string result; //The ping output string
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// bool CheckNetworkComponent::isNetworkConnected(const std::string& host) {

//     struct addrinfo hints, *res;
//     struct sockaddr_in r_addr;
//     socklen_t r_addr_len;
//     int sockfd;
//     bool is_connected = false;

//     // Setup hints
//     memset(&hints, 0, sizeof(hints));
//     hints.ai_family = AF_INET;
//     hints.ai_socktype = SOCK_RAW;
//     hints.ai_protocol = IPPROTO_ICMP;

//     // Get address info -> this will take care of the dns lookup if needed
//     if (getaddrinfo(host.c_str(), NULL, &hints, &res) != 0) {
//         return false;
//     }

//     // Create socket
//     // sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
//     sockfd = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
//     if (sockfd == -1) {
//         std::cout << "Failed to create socket" << std::endl;
//         freeaddrinfo(res);
//         return false;
//     }

//     // Build a packet to send to host
//     struct ping_pkt pckt;
//     memset(&pckt,0,sizeof(pckt));
//     pckt.hdr.type = ICMP_ECHO;
//     pckt.hdr.un.echo.id = getpid();
//     for (std::size_t i = 0; i<sizeof(pckt.msg) - 1; i++)
//         pckt.msg[i] = i + '0';

//     pckt.msg[sizeof(pckt.msg)-1] = 0;
//     pckt.hdr.un.echo.sequence = m_msg_count++;
//     pckt.hdr.checksum = checksum(& pckt, sizeof(pckt));

//     // Send a packet to the host
//     int bytes_sent = sendto(sockfd,&pckt,sizeof(pckt),0,res->ai_addr,res->ai_addrlen);
//     if(bytes_sent == -1) {
//         std::cout << "Failed to send packet address" << std::endl;
//         freeaddrinfo(res);
//         return false;
//     }

//     // Receive a packet
//     char rbuffer[128];
//     int bytes_received = recvfrom(sockfd,rbuffer,sizeof(rbuffer),0,
//                                     (struct sockaddr*) &r_addr, &r_addr_len);
//     if(bytes_received == -1) {
//         std::cout << "Failed to receive packets" << std::endl;
//         is_connected = false;
//     }

//     if(is_connected){
//         std::cout << "is connected!" << std::endl;
//     }

//     freeaddrinfo(res);

//     return is_connected;
// }

