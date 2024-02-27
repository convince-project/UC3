/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef __CHECKNETWORKDATAMODEL_H_
#define __CHECKNETWORKDATAMODEL_H_

#include <rclcpp/rclcpp.hpp>

#include <QScxmlCppDataModel>
#include <QVariantMap>
#include <QTime>
#include <QTimer>
#include <QDebug>

// Ping utility structs
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/ip_icmp.h>

// Packet struct
constexpr int PING_PKT_S = 64;

struct ping_pkt {
    struct icmphdr hdr;
    char msg[PING_PKT_S - sizeof(struct icmphdr)];
};

class CheckNetworkDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    CheckNetworkDataModel() = default;
    void set_name(std::string name);
    bool close();
    void spin();
    void topic_callback();
    bool start() ;
    bool setup(const QVariantMap& initialDataValues) override;
private: 
    std::string m_name = "CheckNetworkDataModel";
    int m_msg_count = 0;
    bool m_is_connected = true;
    std::shared_ptr<std::thread> m_thread;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string m_address_name;
    std::shared_ptr<rclcpp::Node> m_node;
    
    //host can be either the name of the resource (e.g. r1-base) or its ip addr.
    bool isNetworkConnected(const std::string& host);
    std::string exec(const char* cmd);

    // Calculating the Check Sum
    // TODO: understand what is this
    unsigned short checksum(void* b, int len)
    {
        unsigned short* buf = static_cast<unsigned short*>(b);
        unsigned int sum = 0;
        unsigned short result;
    
        for (sum = 0; len > 1; len -= 2)
            sum += *buf++;
        if (len == 1)
            sum += *(unsigned char*)buf;
        sum = (sum > 16) + (sum & 0xFFFF);
        sum += (sum > 16);
        result = ~sum;
        return result;
    }
};

Q_DECLARE_METATYPE(::CheckNetworkDataModel*)

#endif