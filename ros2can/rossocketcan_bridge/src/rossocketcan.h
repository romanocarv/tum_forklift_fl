// Licensed to the Apache Software Foundation (ASF) under one
// or more contributor license agreements.  See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  The ASF licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
// 
//   http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.

/** @file rossocketcan.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg
 *  @brief  bidirectional ROS2 to CAN interface with topics and service
 */

#ifndef __ros_socketcan_H__
#define __ros_socketcan_H__

//#include <termios.h> //n
//#include <fstream> //n
//#include <sstream>//n
//#include <chrono>//n
//#include <iostream>//n
//#include <stdio.h>
//#include <stdint.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <string.h>
//#include <net/if.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <sys/ioctl.h>

//#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/asio.hpp>
//#include <boost/asio/io_service.hpp>
//#include <boost/asio/signal_set.hpp>
//#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
//#include "can_msgs/srv/can_request.hpp"

#include "log.h"

const std::string version = "1.00 from: " + std::string(__DATE__) + " " + std::string(__TIME__);
const std::string programdescr = "ROS 2 to CAN-Bus Bridge\nVersion: " + version;

/**
 * @brief The rossocketcan bridge connects a canbus with the ROS2 topic system. 
 * @details A nodes is provided, which provides the bridge from a ROS topic to the CAN bus and from the CAN bus to a ROS topic. The node functions as a bidirectional bridge and provides a service to publish a message and receive the answer with the fitting message id. 
 * 
 */
class rossocketcan : public rclcpp::Node
{
    public:
        /**
         * @brief constructor for rossocketcan class
         * @details Within the constructor the topic and service naming is done. 
         */
        rossocketcan(std::string can_socket2 = "can0");//boost::asio::io_service& ios);
        
        /**
         * @brief Within the Init() fucntin the ROS and CAN setup is done.
         * @details Within the Init() function the ROS2 publisher, subscriber and the service server is initialized. In addition the socketcan interface is configured and assigned to the socket. The Init function is necessary as the topics need a fully constructed node class to be added to.
         */
        void Init(const char* can_socket = "can0");//boost::asio::io_service& ios);
        /**
         * @brief destructor
         */
        ~rossocketcan();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr actualPublisher_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr canSubscription_;
        //rclcpp::service::Service<can_msgs::srv::CanRequest>::SharedPtr server_ros2can_;
        
        can_msgs::msg::Frame current_frame;
        
        /**
         * @brief The CanSendConfirm function is needed by the .async_write_some function and is called as confirmation for a successfull send process.
         */
        void CanSendConfirm();
	void CanSendConfirm2();
        
        /**
         * @brief The CanPublisher is listening to a ROS2 Topic and calls the CanSend Method.
         */
        void CanPublisher(const can_msgs::msg::Frame::SharedPtr msg);
        
        /**
         * @brief The CanSend method sends a ROS message to the CAN bus.
         * @details The CanSend function is Called by the CanPublisher and ther ros2can_srv. It converts the ROS message to a can_frame and adds the CAN Flags to the message ID.  
         */
        void CanSend(const can_msgs::msg::Frame msg);
	void CanSend2(const can_msgs::msg::Frame msg);

	void CanCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        
        /**
         * @brief The CanListener listens to the CAN Bus and publishes the message to a ROS2 Topic.
         * @details The CanListener function is Called by the .async_read_some when a Message is received on the Can Socket. It converts the message to a ROS Message and publishes it to a ROS2 Topic. Afterwards .async_read_some must be called again to wait for further CAN Messages.
         */
        void CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream);
	void CanListener2(struct can_frame& rec_frame2, boost::asio::posix::basic_stream_descriptor<>& stream2);
        
        /**
         * @brief The ros2can_service provides the possibility to send a can message and wait for a specific can message with a give CAN Message ID.
         */
        //void ros2can_srv(
        //const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        //const std::shared_ptr<can_msgs::srv::CanRequest::Request> request,
        //std::shared_ptr<can_msgs::srv::CanRequest::Response> response);
        
        /**
         * @biref The Stop method is needed as the interuped handler must be configered to the asio libary.
         */
        void stop();
	void stop2();
        
        boost::asio::io_service ios;
	boost::asio::io_service ios2;
        boost::asio::posix::basic_stream_descriptor<> stream;
	boost::asio::posix::basic_stream_descriptor<> stream2;
        boost::asio::signal_set signals;
	boost::asio::signal_set signals2;

        struct sockaddr_can addr;
        struct can_frame frame;
        struct can_frame rec_frame;
        struct ifreq ifr;

	struct sockaddr_can addr2;
        struct can_frame frame2;
        struct can_frame rec_frame2;
        struct ifreq ifr2;

	int8_t speed_block;

	int countUp;
	int countUp2;
	int countUp3;
	int countUp5;
	int wink;
	int16_t result_i;
	int16_t result_i2;
	int16_t canResultHB;
	int16_t canResultLB;
	int16_t canResultHBold;
	int16_t canResultLBold;
	int16_t canResult2HB;
	int16_t canResult2LB;
	int16_t actSin;
	int16_t actCos;
	int16_t targetSin;
	int16_t targetCos;
	int16_t nextSin;
	int16_t nextCos;
	int16_t maxStep;
	int16_t gear;
	int16_t swheel;
	int16_t speed;

	bool sendHub;
	bool sinFrame;

        int16_t tiltLever;
        int16_t tiltLeverDirection;

        int16_t sideshifter;
        int16_t sideshifterDirection;

        int16_t forkPositioner;
        int16_t forkPositionerDirection;

	int16_t hub;
	int16_t hubDirection;

	int16_t slowModus;
	int16_t feststellBremse;

	int16_t horn;

	bool huppeOn;

	can_msgs::msg::Frame frameLenk;
        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	int natsock2 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        std::stringstream topicname_receive;
        std::stringstream topicname_transmit;
        std::stringstream servername;
};
#endif
