// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class HeartbeatFL : public rclcpp::Node
{
public:
  HeartbeatFL()
  : Node("heartbeat_fl"), count_(0)
  {
    timer_param = this->create_wall_timer(1000ms, std::bind(&HeartbeatFL::get_param, this));
    this->declare_parameter<std::uint16_t>("pub_logger_fr_", 1000);
    this->declare_parameter<std::uint16_t>("pub_hb_fr_", 100);
    this->get_parameter("pub_hb_fr_", pub_hb_fr_);
    this->get_parameter("pub_logger_fr_", pub_logger_fr_);
    publisher_ = this->create_publisher<std_msgs::msg::Byte>("forklift1/heartbeat/heartbeat_fl", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(pub_hb_fr_), std::bind(&HeartbeatFL::timer_callback, this));

    std::string subscribtion_names[nbr_subscribtions] = { "forklift1/heartbeat/forklift_controller",
                                                          "forklift1/heartbeat/nodeManager_fl",
                                                          "forklift1/heartbeat/streamManager_fl",
                                                          "remote1/heartbeat/heartbeat_rm"};

    subscribtion_0 = this->create_subscription<std_msgs::msg::Byte>(
      subscribtion_names[0], 10, std::bind(&HeartbeatFL::callback_0, this, _1));
    subscribtion_1 = this->create_subscription<std_msgs::msg::Byte>(
      subscribtion_names[1], 10, std::bind(&HeartbeatFL::callback_1, this, _1));
    subscribtion_2 = this->create_subscription<std_msgs::msg::Byte>(
      subscribtion_names[2], 10, std::bind(&HeartbeatFL::callback_2, this, _1));
    subscribtion_3 = this->create_subscription<std_msgs::msg::Byte>(
      subscribtion_names[3], 10, std::bind(&HeartbeatFL::callback_3, this, _1));
    for(uint8_t i=0;i<sizeof(hb);i++){
      hb[i] = max_time[i] + 1;
    }

      RCLCPP_INFO(this->get_logger(), "Node Start");
    }

private:
  void get_param(){
    this->get_parameter("pub_hb_fr_", pub_hb_fr_);
    this->get_parameter("pub_logger_fr_", pub_logger_fr_);
  }

  void callback_0(const std_msgs::msg::Byte::SharedPtr msg) 
  {
    hb[0] = 0;
    nodeState[0] = static_cast<nodeStates>(msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscribtion_0;

  void callback_1(const std_msgs::msg::Byte::SharedPtr msg) 
  {
    hb[1] = 0;
    nodeState[1] = static_cast<nodeStates>(msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscribtion_1;

  void callback_2(const std_msgs::msg::Byte::SharedPtr msg) 
  {
    hb[2] = 0;
    nodeState[2] = static_cast<nodeStates>(msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscribtion_2;

  void callback_3(const std_msgs::msg::Byte::SharedPtr msg) 
  {
    hb[3] = 0;
    nodeState[3] = static_cast<nodeStates>(msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscribtion_3;

  void timer_callback()
  {
    bool allActive = true;
    bool nodeActive[nbr_subscribtions];
    for(uint8_t i=0;i<sizeof(hb);i++){
      if(active[i] & hb[i]++ > max_time[i]){
        allActive = false;
      }
      else{
        nodeActive[i] = true;
      }
    }
    if(!allActive){ //at least one necessary node is delayed
      state = error;
    }
    else{ // every necessary node is running in time
      if((nodeState[0] == nError && active[0]) || (nodeState[1] == nError && active[1]) || (nodeState[2] == nError && active[2])){
        state = error;
      }
      else if((nodeState[0] == nInactive && active[0]) || (nodeState[1] == nInactive && active[1]) || (nodeState[2] == nInactive && active[2])){
        state = inactive;
      }
      else if((nodeState[0] == nWaiting && active[0]) || (nodeState[1] == nWaiting && active[1]) || (nodeState[2] == nWaiting && active[2])){
        state = waiting_fl;
      }
      else if(nodeState[3] == error || nodeState[3] == inactive || nodeState[3] == waiting_rm){
        state = waiting_rm;
      }
      else{
        state = normal;
      }      
    }
    auto message = std_msgs::msg::Byte();
    message.data = state;
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Node Status FL: '%d'", message.data);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr publisher_;
  size_t count_;
  const static uint8_t nbr_subscribtions = 4;
  const uint8_t max_time[nbr_subscribtions] = {5,5,5,5};
  enum States {error, inactive, waiting_rm, waiting_fl, normal};
  enum nodeStates {nError, nInactive, nWaiting, nNormal};

  uint8_t hb[nbr_subscribtions];
  nodeStates nodeState[nbr_subscribtions];
  
  bool active[nbr_subscribtions] = {false, false, false, true};

  States state = error;

  rclcpp::TimerBase::SharedPtr timer_param;
  std::uint16_t  pub_logger_fr_;
  std::uint16_t  pub_hb_fr_;

  rclcpp::Clock& clk = *this->get_clock();



};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatFL>());
  rclcpp::shutdown();
  return 0;
}
