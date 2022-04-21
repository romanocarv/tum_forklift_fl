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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "gst_pipeline.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class streamManager_fl_cl : public rclcpp::Node
{
public:
  streamManager_fl_cl()
  : Node("streamManager_fl"), count_(0)
  {
    timer_param = this->create_wall_timer(1000ms, std::bind(&streamManager_fl_cl::get_param, this));
    this->declare_parameter<std::uint16_t>("pub_logger_fr_", 1000);
    this->declare_parameter<std::uint16_t>("pub_hb_fr_", 100);
    this->declare_parameter<std::uint16_t>("pub_streamState_fr_", 100);
    this->declare_parameter<std::uint16_t>("streamMan_fr_", 100);
    this->declare_parameter<std::uint16_t>("port_stream_front_A1_", 5400);
    this->declare_parameter<std::uint16_t>("port_stream_left_B2_", 5300);
    this->declare_parameter<std::uint16_t>("port_stream_right_C3_", 5500);
    this->declare_parameter<std::uint16_t>("port_stream_front_small_D4_", 5200);
    this->declare_parameter<std::uint16_t>("port_stream_back_E5_", 5400);
    this->declare_parameter<std::uint16_t>("port_stream_back_small_F6_", 5200);
    this->declare_parameter<std::uint16_t>("port_stream_Fork_G7_", 5200);
    this->declare_parameter<std::string>("ip_fl1_client_", "192.168.178.64");

    this->get_parameter("pub_hb_fr_", pub_hb_fr_);
    this->get_parameter("pub_streamState_fr_", pub_streamState_fr_);
    this->get_parameter("streamMan_fr_", streamMan_fr_);

    this->get_parameter("pub_logger_fr_", pub_logger_fr_);
    this->get_parameter("port_stream_front_A1_", port_stream_front_A1_);
    this->get_parameter("port_stream_left_B2_", port_stream_left_B2_);
    this->get_parameter("port_stream_right_C3_", port_stream_right_C3_);
    this->get_parameter("port_stream_front_small_D4_", port_stream_front_small_D4_);
    this->get_parameter("port_stream_back_E5_", port_stream_back_E5_);
    this->get_parameter("port_stream_back_small_F6_", port_stream_back_small_F6_);
    this->get_parameter("port_stream_Fork_G7_", port_stream_Fork_G7_);
    this->get_parameter("ip_fl1_client_", ip_fl1_client_);
    c_ip_fl1_client_ = ip_fl1_client_.c_str();

    subscr_streamSet = this->create_subscription<std_msgs::msg::ByteMultiArray>("remote1/stream/streamSet", 10, std::bind(&streamManager_fl_cl::streamSet_callback, this, _1));
    subscr_hb_gui = this->create_subscription<std_msgs::msg::Byte>("remote1/heartbeat/gui_rm", 10, std::bind(&streamManager_fl_cl::cb_hb_sub, this, _1));

    publisher_stream_state = this->create_publisher<std_msgs::msg::String>("forklift1/stream/streamState", 10);
    publisher_heartbeat = this->create_publisher<std_msgs::msg::Byte>("forklift1/heartbeat/streamManager_fl", 10);

    timer_pub_heartbeat = this->create_wall_timer(std::chrono::milliseconds(pub_hb_fr_), std::bind(&streamManager_fl_cl::timer_cb_pub_heartbeat, this));
    timer_pub_streamState = this->create_wall_timer(std::chrono::milliseconds(pub_streamState_fr_), std::bind(&streamManager_fl_cl::timer_cb_pub_streamState, this));

    timer_streamMan = this->create_wall_timer(std::chrono::milliseconds(streamMan_fr_), std::bind(&streamManager_fl_cl::cb_timer_streamMan, this));


    RCLCPP_INFO(this->get_logger(), "Create Streaming Pipelines");


    message_heartbeat.data = 1; // set initial node state

    //ToDo: alle Streams hier anlegen
    
    stream_front_A1->create_pipeline(1);
    stream_front_A1->start_pipeline();
    stream_front_A1->pause_pipeline();

    stream_left_B2->create_pipeline(2);
    stream_left_B2->start_pipeline();
    stream_left_B2->pause_pipeline();

    stream_right_C3->create_pipeline(3);
    stream_right_C3->start_pipeline();
    stream_right_C3->pause_pipeline();
    //stream_front_small_D4->create_pipeline(6);
    
    stream_back_E5->create_pipeline(4);
    stream_back_E5->start_pipeline();
    stream_back_E5->pause_pipeline();
    //stream_back_small_F6->create_pipeline(7);
    
    stream_Fork_G7->create_pipeline(5);
    stream_Fork_G7->start_pipeline();
    stream_Fork_G7->pause_pipeline();

    //std::this_thread::sleep_for(std::chrono::milliseconds(500));

    
    
    
    ////stream_back_E5->start_pipeline();
    ////stream_Fork_G7->start_pipeline();


    
    
    
    //stream_front_small_D4->pause_pipeline();
    
    //stream_back_small_F6->pause_pipeline();
    
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    //test_pipe->create_pipeline(1); // webcam
    //test_pipe->start_pipeline();

    //test_pipe1->create_pipeline(4); // webcam
    //test_pipe1->start_pipeline();
    
    RCLCPP_INFO(this->get_logger(), "Stream Start GST");
  
  }

private:

  void get_param(){
    // no param for runtime needed
    //this->get_parameter("my_parameter", parameter_string_);
    //this->get_parameter("pub_hb_fr_", pub_hb_fr_);
    //RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    this->get_parameter("pub_hb_fr_", pub_hb_fr_);
    this->get_parameter("pub_streamState_fr_", pub_streamState_fr_);
    this->get_parameter("streamMan_fr_", streamMan_fr_);

    this->get_parameter("pub_logger_fr_", pub_logger_fr_);
    this->get_parameter("port_stream_front_A1_", port_stream_front_A1_);
    this->get_parameter("port_stream_left_B2_", port_stream_left_B2_);
    this->get_parameter("port_stream_right_C3_", port_stream_right_C3_);
    this->get_parameter("port_stream_front_small_D4_", port_stream_front_small_D4_);
    this->get_parameter("port_stream_back_E5_", port_stream_back_E5_);
    this->get_parameter("port_stream_back_small_F6_", port_stream_back_small_F6_);
    this->get_parameter("port_stream_Fork_G7_", port_stream_Fork_G7_);
    this->get_parameter("ip_fl1_client_", ip_fl1_client_);
    c_ip_fl1_client_ = ip_fl1_client_.c_str();

  }

  //calbacks publisher
  void timer_cb_pub_heartbeat()
  {
    //publisher_streamSet
    
    //RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "HB: '%d'", message_heartbeat.data);
    publisher_heartbeat->publish(message_heartbeat);
  }
  void timer_cb_pub_streamState() //delete -> mit heartbeat
  {
    //message_streamState.data = "Stream active";
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_streamState.data.c_str());
    //publisher_stream_state->publish(message_streamState);
  }
  //callbacks subscriber
  //je erhaltene Nachricht Vairable im Typ der Sensor msg anlegen, die dann verfuegbar ist fuer berechnung
  //Umsetzung nicht mit if els  rclcpp::TimerBase::SharedPtr timer_param;

  void cb_hb_sub(const std_msgs::msg::Byte::SharedPtr msg)
  {
    message_hb_gui_sub.data = msg->data;
    timer_hb_gui = 0;
  }

  void streamSet_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) 
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "stream set callback");
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Debug mode");
    //message_heartbeat.data = 3;
    //streamSet.data[2] = 1;
    //streamSet.data[1] = 2;
    //streamSet.data[3] = 3;
    //streamSet.data[0] = 7;
    //RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "streamSet: '%s'", streamSet.data);
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());


    if (!(message_heartbeat.data == 3))
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Stream error");
      return;
    }

    streamSet.data = msg->data;

    switch (streamSet.data[2]) //0->2
    {
      case 1:
      //pause E5 (Back)
      stream_back_E5->pause_pipeline();
      //start A1 (Front)
      stream_front_A1->start_pipeline();
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Stream front (A1)");
      break;
      
      
      case 5:
      //pause A1 (Front)
      stream_front_A1->pause_pipeline();
      //start E5 (Back)
      stream_back_E5->start_pipeline();
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Stream back (E5)");
      break;
      

      //default
    }

    switch (streamSet.data[0]) //3 -> 0
    {
      case 4:
      //pause F6 & G7
      ////stream_back_small_F6->pause_pipeline();
      stream_Fork_G7->pause_pipeline();
      //start D4
      ////stream_front_small_D4->start_pipeline();
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "front small (D4)");
      break;

      case 6:
      //pause D4 & G7
      ////stream_front_small_D4->pause_pipeline();
      stream_Fork_G7->pause_pipeline();
      //start F6
      ////stream_back_small_F6->start_pipeline();
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "back small (F6)");
      break;

      case 7:
      //pause F6 & D4 
      ////stream_back_small_F6->pause_pipeline();
      ////stream_front_small_D4->pause_pipeline();
      //start G7 
      stream_Fork_G7->start_pipeline();
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Fork (G7)");
      break;

      case 0:
      //pause all
      ////stream_front_small_D4->pause_pipeline();
      ////stream_back_small_F6->pause_pipeline();
      stream_Fork_G7->pause_pipeline();
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Pause top window");
      break;

    }

    if (streamSet.data[1] == 2 && streamSet.data[3] == 3){
      //start B2 und C3
      stream_left_B2->start_pipeline();
      stream_right_C3->start_pipeline();
    }
    else 
    {
      //pause B2 und C3
      ///stream_left_B2->pause_pipeline();
      ///stream_right_C3->pause_pipeline();
    }


    //alt
    /*
    if (streamSet.data == vec_streamSet_front && message_heartbeat.data == 3)
    {
      RCLCPP_INFO(this->get_logger(), "Start Stream Front");
      test_pipe1->pause_pipeline();
      test_pipe->start_pipeline();
      //Kameras Front Setting starten, alle anderen Pausieren
    }
    else if (streamSet.data == vec_streamSet_back && message_heartbeat.data == 3)
    {
      RCLCPP_INFO(this->get_logger(), "Start Stream Back");
      test_pipe->pause_pipeline();
      test_pipe1->start_pipeline();
      //Kameras Back Setting starten, alle anderen Pausieren
    }
    else if (streamSet.data == vec_streamSet_stop && message_heartbeat.data == 3)
    {
      RCLCPP_INFO(this->get_logger(), "Stop Stream");
      test_pipe->pause_pipeline();
      test_pipe1->pause_pipeline();
      //alle Kameras stoppen
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Stream error");
    }
    */
  }



  //auch bei hb_fl alle states verwenden
  void cb_timer_streamMan()
  {

    std::uint8_t stream_man_case = 0;

    timer_hb_gui ++;

    //msg: hb gui
    //RCLCPP_INFO(this->get_logger(), "received hb '%d'", message_hb_gui_sub.data);

    // cl == 0 && fl == 0 -> hb = waiting
    if(((message_hb_gui_sub.data == 0) || (message_hb_gui_sub.data == 1) || (message_hb_gui_sub.data == 2)) && 
      ((message_heartbeat.data == 0) || (message_heartbeat.data == 1) || (message_heartbeat.data == 2)))
    {
      stream_man_case = 1;
    }
    // cl == 1 && fl == 0 -> FL an -> hb danach auf active
    else if ((message_hb_gui_sub.data == 3) && 
      ((message_heartbeat.data == 0) || (message_heartbeat.data == 1) || (message_heartbeat.data == 2)))
    {
      stream_man_case = 2;
    }
    // cl == 0 && fl == 1 -> FL aus (delete pipeline?) -> hb = waiting
    else if (((message_hb_gui_sub.data == 0) || (message_hb_gui_sub.data == 1) || (message_hb_gui_sub.data == 2)) && (message_heartbeat.data == 3))
    {
      stream_man_case = 3;
    }
    // cl == 1 && fl == 1 -> passt -> continue -> hb = normal
    else if ((message_hb_gui_sub.data == 3) && (message_heartbeat.data == 3))
    {
      stream_man_case = 4;
    } 

    //ToDo: Logik mit Timer überprüfen
    if (timer_hb_gui > 4 && (message_heartbeat.data == 3))
    {
      stream_man_case = 3;
    }
    else if (timer_hb_gui > 4 && 
      ((message_heartbeat.data == 0) || (message_heartbeat.data == 1) || (message_heartbeat.data == 2)))
    {
      stream_man_case = 1;
    }



    switch(stream_man_case)
    {
      case 1: // cl == 0 && fl == 0 -> hb = waiting
          RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Wait on Gui Start");
          message_heartbeat.data = 2;
      break;
      case 2: // cl == 1 && fl == 0 -> FL an -> hb danach auf active
          RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Start FL-Stream");

          // -> setze hb auf 3 -> Pipelines können starten


          //test_pipe->create_pipeline(1); // webcam
          // ToDo: gesamte Pipeline hier starten
          ////test_pipe->start_pipeline();

          // Steuerung allein mit streamSet, hier nur hb setzen für streamset
          //test_pipe1->create_pipeline(3); // webcam2
          //test_pipe1->start_pipeline();

          message_heartbeat.data = 3;
      break;
      case 3: // cl == 0 && fl == 1 -> FL aus (delete pipeline?) -> hb = waiting
          RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "Wait on Gui Start, FL switched off");

          //pause / destruct pipeline
          //test_pipe->pause_pipeline(); // durch destruktor ersetzen?
          //test_pipe1->pause_pipeline();

          //stream_front_A1->pause_pipeline();
          //stream_left_B2->pause_pipeline();
          //stream_right_C3->pause_pipeline();
          //stream_front_small_D4->pause_pipeline();
          //stream_back_E5->kill_pipeline();
          //stream_back_small_F6->pause_pipeline();
          //stream_Fork_G7->pause_pipeline();

          message_heartbeat.data = 2; 
      break;
      case 4: // cl == 1 && fl == 1 -> passt -> continue -> hb = normal
          RCLCPP_INFO_THROTTLE(this->get_logger(), clk, pub_logger_fr_, "FL Stream ok");
          message_heartbeat.data = 3;
      break;
    }




  }


  std::uint8_t timer_hb_gui = 0;

  Gst_pipeline *stream_front_A1 = new Gst_pipeline(5400, "192.168.178.61", 926, 526);//5400
  Gst_pipeline *stream_left_B2 = new Gst_pipeline(5300, "192.168.178.61", 463, 263);//5300
  Gst_pipeline *stream_right_C3 = new Gst_pipeline(5500, "192.168.178.61", 463, 263);//5500
  //Gst_pipeline *stream_front_small_D4 = new Gst_pipeline(5200, "192.168.178.67", 1246, 706);
  Gst_pipeline *stream_back_E5 = new Gst_pipeline(5400, "192.168.178.61", 926, 526);//5400
  //Gst_pipeline *stream_back_small_F6 = new Gst_pipeline(5200, "192.168.178.67", 0, 0);
  Gst_pipeline *stream_Fork_G7 = new Gst_pipeline(5200, "192.168.178.61", 926, 263);//5200

  std::vector<uint8_t> vec_streamSet_front{1, 1, 1, 0, 0};
  std::vector<uint8_t> vec_streamSet_back{0, 0, 1, 1, 1};
  std::vector<uint8_t> vec_streamSet_stop{0, 0, 0, 0, 0};

  
  std_msgs::msg::Byte message_heartbeat = std_msgs::msg::Byte();
  std_msgs::msg::Byte message_hb_gui_sub = std_msgs::msg::Byte();
  std_msgs::msg::ByteMultiArray streamSet = std_msgs::msg::ByteMultiArray();
  std_msgs::msg::ByteMultiArray message_streamSet = std_msgs::msg::ByteMultiArray();
  std_msgs::msg::String message_streamState = std_msgs::msg::String();

  rclcpp::TimerBase::SharedPtr timer_pub_streamState;
  rclcpp::TimerBase::SharedPtr timer_pub_heartbeat;
  rclcpp::TimerBase::SharedPtr timer_streamMan;
  
  
  
  rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscr_hb_gui;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscr_streamSet;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_stream_state;
  rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr publisher_heartbeat;

  //Paramter
  std::uint16_t  pub_logger_fr_;
  std::uint16_t  pub_hb_fr_;
  std::uint16_t  pub_streamState_fr_;
  std::uint16_t  streamMan_fr_;
  std::uint16_t  port_stream_front_A1_;
  std::uint16_t  port_stream_left_B2_;
  std::uint16_t  port_stream_right_C3_;
  std::uint16_t  port_stream_front_small_D4_;
  std::uint16_t  port_stream_back_E5_;
  std::uint16_t  port_stream_back_small_F6_;
  std::uint16_t  port_stream_Fork_G7_;
  std::string    ip_fl1_client_;
  const char *   c_ip_fl1_client_;

  rclcpp::TimerBase::SharedPtr timer_param;


  rclcpp::Clock& clk = *this->get_clock();
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  gst_init(&argc, &argv);
  rclcpp::spin(std::make_shared<streamManager_fl_cl>());
  rclcpp::shutdown();
  return 0;
}
