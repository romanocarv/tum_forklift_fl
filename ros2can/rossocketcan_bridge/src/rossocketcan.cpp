#include "rossocketcan.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#define PI 3.14159265

rossocketcan::rossocketcan(std::string can_socket2): Node("ros2" + can_socket2), stream(ios), stream2(ios2), signals(ios, SIGINT, SIGTERM), signals2(ios2, SIGINT, SIGTERM)
{

}

void rossocketcan::Init(const char* can_socket)
{
    printf("Using can socket %s\n", can_socket);
    
    const char* canname = can_socket;
        
    topicname_receive 	<< "CAN/" << canname << "/" << "receive";
    topicname_transmit 	<< "CAN/" << canname << "/" << "transmit";
      
    rclcpp::executors::MultiThreadedExecutor exec;
    
    actualPublisher_		= this->create_publisher<std_msgs::msg::Int32MultiArray>("forklift1/actual/info", 1);
    canSubscription_ 	= this->create_subscription<std_msgs::msg::Int32MultiArray>("forklift1/can/info", 1, std::bind(&rossocketcan::CanCallback, this, _1));

    ///////////////////    

    strcpy(ifr.ifr_name, "can0");
    ioctl(natsock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }

    strcpy(ifr2.ifr_name, "can1");
    ioctl(natsock2, SIOCGIFINDEX, &ifr2);
    
    addr2.can_family  = AF_CAN;
    addr2.can_ifindex = ifr2.ifr_ifindex;
    
    if(bind(natsock2,(struct sockaddr *)&addr2,sizeof(addr2))<0)
    {
        perror("Error in socket bind");
    }


    stream.assign(natsock);
    stream2.assign(natsock2);

    countUp = 0;
    countUp2 = 0;
    countUp3 = 0;
    maxStep = 50;
    gear = 1;
    result_i = 2489;
    result_i2 = 480;
    countUp5 = 0;

    sendHub = false;

    hub = 0;
    hubDirection = 0;
    
    std::cout << "ROS2 to CAN-Bus topic:" << canSubscription_->get_topic_name() 	<< std::endl;
    std::cout << "CAN-Bus to ROS2 topic:" << actualPublisher_->get_topic_name() 	<< std::endl;
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&rossocketcan::CanListener, this,std::ref(rec_frame),std::ref(stream)));

    stream2.async_read_some(boost::asio::buffer(&rec_frame2, sizeof(rec_frame2)),std::bind(&rossocketcan::CanListener2, this,std::ref(rec_frame2),std::ref(stream2)));
    
    signals.async_wait(std::bind(&rossocketcan::stop, this));
    signals2.async_wait(std::bind(&rossocketcan::stop2, this));
    
    boost::system::error_code ec;
    
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    std::thread bt2(std::bind(run, &ios2));
    bt.detach();
    bt2.detach();
    
    rclcpp::spin(shared_from_this());

}

void rossocketcan::CanCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    hub = msg->data[0];
    hubDirection = msg->data[1];
	tiltLever = msg->data[2];
	tiltLeverDirection = msg->data[3];
	sideshifter = msg->data[4];
	sideshifterDirection = msg->data[5];
	forkPositioner = msg->data[6];
	forkPositionerDirection = msg->data[7];
    gear = msg->data[8];
    slowModus = msg->data[9];
    feststellBremse = msg->data[10];
    horn = msg->data[11];
}

void rossocketcan::stop()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios.stop();
    signals.clear();
}

void rossocketcan::stop2()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios2.stop();
    signals2.clear();
}

rossocketcan::~rossocketcan(){printf("\nEnd of Publisher Thread. \n");}

void rossocketcan::CanSend(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;
    
    frame1.can_id = msg.id;
    
    if (msg.eff == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_EFF_FLAG;
    }
    
    if (msg.err == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_ERR_FLAG;
    }
    
    if (msg.rtr == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_RTR_FLAG;
    }

    frame1.can_dlc = msg.dlc;

    for(int i=0;i<(int)frame1.can_dlc;i++)
    {
        frame1.data[i] = msg.data[i];
    }
    
    stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)),std::bind(&rossocketcan::CanSendConfirm, this));
}

void rossocketcan::CanSend2(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;
    
    frame1.can_id = msg.id;
   
    if (msg.eff == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_EFF_FLAG;
    }
    
    if (msg.err == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_ERR_FLAG;
    }
    
    if (msg.rtr == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_RTR_FLAG;
    }
    
    frame1.can_dlc = msg.dlc;

    if(frame1.can_id == 386)
    {
        frame1.data[0] = hub;
        frame1.data[1] = tiltLever;
        frame1.data[2] = sideshifter;
        frame1.data[3] = forkPositioner;
        if(gear == 0)
        {
            frame1.data[4] = 4 + tiltLeverDirection + hubDirection + horn;	
        }
        else if(gear == 1)
        {
            frame1.data[4] = 0 + tiltLeverDirection + hubDirection + horn;
        }
        else
        {
            frame1.data[4] = 2 + tiltLeverDirection + hubDirection + horn;
        }
        frame1.data[5] = sideshifterDirection + forkPositionerDirection;
        frame1.data[6] = 0;
        frame1.data[7] = 0;
    }
	else if(frame1.can_id == 387)
	{
		frame1.data[0] = slowModus;
		frame1.data[1] = feststellBremse;
		frame1.data[2] = 0;
		frame1.data[3] = 0;
		frame1.data[4] = 0;
		frame1.data[5] = 0;
		frame1.data[6] = 0;
		frame1.data[7] = 0;	
	}
	else
	{
        for(int i=0;i<(int)frame1.can_dlc;i++)
        {
            frame1.data[i] = msg.data[i];
        }
    }

	/*for(int i=0;i<(int)frame1.can_dlc;i++)
	{
		frame1.data[i] = msg.data[i];
	}*/
    
    stream2.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)),std::bind(&rossocketcan::CanSendConfirm2, this));
}


void rossocketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg)
{

    can_msgs::msg::Frame msg1;
    msg1.id  = msg->id;
    msg1.dlc = msg->dlc;
    msg1.eff = msg->eff;
    msg1.rtr = msg->rtr;
    msg1.err = msg->err;
    msg1.data= msg->data;
    
    CanSend(msg1);
    
}

void rossocketcan::CanSendConfirm(void)
{
}

void rossocketcan::CanSendConfirm2(void)
{
}

void rossocketcan::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{  
    can_msgs::msg::Frame frame;
    
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
    
    for(int i=0; i<rec_frame.can_dlc; i++)
    {
         frame.data[i]=rec_frame.data[i];
    }	

    CanSend2(frame);
  
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&rossocketcan::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
}

void rossocketcan::CanListener2(struct can_frame& rec_frame2, boost::asio::posix::basic_stream_descriptor<>& stream2)
{
    can_msgs::msg::Frame frame;
    
    std::stringstream s;
    
    frame.id = rec_frame2.can_id; 
    frame.dlc = int(rec_frame2.can_dlc);
    
    for(int i=0; i<rec_frame2.can_dlc; i++)
    {
         frame.data[i]=rec_frame2.data[i];
         s << rec_frame2.data[i];
    }

    if(rec_frame2.can_id == 771)
    {
        auto actualInfoMsg = std_msgs::msg::Int32MultiArray();

	    swheel = (int8_t)rec_frame2.data[3];
	    speed = (int8_t)rec_frame2.data[2];

	    std::vector<int> vec_can_info{(int)swheel, (int)speed, (int)speed_block};
        
        actualInfoMsg.data = vec_can_info;

        actualPublisher_->publish(actualInfoMsg);
    }

    if(rec_frame2.can_id == 387)
    {
	    speed_block = (int8_t)rec_frame2.data[0];
    }

    CanSend(frame);
  
    stream2.async_read_some(boost::asio::buffer(&rec_frame2, sizeof(rec_frame2)),std::bind(&rossocketcan::CanListener2,this, std::ref(rec_frame2),std::ref(stream2)));
    
}

int main(int argc, char *argv[])
{
    std::cout << programdescr << std::endl;
    rclcpp::init(argc, argv);
    
    if (argc < 2)
	{
        auto ros2canptr = std::make_shared<rossocketcan>();
        ros2canptr -> Init();
    }
    else
	{
        auto ros2canptr = std::make_shared<rossocketcan>(argv[1]);
        ros2canptr -> Init(argv[1]);
    }
    
    return 0;
}
