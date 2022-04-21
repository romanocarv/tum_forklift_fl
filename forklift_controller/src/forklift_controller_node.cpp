#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

//#include <linux/can/raw.h>
//#include <boost/asio.hpp>

#define PI 3.14159265

using namespace std::chrono_literals;
using std::placeholders::_1;

class ForkliftController : public rclcpp::Node
{
    public:
        ForkliftController()
        : Node("forklift_controller")
        {
            //Publishers, Subscribers and Main Function Timer Callback
            timer_param = this->create_wall_timer(1000ms, std::bind(&ForkliftController::get_param, this));
            this->declare_parameter<float>("maxSpeed", 1000);
            this->declare_parameter<float>("maxAngle", 90);
            this->declare_parameter<float>("maxLift", 255);

            this->declare_parameter<std::uint16_t>("pub_hb_fr_", 100);
            this->declare_parameter<std::uint16_t>("pub_logger_fr_", 100);

            publisher_heartbeat = this->create_publisher<std_msgs::msg::Byte>("remote1/heartbeat/forklift_controller", 10);
            modePublisher_ = this->create_publisher<std_msgs::msg::Byte>("forklift1/stream/drivingMode", 10);
            infoPublisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("forklift1/gui/info", 10);
            allInfoPublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("forklift1/info", 10);
	        autoCenter_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("forklift1/steeringwheel/auto_center", 10);
	        canInfoPublisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("forklift1/can/info", 1);
	        gasPedalPublisher_ = this->create_publisher<std_msgs::msg::Int16>("forklift1/target/throttle", 1);
            steeringWheelPublisher_ = this->create_publisher<std_msgs::msg::Int16>("forklift1/target/steering", 1);
            steeringWheelSubscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joySteeringWheel", 10, std::bind(&ForkliftController::SteeringWheelCallback, this, _1));
	        actualValuesSubscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "forklift1/actual/info", 10, std::bind(&ForkliftController::ActualValuesCallback, this, _1));
            joystickSubscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joyJoystick", 10, std::bind(&ForkliftController::JoystickCallback, this, _1));
            timer_ = this->create_wall_timer(
            10ms, std::bind(&ForkliftController::MainFunctionCallback, this));
        }

    private:

          void get_param()
          {
            this->get_parameter("pub_hb_fr_", pub_hb_fr_);
            this->get_parameter("pub_logger_fr_", pub_logger_fr_);

            this->get_parameter("maxSpeed", maxSpeed);
            this->get_parameter("maxAngle", maxAngle);
            this->get_parameter("maxLift", maxLift);
        }

        void SendArduinoInfo()
        {
            auto arduinoMsg = std_msgs::msg::Int16();

            arduinoMsg.data = (int16_t)gasPedal;

            gasPedalPublisher_->publish(arduinoMsg);
        }

        void SendArduinoSteeringInfo()
        {
            auto arduinoSteeringMsg = std_msgs::msg::Int16();

            arduinoSteeringMsg.data = (int16_t)steeringWheel;

            steeringWheelPublisher_->publish(arduinoSteeringMsg);
        }
        
        //Handle Joystick Clicks to Change Gear
        void HandleGear()
        {
            if (joyGearLeft == 1 && !gearHandle)
            {
                gearHandle = true;
                gearDown = true;
            }

            if (joyGearRight == 1 && !gearHandle)
            {
                gearHandle = true;
                gearUp = true;
            }

            if (gearHandle)
            {
                if (gearDown)
                {
                    if (joyGearLeft == 0)
                    {
                        gear = gear - 1;
                        if (gear <= 0)
                        {
                            gear = 0;
                        }
                        
                        gearHandle = false;
                        gearDown = false;
                    }
                }
                if (gearUp)
                {
                    if (joyGearRight == 0)
                    {
                        gear = gear + 1;
                        if (gear >= 2)
                        {
                            gear = 2;
                        }
                        
                        gearHandle = false;
                        gearUp = false;
                    }
                }
                
            }
        }

        //Handle Joystick Clicks to Set Up Aid Cameras
        void HandleCameraButtons()
        {
            if (joyCamera == 1)
            {
                if(abs(joyLiftLever) == deadzonePct && joyTiltLever == 0 && joySideshifter == 0 && joyForkPositioner == 0)
                {
                    enableForkPositioner = 0x10;
                    handleForkMovement = false;
		    handleFork = true;
                }		
            }
            else
            {
                enableForkPositioner = 0x00;
                if(abs(joyLiftLever) == deadzonePct && joyTiltLever == 0 && joySideshifter == 0 && joyForkPositioner == 0 && !handleForkMovement)
                {
                    handleForkMovement = true;
		    handleFork = false;
		    counterFork = 0;
                }
            }

            if (joyOppositeCamera)
            {
                if (canBackwards == 1)
                {
                    frontCamera = true;
                    backCamera = false;
                }
                else
                {
                    frontCamera = false;
                    backCamera = true;
                }       
            }
            else
            {
                frontCamera = false;
                backCamera = false;
            }
            
        }

        //Handle Actual Direction from CAN Bus
        void HandleDirection()
        {
            switch (gear)
            {
            case 0:
                canForwards = 0;
                canBackwards = 1;
                break;
            
            case 1:
                canForwards = 0;
                canBackwards = 0;
                break;

            case 2:
                canForwards = 1;
                canBackwards = 0;
                break;
            }
        }

        //Handle Joystick Buttons and Calculate Target Values
        void HandleJoystick()
        {
            if(setNewAngle && !fixingAngle)
            {
                fixingAngle = true;
                float newAngleShift = ((actualAngle - 90) * (maxLimitAngle - minLimitAngle) / (-180)) + minLimitAngle;
                maxLimitAngle = (midAngle-newAngleShift) + maxLimitAngle;
                minLimitAngle = (midAngle-newAngleShift) + minLimitAngle;
                midAngle = midAngle-(newAngleShift-midAngle);
            }
	    
            steeringWheel = (((joySteeringWheel + 1) / 2) * (maxLimitAngle - minLimitAngle)) + minLimitAngle;
            gasPedal = 1000 * (joyGasPedal + 1)/2;	

            if(gear == 1 || gasPedalBlock != 0x01 || breakPedal >= 0.05)
            {
                gasPedal = 0;
            }

            breakPedal = joyBreakPedal + 1;
	    if(joyLiftLever < 0)
	    {
		liftLever = ((joyLiftLever+deadzonePct) / (-1+deadzonePct))  * maxLift;
	    }
	    else
	    {
	    	liftLever = -((joyLiftLever-deadzonePct) / (1-deadzonePct))  * maxLift;
	    }
            //liftLever = -((joyLiftLever-deadzonePct) / (1-deadzonePct))  * maxLift;
	    //liftLever = -joyLiftLever * maxLift;
            tiltLever = joyTiltLever * maxTilt;
            sideshifter = joySideshifter * maxShifter;
	    forkPositioner = joyForkPositioner * maxFork;
	        
            HandleHorn();
            
            if(handleForkMovement)
			{
				HandleHubDirection();
		        HandleTiltDirection();
		        HandleShiftDirection();
			}
			else
			{
				BlockHub();	
			}
	    if(handleFork && counterFork++ >= 100)
	    {
		HandleForkPositionerDirection();
	    }
	    else 
	    {
		BlockPositioner();
		canForkPositionerDirection = enableForkPositioner;
	    }   
            //HandleForkPositionerDirection();
        }

        //Handle Joystick to Set liftLever Target Value
        void HandleHubDirection()
        {
            if (liftLever < deadzonePct)
            {
                liftLever = -liftLever;
		canLiftDirection = 0x08;
            }
            else if (liftLever > deadzonePct)
            {
                canLiftDirection = 0x10;
            }
	    else if (abs(liftLever) == deadzonePct)
	    {
	    	canLiftDirection = 0x00;
	    }
            if (enableForkPositioner == 0x10)
            {
                canLiftDirection = 0x00;
                liftLever = 0;	    
            }
            else
            {
                forkPositioner = 0;
            }
        }

        void HandleHorn()
        {
            if(joyHorn == 1)
            {
                horn = 0x01;
            }
            else
            {
                horn = 0x00;
            }
        }

        void HandleTiltDirection()
        {
            if (tiltLever < 0)
            {
                canTiltDirection = 0x20;
                tiltLever = -tiltLever;
            }
            else if (tiltLever > 0)
            {
                canTiltDirection = 0x40;
            }
	    else
	    {
	    	canTiltDirection = 0x00;
	    }
            if (enableForkPositioner == 0x10)
            {
                canTiltDirection = 0x00;
                tiltLever = 0;	    
            }
            else
            {
                forkPositioner = 0;
            }
        }

        void HandleShiftDirection()
        {
            if (sideshifter < 0)
            {
                canSideshifterDirection = 0x2;
                sideshifter = -sideshifter;
            }
            else if (sideshifter > 0)
            {
                canSideshifterDirection = 0x1;
            }
	    else
	    {
	    	canSideshifterDirection = 0x00;
	    }
            if (enableForkPositioner == 0x10)
            {
                canSideshifterDirection = 0x00;
            sideshifter = 0;	    
            }
            else
            {
                forkPositioner = 0;
            }
        }

        void HandleForkPositionerDirection()
        {
	    if (enableForkPositioner == 0x10)
	    {
		    if (forkPositioner < 0)
		    {
		        forkPositioner = -forkPositioner;
		        canForkPositionerDirection = 0x4 + enableForkPositioner;
		    }
		    else if (forkPositioner > 0)
		    {
		        canForkPositionerDirection = 0x8 + enableForkPositioner;
		    }
		    else
		    {
		        canForkPositionerDirection = 0x00 + enableForkPositioner;
		    }
            }
            else
            {
                forkPositioner = 0;
                canForkPositionerDirection = 0x00 + enableForkPositioner;
            }
        }

        void BlockHub()
        {
            liftLever = 0;
            tiltLever = 0;
            sideshifter = 0;
            canLiftDirection = 0;
            canTiltDirection = 0;
            canSideshifterDirection = 0;
        }

	void BlockPositioner()
        {
            //liftLever = 0;
            //tiltLever = 0;
            //sideshifter = 0;
	    forkPositioner = 0;
            //canLiftDirection = 0;
            //canTiltDirection = 0;
            //canSideshifterDirection = 0;
	    canForkPositionerDirection = 0;
        }


        void SendCanInfo()
        {
            auto canInfoMsg = std_msgs::msg::Int32MultiArray();

            std::vector<int> vec_can_info{(int)liftLever, (int)canLiftDirection, (int)tiltLever, (int)canTiltDirection, (int)sideshifter, (int)canSideshifterDirection, (int)forkPositioner, (int)canForkPositionerDirection, (int)gear, (int)slowModus, (int)parkBreak, (int)horn};
            
            canInfoMsg.data = vec_can_info;

            canInfoPublisher_->publish(canInfoMsg);
        }

        //Send Driving Mode Information
        void SendDrivingMode()
        {
            auto modeMsg = std_msgs::msg::Byte();

            modeMsg.data = (forkCamera << 4) | (frontCamera << 3) | (backCamera << 2) | (canForwards << 1) | (canBackwards);

            modePublisher_->publish(modeMsg);
            message_heartbeat.data = 3;
            publisher_heartbeat->publish(message_heartbeat);
        }

        //Send Information to GUI
        void SendGUIInfo()
        {
            auto infoMsg = std_msgs::msg::ByteMultiArray();
            std::vector<uint8_t> vec_info{(uint8_t)(actualAngle + maxAngle), (uint8_t)actualSpeed, (uint8_t)breakPedal, (uint8_t)liftLever, (uint8_t)gear};
            infoMsg.data = vec_info;

            infoPublisher_->publish(infoMsg);
        }

        //Send Overall Information
        void SendAllInfo()
        {
            auto infoMsg = std_msgs::msg::Float32MultiArray();
            std::vector<float> vec_info{steeringWheel, angleActual, gasPedal, speedCockpitActual, breakPedal, liftLever, (float)gear, (float) gearActual, (float)forkCamera, (float)backCamera, (float)frontCamera};
            infoMsg.data = vec_info;

            allInfoPublisher_->publish(infoMsg);
        }

		float Deadzone(float value)
        {

            if (abs(value) <= (deadzonePct))
            {
                return 0;
            }
            else
            {
                return value;
            }
        }

		float Deadzone2(float value)
        {

            if (abs(value) <= (deadzonePct))
            {
		if(value >= 0)
		{
			return deadzonePct;
		}
		else
		{
                	return -deadzonePct;
		}
            }
            else
            {
                return value;
            }
        }


	    void AutoCenter()
        {
            auto autoCenterMsg = std_msgs::msg::Float32MultiArray();
            std::vector<float> vec_info{joySteeringWheel, gasPedal};
            autoCenterMsg.data = vec_info;

            autoCenter_->publish(autoCenterMsg);
        }

        //Main Function Callback
        void MainFunctionCallback()
        {
            HandleGear();      

	        //HandleCameraButtons();

            HandleDirection();

            HandleJoystick();

            SendCanInfo();

            SendArduinoInfo();

            SendArduinoSteeringInfo();

            HandleCameraButtons();
            
            SendDrivingMode();

            SendGUIInfo();

            AutoCenter();

            CheckForkliftOn();
 
        }

        void ActualValuesCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
        {
            actualSpeed = msg->data[1];
            if(gear == 0)
            {
                actualSpeed = -actualSpeed;
            }
            actualAngle = msg->data[0];
            gasPedalBlock = msg->data[2];
            if(!setNewAngle && countSetNewAngle >= 30)
            {
                setNewAngle = true;
            }
            else
            {
                countSetNewAngle++;
                if(countSetNewAngle >= 200)
                {
                    countSetNewAngle = 150;
                }
            }
	        countForkliftOn = 0;
	    }

        void CheckForkliftOn()
        {
            countForkliftOn++;
            if(countForkliftOn >= 1000)
            {
                countForkliftOn = 500;
            }
            if(countForkliftOn >= 300)
            {
                setNewAngle = false;
                fixingAngle = false;
                countSetNewAngle = 0;
            }
        }

        //Steering Wheel Callback
        void SteeringWheelCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            joySteeringWheel = msg->axes[0];
            joyGasPedal = msg->axes[1];
            joyBreakPedal = msg->axes[2];
            joyBackwards = msg->buttons[7];
            joyGearLeft = msg->buttons[5];
            joyGearRight = msg->buttons[4];
	        joyHorn = msg->buttons[10];
        }

        //Joystick Callback
        void JoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            joyLiftLever = Deadzone2(msg->axes[1]);
            joyCamera = msg->buttons[0];
            joyOppositeCamera = msg->buttons[1];
			joyTiltLever = Deadzone(msg->axes[5]);
			joySideshifter = Deadzone(msg->axes[4]);
			joyForkPositioner = Deadzone(msg->axes[2]);
            joySlow = msg->buttons[2];
            joyFeststellBremse = msg->buttons[3];
        }

        //Publishers, Timers and Subscriptions
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr canPublisher_;
	    
        rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr modePublisher_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr gasPedalPublisher_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steeringWheelPublisher_;
        rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr infoPublisher_;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr canInfoPublisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr allInfoPublisher_;
	    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr autoCenter_;
	    
        rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr publisher_heartbeat;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr steeringWheelSubscription_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSubscription_;
	    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr actualValuesSubscription_;
    
        //Target Values
        float gasPedal = 0;
        float breakPedal;
        float steeringWheel = 180;
        float liftLever;
		float tiltLever;
        float sideshifter;
        float forkPositioner;

        float nextSteer = 180;
	    float newSteer = 180;
        float lastSteer = 180;
        float maxStep = 0.5;
        
        //Joystick Values
        float joyGasPedal;
        float joyBreakPedal;
        float joySteeringWheel;
        float joyLiftLever;
        float joyTiltLever;
        float joySideshifter;
        float joyForkPositioner;
        int joyBackwards;
        int joyGearLeft;
        int joyGearRight;
        int joyCamera;
        int joyOppositeCamera;
        int joySlow;
        int joyFeststellBremse;
        int joyHorn;

        int horn;

	int counterFork;	

        int slowModus;
        int parkBreak;

        //Handle Function Variables
        bool gearDown;
        bool gearHandle;
        bool gearUp;
        int gear;
        int enableForkPositioner;

        uint8_t canForwards;
        uint8_t canBackwards;

        bool forkCamera = true;
        bool frontCamera;
        bool backCamera;

        uint8_t canLiftDirection;
        uint8_t canTiltDirection;
        uint8_t canSideshifterDirection;
        uint8_t canForkPositionerDirection;

	bool handleFork = false;

        bool blockHub;
        bool handleForkMovement = true;

        //Actual Values
        float speedLeftWheelActual;
        float speedRightWheelActual;
        float angleActual;
        float speedCockpitActual;
        float angleCockpitActual;
        int8_t gearActual;

        int midAngle = 180;
        int maxLimitAngle = 900;
        int minLimitAngle = -540;

        //Parameters
        float maxSpeed = 1000;
        float maxAngle = 90;
        float maxLift = 255;
        float maxShifter = 170;
        float maxTilt = 170;
        float maxFork = 100;

	bool liftUp = false;

        float deadzonePct = 0.15;

        int actualSpeed = 0;
        int actualAngle = 0;
	    int gasPedalBlock = 0;

	    bool fixingAngle = false;

        int countForkliftOn;

        bool setNewAngle = false;

	    int countSetNewAngle;

        std::uint16_t pub_logger_fr_;
        std::uint16_t pub_hb_fr_;
        rclcpp::TimerBase::SharedPtr timer_param;
        rclcpp::Clock& clk = *this->get_clock();
        std_msgs::msg::Byte message_heartbeat = std_msgs::msg::Byte();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForkliftController>());
    rclcpp::shutdown();
    return 0;
}
