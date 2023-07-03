#ifndef __EPUCK2_DRIVER__
#define __EPUCK2_DRIVER__
#include <sstream>
#include <math.h>
#include <time.h>
#include <chrono>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#define DEBUG_CONNECTION_INIT 1
#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0
#define DEBUG_IMU 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_LED_RECEIVED 0
#define DEBUG_RGB_RECEIVED 0
#define DEBUG_MAG_FIELD 0
#define DEBUG_BATTERY 0

#define READ_TIMEOUT_SEC 10
#define READ_TIMEOUT_USEC 0
#define MAX_CONNECTION_TRIALS 5

#define WHEEL_DIAMETER 4									  // cm.
#define WHEEL_SEPARATION 5.3								  // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053								  // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER * M_PI) / 100.0) // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE / 1000.0)		  // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035									  // meters.

#define LED_NUMBER 6 // total number of LEDs on the robot (0,2,4,6=leds, 8=body, 9=front)
#define RGB_LED_NUMBER 4

#define DEG2RAD(deg) (deg / 180 * M_PI)
#define GYRO_RAW2DPS (250.0 / 32768.0f) // 250DPS (degrees per second) scale for int16 raw value
#define STANDARD_GRAVITY 9.80665f

#define RESISTOR_R1 220 // kohm
#define RESISTOR_R2 330 // kohm
#define VOLTAGE_DIVIDER (1.0f * RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))
#define VREF 3.0f // volt correspond to the voltage on the VREF+ pin
#define ADC_RESOLUTION 4096
#define COEFF_ADC_TO_VOLT ((1.0f * ADC_RESOLUTION * VOLTAGE_DIVIDER) / VREF) // convertion from adc value to voltage
#define MAX_VOLTAGE 4.2f													 // volt
#define MIN_VOLTAGE 3.4f													 // volt

class epuck2_driver : public rclcpp::Node
{
private:
	// Communication variables
	struct sockaddr_in robot_addr;
	int fd;
	unsigned char command[21];
	unsigned char header, sensor[104];
	int bytes_sent = 0, bytes_recv = 0;
	bool camera_enabled, ground_sensors_enabled;
	uint8_t expected_recv_packets = 0;
	bool newImageReceived = false;
	std::string epuckAddress = "";

	// Sensors data variables
	unsigned char image[160 * 120 * 2];
	float acceleration, orientation, inclination; /**< acceleration data*/
	int16_t accData[3];
	int16_t gyroRaw[3];
	float magneticField[3];
	uint8_t temperature;
	int proxData[8]; /**< proximity sensors data*/
	int lightAvg;	 /**< light sensor data*/
	uint16_t distanceMm;
	uint16_t micVolume[4]; /**< microphone data*/
	int16_t motorSteps[2];
	uint16_t batteryRaw;
	uint8_t microSdState;
	uint8_t irCheck, irAddress, irData;
	uint8_t selector;
	int16_t groundProx[3], groundAmbient[3];
	uint8_t buttonState;

	// ROS variables
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr proxPublisher[8];
	sensor_msgs::msg::Range proxMsg[8];
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPublisher;
	sensor_msgs::msg::LaserScan laserMsg;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
	nav_msgs::msg::Odometry odomMsg;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
	sensor_msgs::msg::Image imgMsg;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
	sensor_msgs::msg::Imu imuMsg;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr microphonePublisher;
	visualization_msgs::msg::Marker microphoneMsg;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr floorPublisher;
	visualization_msgs::msg::Marker floorMsg;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distSensPublisher;
	sensor_msgs::msg::Range distSensMsg;
	rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magFieldPublisher;
	sensor_msgs::msg::MagneticField magFieldMsg;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr magFieldVectorPublisher;
	visualization_msgs::msg::Marker magFieldVectorMsg;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battPublisher;
	sensor_msgs::msg::BatteryState battMsg;

	rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr cmdCtrlSubscriber;
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr cmdLedSubscriber, cmdRgbLedsSubscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber;

	double leftStepsDiff = 0, rightStepsDiff = 0;
	double leftStepsPrev = 0, rightStepsPrev = 0;
	signed long int leftStepsRawPrev = 0, rightStepsRawPrev = 0;
	signed long int motorPositionDataCorrect[2];
	double xPos, yPos, theta;
	double deltaSteps, deltaTheta;
	rclcpp::Time currentTime, lastTime, currentTimeMap, lastTimeMap;
	int overflowCountLeft = 0, overflowCountRight = 0;
	int16_t gyroOffset[3] = {0, 0, 0}; // Used if making an initial calibration of the gyro.
	int speedLeft = 0, speedRight = 0;

	// General variables
	std::string epuckname;

	rclcpp::Clock rclClock;
	rclcpp::TimerBase::SharedPtr updateSensorsAndActuators_timer;
	rclcpp::TimerBase::SharedPtr updateRosInfo_timer;
	std::shared_ptr<tf2_ros::TransformBroadcaster> br;
	double origin_list[24] = {0.035, -0.010, 0.034,
							  0.025, -0.025, 0.034,
							  0.000, -0.030, 0.034,
							  -0.035, -0.020, 0.034,
							  -0.035, 0.020, 0.034,
							  0.000, 0.030, 0.034,
							  0.025, 0.025, 0.034,
							  0.035, 0.010, 0.034};
	double rpy_list[24] = {0, 0, 6.11,
						   0, 0, 5.59,
						   0, 0, 4.71,
						   0, 0, 3.49,
						   0, 0, 2.8,
						   0, 0, 1.57,
						   0, 0, 0.70,
						   0, 0, 0.17};
	int initConnectionWithRobot();
	void closeConnection();
	void updateSensorsAndActuators();
	void RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst);
	void handlerVelocity(const geometry_msgs::msg::Twist &msg);
	void handlerCtrl(const std_msgs::msg::Int16MultiArray &msg);
	void handlerLED(const std_msgs::msg::UInt8MultiArray &msg);
	void handlerRgbLeds(const std_msgs::msg::UInt8MultiArray &msg);
	void updateRosInfo();

public:
	epuck2_driver(/* args */);
	~epuck2_driver();
};

#endif
