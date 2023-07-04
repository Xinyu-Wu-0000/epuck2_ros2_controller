#include "epuck2_driver_cpp.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

epuck2_driver::epuck2_driver(/* args */) : Node("epuck2_driver")
{
	br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
	this->declare_parameter("epuck2_id", 0);
	this->declare_parameter("epuck2_address", "");
	this->declare_parameter("epuck2_name", "epuck2");
	this->declare_parameter("xpos", 0.0);
	this->declare_parameter("ypos", 0.0);
	this->declare_parameter("theta", 0.0);
	this->declare_parameter("cam_en", false);
	this->declare_parameter("floor_en", false);
	int robotId = 0;
	double init_xpos, init_ypos, init_theta;

	robotId = this->get_parameter("epuck2_id").as_int();
	epuckAddress = this->get_parameter("epuck2_address").as_string();
	epuckname = this->get_parameter("epuck2_name").as_string();
	init_xpos = this->get_parameter("xpos").as_double();
	init_ypos = this->get_parameter("ypos").as_double();
	init_theta = this->get_parameter("theta").as_double();
	camera_enabled = this->get_parameter("cam_en").as_bool();
	ground_sensors_enabled = this->get_parameter("floor_en").as_bool();

	if (DEBUG_ROS_PARAMS)
	{
		std::cout << "[" << epuckname << "] "
				  << "epuck id: " << robotId << std::endl;
		std::cout << "[" << epuckname << "] "
				  << "epuck address: " << epuckAddress << std::endl;
		std::cout << "[" << epuckname << "] "
				  << "epuck name: " << epuckname << std::endl;
		std::cout << "[" << epuckname << "] "
				  << "init pose: " << init_xpos << ", " << init_ypos << ", " << init_theta << std::endl;
		std::cout << "[" << epuckname << "] "
				  << "camera enabled: " << camera_enabled << std::endl;
		std::cout << "[" << epuckname << "] "
				  << "ground sensors enabled: " << ground_sensors_enabled << std::endl;
	}

	if (epuckAddress.compare("") == 0)
	{
		std::cerr << "Robot address cannot be empty" << std::endl;
		exit(1);
	}
	int temret = initConnectionWithRobot();
	if (temret < 0)
	{
		std::cerr << "Can't connect to the robot" << std::endl;
		exit(1);
	}

	command[0] = 0x80;
	command[1] = 2;	 // Sensors enabled camera disabled.
	command[2] = 1;	 // Calibrate proximity sensors.
	command[3] = 0;	 // left motor LSB
	command[4] = 0;	 // left motor MSB
	command[5] = 0;	 // right motor LSB
	command[6] = 0;	 // right motor MSB
	command[7] = 0;	 // lEDs
	command[8] = 0;	 // LED2 red
	command[9] = 0;	 // LED2 green
	command[10] = 0; // LED2 blue
	command[11] = 0; // LED4 red
	command[12] = 0; // LED4 green
	command[13] = 0; // LED4 blue
	command[14] = 0; // LED6 red
	command[15] = 0; // LED6 green
	command[16] = 0; // LED6 blue
	command[17] = 0; // LED8 red
	command[18] = 0; // LED8 green
	command[19] = 0; // LED8 blue
	command[20] = 0; // speaker
	expected_recv_packets = 1;

	imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

	for (int i = 0; i < 8; i++)
	{
		std::stringstream ss;
		ss.str("");
		ss << "proximity" << i;
		proxPublisher[i] = this->create_publisher<sensor_msgs::msg::Range>(ss.str(), 10);
		proxMsg[i].radiation_type = sensor_msgs::msg::Range::INFRARED;
		ss.str("");
		ss << epuckname << "/base_prox" << i;
		proxMsg[i].header.frame_id = ss.str();
		proxMsg[i].field_of_view = 0.26; // About 15 degrees...to be checked!
		proxMsg[i].min_range = 0.005;	 // 0.5 cm.
		proxMsg[i].max_range = 0.05;	 // 5 cm.
	}
	laserPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
	odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
	currentTime = rclClock.now();
	lastTime = rclClock.now();

	microphonePublisher = this->create_publisher<visualization_msgs::msg::Marker>("microphone", 10);
	distSensPublisher = this->create_publisher<sensor_msgs::msg::Range>("dist_sens", 10);
	distSensMsg.radiation_type = sensor_msgs::msg::Range::INFRARED;
	std::stringstream ss;
	ss.str("");
	ss << epuckname << "/base_dist_sens";
	distSensMsg.header.frame_id = ss.str();
	distSensMsg.field_of_view = 0.43; // About 25 degrees (+/- 12.5)
	distSensMsg.min_range = 0.005;	  // 5 mm.
	distSensMsg.max_range = 2;		  // 2 m.
	magFieldPublisher = this->create_publisher<sensor_msgs::msg::MagneticField>("mag_field", 10);
	magFieldVectorPublisher = this->create_publisher<visualization_msgs::msg::Marker>("mag_field_vector", 10);
	battPublisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
	if (camera_enabled)
	{
		imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("camera", 1);
		command[1] = 3; // Camera and sensors enabled.
		expected_recv_packets = 2;
	}
	if (ground_sensors_enabled)
	{
		floorPublisher = this->create_publisher<visualization_msgs::msg::Marker>("floor", 10);
	}
	cmdCtrlSubscriber = this->create_subscription<std_msgs::msg::Int16MultiArray>("mobile_base/cmd_ctrl", 10, std::bind(&epuck2_driver::handlerCtrl, this, _1));
	cmdVelSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("mobile_base/cmd_vel", 10, std::bind(&epuck2_driver::handlerVelocity, this, _1));
	cmdLedSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>("mobile_base/cmd_led", 10, std::bind(&epuck2_driver::handlerLED, this, _1));
	cmdRgbLedsSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>("mobile_base/rgb_leds", 10, std::bind(&epuck2_driver::handlerRgbLeds, this, _1));

	theta = init_theta;
	xPos = init_xpos;
	yPos = init_ypos;

	updateRosInfo_timer = this->create_wall_timer(
		20ms, std::bind(&epuck2_driver::updateRosInfo, this));
	updateSensorsAndActuators_timer = this->create_wall_timer(
		20ms, std::bind(&epuck2_driver::updateSensorsAndActuators, this));
}

epuck2_driver::~epuck2_driver()
{
}

int epuck2_driver::initConnectionWithRobot()
{
	int ret_value;
	std::stringstream ss;
	struct timeval tv;
	socklen_t len = sizeof(tv);
	uint8_t trials = 0;
	memset(&robot_addr, 0, sizeof(robot_addr));
	robot_addr.sin_family = AF_INET;
	robot_addr.sin_addr.s_addr = inet_addr(epuckAddress.c_str());
	robot_addr.sin_port = htons(1000);

	if (DEBUG_CONNECTION_INIT)
		fprintf(stdout, "Try to connect to %s:%d (TCP)\n", inet_ntoa(robot_addr.sin_addr), htons(robot_addr.sin_port));

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0)
	{
		perror("TCP cannot create socket: ");
		return -1;
	}

	// Set to non-blocking mode during connection otherwise it will block for too much time if the robot isn't ready to accept connections
	if ((ret_value = fcntl(fd, F_GETFL, 0)) < 0)
	{
		perror("Cannot get flag status: ");
		return -1;
	}
	else
	{
		if ((ret_value & O_NONBLOCK) > 0)
		{
			std::cout << "Non-blocking socket" << std::endl;
		}
		else
		{
			std::cout << "Blocking socket" << std::endl;
		}
	}
	ret_value |= O_NONBLOCK;
	if (fcntl(fd, F_SETFL, ret_value) < 0)
	{
		perror("Cannot set non-blocking mode: ");
		return -1;
	}
	while (trials < MAX_CONNECTION_TRIALS)
	{
		// Connection to the robot (server).
		int temval = 1;
		int ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &temval, sizeof(temval));
		if (ret < 0)
		{
			printf("setsockopt(SO_REUSEADDR) failed");
		}
		ret_value = connect(fd, (sockaddr *)&robot_addr, sizeof(robot_addr));
		if (ret_value == 0)
		{
			std::cout << "connected!!!" << std::endl;
			break;
		}
		else
		{

			trials++;
			if (DEBUG_CONNECTION_INIT)
				fprintf(stderr, "Connection trial %d, ret_value %d\n", trials, ret_value);
			sleep(3);
		}
	}

	if (trials == MAX_CONNECTION_TRIALS)
	{
		ss.str("");
		ss << "[" << epuckname << "] "
		   << "Error, can't connect to tcp socket";
		// if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return -1;
	}

	// Set to blocking mode.
	if ((ret_value = fcntl(fd, F_GETFL, 0)) < 0)
	{
		perror("Cannot get flag status: ");
		return -1;
	}
	else
	{
		if ((ret_value & O_NONBLOCK) > 0)
		{
			std::cout << "Non-blocking socket" << std::endl;
		}
		else
		{
			std::cout << "Blocking socket" << std::endl;
		}
	}
	ret_value &= (~O_NONBLOCK);
	if (fcntl(fd, F_SETFL, ret_value) < 0)
	{
		perror("Cannot set blocking mode: ");
		return -1;
	}

	// Set the reception timeout. This is used when blocking mode is activated after connection.
	tv.tv_sec = READ_TIMEOUT_SEC;
	tv.tv_usec = READ_TIMEOUT_USEC;
	ret_value = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	if (ret_value < 0)
	{
		perror("Cannot set rx timeout: ");
		return -1;
	}
	ret_value = getsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, &len);
	if (ret_value < 0)
	{
		perror("Cannot read rx timeout: ");
	}
	else
	{
		std::cout << "rx timeout: " << tv.tv_sec << " s and " << tv.tv_usec << " us" << std::endl;
	}

	return 0;
}

void epuck2_driver::closeConnection()
{
	std::stringstream ss;
	if (close(fd) < 0)
	{
		ss.str("");
		ss << "[" << epuckname << "] "
		   << "Can't close tcp socket";
		if (DEBUG_CONNECTION_INIT)
			perror(ss.str().c_str());
	}
}

void epuck2_driver::updateSensorsAndActuators()
{
	long unsigned int bytes_sent = 0, bytes_recv = 0, ret_value;
	long mantis = 0;
	short exp = 0;
	float flt = 0.0;

	bytes_sent = 0;
	while (bytes_sent < sizeof(command))
	{
		bytes_sent += send(fd, (char *)&command[bytes_sent], sizeof(command) - bytes_sent, 0);
	}
	command[2] = 0; // Stop proximity calibration.
	while (expected_recv_packets > 0)
	{
		bytes_recv = recv(fd, (char *)&header, 1, 0);
		if (bytes_recv <= 0)
		{
			closeConnection();
			if (initConnectionWithRobot() < 0)
			{
				std::cerr << "Lost connection with the robot" << std::endl;
				exit(1);
			}
			else
			{
				return; // Wait for the next sensor request
			}
		}
		// return;
		switch (header)
		{
		case 0x01: // Camera.
			bytes_recv = 0;
			while (bytes_recv < sizeof(image))
			{
				ret_value = recv(fd, (char *)&image[bytes_recv], sizeof(image) - bytes_recv, 0);
				if (ret_value <= 0)
				{
					closeConnection();
					if (initConnectionWithRobot() < 0)
					{
						std::cerr << "Lost connection with the robot" << std::endl;
						exit(1);
					}
					else
					{
						return; // Wait for the next sensor request
					}
				}
				else
				{
					bytes_recv += ret_value;
					// std::cout << "image read = " << bytes_recv << std::endl;
				}
			}

			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "camera read correctly" << std::endl;
			newImageReceived = true;

			// red = image[0] & 0xf8;
			// green = image[0] << 5;
			// green += (image[1] & 0xf8) >> 3;
			// blue = image[1] << 3;
			// printf("1st pixel = %d, %d, %d\r\n", red, green, blue);
			break;

		case 0x02: // Sensors.
			bytes_recv = 0;
			while (bytes_recv < sizeof(sensor))
			{
				ret_value = recv(fd, (char *)&sensor[bytes_recv], sizeof(sensor) - bytes_recv, 0);
				if (ret_value <= 0)
				{
					closeConnection();
					if (initConnectionWithRobot() < 0)
					{
						std::cerr << "Lost connection with the robot" << std::endl;
						exit(1);
					}
					else
					{
						return; // Wait for the next sensor request
					}
				}
				else
				{
					bytes_recv += ret_value;
					// std::cout << "sensors read = " << bytes_recv << std::endl;
				}
			}

			accData[0] = sensor[0] + sensor[1] * 256;
			accData[1] = sensor[2] + sensor[3] * 256;
			accData[2] = sensor[4] + sensor[5] * 256;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;

			// Compute acceleration
			mantis = (sensor[6] & 0xff) + ((sensor[7] & 0xffl) << 8) + (((sensor[8] & 0x7fl) | 0x80) << 16);
			exp = (sensor[9] & 0x7f) * 2 + ((sensor[8] & 0x80) ? 1 : 0);
			if (sensor[9] & 0x80)
			{
				mantis = -mantis;
			}
			flt = (mantis || exp) ? ((float)ldexp(mantis, (exp - 127 - 23))) : 0;
			acceleration = flt;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "acceleration: " << acceleration << std::endl;

			// Compute orientation.
			mantis = (sensor[10] & 0xff) + ((sensor[11] & 0xffl) << 8) + (((sensor[12] & 0x7fl) | 0x80) << 16);
			exp = (sensor[13] & 0x7f) * 2 + ((sensor[12] & 0x80) ? 1 : 0);
			if (sensor[13] & 0x80)
				mantis = -mantis;
			flt = (mantis || exp) ? ((float)ldexp(mantis, (exp - 127 - 23))) : 0;
			orientation = flt;
			if (orientation < 0.0)
				orientation = 0.0;
			if (orientation > 360.0)
				orientation = 360.0;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "orientation: " << orientation << std::endl;

			// Compute inclination.
			mantis = (sensor[14] & 0xff) + ((sensor[15] & 0xffl) << 8) + (((sensor[16] & 0x7fl) | 0x80) << 16);
			exp = (sensor[17] & 0x7f) * 2 + ((sensor[16] & 0x80) ? 1 : 0);
			if (sensor[17] & 0x80)
				mantis = -mantis;
			flt = (mantis || exp) ? ((float)ldexp(mantis, (exp - 127 - 23))) : 0;
			inclination = flt;
			if (inclination < 0.0)
				inclination = 0.0;
			if (inclination > 180.0)
				inclination = 180.0;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "inclination: " << inclination << std::endl;

			// Gyro
			gyroRaw[0] = sensor[18] + sensor[19] * 256;
			gyroRaw[1] = sensor[20] + sensor[21] * 256;
			gyroRaw[2] = sensor[22] + sensor[23] * 256;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "gyro: " << gyroRaw[0] << "," << gyroRaw[1] << "," << gyroRaw[2] << std::endl;

			// Magnetometer
			magneticField[0] = *((float *)&sensor[24]);
			magneticField[1] = *((float *)&sensor[28]);
			magneticField[2] = *((float *)&sensor[32]);
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "mag: " << magneticField[0] << "," << magneticField[1] << "," << magneticField[2] << std::endl;

			// Temperature.
			temperature = sensor[36];
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "temperature: " << (int)temperature << std::endl;

			// Proximity sensors data.
			proxData[0] = sensor[37] + sensor[38] * 256;
			proxData[1] = sensor[39] + sensor[40] * 256;
			proxData[2] = sensor[41] + sensor[42] * 256;
			proxData[3] = sensor[43] + sensor[44] * 256;
			proxData[4] = sensor[45] + sensor[46] * 256;
			proxData[5] = sensor[47] + sensor[48] * 256;
			proxData[6] = sensor[49] + sensor[50] * 256;
			proxData[7] = sensor[51] + sensor[52] * 256;
			if (proxData[0] < 0)
			{
				proxData[0] = 0;
			}
			if (proxData[1] < 0)
			{
				proxData[1] = 0;
			}
			if (proxData[2] < 0)
			{
				proxData[2] = 0;
			}
			if (proxData[3] < 0)
			{
				proxData[3] = 0;
			}
			if (proxData[4] < 0)
			{
				proxData[4] = 0;
			}
			if (proxData[5] < 0)
			{
				proxData[5] = 0;
			}
			if (proxData[6] < 0)
			{
				proxData[6] = 0;
			}
			if (proxData[7] < 0)
			{
				proxData[7] = 0;
			}
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;

			// Compute abmient light.
			lightAvg += (sensor[53] + sensor[54] * 256);
			lightAvg += (sensor[55] + sensor[56] * 256);
			lightAvg += (sensor[57] + sensor[58] * 256);
			lightAvg += (sensor[59] + sensor[60] * 256);
			lightAvg += (sensor[61] + sensor[62] * 256);
			lightAvg += (sensor[63] + sensor[64] * 256);
			lightAvg += (sensor[65] + sensor[66] * 256);
			lightAvg += (sensor[67] + sensor[68] * 256);
			lightAvg = (int)(lightAvg / 8);
			lightAvg = (lightAvg > 4000) ? 4000 : lightAvg;
			if (lightAvg < 0)
			{
				lightAvg = 0;
			}
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "lightAvg: " << lightAvg << std::endl;

			// ToF
			distanceMm = (uint16_t)((uint8_t)sensor[70] << 8) | ((uint8_t)sensor[69]);
			if (distanceMm > 2000)
			{
				distanceMm = 2000;
			}
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "distanceMm: " << distanceMm << "(" << (int)sensor[69] << "," << (int)sensor[70] << ")" << std::endl;

			// Microphone
			micVolume[0] = ((uint8_t)sensor[71] + (uint8_t)sensor[72] * 256);
			micVolume[1] = ((uint8_t)sensor[73] + (uint8_t)sensor[74] * 256);
			micVolume[2] = ((uint8_t)sensor[75] + (uint8_t)sensor[76] * 256);
			micVolume[3] = ((uint8_t)sensor[77] + (uint8_t)sensor[78] * 256);
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "mic: " << micVolume[0] << "," << micVolume[1] << "," << micVolume[2] << "," << micVolume[3] << std::endl;

			// Left steps
			motorSteps[0] = (sensor[79] + sensor[80] * 256);
			// Right steps
			motorSteps[1] = (sensor[81] + sensor[82] * 256);
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "steps: " << motorSteps[0] << "," << motorSteps[1] << std::endl;

			// Battery
			batteryRaw = (uint8_t)sensor[83] + (uint8_t)sensor[84] * 256;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "batteryRaw: " << batteryRaw << std::endl;

			// Micro sd state.
			microSdState = sensor[85];
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "microSdState: " << (int)microSdState << std::endl;

			// Tv remote.
			irCheck = sensor[86];
			irAddress = sensor[87];
			irData = sensor[88];
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "tv remote: " << (int)irCheck << "," << (int)irAddress << "," << (int)irData << std::endl;

			// Selector.
			selector = sensor[89];
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "selector: " << (int)selector << std::endl;

			// Ground sensor proximity.
			groundProx[0] = sensor[90] + sensor[91] * 256;
			groundProx[1] = sensor[92] + sensor[93] * 256;
			groundProx[2] = sensor[94] + sensor[95] * 256;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "groundProx: " << groundProx[0] << "," << groundProx[1] << "," << groundProx[2] << std::endl;

			// Ground sensor ambient light.
			groundAmbient[0] = sensor[96] + sensor[97] * 256;
			groundAmbient[1] = sensor[98] + sensor[99] * 256;
			groundAmbient[2] = sensor[100] + sensor[101] * 256;
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "groundAmbient: " << groundAmbient[0] << "," << groundAmbient[1] << "," << groundAmbient[2] << std::endl;

			// Button state.
			buttonState = sensor[102];
			if (DEBUG_UPDATE_SENSORS_DATA)
				std::cout << "[" << epuckname << "] "
						  << "buttonState: " << (int)buttonState << std::endl;

			break;

		case 0x03:
		{
			printf("empty packet\r\n");
			break;
		}
		default:
			printf("unexpected packet\r\n");
			closeConnection();
			if (initConnectionWithRobot() < 0)
			{
				std::cerr << "Lost connection with the robot" << std::endl;
				exit(1);
			}
			else
			{
				return; // Wait for the next sensor request
			}
			break;
		}
		expected_recv_packets--;
	}

	if (camera_enabled)
	{
		expected_recv_packets = 2;
	}
	else
	{
		expected_recv_packets = 1;
	}
}

void epuck2_driver::RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
	int line, column;
	int index_src = 0, index_dst = 0;

	for (line = 0; line < height; ++line)
	{
		for (column = 0; column < width; ++column)
		{
			dst[index_dst++] = (unsigned char)(src[index_src] & 0xF8);
			dst[index_dst++] = (unsigned char)((src[index_src] & 0x07) << 5) | (unsigned char)((src[index_src + 1] & 0xE0) >> 3);
			dst[index_dst++] = (unsigned char)((src[index_src + 1] & 0x1F) << 3);
			index_src += 2;
		}
	}
}
void epuck2_driver::handlerCtrl(const std_msgs::msg::Int16MultiArray &msg)
{
	command[3] = msg.data[0] & 0xFF; // left motor LSB
	command[4] = msg.data[0] >> 8;	 // left motor MSB
	command[5] = msg.data[1] & 0xFF; // right motor LSB
	command[6] = msg.data[1] >> 8;	 // right motor MSB

	if (DEBUG_SPEED_RECEIVED)
		std::cout << "[" << epuckname << "] "
				  << "new speed: " << msg.data[0] << ", " << msg.data[1] << std::endl;
}
void epuck2_driver::handlerVelocity(const geometry_msgs::msg::Twist &msg)
{
	// Controls the velocity of each wheel based on linear and angular velocities.
	double linear = msg.linear.x;
	double angular = msg.angular.z;

	// Kinematic model for differential robot.
	double wl = (linear - (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;
	double wr = (linear + (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;

	// At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
	speedLeft = int(wl * 1000.0);
	speedRight = int(wr * 1000.0);

	command[3] = speedLeft & 0xFF;	// left motor LSB
	command[4] = speedLeft >> 8;	// left motor MSB
	command[5] = speedRight & 0xFF; // right motor LSB
	command[6] = speedRight >> 8;	// right motor MSB

	if (DEBUG_SPEED_RECEIVED)
		std::cout << "[" << epuckname << "] "
				  << "new speed: " << speedLeft << ", " << speedRight << std::endl;
}

void epuck2_driver::handlerLED(const std_msgs::msg::UInt8MultiArray &msg)
{
	// Controls the state of each LED on the standard robot
	for (int i = 0; i < LED_NUMBER; i++)
	{
		if (msg.data[i] == 0)
		{
			command[7] &= ~(1 << i);
		}
		else
		{
			command[7] |= (1 << i);
		}
	}

	if (DEBUG_LED_RECEIVED)
	{
		std::cout << "[" << epuckname << "] "
				  << "new LED status: " << std::endl;
		for (int i = 0; i < LED_NUMBER; i++)
			std::cout << msg.data[i] << ", ";
	}
}

void epuck2_driver::handlerRgbLeds(const std_msgs::msg::UInt8MultiArray &msg)
{
	command[8] = msg.data[0];	// LED2 red
	command[9] = msg.data[1];	// LED2 green
	command[10] = msg.data[2];	// LED2 blue
	command[11] = msg.data[3];	// LED4 red
	command[12] = msg.data[4];	// LED4 green
	command[13] = msg.data[5];	// LED4 blue
	command[14] = msg.data[6];	// LED6 red
	command[15] = msg.data[7];	// LED6 green
	command[16] = msg.data[8];	// LED6 blue
	command[17] = msg.data[9];	// LED8 red
	command[18] = msg.data[10]; // LED8 green
	command[19] = msg.data[11]; // LED8 blue

	if (DEBUG_RGB_RECEIVED)
	{
		std::cout << "[" << epuckname << "] "
				  << "new RGB status: " << std::endl;
		for (int i = 0; i < RGB_LED_NUMBER; i++)
		{
			std::cout << i << ": " << msg.data[i * 3] << ", " << msg.data[i * 3 + 1] << ", " << msg.data[i * 3 + 2];
		}
	}
}

void epuck2_driver::updateRosInfo()
{
	std::stringstream ss;
	geometry_msgs::msg::Quaternion orientQuat;
	std::stringstream parent;
	std::stringstream child;
	tf2::Transform transform;
	tf2::Quaternion q;
	geometry_msgs::msg::TransformStamped transformGeom;
	std_msgs::msg::Header msgHeader;
	// e-puck proximity positions (cm), x pointing forward, y pointing left
	//           P7(3.5, 1.0)   P0(3.5, -1.0)
	//       P6(2.5, 2.5)           P1(2.5, -2.5)
	//   P5(0.0, 3.0)                   P2(0.0, -3.0)
	//       P4(-3.5, 2.0)          P3(-3.5, -2.0)
	//
	// e-puck proximity orentations (degrees)
	//           P7(10)   P0(350)
	//       P6(40)           P1(320)
	//   P5(90)                   P2(270)
	//       P4(160)          P3(200)

	for (int i = 0; i < 8; i++)
	{
		if (proxData[i] > 0)
		{
			proxMsg[i].range = 0.5 / sqrt(proxData[i]); // Transform the analog value to a distance value in meters (given from field tests).
		}
		else
		{
			proxMsg[i].range = proxMsg[i].max_range;
		}
		if (proxMsg[i].range > proxMsg[i].max_range)
		{
			proxMsg[i].range = proxMsg[i].max_range;
		}
		if (proxMsg[i].range < proxMsg[i].min_range)
		{
			proxMsg[i].range = proxMsg[i].min_range;
		}
		proxMsg[i].header.stamp = rclClock.now();
		proxPublisher[i]->publish(proxMsg[i]);
	}
	child << epuckname << "/base_link";
	for (int i = 0; i < 8; i++)
	{
		transform.setOrigin(tf2::Vector3(origin_list[i * 3], origin_list[i * 3 + 1], origin_list[i * 3 + 2]));
		q.setRPY(rpy_list[i * 3], rpy_list[i * 3 + 1], rpy_list[i * 3 + 2]);
		transform.setRotation(q);
		parent.str("");
		parent << epuckname << "/base_prox" + std::to_string(i);
		transformGeom.set__transform(tf2::toMsg(transform));
		transformGeom.set__child_frame_id(child.str());
		msgHeader.set__stamp(rclClock.now());
		msgHeader.set__frame_id(parent.str());
		transformGeom.set__header(msgHeader);
		br->sendTransform(transformGeom);
	}

	// We use the information from the 6 proximity sensors on the front side of the robot to get 19 laser scan points. The interpolation used is the following:
	// -90 degrees: P2
	// -80 degrees: 4/5*P2 + 1/5*P1
	// -70 degrees: 3/5*P2 + 2/5*P1
	// -60 degrees: 2/5*P2 + 3/5*P1
	// -50 degrees: 1/5*P2 + 4/5*P1
	// -40 degrees: P1
	// -30 degrees: 2/3*P1 + 1/3*P0
	// -20 degrees: 1/3*P1 + 2/3*P0
	// -10 degrees: P0
	// 0 degrees: 1/2*P0 + 1/2*P7
	// 10 degrees: P7
	// 20 degrees: 1/3*P6 + 2/3*P7
	// 30 degrees: 2/3*P6 + 1/3*P7
	// 40 degrees: P6
	// 50 degrees: 1/5*P5 + 4/5*P6
	// 60 degrees: 2/5*P5 + 3/5*P6
	// 70 degrees: 3/5*P5 + 2/5*P6
	// 80 degrees: 4/5*P5 + 1/5*P6
	// 90 degrees: P5
	currentTimeMap = rclClock.now();
	parent.str("");
	parent << epuckname << "/base_laser";
	laserMsg.header.stamp = rclClock.now();
	laserMsg.header.frame_id = parent.str();
	laserMsg.angle_min = -M_PI / 2.0;
	laserMsg.angle_max = M_PI / 2.0;
	laserMsg.angle_increment = M_PI / 18.0; // 10 degrees.
	// laserMsg.time_increment = (currentTimeMap-lastTimeMap).toSec()/180; //0.003; //(1 / laser_frequency) / (num_readings);
	// laserMsg.scan_time = (currentTimeMap-lastTimeMap).toSec();
	//  The laser is placed in the center of the robot, but the proximity sensors are placed around the robot thus add "ROBOT_RADIUS" to get correct values.
	laserMsg.range_min = 0.005 + ROBOT_RADIUS; // 0.5 cm + ROBOT_RADIUS.
	laserMsg.range_max = 0.05 + ROBOT_RADIUS;  // 5 cm + ROBOT_RADIUS.
	laserMsg.ranges.resize(19);
	laserMsg.intensities.resize(19);
	lastTimeMap = rclClock.now();

	double prox_list[] = {1.0 * proxData[2], proxData[2] * 4.0 / 5 + proxData[1] * 1.0 / 5, proxData[2] * 3.0 / 5.0 + proxData[1] * 2.0 / 5,
						  proxData[2] * 2.0 / 5 + proxData[1] * 3.0 / 5, proxData[2] * 1.0 / 5 + proxData[1] * 4.0 / 5, 1.0 * proxData[1],
						  proxData[1] * 2.0 / 3 + proxData[0] * 1.0 / 3, proxData[1] * 1.0 / 3 + proxData[0] * 2.0 / 3, 1.0 * proxData[0],
						  ((proxData[0] + proxData[7]) >> 1) * 1.0, 1.0 * proxData[7], proxData[7] * 2.0 / 3 + proxData[6] * 1.0 / 3,
						  proxData[7] * 1.0 / 3 + proxData[6] * 2.0 / 3, 1.0 * proxData[6], proxData[6] * 4.0 / 5 + proxData[5] * 1.0 / 5,
						  proxData[6] * 3.0 / 5 + proxData[5] * 2.0 / 5, proxData[6] * 2.0 / 5 + proxData[5] * 3.0 / 5, proxData[6] * 1.0 / 5 + proxData[5] * 4.0 / 5,
						  1.0 * proxData[5]};
	for (int i = 0; i < 19; i++)
	{
		if (prox_list[i] > 0)
		{
			laserMsg.ranges[i] = (0.5 / sqrt(prox_list[i])) + ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
			laserMsg.intensities[i] = prox_list[i];
		}
		else
		{ // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
			laserMsg.ranges[i] = laserMsg.range_max;
			laserMsg.intensities[i] = 0;
		}
	}

	for (int i = 0; i < 19; i++)
	{
		if (laserMsg.ranges[i] > laserMsg.range_max)
		{
			laserMsg.ranges[i] = laserMsg.range_max;
		}
		if (laserMsg.ranges[i] < laserMsg.range_min)
		{
			laserMsg.ranges[i] = laserMsg.range_min;
		}
	}
	transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.034));
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	parent.str("");
	child.str("");
	parent << epuckname << "/base_laser";
	child << epuckname << "/base_link";
	transformGeom.set__transform(tf2::toMsg(transform));
	transformGeom.set__child_frame_id(child.str());
	msgHeader.set__stamp(rclClock.now());
	msgHeader.set__frame_id(parent.str());
	transformGeom.set__header(msgHeader);
	br->sendTransform(transformGeom);
	laserPublisher->publish(laserMsg);
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Odometry topic
	//  The encoders values coming from the e-puck are 2 bytes signed int thus we need to handle the overflows otherwise the odometry will be wrong after a while (about 4 meters).
	if ((leftStepsRawPrev > 0) && (motorSteps[0] < 0) && (abs(motorSteps[0] - leftStepsRawPrev) > 30000))
	{ // Overflow detected (positive).
		overflowCountLeft++;
	}
	if ((leftStepsRawPrev < 0) && (motorSteps[0] > 0) && (abs(motorSteps[0] - leftStepsRawPrev) > 30000))
	{ // Overflow detected (negative).
		overflowCountLeft--;
	}
	motorPositionDataCorrect[0] = (overflowCountLeft * 65536) + motorSteps[0];

	if ((rightStepsRawPrev > 0) && (motorSteps[1] < 0) && (abs(motorSteps[1] - rightStepsRawPrev) > 30000))
	{ // Overflow detected (positive).
		overflowCountRight++;
	}
	if ((rightStepsRawPrev < 0) && (motorSteps[1] > 0) && (abs(motorSteps[1] - rightStepsRawPrev) > 30000))
	{ // Overflow detected (negative).
		overflowCountRight--;
	}
	motorPositionDataCorrect[1] = (overflowCountRight * 65536) + motorSteps[1];

	leftStepsRawPrev = motorSteps[0];
	rightStepsRawPrev = motorSteps[1];

	if (DEBUG_ODOMETRY)
		std::cout << "[" << epuckname << "] "
				  << "left, right raw: " << motorSteps[0] << ", " << motorSteps[1] << std::endl;
	if (DEBUG_ODOMETRY)
		std::cout << "[" << epuckname << "] "
				  << "left, right raw corrected: " << motorPositionDataCorrect[0] << ", " << motorPositionDataCorrect[1] << std::endl;

	// Compute odometry.
	leftStepsDiff = motorPositionDataCorrect[0] * MOT_STEP_DIST - leftStepsPrev;   // Expressed in meters.
	rightStepsDiff = motorPositionDataCorrect[1] * MOT_STEP_DIST - rightStepsPrev; // Expressed in meters.
	if (DEBUG_ODOMETRY)
		std::cout << "[" << epuckname << "] "
				  << "left, right steps diff: " << leftStepsDiff << ", " << rightStepsDiff << std::endl;

	deltaTheta = (rightStepsDiff - leftStepsDiff) / WHEEL_DISTANCE; // Expressed in radiant.
	deltaSteps = (rightStepsDiff + leftStepsDiff) / 2;				// Expressed in meters.
	if (DEBUG_ODOMETRY)
		std::cout << "[" << epuckname << "] "
				  << "delta theta, steps: " << deltaTheta << ", " << deltaSteps << std::endl;

	xPos += deltaSteps * cos(theta + deltaTheta / 2); // Expressed in meters.
	yPos += deltaSteps * sin(theta + deltaTheta / 2); // Expressed in meters.
	theta += deltaTheta;							  // Expressed in radiant.
	if (DEBUG_ODOMETRY)
		std::cout << "[" << epuckname << "] "
				  << "x, y, theta: " << xPos << ", " << yPos << ", " << theta << std::endl;

	leftStepsPrev = motorPositionDataCorrect[0] * MOT_STEP_DIST;  // Expressed in meters.
	rightStepsPrev = motorPositionDataCorrect[1] * MOT_STEP_DIST; // Expressed in meters.

	// Publish the odometry message over ROS.
	odomMsg.header.stamp = rclClock.now();
	odomMsg.header.frame_id = "odom";
	ss << epuckname << "/base_link";
	odomMsg.child_frame_id = ss.str();
	odomMsg.pose.pose.position.x = xPos;
	odomMsg.pose.pose.position.y = yPos;
	odomMsg.pose.pose.position.z = 0;
	odomMsg.pose.covariance[0] = 0.001;
	odomMsg.pose.covariance[7] = 0.001;
	odomMsg.pose.covariance[14] = 1000.0;
	odomMsg.pose.covariance[21] = 1000.0;
	odomMsg.pose.covariance[28] = 1000.0;
	odomMsg.pose.covariance[35] = 0.001;
	// Since all odometry is 6DOF we'll need a quaternion created from yaw.
	q.setRPY(0, 0, theta);
	orientQuat = tf2::toMsg(q);
	// orientQuat = tf2::createQuaternionMsgFromYaw(theta);
	odomMsg.pose.pose.orientation = orientQuat;
	currentTime = rclClock.now();
	odomMsg.twist.twist.linear.x = deltaSteps / ((currentTime - lastTime).seconds());  // "deltaSteps" is the linear distance covered in meters from the last update (delta distance);
																					   // the time from the last update is measured in seconds thus to get m/s we multiply them.
	odomMsg.twist.twist.angular.z = deltaTheta / ((currentTime - lastTime).seconds()); // "deltaTheta" is the angular distance covered in radiant from the last update (delta angle);
																					   // the time from the last update is measured in seconds thus to get rad/s we multiply them.
	if (DEBUG_ODOMETRY)
		std::cout << "[" << epuckname << "] "
				  << "time elapsed = " << (currentTime - lastTime).seconds() << " seconds" << std::endl;
	lastTime = rclClock.now();
	odomPublisher->publish(odomMsg);

	// Publish the transform over tf.
	geometry_msgs::msg::TransformStamped odomTrans;
	odomTrans.header.stamp = odomMsg.header.stamp;
	odomTrans.header.frame_id = odomMsg.header.frame_id;
	odomTrans.child_frame_id = odomMsg.child_frame_id;
	odomTrans.transform.translation.x = xPos;
	odomTrans.transform.translation.y = yPos;
	odomTrans.transform.translation.z = 0.0;
	odomTrans.transform.rotation = orientQuat;
	br->sendTransform(odomTrans);
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  IMU topic
	ss.str(std::string());
	ss << epuckname << "/base_link";
	imuMsg.header.frame_id = ss.str();
	imuMsg.header.stamp = rclClock.now();
	// Exchange x and y to be compatible with e-puck 1.x
	imuMsg.linear_acceleration.x = (accData[1]) / 800.0 * STANDARD_GRAVITY; // 1 g = about 800, then transforms in m/s^2.
	imuMsg.linear_acceleration.y = (accData[0]) / 800.0 * STANDARD_GRAVITY;
	imuMsg.linear_acceleration.z = (accData[2]) / 800.0 * STANDARD_GRAVITY;
	imuMsg.linear_acceleration_covariance[0] = 0.01;
	imuMsg.linear_acceleration_covariance[1] = 0.0;
	imuMsg.linear_acceleration_covariance[2] = 0.0;
	imuMsg.linear_acceleration_covariance[3] = 0.0;
	imuMsg.linear_acceleration_covariance[4] = 0.01;
	imuMsg.linear_acceleration_covariance[5] = 0.0;
	imuMsg.linear_acceleration_covariance[6] = 0.0;
	imuMsg.linear_acceleration_covariance[7] = 0.0;
	imuMsg.linear_acceleration_covariance[8] = 0.01;
	if (DEBUG_IMU)
		std::cout << "[" << epuckname << "] "
				  << "accel raw: " << accData[0] << ", " << accData[1] << ", " << accData[2] << std::endl;
	if (DEBUG_IMU)
		std::cout << "[" << epuckname << "] "
				  << "accel (m/s2): " << imuMsg.linear_acceleration.x << ", " << imuMsg.linear_acceleration.y << ", " << imuMsg.linear_acceleration.z << std::endl;
	imuMsg.angular_velocity.x = (gyroRaw[0] - gyroOffset[0]) * DEG2RAD(GYRO_RAW2DPS); // rad/s
	imuMsg.angular_velocity.y = (gyroRaw[1] - gyroOffset[1]) * DEG2RAD(GYRO_RAW2DPS);
	imuMsg.angular_velocity.z = (gyroRaw[2] - gyroOffset[2]) * DEG2RAD(GYRO_RAW2DPS);
	imuMsg.angular_velocity_covariance[0] = 0.01;
	imuMsg.angular_velocity_covariance[1] = 0.0;
	imuMsg.angular_velocity_covariance[2] = 0.0;
	imuMsg.angular_velocity_covariance[3] = 0.0;
	imuMsg.angular_velocity_covariance[4] = 0.01;
	imuMsg.angular_velocity_covariance[5] = 0.0;
	imuMsg.angular_velocity_covariance[6] = 0.0;
	imuMsg.angular_velocity_covariance[7] = 0.0;
	imuMsg.angular_velocity_covariance[8] = 0.01;
	if (DEBUG_IMU)
		std::cout << "[" << epuckname << "] "
				  << "gyro raw: " << gyroRaw[0] << ", " << gyroRaw[1] << ", " << gyroRaw[2] << std::endl;
	if (DEBUG_IMU)
		std::cout << "[" << epuckname << "] "
				  << "gyro (rad/s): " << imuMsg.angular_velocity.x << ", " << imuMsg.angular_velocity.y << ", " << imuMsg.angular_velocity.z << std::endl;
	// Pitch and roll computed assuming the aerospace rotation sequence Rxyz
	double roll = atan2(imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z);
	double pitch = atan2(-imuMsg.linear_acceleration.x, sqrt(imuMsg.linear_acceleration.y * imuMsg.linear_acceleration.y + imuMsg.linear_acceleration.z * imuMsg.linear_acceleration.z));
	double yaw = 0.0;
	q.setRPY(roll, pitch, yaw);
	orientQuat = tf2::toMsg(q);
	imuMsg.orientation = orientQuat;
	imuMsg.orientation_covariance[0] = 0.01;
	imuMsg.orientation_covariance[1] = 0.0;
	imuMsg.orientation_covariance[2] = 0.0;
	imuMsg.orientation_covariance[3] = 0.0;
	imuMsg.orientation_covariance[4] = 0.01;
	imuMsg.orientation_covariance[5] = 0.0;
	imuMsg.orientation_covariance[6] = 0.0;
	imuMsg.orientation_covariance[7] = 0.0;
	imuMsg.orientation_covariance[8] = 0.01;
	if (DEBUG_IMU)
		std::cout << "[" << epuckname << "] "
				  << "roll=" << roll << ", pitch=" << pitch << std::endl;
	imuPublisher->publish(imuMsg);
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Magnetic field topic
	ss.str(std::string());
	ss << epuckname << "/base_link";
	magFieldMsg.header.frame_id = ss.str();
	magFieldMsg.header.stamp = rclClock.now();
	magFieldMsg.magnetic_field_covariance[0] = 0.01;
	magFieldMsg.magnetic_field_covariance[4] = 0.01;
	magFieldMsg.magnetic_field_covariance[8] = 0.01;
	magFieldMsg.magnetic_field.x = magneticField[0] / 1000000.0; // given in Tesla
	magFieldMsg.magnetic_field.y = magneticField[1] / 1000000.0; // given in Tesla
	magFieldMsg.magnetic_field.z = magneticField[2] / 1000000.0; // given in Tesla
	if (DEBUG_MAG_FIELD)
		std::cout << "[" << epuckname << "] "
				  << "mag field (Tesla): " << magFieldMsg.magnetic_field.x << ", " << magFieldMsg.magnetic_field.y << ", " << magFieldMsg.magnetic_field.z << std::endl;
	magFieldPublisher->publish(magFieldMsg);
	// Magnetic field vector (normalized) topic
	// The resulting vector will have a length of 1 meter
	ss.str(std::string());
	ss << epuckname << "/base_link";
	magFieldVectorMsg.header.frame_id = ss.str();
	magFieldVectorMsg.header.stamp = rclClock.now();
	magFieldVectorMsg.type = visualization_msgs::msg::Marker::ARROW;
	geometry_msgs::msg::Point p;
	// Start point
	p.x = 0.0;
	p.y = 0.0;
	p.z = 0.0;
	magFieldVectorMsg.points.clear();
	magFieldVectorMsg.points.push_back(p);
	double mod = sqrt(magFieldMsg.magnetic_field.x * magFieldMsg.magnetic_field.x + magFieldMsg.magnetic_field.y * magFieldMsg.magnetic_field.y);
	// End point
	p.x = magFieldMsg.magnetic_field.x / mod;
	p.y = magFieldMsg.magnetic_field.y / mod;
	magFieldVectorMsg.points.push_back(p);
	magFieldVectorMsg.scale.x = 0.002;
	magFieldVectorMsg.scale.y = 0.003;
	magFieldVectorMsg.scale.z = 0.005;
	magFieldVectorMsg.color.a = 1.0;
	magFieldVectorMsg.color.r = 1.0;
	magFieldVectorMsg.color.g = 1.0;
	magFieldVectorMsg.color.b = 0.0;
	magFieldVectorPublisher->publish(magFieldVectorMsg);
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Microphone topic
	ss.str(std::string());
	ss << epuckname << "/base_link";
	microphoneMsg.header.frame_id = ss.str();
	microphoneMsg.header.stamp = rclClock.now();
	microphoneMsg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	microphoneMsg.pose.position.x = 0.15;
	microphoneMsg.pose.position.y = 0;
	microphoneMsg.pose.position.z = 0.11;
	q.setRPY(0, 0, 0);
	orientQuat = tf2::toMsg(q);
	microphoneMsg.pose.orientation = orientQuat;
	microphoneMsg.scale.z = 0.01;
	microphoneMsg.color.a = 1.0;
	microphoneMsg.color.r = 1.0;
	microphoneMsg.color.g = 1.0;
	microphoneMsg.color.b = 1.0;
	ss.str(std::string());
	ss << "mic: [" << micVolume[0] << ", " << micVolume[1] << ", " << micVolume[2] << ", " << micVolume[3] << "]";
	microphoneMsg.text = ss.str();
	microphonePublisher->publish(microphoneMsg);
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Time of flight topic
	distSensMsg.range = (float)distanceMm / 1000.0;
	if (distSensMsg.range > distSensMsg.max_range)
	{
		distSensMsg.range = distSensMsg.max_range;
	}
	if (distSensMsg.range < distSensMsg.min_range)
	{
		distSensMsg.range = distSensMsg.min_range;
	}
	distSensMsg.header.stamp = rclClock.now();
	distSensPublisher->publish(distSensMsg);
	transform.setOrigin(tf2::Vector3(0.035, 0.0, 0.034));
	q.setRPY(0, -0.21, 0.0);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_dist_sens";
	child.str("");
	child << epuckname << "/base_link";
	transformGeom.set__transform(tf2::toMsg(transform));
	transformGeom.set__child_frame_id(child.str());
	msgHeader.set__stamp(rclClock.now());
	msgHeader.set__frame_id(parent.str());
	transformGeom.set__header(msgHeader);
	br->sendTransform(transformGeom);
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Battery topic
	ss.str(std::string());
	ss << epuckname << "/base_link";
	battMsg.header.frame_id = ss.str();
	battMsg.header.stamp = rclClock.now();
	battMsg.voltage = (float)batteryRaw / COEFF_ADC_TO_VOLT;
	battMsg.percentage = (battMsg.voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
	battMsg.present = 1;
	if (DEBUG_BATTERY)
		std::cout << "[" << epuckname << "] "
				  << "battery V: " << battMsg.voltage << ", " << battMsg.percentage * 100.0 << " %" << std::endl;
	battPublisher->publish(battMsg);

	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Camera image topic
	if (camera_enabled)
	{
		if (newImageReceived)
		{
			newImageReceived = false;
			cv::Mat rgb888;
			cv_bridge::CvImage out_msg;
			out_msg.header.stamp = rclClock.now();
			// Same timestamp and tf frame as input image
			rgb888 = cv::Mat(120, 160, CV_8UC3);
			RGB565toRGB888(160, 120, &image[0], rgb888.data);
			out_msg.encoding = sensor_msgs::image_encodings::RGB8;
			out_msg.image = rgb888;
			imagePublisher->publish(*out_msg.toImageMsg());
		}
	}
	// #############################################################################################################################################

	// #############################################################################################################################################
	//  Ground sensor topic
	if (ground_sensors_enabled)
	{
		ss.str(std::string());
		ss << epuckname << "/base_link";
		floorMsg.header.frame_id = ss.str();
		floorMsg.header.stamp = rclClock.now();
		floorMsg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		floorMsg.pose.position.x = 0.15;
		floorMsg.pose.position.y = 0;
		floorMsg.pose.position.z = 0.13;
		q.setRPY(0, 0, 0);
		orientQuat = tf2::toMsg(q);
		floorMsg.pose.orientation = orientQuat;
		floorMsg.scale.z = 0.01;
		floorMsg.color.a = 1.0;
		floorMsg.color.r = 1.0;
		floorMsg.color.g = 1.0;
		floorMsg.color.b = 1.0;
		ss.str(std::string());
		ss << "floor: [" << groundProx[0] << ", " << groundProx[1] << ", " << groundProx[2] << ", " << groundProx[3] << ", " << groundProx[4] << "]";
		floorMsg.text = ss.str();
		floorPublisher->publish(floorMsg);
	}
	// #############################################################################################################################################
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<epuck2_driver>());
	rclcpp::shutdown();
	return 0;
}
