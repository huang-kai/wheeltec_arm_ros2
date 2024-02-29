
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_
// 头文件
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wheeltec_arm_pick_msgs/msg/color_ik_result.hpp>
#include <wheeltec_arm_pick_msgs/msg/arm_target_position.hpp>
#include <wheeltec_arm_pick_msgs/msg/gesture.hpp>
#include <wheeltec_arm_pick_msgs/msg/data.hpp>

using namespace std;
#define SEND_DATA_CHECK   1     //标志位，发送端做校验位
#define READ_DATA_CHECK   0     //标志位，接收端做校验位
#define FRAME_HEADER      0X7B  //帧头，和下位机一致
#define FRAME_TAIL  0X7D //帧尾
#define FRAME_HEADER_ARM      0XAA  //帧头，和下位机一致
#define FRAME_TAIL_ARM  0XBB //帧尾
#define RECEIVE_DATA_SIZE		24//下位机发过来的数据的长度
#define SEND_DATA_SIZE			16//ROS发给下位机的数据的长度 考虑到时效应短尽短
#define default_mode  1
#define follower 2
#define PI 				3.1415926f//圆周率
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define GYROSCOPE_RATIO	0.00026644f	// 1/65.5/57.30=0.00026644 陀螺仪原始数据换成弧度单位
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define ACCEl_RATIO 	16384.0f  	// 量程±2g，重力加速度定义为1g等于9.8米每平方秒。
extern sensor_msgs::msg::Imu Mpu6050;
//协方差
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
 
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
// 速度/位置结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}MPU6050_DATA;

typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail;    //1个字节  帧尾 校验位 

}SEND_DATA;

typedef struct _SEND_DATA2_  
{
	    uint8_t tx[SEND_DATA_SIZE-5];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail;    //1个字节  帧尾 校验位 

}SEND_DATA2;

typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header; //1个字节 帧头
		float X_speed;  
		float Y_speed;  
		float Z_speed;  
		float Power_Voltage;	
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}RECEIVE_DATA;

//DATE：2024-2-29
//类，巧妙使用构造函数初始化数据和发布话题等
class turn_on_robot : public rclcpp::Node

{
	public:
		turn_on_robot();
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码 
    void update_tfupdate_tf(geometry_msgs::msg::TransformStamped::SharedPtr odom_tf);
		void Publish_Odom();      //Pub the speedometer topic //发布里程计话题
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象 
		//explicit turn_on_robot(
		 // const std::string & name = "wheeltec_robot");

	private:

        rclcpp::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
        float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_State_Sub;
		

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;         // CHANGE
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;         // CHANGE
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;         // CHANGE

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher;         // CHANGE
		rclcpp::Publisher<wheeltec_arm_pick_msgs::msg::Data>::SharedPtr robotpose_publisher;         // CHANGE
        rclcpp::Publisher<wheeltec_arm_pick_msgs::msg::Data>::SharedPtr robotvel_publisher;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tf_pub_;
        //rclcpp::Publisher<wheeltec_robot_msg::msg::Supersonic>::SharedPtr distance_publisher;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro;
        rclcpp::TimerBase::SharedPtr test_timer;

        rclcpp::TimerBase::SharedPtr odom_timer;
        rclcpp::TimerBase::SharedPtr imu_timer;
        rclcpp::TimerBase::SharedPtr voltage_timer;

        rclcpp::TimerBase::SharedPtr robotpose_timer;
        rclcpp::TimerBase::SharedPtr robotvel_timer;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void declare_parameters();
        void get_parameters();


        void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
        //void Akm_Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
        void Publish_ImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
        void Publish_Voltage();   //Pub the power supply voltage topic //发布电源电压话题
        void Publish_distance();//发布超声波距离
        void joint_states_Callback(const sensor_msgs::msg::JointState arm_joint);
        void face_joint_states_Callback(const wheeltec_arm_pick_msgs::msg::ColorIkResult angle);
		void color_joint_states_Callback(const wheeltec_arm_pick_msgs::msg::ColorIkResult color_angle);
		void gesture_joint_states_Callback(const sensor_msgs::msg::JointState gesture_angle);
		void arm_teleop_Callback(const sensor_msgs::msg::JointState arm_joint);
		void voice_joint_states_Callback(const sensor_msgs::msg::JointState preset_angle);
        void init_joint_states();

        auto createQuaternionMsgFromYaw(double yaw);
 
        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool Get_Sensor_Data();   
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        unsigned char Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read //IMU数据转化读取
		    float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id, akm_cmd_vel, test, joint_states; //Define the related variables //定义相关变量
        std::string cmd_vel;
        int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体
		SEND_DATA2 Send_Data2;     //11位
        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
        MPU6050_DATA Mpu6050_Data; //IMU data //IMU数据
        float Power_voltage;       //Power supply voltage //电源电压
        bool Charging=0;           //Whether the robot is charging the flag bit //机器人是否在充电的标志位
        float Charging_Current=0;  //Charging_Current //充电电流
        bool Red=0;                //Whether the robot finds the marker bit of infrared signal (charging pile)  //机器人是否寻找到红外信号(充电桩)的标志位 
		int joint_num;
    size_t count_;
};
#endif
