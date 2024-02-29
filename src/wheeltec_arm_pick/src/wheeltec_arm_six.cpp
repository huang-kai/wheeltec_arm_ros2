#include "wheeltec_arm_pick/wheeltec_arm_six.h"
#include "wheeltec_arm_pick/quaternion_solution.h"
//sensor_msgs::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象 
sensor_msgs::msg::Imu Mpu6050;
using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;
/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/

int main(int argc, char *argv[])
{
  // ros::init(argc, argv, "wheeltec_arm_six");//ROS初始化 并设置节点名称，可修改
  rclcpp::init(argc, argv);
  // ROS_INFO("wheeltec_arm_six node has turned on ");//显示状态
  turn_on_robot Robot_Control; //实例化一个对象
  Robot_Control.Control();  //循环执行数据采集和发布topic等操作
  return 0;
} 


/**************************************
Date: June 29, 2020
Function: 数据传输转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;  
      transition_16 |=  Data_Low;  
  return transition_16;
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;  //获取数据的高8位
      transition_16 |=  Data_Low;     //获取数据的低8位
      data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; //(发送端将数据放大1000倍发送，这里需要将数据单位还原)
  return data_return;
}

/**************************************
Date: June 29, 2020
Function: 订阅回调函数Callback，根据订阅的指令向串口发指令控制下位机
***************************************/
//void turn_on_robot::Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl) 
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short  transition;  //intermediate variable //中间变量
  //if(akm_cmd_vel=="none") {RCLCPP_INFO(this->get_logger(),"not akm");} //Prompt message //提示信息
  Send_Data2.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
  Send_Data2.tx[1] = 0; //set aside //预留位
  Send_Data2.tx[2] = 0; //set aside //预留位

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  transition=0;
  transition = twist_aux->linear.x*1000; //将浮点数放大一千倍，简化传输
  Send_Data2.tx[4] = transition;     //取数据的低8位
  Send_Data2.tx[3] = transition>>8;  //取数据的高8位

  //The target velocity of the Y-axis of the robot
  //机器人y轴的目标线速度
  transition=0;
  transition = twist_aux->linear.y*1000;
  Send_Data2.tx[6] = transition;
  Send_Data2.tx[5] = transition>>8;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  transition=0;
  transition = twist_aux->angular.z*1000;
  Send_Data2.tx[8] = transition;
  Send_Data2.tx[7] = transition>>8;

  Send_Data2.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BBC check bits, see the Check_Sum function //BBC校验位，规则参见Check_Sum函数
  Send_Data2.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D

  try
  {
    if(akm_cmd_vel=="none")  
 {Stm32_Serial.write(Send_Data2.tx,sizeof (Send_Data2.tx));} //Sends data to the downloader via serial port //通过串口向下位机发送数据 
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

void turn_on_robot::joint_states_Callback(const sensor_msgs::msg::JointState arm_joint)
{
  //ROS_INFO_STREAM("ok");//ready显示状态
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值
  
  transition=0;
  transition = arm_joint.position[joint_num]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint.position[joint_num+1]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = arm_joint.position[joint_num+2]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint.position[joint_num+3]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = arm_joint.position[joint_num+4]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint.position[joint_num+5]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = default_mode;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）

  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}

void turn_on_robot::init_joint_states()
{
  RCLCPP_INFO_STREAM(this->get_logger(),"arm is ready");//ready显示状态
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值

  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 1.57*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = default_mode ;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）

  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}

void turn_on_robot::face_joint_states_Callback(const wheeltec_arm_pick_msgs::msg::ColorIkResult angle)
{
  short  transition;  //中间变量
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"face is ready");//ready显示状态
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值

  transition=0;
  transition = angle.pedestal_angle*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = ((angle.arm_angle>0.7)?0.7:angle.arm_angle)*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = ((angle.arm_angle>0.7)?(0.7-0.06981317008):(angle.arm_angle-0.06981317008))*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 1.57*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = follower ;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）
  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}

void turn_on_robot:: color_joint_states_Callback(const wheeltec_arm_pick_msgs::msg::ColorIkResult color_angle)
{
  RCLCPP_INFO_STREAM(this->get_logger(),"color is ready");//ready显示状态
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值

  transition=0;
  transition = color_angle.pedestal_angle*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = ((color_angle.arm_angle>0.7)?0.7:color_angle.arm_angle)*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = ((color_angle.arm_angle>0.7)?0.7:color_angle.arm_angle)*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 1.57*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = 0*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = follower ;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）
  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}
void turn_on_robot:: gesture_joint_states_Callback(const sensor_msgs::msg::JointState gesture_angle)
{
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"gesture is ready");//ready显示状态
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值

  transition=0;
  transition = gesture_angle.position[0]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = gesture_angle.position[1]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = -gesture_angle.position[2]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = -gesture_angle.position[3]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = gesture_angle.position[4]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = gesture_angle.position[5]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = default_mode ;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）
  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}

void turn_on_robot::arm_teleop_Callback(const sensor_msgs::msg::JointState arm_joint)
{
  RCLCPP_INFO_STREAM(this->get_logger(),"ok");//ready显示状态
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值

  transition=0;
  transition = arm_joint.position[0]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint.position[1]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = arm_joint.position[2]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint.position[3]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = arm_joint.position[4]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint.position[5]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = default_mode;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）

  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}

void turn_on_robot::voice_joint_states_Callback(const sensor_msgs::msg::JointState preset_angle)
{
  RCLCPP_INFO_STREAM(this->get_logger(),"ok");//ready显示状态
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER_ARM;//帧头 固定值

  transition=0;
  transition = preset_angle.position[0]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",preset_angle.position[0]); 
  Send_Data.tx[2] = transition;     //取数据的低8位
  Send_Data.tx[1] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = preset_angle.position[1]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  transition=0;
  transition = preset_angle.position[2]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[6] = transition;     //取数据的低8位
  Send_Data.tx[5] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = preset_angle.position[3]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[8] = transition;     //取数据的低8位
  Send_Data.tx[7] = transition>>8;  //取数据的高8位

  transition=0;
  transition = preset_angle.position[4]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[10] = transition;     //取数据的低8位
  Send_Data.tx[9] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = preset_angle.position[5]*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[12] = transition;     //取数据的低8位
  Send_Data.tx[11] = transition>>8;  //取数据的高8位

  Send_Data.tx[13] = default_mode;

  Send_Data.tx[14]=Check_Sum(14,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[15]=FRAME_TAIL_ARM;  //数据的最后一位是帧尾（固定值）

  try
  {
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}
/**************************************
Date: May 31, 2020
Function: 发布IMU数据
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; //Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id; //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack 
                                                //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  

  imu_publisher->publish(Imu_Data_Pub);
 
}

/**************************************
Date: May 31, 2020
Function: 发布里程计相关信息
***************************************/
void turn_on_robot::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
     //把Z轴转角转换为四元数进行表达

    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);

    wheeltec_arm_pick_msgs::msg::Data robotpose;
    wheeltec_arm_pick_msgs::msg::Data robotvel;
    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据

    odom.header.stamp = rclcpp::Node::now(); 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标

    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    //odom.pose.pose.position.z = 0;
    odom.pose.pose.position.z = Robot_Pos.Z; 
    odom.pose.pose.orientation = odom_quat;
    //设置速度 
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

 /*   geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.header.stamp = rclcpp::Node::now();

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;

    tf_bro->sendTransform(odom_tf);

*/
    //There are two types of this matrix, which are used when the robot is at rest 
    //and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    //tf_pub_->publish(odom_tf);
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话题
    robotpose_publisher->publish(robotpose); //Pub odometer topic //发布里程计话题
    robotvel_publisher->publish(robotvel); //Pub odometer topic //发布里程计话题
}


// void turn_on_robot::Publish_Pose()
// {
//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
//     geometry_msgs::Pose pose;//里程计话题消息数据类型

//     pose.position.x = Robot_Pos.X;//位置
//     pose.position.y = Robot_Pos.Y;
//     pose.position.z = Robot_Pos.Z;
//     pose.orientation = odom_quat;
     
//     pose_publisher.publish(pose);//发布这个话题 消息类型是nav_msgs::Odometry
// }

/**************************************
Date: May 31, 2020
Function: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; //Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
      {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage; //The power supply voltage is obtained //电源供电的电压获取
        voltage_publisher->publish(voltage_msgs); //Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
      }
}/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BBC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BBC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
    //机械臂校验位
    if(Count_Number==14){
      for(k=0;k<Count_Number;k++)//Count_Number是发送数组位数减1
      {
      check_sum=check_sum^Send_Data.tx[k];//按位异或
      }
    }
    //底盘校验位
    if(Count_Number==9){
      for(k=0;k<Count_Number;k++)//Count_Number是发送数组位数减1
      {
      check_sum=check_sum^Send_Data2.tx[k];//按位异或
      }
    }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data()
{ 
  short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0};
  Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据

    //ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
    //Receive_Data_Pr[0],Receive_Data_Pr[1],Receive_Data_Pr[2],Receive_Data_Pr[3],Receive_Data_Pr[4],Receive_Data_Pr[5],Receive_Data_Pr[6],Receive_Data_Pr[7],
    //Receive_Data_Pr[8],Receive_Data_Pr[9],Receive_Data_Pr[10],Receive_Data_Pr[11],Receive_Data_Pr[12],Receive_Data_Pr[13],Receive_Data_Pr[14],Receive_Data_Pr[15],
    //Receive_Data_Pr[16],Receive_Data_Pr[17],Receive_Data_Pr[18],Receive_Data_Pr[19],Receive_Data_Pr[20],Receive_Data_Pr[21],Receive_Data_Pr[22],Receive_Data_Pr[23]);
   
    for(j=0;j<24;j++)
    {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
    }
    //ROS_INFO("%x-%x",Header_Pos,Tail_Pos);
    if(Tail_Pos==(Header_Pos+23))
    {
         //ROS_INFO("1----");
      memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if(Header_Pos==(1+Tail_Pos))
    {
        //ROS_INFO("2----");
        for(j=0;j<24;j++)
        Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%24];
    }
    else 
    {
     //ROS_INFO("3----");
     return false;
    }    
    //ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
    //Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6],Receive_Data.rx[7],
    //Receive_Data.rx[8],Receive_Data.rx[9],Receive_Data.rx[10],Receive_Data.rx[11],Receive_Data.rx[12],Receive_Data.rx[13],Receive_Data.rx[14],Receive_Data.rx[15],
    //Receive_Data.rx[16],Receive_Data.rx[17],Receive_Data.rx[18],Receive_Data.rx[19],Receive_Data.rx[20],Receive_Data.rx[21],Receive_Data.rx[22],Receive_Data.rx[23]); 
  
  Receive_Data.Frame_Header= Receive_Data.rx[0]; //数据的第一位是帧头（固定值）
  Receive_Data.Frame_Tail= Receive_Data.rx[23];  //数据的最后一位是帧尾（数据校验位）

 if (Receive_Data.Frame_Header == FRAME_HEADER )//判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) //判断帧尾
    { 
      //BBC check passes or two packets are interlaced //BBC校验通过或者两组数据包交错
      if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK)||(Header_Pos==(1+Tail_Pos))) 
      {
        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
        //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); 
        //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); 
                                                                         //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
        
        //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
        //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度  
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度  
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度  
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度  
        //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
        //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
        //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
        //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;
        //Robot_Vel.Z = Mpu6050.angular_velocity.z;
        //获取电池电压
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[20]<<8;
        transition_16 |=  Receive_Data.rx[21];  
        Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;//(发送端将数据放大1000倍发送，这里需要将数据单位还原)
        return true;
     }
    }
  } 
 return false;
}

/**************************************
Date: May 31, 2020
Function: 这是相关控制代码，代码循环执行
***************************************/
void turn_on_robot::Control()
{
  RCLCPP_INFO(this->get_logger(),"control");
  rclcpp::Time current_time, last_time;
  current_time = rclcpp::Node::now();
  last_time = rclcpp::Node::now();
  while(rclcpp::ok())
  {
    current_time = rclcpp::Node::now();
    //Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage) 
    //获取时间间隔，用于积分速度获得位移(里程)
    Sampling_Time = (current_time - last_time).seconds(); 

    //The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
    //通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    if (true == Get_Sensor_Data()) 
                                   
    {
      //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
      Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m 
      Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 
      Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time;

      //Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
      //通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);//四元数解算
      Publish_Odom();        //发布里程计话题
      //Publish_Pose();
      Publish_ImuSensor();  //发布话题    
      Publish_Voltage(); //发布电源电压
      rclcpp::spin_some(this->get_node_base_interface());
    }

    last_time = current_time; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔

    }
}
/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot()
: rclcpp::Node ("wheeltec_arm")
{
  RCLCPP_INFO(this->get_logger(),"turn_on_robot");
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); //构造函数初始化
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Send_Data2, 0, sizeof(Send_Data2));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  //ros::NodeHandle private_nh("~");
  //把以上的类成员参数注册到参数服务器，这样在launch文件里面即可修改
  //3个入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  //private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/wheeltec_controller"); //固定串口
  //private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //和下位机底层波特率115200 不建议更高的波特率了
  //private_nh.param<std::string>("smoother_cmd_vel", smoother_cmd_vel, "/smoother_cmd_vel");//平滑控制指令 
  //private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");//ID
  //private_nh.param<int>("joint_num", joint_num, 4);//机械臂关节起始编号
  int serial_baud_rate = 115200;

  this->declare_parameter<int>("serial_baud_rate");
  this->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<std::string>("joint_states", "joint_states");
  this->declare_parameter<int>("joint_num",6);
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("joint_states", joint_states);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  //发布话题
  //RCLCPP_INFO(this->get_logger(),"create topics start");
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
 
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);    // CHANGE
  
  //imu_timer = create_wall_timer(1s/100, [=]() { Publish_ImuSensor(); });
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
  //voltage_timer = create_wall_timer(1s/100, [=]() { Publish_Voltage(); });    
  //tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  robotpose_publisher = create_publisher<wheeltec_arm_pick_msgs::msg::Data>("robotpose", 10);
  robotvel_publisher = create_publisher<wheeltec_arm_pick_msgs::msg::Data>("robotvel", 10);
  
  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  //voltage_publisher = n.advertise<std_msgs::Float32>("PowerVoltage", 10);//电池电压数据发布
  //odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);//里程计数据发布
  //imu_publisher  = n.advertise<sensor_msgs::Imu>("mobile_base/sensors/imu_data", 20);//IMU数据发布
  //imu_publisher  = n.advertise<sensor_msgs::Imu>("imu", 20);//IMU数据发布
  
  //订阅话题
  
  //Cmd_Vel_Sub = n.subscribe("cmd_vel", 100, &turn_on_robot::Cmd_Vel_Callback, this);//因为官方的平滑包只支持X和W，没有Y，所以这里不使用平滑包
  //joint_state_Sub = n.subscribe("joint_states", 100, &turn_on_robot::joint_states_Callback, this);
  //face_joint_state_Sub = n.subscribe("face_ik_result", 100,&turn_on_robot::face_joint_states_Callback, this);
  //color_joint_state_Sub = n.subscribe("color_result", 100,&turn_on_robot::color_joint_states_Callback, this);
  //gesture_joint_state_Sub = n.subscribe("gesture_arm", 100,&turn_on_robot::gesture_joint_states_Callback, this);
  //arm_teleop_Sub = n.subscribe("arm_teleop", 100, &turn_on_robot::arm_teleop_Callback, this);
  //voice_joint_state_Sub = n.subscribe("voice_joint_states", 100,&turn_on_robot::voice_joint_states_Callback, this); 
  //ROS_INFO_STREAM("Data ready");//ready显示状态
  
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 1, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));
	
  Joint_State_Sub = create_subscription<sensor_msgs::msg::JointState>(
      joint_states, 1, std::bind(&turn_on_robot::joint_states_Callback, this, _1));

  //RCLCPP_INFO(this->get_logger(),"create topics finish");
  //初始化串口 
  try{ 
         Stm32_Serial.setPort(usart_port_name);//选择哪个口，如果选择的口没有接串口外设初始化会失败
         Stm32_Serial.setBaudrate(serial_baud_rate);//设置波特率
         serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
         Stm32_Serial.setTimeout(_time);
         Stm32_Serial.open();//串口开启
    }
  catch (serial::IOException& e){
     RCLCPP_ERROR(this->get_logger(),"wheeltec_robot can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"wheeltec_robot serial port opened"); //Serial port opened successfully //串口开启成功提示
  }

  init_joint_states(); //开机过程机械臂运动到预设位置
}
/**************************************
Date: May 31, 2020
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  //Sends the stop motion command to the lower machine before the turn_on_robot object ends
  //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data2.tx[0]=FRAME_HEADER;
  Send_Data2.tx[1] = 0;  
  Send_Data2.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data2.tx[4] = 0;     
  Send_Data2.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data2.tx[6] = 0;
  Send_Data2.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data2.tx[8] = 0;  
  Send_Data2.tx[7] = 0;    
  Send_Data2.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data2.tx[10]=FRAME_TAIL; 

  try
  {
  // if(Receive_Data.Flag_Stop==0) 
    Stm32_Serial.write(Send_Data2.tx,sizeof (Send_Data2.tx)); //向串口发数据
    RCLCPP_ERROR(this->get_logger(),"New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //如果try失败,打印错误信息
  }
  Stm32_Serial.close();//关闭串口
  RCLCPP_INFO(this->get_logger(),"Shutting down");//close
}
