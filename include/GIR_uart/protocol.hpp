//--ros库函数
#include "ros/ros.h"
#include <serial/serial.h>
#include <ros/package.h>
//--c++库函数
#include "bits/stdc++.h"
using namespace std;
#include <sys/timeb.h>
//参数读取
#include "Jason/jason.hpp"
using json = nlohmann::json;
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <array>
#include <string>

class protocolSolution  //协议解决器类
{
public:
  //--用户调用接口
  protocolSolution();  //构造
  ~protocolSolution();  //析构

private:
  //--串口配置
  void uartDeviceConfig(void);
  //--信息流
  void msgFlowConfig(void);
  //--串口读取数据
  void readData(void);
  //--解析
  void analyzePackageRTK(); 
  void analyzePackageIMU();
  //--终端tag
  void displayTag(std::string str);
  //--串口缓冲区
  deque<uint8_t> uartBuffer;
  deque<uint8_t> uartBuffer_1;
  //--串口配置器
  serial::Serial uartSolver;
  //jason参数
  json param;
  //--ros配置
  ros::NodeHandle n;
  //发布传感器信息
  ros::Publisher IMUReceptionPub;
  //发布裁判系统信息
  ros::Publisher GNSS_RTKReceptionPub;
  //IMU、RTK消息
  sensor_msgs::NavSatFix GNSS_RTKReceptionMsg;
  sensor_msgs::Imu IMUReceptionMsg;
  //CRC校验
  unsigned char IMU_A_CRCbuffer[68];
  unsigned char IMU_B_CRCbuffer[154];
  unsigned long CRC32Value(int i);
  unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer );
};