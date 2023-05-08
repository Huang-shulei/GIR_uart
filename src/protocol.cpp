#include "GIR_uart/protocol.hpp"

#define CRC32_POLYNOMIAL 0xEDB88320L	
float FREQ = 100.0;



unsigned long protocolSolution::CRC32Value(int i) 
{
  int j;
  unsigned long ulCRC;
  ulCRC = i;
  for ( j = 8 ; j > 0; j-- ) 
  {
    if ( ulCRC & 1 )
    {
      ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
    }
    else
    {
      ulCRC >>= 1;
    }
  }
  return ulCRC;
}


unsigned long protocolSolution::CalculateBlockCRC32( unsigned long ulCount, unsigned char *ucBuffer ) 
{
  unsigned long ulTemp1;
  unsigned long ulTemp2;
  unsigned long ulCRC = 0;
  while ( ulCount-- != 0 ) 
  {
    ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
    ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xFF );
    ulCRC = ulTemp1 ^ ulTemp2;
  }
  return( ulCRC );
}



protocolSolution::protocolSolution()  //构造函数
{
  uartDeviceConfig(); //串口设置
  //测试串口是否有缓存数据
  size_t receiverSize = uartSolver.available(); 
  if (receiverSize != 0)  //若有数据
  {
    uint8_t receiver[4095]; //4095个uint8数据的空数组
    receiverSize = uartSolver.read(receiver, receiverSize); //将已有的数据读到数组中
  }
   
  //消息流配置
  msgFlowConfig();
  
  while (ros::ok())  //循环接收
  {
    //--读取串口数据
    this->readData();  //读数据，循环读数据
  }
}



protocolSolution::~protocolSolution() //析构
{
  uartSolver.close();  //关闭串口
}



void protocolSolution::uartDeviceConfig()  //串口设置
{
  //--json参数读取配置
  ifstream(ros::package::getPath("GIR_uart") + "/include/Jason/param.json") >> param;  //读取param.json文件
  //--shell终端赋权限
  string linuxCmd;
  //给串口权限
  linuxCmd = linuxCmd + "echo " + (string)param["uartParam"].at("passWord") + " | sudo -S chmod 777 " + (string)param["uartParam"].at("usbPath");
  //运行shell命令
  system(linuxCmd.c_str());  
  //--串口配置
  serial::Timeout to = serial::Timeout::simpleTimeout(10); //通讯容错时间milliseconds级
  uartSolver.setTimeout(to);  //设置串口的等待时间
  uartSolver.setPort((string)param["uartParam"].at("usbPath")); //串口路径 ls /dev/ttyUSB* 查询可使用的串口设备
  uartSolver.setBaudrate(460800);                               //串口波特率
  //--串口异常检测
  try
  {
    uartSolver.open(); //打开串口
  }
  catch (serial::IOException &e) //捕捉异常
  {
    cerr<< "请检查串口号是否正确"<< endl; 
    exit(EXIT_FAILURE);  //return 1
  }
  if (uartSolver.isOpen())  //如果可以打开串口
  {
    cout << " 串口已经打开" << endl;  //输出打开
  }
  cout << "串口初始化完成" << endl;
}



void protocolSolution::readData()  //读取串口数据
{
  //--part1 读取串口数据到缓冲区
  size_t receiverSize = uartSolver.available();//可读取数据的大小
  if (receiverSize != 0)
  {
    uint8_t receiver[23000];
    receiverSize = uartSolver.read(receiver, receiverSize);
    for (int index = 0; index < receiverSize; index++)
    {
      uartBuffer.push_back(receiver[index]);  //将数据加入缓冲队列
    }
  }

  //--part 2缓冲区溢出处理
  if (uartBuffer.size() > 999)  //如果数据量大于999
  {
    uartBuffer.clear();  //则清空
  }

  //--part3 缓冲区数据解析
  if (uartBuffer.size() >= 230) // IMU包72字节(header28个字节、消息类型长度40个字节、最后四位为CRC校验码)+B包22位
  {
    //解析IMU包
    if (uartBuffer.at(0) == 0xAA && uartBuffer.at(1) == 0x44 &&
        uartBuffer.at(2) == 0x12 && uartBuffer.at(3) == 0x1C &&
        uartBuffer.at(8) == 0x28)  //判断帧头帧尾
    { 
      for(int i=0; i < 68; i++)
      {
        this->IMU_A_CRCbuffer[i] = uartBuffer.at(i);
      }
      unsigned long CRCle = this->CalculateBlockCRC32(sizeof(this->IMU_A_CRCbuffer), this->IMU_A_CRCbuffer);
      if(CRCle == (unsigned long)(uartBuffer[71] << 24 | uartBuffer[70] << 16 | uartBuffer[69] << 8 | uartBuffer[68]) ||
         CRCle == ((unsigned long)(uartBuffer[71] << 24 | uartBuffer[70] << 16 | uartBuffer[69] << 8 | uartBuffer[68])-(unsigned long)(18446744069414584320)))
      {
        displayTag("IMU");
        this->analyzePackageIMU();  //解析A包
      }
    }

    //解析RTK包
    if (uartBuffer.at(0) == 0xAA && uartBuffer.at(1) == 0x44 &&
        uartBuffer.at(2) == 0x12 && uartBuffer.at(3) == 0x1C &&
        uartBuffer.at(8) == 0x7E)  //判断帧头帧尾
    { 
      for(int i=0; i < 154; i++)
      {
        this->IMU_B_CRCbuffer[i] = uartBuffer.at(i);
      }
      unsigned long CRCle = this->CalculateBlockCRC32(sizeof(this->IMU_B_CRCbuffer), this->IMU_B_CRCbuffer);
      if(CRCle == (unsigned long)(uartBuffer[157] << 24 | uartBuffer[156] << 16 | uartBuffer[155] << 8 | uartBuffer[154]) ||
         CRCle == ((unsigned long)(uartBuffer[157] << 24 | uartBuffer[156] << 16 | uartBuffer[155] << 8 | uartBuffer[154])-(unsigned long)(18446744069414584320)))
      {
        displayTag("RTK");
        this->analyzePackageRTK();  //解析A包
      }
    }

    IMUReceptionPub.publish(IMUReceptionMsg);
    GNSS_RTKReceptionPub.publish(GNSS_RTKReceptionMsg);
    uartBuffer.pop_front();
  }
}




void protocolSolution::analyzePackageIMU()  //IMU包解析
{
  //yaw、pitch、roll轴角速度
  float yaw_rad_speed = (float)(uartBuffer[59] << 24 | uartBuffer[58] << 16 | uartBuffer[57] << 8 | uartBuffer[56]) / 1000000.0f * FREQ;
  float pitch_rad_speed = (float)(uartBuffer[67] << 24 | uartBuffer[66] << 16 | uartBuffer[65] << 8 | uartBuffer[64]) / 1000000.0f * FREQ;
  float roll_rad_speed = -(float)(uartBuffer[63] << 24 | uartBuffer[62] << 16 | uartBuffer[61] << 8 | uartBuffer[60]) / 1000000.0f * FREQ;
  //yaw、pitch、roll轴线速度
  float yaw_speed = (float)(uartBuffer[47] << 24 | uartBuffer[46] << 16 | uartBuffer[45] << 8 | uartBuffer[44]) / 1000000.0f;
  float pitch_speed = (float)(uartBuffer[55] << 24 | uartBuffer[54] << 16 | uartBuffer[53] << 8 | uartBuffer[52]) / 1000000.0f;
  float roll_speed = -(float)(uartBuffer[51] << 24 | uartBuffer[50] << 16 | uartBuffer[49] << 8 | uartBuffer[48]) / 1000000.0f;
  //yaw、pitch、roll轴线加速度
  float yaw_acc = yaw_speed * FREQ;
  float pitch_acc = pitch_speed * FREQ;
  float roll_acc = roll_speed * FREQ;
  //赋值给消息
  timeb t;  
  ftime(&t);
  IMUReceptionMsg.header.stamp.sec = t.time;
  IMUReceptionMsg.header.stamp.nsec = t.millitm;
  IMUReceptionMsg.angular_velocity.x = pitch_rad_speed;
  IMUReceptionMsg.angular_velocity.y = roll_rad_speed;
  IMUReceptionMsg.angular_velocity.z = yaw_rad_speed;
  IMUReceptionMsg.linear_acceleration.x = pitch_acc;
  IMUReceptionMsg.linear_acceleration.y = roll_acc;
  IMUReceptionMsg.linear_acceleration.z = yaw_acc;
}


void protocolSolution::analyzePackageRTK()  //IMU包解析
{
  //经纬度、海拔
  float lat = (float)(uartBuffer[43] << 56 | uartBuffer[42] << 48 | uartBuffer[41] << 40 | uartBuffer[40]<< 32 | uartBuffer[39] << 24 | uartBuffer[38] << 16 | uartBuffer[37] << 8 | uartBuffer[36]);
  float lon = (float)(uartBuffer[51] << 56 | uartBuffer[50] << 48 | uartBuffer[49] << 40 | uartBuffer[48]<< 32 | uartBuffer[47] << 24 | uartBuffer[46] << 16 | uartBuffer[45] << 8 | uartBuffer[44]);
  float hgt = (float)(uartBuffer[59] << 56 | uartBuffer[58] << 48 | uartBuffer[57] << 40 | uartBuffer[56]<< 32 | uartBuffer[55] << 24 | uartBuffer[54] << 16 | uartBuffer[53] << 8 | uartBuffer[52]);
  //赋值给消息
  timeb t;  
  ftime(&t);
  GNSS_RTKReceptionMsg.header.stamp.sec = t.time;
  GNSS_RTKReceptionMsg.header.stamp.nsec = t.millitm;
  GNSS_RTKReceptionMsg.latitude = lat;
  GNSS_RTKReceptionMsg.longitude = lon;
  GNSS_RTKReceptionMsg.altitude = hgt;
}


void protocolSolution::msgFlowConfig()
{
  IMUReceptionPub = n.advertise<sensor_msgs::Imu>("/IMU", 1);  //广播从IMU接收到的数据
  GNSS_RTKReceptionPub = n.advertise<sensor_msgs::NavSatFix>("/GNSS_RTK", 1); //广播从GNSS/INS-RTK接收到的数据
}


void protocolSolution::displayTag(std::string str)
{
  if (param["uartParam"].at("isDisplay") == 0)
    return;

  timeb t;  //1970年1月1日至今的秒数
  ftime(&t); //获取当前的时间和日期
  if (str == "RTK")
  {
    cout << "[" << t.time * 1000 + t.millitm << "]"
         << "RTKDataReception sending to NUC 100hz" << endl;
  }
  else if (str == "IMU")
  {
    cout << "[" << t.time * 1000 + t.millitm << "]"
         << "imuReception sending to NUC 100hz" << endl;
  }
}