#ifndef __UAV_PROTOCOL_H__
#define __UAV_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
   

typedef  float float32_t;
typedef  double float64_t;

#define SIZEREPEATARRAY    50

#define BASIC_HEADER	'B'
#define BASIC_ADDRESS	0x0000
#define BASIC_LENGTH	20

#define FLYING_HEADER	'F'
#define FLYING_ADDRESS	BASIC_ADDRESS+BASIC_LENGTH
#define FLYING_LENGTH	58

#define TRAJ_HEADER		'T'
#define TRAJ_ADDRESS	FLYING_ADDRESS+FLYING_LENGTH
#define TRAJ_LENGTH		122

#define TRAJ_APPEND_HEADER		  'A'
#define TRAJ_APPEND_ADDRESS	    TRAJ_ADDRESS+TRAJ_LENGTH
#define TRAJ_APPEND_LENGTH		  168

#define CTRL_HEADER		'C'
#define CTRL_ADDRESS	TRAJ_APPEND_ADDRESS+TRAJ_APPEND_LENGTH
#define CTRL_LENGTH		36

#define IMG_HEADER		'I'
#define IMG_ADDRESS		CTRL_ADDRESS+CTRL_LENGTH
#define IMG_LENGTH		6



#define OwnAddr  0x04; // 本设备地址号, 输出驱动和测量板		04?
#define GCSAddr  0x08 //	地面站作为接收者地址


typedef struct  __attribute__((packed)) 
{
	uint8_t header[2];
	uint8_t length;
	unsigned short pose_index;
	double Pos_Origin[3]; // 预设位置 (经度 纬度  )  0:起飞(原点) 
}flightPoint;


typedef struct  __attribute__((packed)) 
{
	unsigned char ControlMode;      // 飞行器控制模式 0:保护状态  1：有操纵增稳 2：姿态控	3: 轨迹控制（位置模式)  4: 轨迹控制（速度模式)
	unsigned char FlightState;      // 飞行工作状态  0自检模式；1 停机 2：起飞模式 3：自主飞行  4：返程 5：降落模式 6：降落完成

	unsigned char HandState;		//
	unsigned char FlowStatus; 	    // 任务流程
	unsigned char RobostStatue;     //  全自主状态  0:手动  1：自主控制
	unsigned char WorkMode;		    // 工作模式  0 保护停机  1:	遥控调试 2： 自主模式  3：地面站控制

	uint32_t Systime;					//
	float motorSpeed;				//
	unsigned short System_vol;		//
	unsigned short Motor_vol;		//
	unsigned short sysTemp;   		//


	short 	 gyro_xyz[3];          //(24) IMU 角速率 	放大 1000倍    		 rad 
	short 	 accl_xyz[3];          //(30) IMU 加速度 	放大 1000倍    		 g

	short 	 	atti[3];            //  姿态角 滚转 俯仰角 偏航 	放大 1000倍  rad
	short	 	veloN[3];	        // 放大100倍	 北东地速度
	short	 	veloB[3];	        // 放大100倍	 前右下速度
	double 		posi[3]; 			// 当前飞行器经纬度	(经度  纬度    高度)
	
	unsigned char 	GpsSol_Flags;   //
	unsigned short 	GpsSol_pDOP; 	// 0.01  移动平台的Postition DOP
	unsigned char  	GpsSol_numSV; 	// 移动平台的Number of Svs used in navigation solution;
	
	unsigned short pose_index;
	double Pos_Origin[12][3]; // 预设位置 (经度 纬度  )  0:起飞(原点) 

	float	roll_obj;
	float 	pitch_obj;
	float 	yaw_obj;
	float 	veloB_x_obj;
	float 	veloB_y_obj;
	float 	veloB_z_obj;
	float 	posiB_x_obj;
	float 	posiB_y_obj;
	float 	posiB_z_obj;


	unsigned char ImgState;     // 图像辨识 状态 0：无数据 1 ：目标丢失 2 图像定位中
	unsigned char ImgMode;      // 上电运行成功 2: 上电无图像
	short	ImgDist[2];         // short 单位 mm  范围 ±327675mm


} UAVstatus;






// 发送数据队列格式
typedef struct  __attribute__((packed)) 
{
	uint8_t index;
	uint8_t enable;  		// 是否处于激活状态  0：失效状态  1：使能状态
	uint16_t n_1ms;		    // 该队列 运行时间，定义运行时间变量，最小分辨率为1ms
	uint16_t period_ms;		// 该队列 连续发送周期，指所要求的定时发送周期
	uint16_t regaddr;		// 寄存器变量起始地址
	uint16_t objAddr;		// 寄存器变量起始地址  
	uint8_t  length;			// 数据长度
	uint8_t  header;          //(方)添加，用于判断将要打包发送的数据属于哪个数据文件中

} sendbufQ;


typedef struct QuaternionData
{
  float32_t q0;
  float32_t q1;
  float32_t q2;
  float32_t q3;
} QuaternionDataU;

//! @warning this struct will be deprecated in the next release and renamed to Vector3fData. Use Vector3fData instead.
typedef struct __attribute__((packed)) 
{
  float32_t x;
  float32_t y;
  float32_t z;
} CommonDataU;

//! @note this struct will replace CommonData in the next release.
//! Eigen-like naming convention
typedef struct __attribute__((packed)) 
{
  float32_t x;
  float32_t y;
  float32_t z;
} Vector3fDataU;

typedef struct __attribute__((packed)) 
{
  float32_t x;
  float32_t y;
  float32_t z;
  uint8_t health : 1;
  uint8_t sensorID : 4;
  uint8_t reserve : 3;
} VelocityDataU;

typedef struct __attribute__((packed)) 
{
  float64_t latitude;
  float64_t longitude;
  //! @warning the 'altitude' field will be renamed in a future release.
  //! @note the altitude value can be configured to output GPS-only data
  //!       or a fusion of GPS and Baro in Assistant 2's SDK Tab, 'ALTI' 
  float32_t altitude;

  //! @warning the 'height' field will be renamed in a future release.
  //! @note the height value can be configured to output AGL height
  //!       or height relative to takeoff in Assistant 2's SDK Tab, 'HEIGHT'
  float32_t height;

  uint8_t health;
} PositionDataU;

 //! @warning the 'RadioData' struct will be deprecated in the next release and renamed to RCData. Use RCData instead.
typedef struct __attribute__((packed)) 
{
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
  int16_t mode;
  int16_t gear;
} RadioDataU;

//! @note This struct will replace RadioData in the next release. 
typedef struct __attribute__((packed)) 
{
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
  int16_t mode;
  int16_t gear;
} RCDataU;

//! @warning the 'MagnetData' struct will be deprecated in the next release and renamed to MagData. Use MagData instead.
typedef struct __attribute__((packed)) 
{
  int16_t x;
  int16_t y;
  int16_t z;
} MagnetDataU;

//! @note This struct will replace MagnetData in the next release.
typedef struct __attribute__((packed)) 
{
  int16_t x;
  int16_t y;
  int16_t z;
} MagDataU;

//! @note This struct is provided as a means for users to provide sigle GPS points to the SDK.
//!       It does not follow standard SDK GPS datatypes. This may change in a future release. 
typedef struct __attribute__((packed)) 
{
  float64_t latitude;
  float64_t longitude;
  //! @warning please provide relative height in the altitude field. The name will change in a future release.
  float64_t altitude;

} GPSPositionDataU;

typedef struct __attribute__((packed)) 
{
  uint8_t mode;
  //! @todo mode remote to enums
  uint8_t deviceStatus : 3; /*0->rc  1->app  2->serial*/
  uint8_t flightStatus : 1; /*1->opensd  0->close*/
  uint8_t vrcStatus : 1;
  uint8_t reserved : 3;
} CtrlInfoDataU;

typedef struct __attribute__((packed)) 
{
  //! @todo type modify
  uint32_t time;
  uint32_t nanoTime;
  uint8_t syncFlag;
} TimeStampDataU;

typedef struct __attribute__((packed)) 
{
  float32_t roll;
  float32_t pitch;
  float32_t yaw;
  uint8_t pitchLimit : 1;
  uint8_t rollLimit : 1;
  uint8_t yawLimit : 1;
  uint8_t reserved : 5;
} GimbalDataU;

typedef uint8_t FlightStatus;

typedef struct __attribute__((packed)) 
{
  unsigned char cmdSequence;
  unsigned char cmdData;
} TaskDataU;

//! @todo rename to a final version
//! RTKData from the A3. This is not available on the M100.
typedef struct __attribute__((packed)) 
{
  uint32_t date;
  uint32_t time;
  float64_t longitude;
  float64_t latitude;
  //! @warning the 'Hmsl' field will be renamed in a future release.
  float32_t Hmsl;

  float32_t velocityNorth;
  float32_t velocityEast;
  //! @warning the 'velocityGround' field will be renamed to velocityDown in the next release.
  float32_t velocityGround;
  
  int16_t yaw;
  uint8_t posFlag;
  uint8_t yawFlag;

} RTKDataU;

//! @todo rename to a final version
//! Detailed GPSData from the A3. This is not available on the M100.
typedef struct __attribute__((packed)) 
{
  uint32_t date;
  uint32_t time;
  int32_t longitude;
  int32_t latitude;
  //! @warning the 'Hmsl' field will be renamed in a future release.    
  int32_t Hmsl;

  float32_t velocityNorth;
  float32_t velocityEast;
  //! @warning the 'velocityGround' field will be renamed to velocityDown in the next release.
  float32_t velocityGround;

} GPSDataU;



//! @todo
typedef struct __attribute__((packed)) 
{
  unsigned short dataFlag;
  TimeStampDataU timeStamp;
  QuaternionDataU q;
  //! @warning the CommonData type will change to Vector3fData in a future release
  CommonDataU a;
  VelocityDataU v;
  //! @warning the CommonData type will change to Vector3fData in a future release
  CommonDataU w;
  PositionDataU pos;
  //! @warning the MagnetData type will change to MagData in a future release
  MagnetDataU mag;
  GPSDataU gps;
  RTKDataU rtk;
  //! @warning the RadioData type will change to RCData in a future release
  RadioDataU rc;
  GimbalDataU gimbal;
  uint8_t status; //! @todo define enum
  uint8_t battery;
  CtrlInfoDataU ctrlInfo;

  //! @note this variable is not set by the FC but populated by the API
  uint8_t activation;
} BroadcastDataU;


void CommProtocol_init(void);
void CommProtocol_task(void);

void received_task(char *rec_buf,uint8_t len);
void getFlowPosition(unsigned short index,double *posi);

void setUAVstatus(uint8_t *onboard,uint32_t len);
void getUAVstatus(void);

uint8_t getControlStatus(void);

void setImageStatus(uint8_t *img);
void setFlowStatus(uint8_t *flow);

#ifdef __cplusplus
}
#endif

 #endif