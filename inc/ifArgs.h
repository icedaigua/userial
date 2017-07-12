#include <stdint.h>

typedef  float float32_t;
typedef  double float64_t;


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


typedef struct __attribute__((packed))
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


typedef struct
{
  double yaw;
  double roll;
  double pitch;
}EulerAngleU;