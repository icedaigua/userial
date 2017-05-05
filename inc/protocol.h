#ifndef __UAV_PROTOCOL_H__
#define __UAV_PROTOCOL_H__

#include <stdint.h>
   

#define SIZEREPEATARRAY    50

#define BASIC_HEADER	'B'
#define BASIC_ADDRESS	0x0000
#define BASIC_LENGTH	20

#define FLYING_HEADER	'F'
#define FLYING_ADDRESS	0x0100
#define FLYING_LENGTH	59

#define TRAJ_HEADER		'T'
#define TRAJ_ADDRESS	0x0200
#define TRAJ_LENGTH		80

#define CTRL_HEADER		'C'
#define CTRL_ADDRESS	0x0300
#define CTRL_LENGTH		36

#define IMG_HEADER		'I'
#define IMG_ADDRESS		0x0400
#define IMG_LENGTH		7



#define OwnAddr  0x04; // 本设备地址号, 输出驱动和测量板		04?
#define GCSAddr  0x08 //	地面站作为接收者地址

typedef struct  __attribute__((packed)) 
{
	unsigned char ControlMode;      // 飞行器控制模式 0:保护状态  1：有操纵增稳 2：姿态控	3: 轨迹控制（位置模式)  4: 轨迹控制（速度模式)
	unsigned char FlightState;      // 飞行工作状态  0自检模式；1 停机 2：起飞模式 3：自主飞行  4：返程 5：降落模式 6：降落完成

	unsigned char HandState;		//
	unsigned char FlowStatus; 	    // 任务流程
	unsigned char RobostStatue;     //  全自主状态  0:手动  1：自主控制
	unsigned char WorkMode;		    // 工作模式  0 保护停机  1:	遥控调试 2： 自主模式  3：地面站控制

	float Systime;					//
	float motorSpeed;				//
	unsigned short System_vol;		//
	unsigned short Motor_vol;		//
	unsigned short sysTemp;   		//(4) // 遥控器输入PWM信号 高电平 us长度


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
	double Pos_Origin[5][2]; // 预设位置 (经度 纬度  )  0:起飞(原点) 

	float	roll_obj;
	float 	pitch_obj;
	float 	yaw_obj;
	float 	veloB_x_obj;
	float 	veloB_y_obj;
	float 	veloB_z_obj;
	float 	posiB_x_obj;
	float 	posiB_x_obj;
	float 	posiB_x_obj;


	unsigned char ImgState;     // 图像辨识 状态 0：无数据 1 ：目标丢失 2 图像定位中
	unsigned char ImgMode;      // 上电运行成功 2: 上电无图像
	short	ImgDist[3];         // short 单位 mm  范围 ±327675mm


} UAVstatus;



// 发送数据队列格式
typedef struct  __attribute__((packed)) 
{
	uint8_t index;
	uint8_t enable;  		// 是否处于激活状态  0：失效状态  1：使能状态
	uint16_t n_1ms;		    // 该队列 运行时间，定义运行时间变量，最小分辨率为1ms
	uint16_t period_ms;		// 该队列 连续发送周期，指所要求的定时发送周期
	uint16_t regaddr;		// 寄存器变量起始地址 
	uint8_t length;			// 数据长度
	char header;          //(方)添加，用于判断将要打包发送的数据属于哪个数据文件中

	// struct sendbufQ *next;
} sendbufQ;

 

 #endif