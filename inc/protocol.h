   #include <stdint.h>
   
   typedef struct  __attribute__((packed)) 
   {//符号	物理变量名称	宏标记	字节长度	数据类型（字长）	读写状态	默认值	数值范围（单位）	地址	H地址
	unsigned char ControlMode;      // 飞行器控制模式 0:保护状态  1：有操纵增稳 2：姿态控	3: 轨迹控制（位置模式)  4: 轨迹控制（速度模式)
	unsigned char FlightState;      // 飞行工作状态  0自检模式；1 停机 2：起飞模式 3：自主飞行  4：返程 5：降落模式 6：降落完成
	unsigned char bSensorState;     // 各传感器工作状态，按位显示 1正常 0不正常  bit_0: IMU  bit_1 GPS bit_2:超声波	 bit_3 接收机
	// unsigned char isTakeoff; 	    // 0:停机 1：离地
	// unsigned char HandState; 	// 0-10  机械手状态  0：收起 1：松臂中，2：空闲 3：待抓 4:抓桶成功 5 抓桶失败 6 放桶  7: 收臂	
	unsigned char FlowStatus; 	    // 任务流程
	unsigned char RobostStatue;     //  全自主状态  0:手动  1：自主控制
	unsigned char WorkMode;		    // 工作模式  0 保护停机  1:	遥控调试 2： 自主模式  3：地面站控制

	unsigned short HandPwm_H[10];   //(4) // 遥控器输入PWM信号 高电平 us长度
	short 	 IMU_R_xyz[3];          //(24) IMU 角速率 	放大 1000倍    		 rad 
	short 	 IMU_A_xyz[3];          //(30) IMU 加速度 	放大 1000倍    		 g

 	short 	 ATI_xyz[3];            //  姿态角 滚转 俯仰角 偏航 	放大 1000倍  rad
	short	 Veloc[3];	            // 放大100倍	 北东地速度
	short	 uvw[3];	            // 放大100倍	 前右下速度
	// short	 Magnet[3];// 磁航向值

	// unsigned short OutPwm_H[12];//(60)	  // 舵机高电平信号值
	 
	// short Temp[3];   //(84) 放大100倍 飞控盒温度 发动机缸1温度 发动机缸 2 温度
	unsigned short Votage[2]; //(90) 放大100倍 系统供电电池电压 舵机电池电压
	// unsigned short UltraHeiht; //(m) 放大1000倍超声波高度
//	unsigned short HandHeight; //(m ) 放大1000倍超声波高度
	// unsigned short RotorSpeed_U; //rpm  放大 10倍 上旋翼转速
	// unsigned short RotorSpeed_D; //rpm  放大 10倍 下旋翼转速
	
	
	// float PressHeight;  // (m) 气压高度
	// float ControlValue[5]; // 操纵量(ms):总距/横向/纵向/偏航/油门 	

	float Height_Control_obj; // m 预定控制高度
	float Height_Control_cur;  // m 当前飞行高度
 
	double Pos[3]; // 当前飞行器经纬度	(经度  纬度    高度)
	double Pos_Origin[10][2]; // 预设位置 (经度 纬度  )  0:起飞(原点) 
	double Pos_Rang[4][2]; 	 // 边界位置点 (经度 纬度  )

	float POS_Move[2]; // 飞机起飞点移动位移 米 [北 东] 向 

	unsigned char ImgState;     // 图像辨识 状态 0：无数据 1 ：目标丢失 2 图像定位中
	unsigned char ImgMode;      // 上电运行成功 2: 上电无图像
	short	ImgDist[3];         // short 单位 mm  范围 ±327675mm
	short	ImgVeloc[2];        //short 单位 mm/s 范围 ±327675mm/s
	short	ImgHeading;	        //相对角度	单位 ° (*0.1)  范围 ±90°(-90~90)
	// short Heading_obj; // rad
	
	// unsigned int	GpsSol_iTow; // ms :GPS millisecond Time of Week
	// short GpsSol_Week ;// GPS week(GPS time)
	// unsigned char GpsSol_gpsFix;  // Gpsfix Type  0x00 No fix; 0x01: dead reckoning only  0x02: 2D-fix; 0x03: 3D-fix;0x04:GPS+dead reckoning combined
	// unsigned char GpsSol_Flags;  // fix Status flags: 0x01:GPSfixOK;0x02:DiffSoln(is DGPS used); 0x04: WKNSET(is Week Number valid); 0x08:TOWSET(is Time of Weed valid);
	// int GpsSol_ecefX; // cm;  ECEF X coordinate
	// int GpsSol_ecefY; // cm;  ECEF Y coordinate
	// int GpsSol_ecefZ; // cm;  ECEF Z coordinate
	// unsigned int GpsSol_pAcc;  // cm; 3D position Accuracy Estimate
	// int	GpsSol_ecefVX; // cm/s  ECEF X velocity
	// int	GpsSol_ecefVY; // cm/s  ECEF Y velocity
	// int	GpsSol_ecefVZ; // cm/s  ECEF Z velocity
	// unsigned int GpsSol_sAcc; // cm/s Speed Accuracy Estimate
	
	// unsigned short GpsSol_pDOP; // 0.01 Postition DOP
	// unsigned char  GpsSol_numSV; // Number of Svs used in navigation solution;
   	char endData;
	int temp;			   // 对齐 预留

	float outOutLoopControlLaw[2];	// 速度回路输出期望 俯仰和操纵角
	float gSetVeloc_B[2];		   	// 机体下期望控制速度
	float PosError_N[2];            // 地理坐标系系位置偏差
	float PosError_B[2];            // 载体系下位置偏差

	// float Horg[3]; // 观察数据
	// float GPS2_Height;
	// double GPS2_Pos[2];	//	 移动平台的经纬度	(经度  纬度） 

			 
   	// short  GPS2_Veloc[3];	// 放大100倍	 移动平台的北东地速度
	// unsigned short GPS2_GpsSol_pDOP; // 0.01  移动平台的Postition DOP

 	// unsigned char  GPS2_GpsSol_numSV; // 移动平台的Number of Svs used in navigation solution;
	// unsigned char GPS2_GpsSol_Flags;   // 移动平台的fix Status flags: 0x01:GPSfixOK;0x02:DiffSoln(is DGPS used); 0x04: WKNSET(is Week Number valid); 0x08:TOWSET(is Time of Weed valid);
	// s16 PowerVotage;
	float Systime;
 } UAVstatus;