
#include "protocol.h"
#include "gcs_thread.h"
#include "serialib.h"

#include <sys/time.h>
#include <string.h>
#include <math.h>

void FlowPosition_init(void);

void SetSendBufferData(sendbufQ iArray);
uint8_t* GetRegAddress(char cHeader, uint16_t regaddr);
uint16_t CalBaseRegAddress(char cHeader, uint16_t regaddr);
void SetSendBuffer(uint8_t Master,uint8_t Slave,uint16_t RegAddr,uint8_t ByteLength,
					uint8_t *data,uint8_t ReturnFlag);

void SetContiuneDefaultData(void);
void SetSendingData(char header, uint16_t period_ms, uint16_t regaddr, uint8_t length);
uint8_t MdfRepeatArray(sendbufQ *pNewRepeat);


void setFlightPonit(flightPoint fP);
void ReceivedComPortDataEvent(char *buf);
void PutFunction02(char *buf);
void PutFunction03(char *buf);
void stopRepeatArray(void);

void QtoEulerAngle(QuaternionDataU quaternionData,short int *atti);


sendbufQ sendbufArray[SIZEREPEATARRAY];

UAVstatus m600Status;

BroadcastDataU onBoardStatus;


char RecvBuff[100]={0};

uint8_t controlStatus = 0;

void CommProtocol_task(void)
{

	 uint8_t i;

	 for(i=0;i<SIZEREPEATARRAY;i++)
	 {
		
	 	if(sendbufArray[i].enable ==1)			  //发送使能
		{
			sendbufArray[i].n_1ms ++;			  //时间变量累加1ms

			if(sendbufArray[i].n_1ms>=sendbufArray[i].period_ms)	//时间到达发送定时周期
			{													
				sendbufArray[i].n_1ms = 0;		//时间变量复位，待发送数据帧进入收发缓冲池

				SetSendBufferData(sendbufArray[i]);	      				
			}
			
		}
	 }
  
}


void received_task(char *rec_buf,uint8_t len)
{
	uint8_t kc = 0;
	flightPoint fP = {0};
	for(kc=0;kc<len-1;kc++)
	{
		if((rec_buf[kc]=='$')&&(rec_buf[kc+1]=='P'))
			break;
		// continue;
        if((rec_buf[kc]=='$')&&(rec_buf[kc+1]=='C'))
        {
            controlStatus = 1;
			printf("received take off cmd\n");			
            return;
        }
	}
	printf("received cmd\n");
	if(kc<(len-1))
	{	
		memcpy((char *)&fP,&rec_buf[kc],rec_buf[kc+2]);

		setFlightPonit(fP);
	}
}

void setFlightPonit(flightPoint fP)
{
	if(fP.pose_index == 20)
		m600Status.pose_index = fP.pose_index - 10;
	else
		m600Status.pose_index = fP.pose_index;
	
	m600Status.Pos_Origin[m600Status.pose_index][0]
						= fP.Pos_Origin[0];
	m600Status.Pos_Origin[m600Status.pose_index][1]
						= fP.Pos_Origin[1];
	m600Status.Pos_Origin[m600Status.pose_index][2]
						= fP.Pos_Origin[2];
}





void CommProtocol_init(void)
{
	uint8_t i;
	for(i=0;i<SIZEREPEATARRAY;i++)
	{
		sendbufArray[i].index  = i;
		sendbufArray[i].enable = 0;
	}

	SetContiuneDefaultData();


	FlowPosition_init();

}

void SetSendBufferData(sendbufQ iArray)
{
	 SetSendBuffer(0xA0, iArray.header,
	 	CalBaseRegAddress(iArray.header, iArray.regaddr),
	 	iArray.length,											//发送基地址,字节长度
	 	GetRegAddress(iArray.header, iArray.regaddr),0);		//发送的起始数据地址指针,返回标志，帧头	  

}


void SetSendBuffer(uint8_t Master,uint8_t Slave,uint16_t RegAddr,uint8_t ByteLength,
					uint8_t *data,uint8_t ReturnFlag)
{ //数据打包过程 
	char m_btSendBuffer[255]={0};
	uint8_t nIndex=0;
	uint8_t sum = 0;
	uint8_t i = 0;

	m_btSendBuffer[0] = '$';							//cHeader;
	m_btSendBuffer[1] = Slave;//(((Master<< 4) & 0x0F0) | (Slave & 0x0F));
	m_btSendBuffer[2] = ByteLength;        				//表示的是变量字节数，不含头部和尾部
	m_btSendBuffer[3] = 0xA2;							//功能码
	m_btSendBuffer[4] = (uint8_t)(RegAddr&0x0FF);			//基地址低8位
	m_btSendBuffer[5] = (uint8_t)((RegAddr&0x0FF00)>>8);	//基地址高8位
	// m_btSendBuffer[4] = 0x30;			//基地址低8位
	// m_btSendBuffer[5] = 0x31;	//基地址高8位


	if(ByteLength>0)
	{
		for (nIndex = 0;nIndex < ByteLength; nIndex++)
		{
			m_btSendBuffer[6+nIndex] = *(data+nIndex);
		}
	}
	m_btSendBuffer[nIndex+6] = ReturnFlag;
	for (i = 0; i < nIndex+7; i++)
	{
	    sum += m_btSendBuffer[i];
	}
	m_btSendBuffer[nIndex+7] = sum;

	// printf("sendbuf is \n");
	// for(int kc=0;kc<20;kc++)
	// 	printf(" %2X	\t",m_btSendBuffer[kc]);
	// printf("\n");

	serial_writesb(get_local_port(),m_btSendBuffer, ByteLength+8);
	// serial_write(get_local_port(),m_btSendBuffer);


}

uint16_t CalBaseRegAddress(char cHeader, uint16_t regaddr)
{

	if(cHeader=='B') 		return BASIC_ADDRESS + regaddr;
	else if(cHeader=='F') 	return FLYING_ADDRESS + regaddr;
	else if(cHeader=='C')	return CTRL_ADDRESS + regaddr;
	else if(cHeader=='T')	return TRAJ_ADDRESS + regaddr;
	else if(cHeader=='A')	return TRAJ_APPEND_ADDRESS + regaddr;
	else if(cHeader=='I')	return IMG_ADDRESS + regaddr;
	else return 0xFFFF;
	
	//for(i=0;i<index;i++)
	//{
	//	base += Base[i];
	//}

	//return regaddr+base;
}


uint8_t* GetRegAddress(char cHeader, uint16_t regaddr)
{//确定发送的起始数据地址指针


	if (cHeader == 'B') 		return (uint8_t *)&m600Status+BASIC_ADDRESS + regaddr;
	else if (cHeader == 'F') 	return (uint8_t *)&m600Status+FLYING_ADDRESS + regaddr;
	else if (cHeader == 'C')	return (uint8_t *)&m600Status+CTRL_ADDRESS + regaddr;
	else if (cHeader == 'T')	return (uint8_t *)&m600Status+TRAJ_ADDRESS + regaddr;
	else if (cHeader == 'A')	return (uint8_t *)&m600Status+TRAJ_APPEND_ADDRESS + regaddr;
	else if (cHeader == 'I')	return (uint8_t *)&m600Status+IMG_ADDRESS + regaddr;
	else return NULL;

}


// @brief  设置默认连续数据上报
void SetContiuneDefaultData(void)
{

	SetSendingData('B', 1, 0, BASIC_LENGTH);		//自己添加的实验数据，返回控制模式等数据
	SetSendingData('F', 5, 0, FLYING_LENGTH);		//自己添加的实验数据，返回控制模式等数据
	SetSendingData('T', 5, 0, TRAJ_LENGTH);		//自己添加的实验数据，返回控制模式等数据
	SetSendingData('A', 5, 0, TRAJ_APPEND_LENGTH);		//自己添加的实验数据，返回控制模式等数据		
	SetSendingData('C', 5, 0, CTRL_LENGTH);		//自己添加的实验数据，返回控制模式等数据	
	SetSendingData('I', 5, 0, IMG_LENGTH);		//自己添加的实验数据，返回控制模式等数据		
}



void SetSendingData(char header, uint16_t period_ms, uint16_t regaddr, uint8_t length)
{
	sendbufQ newSendbuf;
	
	newSendbuf.enable 		= 1;
	newSendbuf.n_1ms	 	= 0;	  		//定时变量置0
 	newSendbuf.objAddr		= GCSAddr;		//地面站作为接收者地址	0x08


	newSendbuf.header 	= header;		//
	newSendbuf.period_ms	= period_ms;	//
	newSendbuf.regaddr   = regaddr;		//
	newSendbuf.length    = length; 		//
	MdfRepeatArray(&newSendbuf);	
}

/**
  * @brief  修改定时发送队列
  * @param   pNewRepeat 需要修改/添加的队列元素
  * @retval : 0：失败（超容量）  1：成功	 
  */
uint8_t MdfRepeatArray(sendbufQ *pNewRepeat)
{
	uint8_t i;
	// 遍历所有， 先遍历原来就有的，如果数据缓冲池中有这数据的内存空间，则检查其值及周期，将其更新
	for(i=0;i<SIZEREPEATARRAY;i++)
	{	  	// 地址和长度相同，修改周期列表 
		if(	(sendbufArray[i].enable==1)&&
		  	(sendbufArray[i].regaddr==pNewRepeat->regaddr)&&
		  	(sendbufArray[i].length==pNewRepeat->length)&&
		  	(sendbufArray[i].header==pNewRepeat->header))
		{
			if(pNewRepeat->period_ms==0)    // 特殊 ，删除
			{
				sendbufArray[i].enable = 0; // 删除 ;
				return 1;
			}
			else
			{
				sendbufArray[i].period_ms = pNewRepeat->period_ms; // 修改周期
				return 1;	
			}
		}
	}

	// 添加新的
	if(pNewRepeat->period_ms<1) return 0; 

	for(i=0;i<SIZEREPEATARRAY;i++)
	{
		if(sendbufArray[i].enable ==0)			//寻找周期发送已停止的单元
			break;
		else 
			continue;
	}
	
	if(i>=SIZEREPEATARRAY) return 0;
		
	sendbufArray[i].n_1ms = 		pNewRepeat->n_1ms;
	sendbufArray[i].period_ms = 	pNewRepeat->period_ms;
	sendbufArray[i].regaddr = 	pNewRepeat->regaddr;
	sendbufArray[i].length = 	pNewRepeat->length;

	sendbufArray[i].objAddr = 	pNewRepeat->objAddr;
	sendbufArray[i].header = 	pNewRepeat->header;
	
	sendbufArray[i].enable = 1;

	return 1;
}


void stopRepeatArray(void)
{
	uint8_t i;
	for(i=0;i<SIZEREPEATARRAY;i++)
	{
		sendbufArray[i].enable = 0;	   //停止所有的定时周期发送任务
	}
}



void setUAVstatus(uint8_t *onboard,uint32_t len)
{
  

	int kc =0;
	//onBoardStatus = onboard;
	memcpy((uint8_t *)&onBoardStatus,onboard,len);

    struct timeval tv;
    gettimeofday(&tv, NULL);
	//printf("gps : %u	%u\n",onBoardStatus.gps.date,onBoardStatus.gps.longitude);

	//printf("rtk : %u	%10.7f\n",onBoardStatus.rtk.date,onBoardStatus.rtk.longitude);
	

	m600Status.ControlMode   = 1;  
	m600Status.FlightState   = 2;      

	m600Status.HandState     = 3;		
	//m600Status.FlowStatus    = 4; 	    
	m600Status.RobostStatue  = 5;     
	m600Status.WorkMode      = 6;		    

    m600Status.Systime	    = tv.tv_sec;//onBoardStatus.timeStamp.time;

    m600Status.motorSpeed   = 0.0;

    m600Status.System_vol	= onBoardStatus.battery;
	m600Status.Motor_vol	= 8;		
	m600Status.sysTemp		= 9;   		

    m600Status.gyro_xyz[0] = (short)(onBoardStatus.w.x*1000);
    m600Status.gyro_xyz[1] = (short)(onBoardStatus.w.y*1000);
    m600Status.gyro_xyz[2] = (short)(onBoardStatus.w.z*1000);


    m600Status.accl_xyz[0] = (short)(onBoardStatus.a.x*1000);
    m600Status.accl_xyz[1] = (short)(onBoardStatus.a.y*1000);
    m600Status.accl_xyz[2] = (short)(onBoardStatus.a.z*1000);

    QtoEulerAngle(onBoardStatus.q,m600Status.atti);

	for ( kc = 0; kc < 3; kc++)
	{
//        m600Status.atti[kc]		= 16+kc;
        m600Status.veloB[kc]	= 22+kc;		
	}

	m600Status.veloN[0]    =(short)(onBoardStatus.v.x*1000);
	m600Status.veloN[1]    =(short)(onBoardStatus.v.y*1000);
	m600Status.veloN[2]    =(short)(onBoardStatus.v.z*1000);

	m600Status.posi[0]    =onBoardStatus.pos.longitude;
	m600Status.posi[1]    =onBoardStatus.pos.latitude;
	m600Status.posi[2]    =onBoardStatus.pos.altitude;

	
	m600Status.GpsSol_Flags = onBoardStatus.rtk.posFlag;  
	m600Status.GpsSol_pDOP	= 26; 	
	m600Status.GpsSol_numSV	= 27; 	
	


	for( kc=0;kc<2;kc++)
		m600Status.ImgDist[kc]=53+kc;  

}

void QtoEulerAngle(QuaternionDataU quaternionData, short int *atti)
{
//  EulerAngleU ans;

  double q2sqr = quaternionData.q2 * quaternionData.q2;
  double t0 = -2.0 * (q2sqr + quaternionData.q3 * quaternionData.q3) + 1.0;
  double t1 = +2.0 * (quaternionData.q1 * quaternionData.q2 + quaternionData.q0 * quaternionData.q3);
  double t2 = -2.0 * (quaternionData.q1 * quaternionData.q3 - quaternionData.q0 * quaternionData.q2);
  double t3 = +2.0 * (quaternionData.q2 * quaternionData.q3 + quaternionData.q0 * quaternionData.q1);
  double t4 = -2.0 * (quaternionData.q1 * quaternionData.q1 + q2sqr) + 1.0;

  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;

    atti[0] = (short)(asin(t2)*1000.0);
    atti[1]  = (short)(atan2(t3, t4)*1000.0);
    atti[2]  = (short)(atan2(t1, t0)*1000.0);


//  ans.pitch = asin(t2);
//  ans.roll = atan2(t3, t4);
//  ans.yaw = atan2(t1, t0);

//  return ans;
}

void setImageStatus(int *img)
{
    m600Status.ImgState = (char)img[0];

//	m600Status.ImgMode	= img[1];
}

void setFlowStatus(uint8_t *flow)
{
	m600Status.FlowStatus = flow[0];    
//	m600Status.ImgMode	= img[1];
}

uint8_t getControlStatus(void)
{
  return controlStatus;
}

void getFlowPosition(unsigned short index,double *posi)
{
	posi[0] = m600Status.Pos_Origin[index][0];
	posi[1] = m600Status.Pos_Origin[index][1];
    posi[2] = m600Status.Pos_Origin[index][2];
}

void FlowPosition_init(void){
	m600Status.Pos_Origin[0][0] = 0.0; 
	m600Status.Pos_Origin[0][1] = 0.0;

	m600Status.Pos_Origin[1][0] = 0.0; 
	m600Status.Pos_Origin[1][1] = 0.0;

	m600Status.Pos_Origin[2][0] = 0.0; 
	m600Status.Pos_Origin[2][1] = 0.0;

	m600Status.Pos_Origin[3][0] = 0.0; 
	m600Status.Pos_Origin[3][1] = 0.0;

	m600Status.Pos_Origin[4][0] = 0.0; 
	m600Status.Pos_Origin[4][1] = 0.0;
}

void ReceivedComPortDataEvent(char *buf)
{
	switch(buf[2])
	{
		case 0x02:	   	// 定时返回请求
			PutFunction02(buf);
			break;
		case 0x03:	   	// 读寄存器命令
		
			break;
		case 0x06:		// 写寄存器命令	
			PutFunction03(buf);
			break;
		case 0x07: 		// 请求应答返回
			break;
		case 0x08:	 	// 异常突发事件
			break;
		default:

			break;
	}				
}															

void PutFunction02(char *buf)
{
	sendbufQ newSendbuf;
	newSendbuf.enable = 1;
	newSendbuf.n_1ms	 = 0;	  //定时变量置0
	newSendbuf.period_ms	= buf[6]|(buf[7]<<8);
	newSendbuf.regaddr   = buf[4]|(buf[5]<<8);
	newSendbuf.length    = buf[3];
	newSendbuf.objAddr	= (buf[1]>>4)&0x0f;
    newSendbuf.header = buf[0];	  //保存帧头字符

	if(newSendbuf.regaddr==0x0FFFF)	//特定基地址 
		stopRepeatArray();	  //停止所有定时周期发送任务
	else 
		MdfRepeatArray(&newSendbuf);
}
void PutFunction03(char *buf)
{
	memcpy((uint8_t *)&m600Status+TRAJ_ADDRESS+buf[4]*2*sizeof(double),&buf[5],2*sizeof(double));
}



void getUAVstatus(void)
{
	// time_t now;
	// struct tm *timenow;

	// time(&now);
	// timenow = localtime(&now);
  
    int kc =0,kj = 0;

	static uint32_t now = 0;

	m600Status.ControlMode   = 1;  
	m600Status.FlightState   = 2;      

	m600Status.HandState     = 3;		
	m600Status.FlowStatus    = 4; 	    
	m600Status.RobostStatue  = 5;     
	m600Status.WorkMode      = 6;		    

	now+=1;
	m600Status.Systime	    = now;
	m600Status.motorSpeed   = 123.456;		
	m600Status.System_vol	= 7;		
	m600Status.Motor_vol	= 8;		
	m600Status.sysTemp		= 9;   		

	for (kc = 0; kc < 3; kc++)
	{
		m600Status.gyro_xyz[kc] = 10+kc;
		m600Status.accl_xyz[kc] = 13+kc;

		m600Status.atti[kc]		= 16+kc;
		m600Status.veloN[kc]    =19+kc;
		m600Status.veloB[kc]	=22+kc;
		m600Status.posi[kc]		=12.34*kc;
	}
	m600Status.GpsSol_Flags = 25;  
	m600Status.GpsSol_pDOP	= 26; 	
	m600Status.GpsSol_numSV	= 27; 	
	
	m600Status.pose_index = 30;
    for(kc=0;kc<12;kc++)
        for(kj=0;kj<3;kj++)
	{
			m600Status.Pos_Origin[kc][kj] = (kc+kj)*1.0;
	}
	

	m600Status.roll_obj	= 41.0;
	m600Status.pitch_obj	= 42.0;
	m600Status.yaw_obj	= 43.0;
	m600Status.veloB_x_obj = 44.0;
	m600Status.veloB_y_obj = 45.0;
	m600Status.veloB_z_obj = 46.0;
	m600Status.posiB_x_obj = 47.0;
	m600Status.posiB_y_obj = 48.0;
	m600Status.posiB_z_obj = 49.0;


	m600Status.ImgState = 51;    
	m600Status.ImgMode	= 52;
	for( kc=0;kc<2;kc++)
		m600Status.ImgDist[kc]=53+kc;  

	
}
