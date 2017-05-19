
#include "protocol.h"
#include "gcs_thread.h"
#include "serialib.h"

#include <string.h>

void SetSendBufferData(sendbufQ iArray);
uint8_t* GetRegAddress(char cHeader, uint16_t regaddr);
uint16_t CalBaseRegAddress(char cHeader, uint16_t regaddr);
void SetSendBuffer(uint8_t Master,uint8_t Slave,uint16_t RegAddr,uint8_t ByteLength,
					uint8_t *data,uint8_t ReturnFlag);

void SetContiuneDefaultData(void);
void SetSendingData(char header, uint16_t period_ms, uint16_t regaddr, uint8_t length);
uint8_t MdfRepeatArray(sendbufQ *pNewRepeat);

void analysisBuf(char R_data);
void ReceivedComPortDataEvent(char *buf);
void PutFunction02(uint8_t *buf);
void PutFunction03(uint8_t *buf);
void stopRepeatArray(void);

uint16_t Base[5]={BASIC_LENGTH,FLYING_LENGTH,TRAJ_LENGTH,CTRL_LENGTH,IMG_LENGTH};
sendbufQ sendbufArray[SIZEREPEATARRAY];

UAVstatus m600Status;

uint8_t sendbuf[10][100]={"ABCDEF","1234567",
"abcdefgh"
};

char RecvBuff[100]={0};

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
	for(int kc=0;kc<len;kc++)
		analysisBuf(rec_buf[kc]);
}

void analysisBuf(char R_data)
{
	static uint8_t R_state = 0,checksum =0,rec_cnt = 0;
	static uint8_t inbuf_len = 0;
    switch (R_state)				  //接收数据帧解包用的结构变量
    {
        case 0:
            {        
                if (R_data=='$')		//header0
                {
					rec_cnt = 0;
					checksum  = 0;

                    RecvBuff[rec_cnt++] = R_data; 
					checksum +=  R_data;
					
					R_state = 1;
                }		
                break;
            } 
        case 1:
            {
				if (R_data=='T')
                {
                	RecvBuff[rec_cnt++] = R_data; 			
					checksum += R_data;
					R_state=2; 
				}
				else
				{
					R_state = 0;
				}
				break;
            }
       case 2:
            {
                RecvBuff[rec_cnt++] = R_data; 		 // 功能码字节		
				checksum += R_data;
				R_state=3; 
				break ;               
            }
       case 3:
            {
              	RecvBuff[rec_cnt++] = R_data; 		 // length
				checksum += R_data;
				inbuf_len = R_data;
				
				R_state=4; 
				break ;
            }
		case 4:
            {
                RecvBuff[rec_cnt++] = R_data; 		 // data
				checksum += R_data;
				if(rec_cnt >= inbuf_len+4)
				{
					R_state=5; 	
				}
                break;
            }			
        case 5: //接收数据
            { //	RecvBuff 	 接收的字符串缓冲器
             	RecvBuff[rec_cnt++] = R_data; 		 // data
            	if(checksum == R_data) // 校验和通过
				{
					ReceivedComPortDataEvent(RecvBuff);
				}
				R_state = 0;
            	break ;
            }
        default:
            {
                R_state = 0;
                break ;
            }
    }
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

void PutFunction02(uint8_t *buf)
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
void PutFunction03(uint8_t *buf)
{
	memcpy((uint8_t *)&m600Status+TRAJ_ADDRESS+buf[4]*2*sizeof(double),&buf[5],2*sizeof(double));
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

}

void SetSendBufferData(sendbufQ iArray)
{
	 SetSendBuffer(0xA0, iArray.header,
	 	CalBaseRegAddress(iArray.header, iArray.regaddr),
	 	iArray.length,											//发送基地址,字节长度
	 	GetRegAddress(iArray.header, iArray.regaddr),0);		//发送的起始数据地址指针,返回标志，帧头	  

	//SetSendBuffer(0xA0,0xA1,
	//	0x100,
	//	5,											//发送基地址,字节长度
	//	sendbuf[iArray.index],0);		
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

	serial_writesb(get_local_port(),m_btSendBuffer, ByteLength+7);
	// serial_write(get_local_port(),m_btSendBuffer);


}

uint16_t CalBaseRegAddress(char cHeader, uint16_t regaddr)
{

	if(cHeader=='B') 		return BASIC_ADDRESS + regaddr;
	else if(cHeader=='F') 	return FLYING_ADDRESS + regaddr;
	else if(cHeader=='C')	return CTRL_ADDRESS + regaddr;
	else if(cHeader=='T')	return TRAJ_ADDRESS + regaddr;
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
	else if (cHeader == 'I')	return (uint8_t *)&m600Status+IMG_ADDRESS + regaddr;
	else return NULL;

	//uint8_t i = 0;
	//uint8_t index = 0;
	//uint16_t base = 0;

	//if(cHeader=='B') 		index = 0;
	//else if(cHeader=='F') 	index = 1;
	//else if(cHeader=='C')	index = 2;
	//else if(cHeader=='T')	index = 3;
	//else if(cHeader=='I')	index = 3;
	//else	return (uint8_t *)(-1);
	//
	//for(i=0;i<index;i++)
	//{
	//	base += Base[i];
	//}

	//return (uint8_t *)&m600Status+regaddr+base;

}


// @brief  设置默认连续数据上报
void SetContiuneDefaultData(void)
{

	SetSendingData('T', 5, 0, TRAJ_LENGTH);		//自己添加的实验数据，返回控制模式等数据
	SetSendingData('F', 5, 0, FLYING_LENGTH);		//自己添加的实验数据，返回控制模式等数据
	// SetSendingData('T', 500, 80, 64);		//自己添加的实验数据，返回控制模式等数据	
	// SetSendingData('T', 500, 144, 64);		//自己添加的实验数据，返回控制模式等数据	
	// SetSendingData('T', 500, 208, 64);		//自己添加的实验数据，返回控制模式等数据		

//需要发回地面站的数据再添加 
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



void setUAVstatus(void)
{

}

void getUAVstatus(void)
{
	time_t now;
	// struct tm *timenow;

	time(&now);
	// timenow = localtime(&now);

	m600Status.ControlMode = 1;  
	m600Status.FlightState = 2;      

	m600Status.HandState   = 3;		
	m600Status.FlowStatus  = 4; 	    
	m600Status.RobostStatue= 5;     
	m600Status.WorkMode    = 6;		    

	m600Status.Systime	  = now;
	m600Status.motorSpeed  = 123.456;		
	m600Status.System_vol	= 7;		
	m600Status.Motor_vol		= 8;		
	m600Status.sysTemp		= 9;   		

	for (int kc = 0; kc < 3; kc++)
	{
		m600Status.gyro_xyz[kc] = 10+kc;
		m600Status.accl_xyz[kc] = 13+kc;

		m600Status.atti[kc]		=16+kc;
		m600Status.veloN[kc]		=19+kc;
		m600Status.veloB[kc]		=22+kc;
		m600Status.posi[kc]		=12.34*kc;
	}
	m600Status.GpsSol_Flags = 25;  
	m600Status.GpsSol_pDOP	= 26; 	
	m600Status.GpsSol_numSV	= 27; 	
	
	m600Status.pose_index = 30;
	for(int kc=0;kc<5;kc++)
		for(int kj=0;kj<2;kj++)
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
	for(int kc=0;kc<3;kc++)
		m600Status.ImgDist[kc]=53+kc;  

	
}