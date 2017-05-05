
#include "protocol.h"

void SetSendBufferData(sendbufQ iArray);
uint8_t* GetRegAddress(char cHeader, uint16_t regaddr);
uint16_t CalBaseRegAddress(char cHeader, uint16_t regaddr);
void SetSendBuffer(uint8_t Master,uint8_t Slave,uint16_t RegAddr,uint8_t ByteLength,
					uint8_t *data,uint8_t ReturnFlag);

void SetContiuneDefaultData(void);

uint16_t Base[5]={BASIC_LENGTH,FLYING_LENGTH,TRAJ_LENGTH,CTRL_LENGTH,IMG_LENGTH};
sendbufQ sendbufArray[SIZEREPEATARRAY];

UAVstatus m600Status;

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
	SetSendBuffer(0xA0,0xA1,
		CalBaseRegAddress(iArray.header, iArray.regaddr),
		iArray.length,											//发送基地址,字节长度
		GetRegAddress(iArray.header, iArray.regaddr),0);		//发送的起始数据地址指针,返回标志，帧头	  
}


void SetSendBuffer(uint8_t Master,uint8_t Slave,uint16_t RegAddr,uint8_t ByteLength,
					uint8_t *data,uint8_t ReturnFlag)
{ //数据打包过程 
	uint8_t m_btSendBuffer[255];
	uint8_t nIndex=0;
	uint8_t sum = 0;
	uint8_t i = 0;

	m_btSendBuffer[0] = '$';							//cHeader;
	m_btSendBuffer[1] = ((Master<< 4) & 0x0F0 | (Slave & 0x0F));
	m_btSendBuffer[2] = 0xA2;							//功能码
	m_btSendBuffer[3] = ByteLength;        				//表示的是变量字节数，不含头部和尾部
	m_btSendBuffer[4] = (uint8_t)(RegAddr&0x0FF);			//基地址低8位
	m_btSendBuffer[5] = (uint8_t)((RegAddr&0x0FF00)>>8);	//基地址高8位
	
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



}

uint16_t CalBaseRegAddress(char cHeader, uint16_t regaddr)
{
	uint8_t i = 0;
	uint8_t index = 0;
	uint16_t base = 0;

	if(cHeader=='B') 		index = 0;
	else if(cHeader=='F') 	index = 1;
	else if(cHeader=='C')	index = 2;
	else if(cHeader=='T')	index = 3;
	else if(cHeader=='I')	index = 3;
	else index = 0;
	
	for(i=0;i<index;i++)
	{
		base += Base[i];
	}

	return regaddr+base;
}


uint8_t* GetRegAddress(char cHeader, uint16_t regaddr)
{//确定发送的起始数据地址指针

	uint8_t i = 0;
	uint8_t index = 0;
	uint16_t base = 0;

	if(cHeader=='B') 		index = 0;
	else if(cHeader=='F') 	index = 1;
	else if(cHeader=='C')	index = 2;
	else if(cHeader=='T')	index = 3;
	else if(cHeader=='I')	index = 3;
	else	return (uint8_t *)(-1);
	
	for(i=0;i<index;i++)
	{
		base += Base[i];
	}

	return (uint8_t *)m600Status+regaddr+base;

}


// @brief  设置默认连续数据上报
void SetContiuneDefaultData(void)
{

	SetSendingData('T', 500, 0, 80);		//自己添加的实验数据，返回控制模式等数据
	// SetSendingData('T', 500, 80, 64);		//自己添加的实验数据，返回控制模式等数据	
	// SetSendingData('T', 500, 144, 64);		//自己添加的实验数据，返回控制模式等数据	
	// SetSendingData('T', 500, 208, 64);		//自己添加的实验数据，返回控制模式等数据		

//需要发回地面站的数据再添加 
}



void SetSendingData(char header, u16 period_ms, u16 regaddr, u8 length)
{
	sendbufQ newSendbuf;
	
	newSendbuf.enable 		= 1;
	newSendbuf.n_1ms	 	= 0;	  		//定时变量置0
 	newSendbuf.objAddr		= GCSAddr;		//地面站作为接收者地址	0x08


	newSendbuf.header 	= header;		//
	newSendbuf.period_ms	= period_ms;	//
	newSendbuf.regaddr   = regaddr;		//
	newSendbuf.length    = length; 		//
	MdfRepeatArray(&NewRepeat);	
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

#if 0

#include "CommProtocol.h"
#include "PC104_Usart.h"
#include "XW5630_Usart.h"
#include "XBee_Usart.h"

#include "proglobe.h"
#include "ControlLaw.H"
#include "ControlLawOutLoop.h"

u8 OwnAddr = CPU_DRIVER; // 本设备地址号, 输出驱动和测量板		04?
u8 GCSAddr = CPU_GCSPCX; //	地面站作为接收者地址

union RegAddr_type EE_ParaData;
Repeat_type RepeatArray[SIZEREPEATARRAY];


union RegV UV;
bool m_bTimingA3;		//有03读请求指令，定时发送返回在有查询请求时把发送优先级给该请求读指令
						//TRUE为定时返回发送有效，FALSE为无效
u16 m_nsBaseAddress = 504;

void CommProtocol_task(void)
{
	if(N_1ms_EventUpData>0)
	{
	///**
	 u8 i;
	 N_1ms_EventUpData = 0;				   //每隔一毫秒都要执行一次  
	 for(i=0;i<SIZEREPEATARRAY;i++)
	 {
		
	 	if(RepeatArray[i].enable ==1)			  //发送使能
		{
			RepeatArray[i].n_1ms ++;			  //时间变量累加1ms
			if(RepeatArray[i].period_ms==0)
			{
				RepeatArray[i].n_1ms = 0;		//时间变量复位，待发送数据帧进入收发缓冲池				
			}
			else if(RepeatArray[i].n_1ms>=RepeatArray[i].period_ms)	//时间到达发送定时周期
			{													
				RepeatArray[i].n_1ms = 0;		//时间变量复位，待发送数据帧进入收发缓冲池
				if(m_bTimingA3) 
					SetSendBufferData(RepeatArray[i]);	      				
			}
			
			//USART_SendStr(&XBee_Usart,"11111112222",20);
		}
		else return ;
	 }
 //**/
  
	}
}

u8 CalHeaderAndAddr(u16 *addr)
{
	s32 address = 0;
	u8 header = '$';
	u8 i=0,j=0;

	address = *addr;
	for(i=0;i<9;i++)
	{
	 	address -= Base[i];
		if(address < 0)break;
	}
	if(i==0)header = 'C';
	else if(i==1)header = 'T';
	else if(i==2)header = 'S';
	else if(i==3)header = 'P';
	else if(i==4)header = 'M';
	else if(i==5)header = 'A';
	else if(i==6)header = 'G';
	else if(i==7)header = 'I';
	else if(i==8)header = 'V';
	
	for(j=0;j<i;j++)
	{
		*addr -= Base[j];
	}	

	return header;
}


void SetSendBufferData(Repeat_type iArray)
{
	SetSendBuffer(iArray.pUSART,0xA2,			//串口,功能码0XA2定时返回数据
		OwnAddr,iArray.objAddr,					//本设备地址OwnAddr=0x04；发送者地址要根据程序作相应变动
		CalBaseRegAddress(iArray.header, iArray.regaddr),iArray.length,			//发送基地址,字节长度
		GetRegAddress(iArray.header, iArray.regaddr),0, iArray.header);		//发送的起始数据地址指针,返回标志，帧头	  
}



void SetSendBufferA3(struct USART_TypeDefStruct *pUSART, u8 Slave,u16 RegAddr,u8 ByteLength, u8 cHeader)
{//以A3功能号返回，参数为：串口,功能码0XA3, 发送者,接收者，基地址,字节长度, 发送的起始数据地址指针,返回标志
	SetSendBuffer(pUSART,0xA2,OwnAddr, Slave,				//此处原为0xA3							
		CalBaseRegAddress(cHeader, RegAddr),ByteLength,	GetRegAddress(cHeader, RegAddr),0, cHeader);

}

void SetSendBuffer(struct USART_TypeDefStruct *pUSART, u8 CodeID,u8 Master,u8 Slave,u16 RegAddr,u8 ByteLength,u8 *data,u8 ReturnFlag, u8 cHeader)
{ //数据打包过程 
	u8 m_btSendBuffer[255];
	u8 nIndex=0;
	u8 sum = 0;
	u8 i = 0;
	m_btSendBuffer[0] = '$';						//cHeader;
	m_btSendBuffer[1] = ((Master<< 4) & 0x0F0 | (Slave & 0x0F));
	m_btSendBuffer[2] = CodeID;						//功能码
	m_btSendBuffer[3] = ByteLength;        			//表示的是变量字节数，不含头部和尾部
	m_btSendBuffer[4] = (u8)(RegAddr&0x0FF);		//基地址低8位
	m_btSendBuffer[5] = (u8)((RegAddr&0x0FF00)>>8);	//基地址高8位
	
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

	USART_SendStr(pUSART, m_btSendBuffer,nIndex+8); //发送数据，添加到发送缓冲池
}

/**
  * @brief  修改定时发送队列
  * @param   pNewRepeat 需要修改/添加的队列元素
  * @retval : 0：失败（超容量）  1：成功	 
  */
u8 MdfRepeatArray(Repeat_type *pNewRepeat)
{
	u8 i;
	// 遍历所有， 先遍历原来就有的，如果数据缓冲池中有这数据的内存空间，则检查其值及周期，将其更新
	for(i=0;i<SIZEREPEATARRAY;i++)
	{	  	// 地址和长度相同，修改周期列表 
		  if(	(RepeatArray[i].enable==1)&&
		  		(RepeatArray[i].regaddr==pNewRepeat->regaddr)&&
		  		(RepeatArray[i].length==pNewRepeat->length)&&
		  		(RepeatArray[i].header==pNewRepeat->header)&&
				(RepeatArray[i].pUSART==pNewRepeat->pUSART))
			{
				 if(pNewRepeat->period_ms==0)    // 特殊 ，删除
				{
					RepeatArray[i].enable = 0; // 删除 ;
					sortRepeatArray();
					return 1;
				}
				else
				{
					RepeatArray[i].period_ms = pNewRepeat->period_ms; // 修改周期
					sortRepeatArray();
					return 1;	
				}
			}
	}
	// 添加新的
	if(pNewRepeat->period_ms<1) return 0; 
	for(i=0;i<SIZEREPEATARRAY;i++)
	{
		if(RepeatArray[i].enable ==0)			//寻找周期发送已停止的单元
			break;
		else 
			continue;
	}
	if(i>=SIZEREPEATARRAY) return 0;
		
	RepeatArray[i].n_1ms = 		pNewRepeat->n_1ms;
	RepeatArray[i].period_ms = 	pNewRepeat->period_ms;
	RepeatArray[i].regaddr = 	pNewRepeat->regaddr;
	RepeatArray[i].length = 	pNewRepeat->length;
	RepeatArray[i].pUSART = 	pNewRepeat->pUSART;
	RepeatArray[i].objAddr = 	pNewRepeat->objAddr;
	RepeatArray[i].header = 	pNewRepeat->header;
	RepeatArray[i].enable = 1;
	sortRepeatArray();
	return 1;
}

void swapRepeatData(Repeat_type *p1,Repeat_type *p2)
{//互换内容
	Repeat_type ptmep;
	ptmep.enable = p1->enable; 
	ptmep.n_1ms  = p1->n_1ms ;
	ptmep.period_ms = p1->period_ms; 
	ptmep.regaddr = p1->regaddr; 
	ptmep.length = p1->length; 
	ptmep.objAddr = p1->objAddr; 
	ptmep.pUSART = p1->pUSART; 
	ptmep.header = p1->header;
	
	p1->enable = p2->enable; 
	p1->n_1ms  = p2->n_1ms ;
	p1->period_ms = p2->period_ms; 
	p1->regaddr = p2->regaddr; 
	p1->length = p2->length; 
	p1->objAddr = p2->objAddr; 
	p1->pUSART = p2->pUSART; 
	p1->header = p2->header;

	p2->enable = ptmep.enable; 
	p2->n_1ms  = ptmep.n_1ms ;
	p2->period_ms = ptmep.period_ms; 
	p2->regaddr = ptmep.regaddr; 
	p2->length = ptmep.length; 
	p2->objAddr = ptmep.objAddr; 
	p2->pUSART= ptmep.pUSART; 		
	p2->header= ptmep.header; 		
}
/**
  * @brief  将周期待发数据靠前排放
  * @param   pNewRepeat 需要修改/添加的队列元素
  * @retval : 0：失败（超容量）  1：成功	 
  */
void sortRepeatArray(void)
{
	u8 i;
	u8 j;
	for(i=0;i<SIZEREPEATARRAY-1;i++)
	{
		if(RepeatArray[i].enable==0)
		{
			for(j=i+1;j<SIZEREPEATARRAY;j++)
			{
				if(RepeatArray[j].enable==1)
				{
					swapRepeatData(&RepeatArray[i],&RepeatArray[j]);	
				}
			}
		} 	
	}
}

void stopRepeatArray(void)
{
	u8 i;
	for(i=0;i<SIZEREPEATARRAY;i++)
	{
		RepeatArray[i].enable = 0;	   //停止所有的定时周期发送任务
	}
}


//串口接收处理函数
/**
  * @brief  接受收据数据帧解包
  * @param   R_data 逐个接收到的数据
  			pUSART： 串口号
  * @retval : 解包状态
  */
void ReceivedByte_SUART(u8 R_data, struct USART_TypeDefStruct *pUSART)
{
 if(pUSART==&XBee_Usart)	
  {
    switch (pUSART->RecvParse.R_state)				  //接收数据帧解包用的结构变量
    {
        case 0:
            {        
                if (isHeader(R_data))
                {
                    pUSART->RecvBuff[0] = R_data; 
					pUSART->RecvParse.R_state = 1;
					pUSART->RecvParse.sum =  R_data;
					return ;
                }		
                break;
            } 
        case 1:
            {
                pUSART->RecvBuff[1] = R_data; 		 // MASTER +SLAVER
				pUSART->RecvParse.R_state++; 
				pUSART->RecvParse.sum += R_data;
				return ;               
            }
       case 2:
            {
                pUSART->RecvBuff[2] = R_data; 		 // 功能码字节
				pUSART->RecvParse.R_state++; 
				pUSART->RecvParse.sum += R_data;
				return ;               
            }
       case 3:
            {
                pUSART->RecvBuff[3] = R_data; 	// Rec Data  Length;  字节长度
				pUSART->RecvParse.R_state++; 
				pUSART->RecvParse.sum += R_data;
				pUSART->RecvParse.p_rec_num = 4;		//接收数据缓冲器指针
				switch(pUSART->RecvBuff[2])
				{
				case 0x02:
				  pUSART->RecvParse.lenth_rec_num = 9;   //lenth_rec_num 接收长度-1
				  break;
				case 0x03:
				case 0x07:
				  pUSART->RecvParse.lenth_rec_num = 7; 	  //lenth_rec_num 接收长度-1
				  break;
				default:
				  pUSART->RecvParse.lenth_rec_num = 7+pUSART->RecvBuff[3]; 	  	//lenth_rec_num 接收长度-1
				  break;
				}
				return ;
            }
		case 4:
            {
                pUSART->RecvBuff[pUSART->RecvParse.p_rec_num++] = R_data;
				pUSART->RecvParse.sum += R_data;
				if(pUSART->RecvParse.p_rec_num >= pUSART->RecvParse.lenth_rec_num)
				{
					pUSART->RecvParse.R_state = 5; 	
				}
                return;
            }			
        case 5: //接收数据
            { //	RecvBuff 	 接收的字符串缓冲器
            pUSART->RecvBuff[pUSART->RecvParse.p_rec_num++] = R_data;
            if(pUSART->RecvParse.sum == R_data) // 校验和通过
			{
				ReceivedComPortDataEvent(pUSART->RecvBuff,pUSART);
			}
			pUSART->RecvParse.R_state = 0;
            return ;
            }
        default:
            {
                pUSART->RecvParse.R_state = 0;
                return ;
            }
    }
    pUSART->RecvParse.R_state = 0;
    return ;
	}	
}

 /**************************************************/
void ReceivedComPortDataEvent(u8 *buf, struct USART_TypeDefStruct *pUSART)
{
//	u8 length;
//	length = pUSART->RecvParse.lenth_rec_num+1;				//接收到的当前数据帧长度
	pUSART->NewDataFrame = true;

	switch(buf[2])
	{
		case 0x02:	   	// 定时返回请求
			PutFunction02(buf, pUSART);
			break;
		case 0x03:	   	// 读寄存器命令
			PutFunction03(buf, pUSART);
			break;
		case 0x06:		// 写寄存器命令	
			PutFunction06(buf, pUSART);
			break;
		case 0x07: 		// 请求应答返回
			break;
		case 0x08:	 	// 异常突发事件
			break;
		case 0xA2:		// 定时返回数据
		case 0xA3:		// 查询返回数据
			PutFunctionA23(buf);
			break;
	}				
}															

void PutFunction02(u8 *buf, struct USART_TypeDefStruct *pUSART)
{
	Repeat_type NewRepeat;
	NewRepeat.enable = 1;
	NewRepeat.n_1ms	 = 0;	  //定时变量置0
	NewRepeat.period_ms	= buf[6]|(buf[7]<<8);
	NewRepeat.regaddr   = buf[4]|(buf[5]<<8);
	NewRepeat.length    = buf[3];
	NewRepeat.objAddr	= (buf[1]>>4)&0x0f;
	NewRepeat.pUSART    = pUSART;
    NewRepeat.header = buf[0];	  //保存帧头字符

	if(NewRepeat.regaddr==0x0FFFF)	//特定基地址 
		stopRepeatArray();	  //停止所有定时周期发送任务
	else MdfRepeatArray(&NewRepeat);
}
void PutFunction03(u8 *buf, struct USART_TypeDefStruct *pUSART)
{
	u16 regaddr;
	u8 length;
	u8 objAddr;
	m_bTimingA3 = false;
	regaddr   = buf[4]|(buf[5]<<8);
	length    = buf[3];
	objAddr	= (buf[1]>>4)&0x0f;
	USART_SendBufferClear(pUSART);	//需要把还没有发送出去的数据放弃掉
	SetSendBufferA3(pUSART,objAddr,regaddr,length,buf[0]);		
	SetSendBufferA3(pUSART,objAddr,regaddr,length,buf[0]);		
	BeepSound(BEEP_LONG,1);  		//读回指定单元数据，长声1下
	m_bTimingA3 = true;
}

void PutFunction06(u8 *buf, struct USART_TypeDefStruct *pUSART)
{																  
	PutFunctionA23(buf);
	PutBackMotorRPMtoGCSState(buf);
	PutControlParamChangedVoice(buf);
	PutMovingPlatformRevise(buf, pUSART);
}

void PutControlParamChangedVoice(u8 *buf)
{
	u16 regaddr = buf[4]|(buf[5]<<8);
	short nsValue = buf[6]|(buf[7]<<8);
	if(buf[0]=='T') 
	{
 		SetControlParamUpdate();  //更新控制律参数
		BeepSound(BEEP_SHORT,2);  //控制参数修改，短声2下
	}
	else if(buf[0]=='C')
	{
		BeepSound(BEEP_SHORT,2);  //控制变量修改，短声1下
		if((regaddr>=128)&&(regaddr<144))
			SetAnyArmPalmHead();  //需要更新角度
		else if(regaddr==4) m_bHandAuto = false;
	}
	else if(buf[0]=='U')
	{
		if(regaddr==4) m_bHandAuto = false;
		else if((regaddr>=128)&&(regaddr<144))
			SetAnyArmPalmHead();  		//需要更新角度
 		else if((regaddr>=160)&&(regaddr<432))
 			SetControlParamUpdate();  	//更新控制律参数
 		else if(regaddr==454)			//GPS点设置标号
		{
			if(!PutObjectivePointCondition(nsValue))
				return;
		}
 		else if(regaddr==780)
			bNewXsenseData = 1;
		BeepSound(BEEP_SHORT,2);  		//控制参数修改，短声2下
	}
}

bool PutObjectivePointCondition(short nsValue)
{
	if((m_nsBaseAddress>=504)&&(m_nsBaseAddress<728))
		SetSendingData(&XBee_Usart, 'U', 0, m_nsBaseAddress, 16);	//停止原先双精度经纬度发送
	else  
		SetSendingData(&XBee_Usart, 'U', 0, m_nsBaseAddress, 4);	//停止原先双精度经纬度发送
	if((nsValue<0)||(nsValue>10))
	{
		BeepSound(BEEP_SHORT,3); 
		return false;
	}
	if(nsValue==10) m_nsBaseAddress = 460+4*(nsValue-10);	//暂存当前基地址  
	else m_nsBaseAddress = 504+16*nsValue;					//暂存当前基地址  
	SetSendingData(&XBee_Usart, 'U', 500, m_nsBaseAddress, (nsValue==10)?4:16);		//0.5秒定时发送双精度经纬度值
	return true; 
}




void PutFunctionA23(u8 *buf)
{
	u8 i;
	u16 regaddr;
	if(buf[3]>0)
	{//数据长度（字节数）大于0
		regaddr = (buf[5]<<8)|buf[4]; 			//基地址
		buf[0]=CalHeaderAndAddr(&regaddr);		//还原基地址并计算实际的帧头
		for(i=0;i<buf[3];i++)
		{//RegAddress0根据数据帧的帧头选择数据集
	 		GetRegAddress(buf[0], 0)[regaddr+i] =buf[6+i]; 
		}
	 }
}

// @brief  设置默认连续数据上报
void SetContiuneDefaultData(void)
{
//	SetSendingData(&Out_Usart, 'S', 100, 0, 20);		// 遥控器PWM0-PWM9		//不是XBEE串口，所以先前打开次句，板子没有向电脑发回数据
	
//
////union1
//	SetSendingData(&XBee_Usart, 'C', 100, 0, 80);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'C', 100, 80, 48);		//自己添加的实验数据，返回控制模式等数据	
//	SetSendingData(&XBee_Usart, 'C', 100, 128, 32);		//自己添加的实验数据，返回控制模式等数据
//
//union2	
	SetSendingData(&XBee_Usart, 'T', 500, 0, 80);		//自己添加的实验数据，返回控制模式等数据
	SetSendingData(&XBee_Usart, 'T', 500, 80, 64);		//自己添加的实验数据，返回控制模式等数据	
	SetSendingData(&XBee_Usart, 'T', 500, 144, 64);		//自己添加的实验数据，返回控制模式等数据	
	SetSendingData(&XBee_Usart, 'T', 500, 208, 64);		//自己添加的实验数据，返回控制模式等数据		
//
////union3
//  	SetSendingData(&XBee_Usart, 'S', 100, 0, 72);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'S', 100, 72, 160);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'S', 100, 232, 64);		//自己添加的实验数据，返回控制模式等数据
//
//
////union4
//	SetSendingData(&XBee_Usart, 'P', 100, 0, 56);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'P', 100, 56, 56);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'P', 100, 112, 56);		//自己添加的实验数据，返回控制模式等数据
//
//
////union5
//	SetSendingData(&XBee_Usart, 'M', 500, 0, 64);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'M', 500, 96, 64);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'M', 500, 160, 64);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'M', 500, 224, 128);	//自己添加的实验数据，返回控制模式等数据
//	
////union6	
//	SetSendingData(&XBee_Usart, 'A', 500, 0, 48);		//自己添加的实验数据，返回控制模式等数据
//
////union7
//	SetSendingData(&XBee_Usart, 'G', 100, 0, 64);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'G', 100, 64, 56);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'G', 100, 120, 16);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'G', 100, 136, 80);		//自己添加的实验数据，返回控制模式等数据
//	SetSendingData(&XBee_Usart, 'G', 100, 216, 80);		//自己添加的实验数据，返回控制模式等数据
//
////union8
// 	SetSendingData(&XBee_Usart, 'I', 100, 0, 16);		//自己添加的实验数据，返回控制模式等数据
//
////union9
// 	SetSendingData(&XBee_Usart, 'V', 100, 0, 32);		//自己添加的实验数据，返回控制模式等数据



	//SetSendingData(&XBee_Usart, 'C', 100, 124, 8);
	
	SetSendingData(&XBee_Usart, 'C', 100, 144, 4);      //系统时间
	
	//SetSendingData(&XBee_Usart, 'M', 100, 0, 4);
	SetSendingData(&XBee_Usart, 'S', 100, 0, 136);		// 遥控器PWM0-PWM9
	
	SetSendingData(&XBee_Usart, 'P', 100, 0, 156);		// 飞控盒温度  发动机缸1温度  发动机缸2温度	 
	                                                    // 系统工作电压  舵机工作电压 超声波高度 
														// 上旋翼转速  下旋翼转速  气压高度
	//SetSendingData(&XBee_Usart, 'P', 100, 0, 12);		// IMU 角速率 	放大 1000倍    		 rad 
	//SetSendingData(&XBee_Usart, 'P', 100, 12, 6);		// 姿态角 滚转 俯仰角 偏航 	放大 1000倍  rad 
	//SetSendingData(&XBee_Usart, 'P', 100, 24, 6);		// 放大100倍 前右下速度	
	//SetSendingData(&XBee_Usart, 'P', 100, 48, 4);		// 放大 10倍 上旋翼转速、下旋翼转速 
	//SetSendingData(&XBee_Usart, 'P', 100, 120, 12);		// 惯性航向角、磁航向角、二航向角差  rad 
	//SetSendingData(&XBee_Usart, 'P', 100, 52, 28);		// 气压高度、经度  纬度  高度 
	//SetSendingData(&XBee_Usart, 'P', 100, 88, 8);		// 当前飞行高度 
	
	//SetSendingData(&XBee_Usart, 'P', 100, 46, 6);		// 超声波高度1、超声波高度2、2高度融合值  放大 1000倍 
	
	SetSendingData(&XBee_Usart, 'C', 100, 0, 28);		// 工作状态，操纵量(ms):总距/横向/纵向/偏航/油门
	SetSendingData(&XBee_Usart, 'M', 100, 0, 32);		// 0-11舵机高电平信号
    SetSendingData(&XBee_Usart, 'A', 200, 0, 48);		//自己添加的实验数据，返回控制模式等数据
//需要发回地面站的数据再添加 
}

void SetSendingOne(struct USART_TypeDefStruct *pUSART, char header, u16 regaddr, u8 length)
{
	u8 objAddr;
	objAddr	= GCSAddr;
	SetSendBufferA3(pUSART,objAddr,regaddr,length, header);		//基地址,字节长度, 发送的起始数据地址指针,返回标志
}

void SetSendingData(struct USART_TypeDefStruct *pUSART, char header, u16 period_ms, u16 regaddr, u8 length)
{
	Repeat_type NewRepeat;
	
	NewRepeat.enable 	= 1;
	NewRepeat.n_1ms	 	= 0;	  		//定时变量置0
 	NewRepeat.objAddr	= GCSAddr;		//地面站作为接收者地址	0x08
	
	NewRepeat.pUSART    = pUSART;		//		
	NewRepeat.header 	= header;		//
	NewRepeat.period_ms	= period_ms;	//
	NewRepeat.regaddr   = regaddr;		//
	NewRepeat.length    = length; 		//
	MdfRepeatArray(&NewRepeat);	
}

void SetSendingTestData(struct USART_TypeDefStruct *pUSART)
{
	Repeat_type NewRepeat;
	NewRepeat.enable = 1;
	NewRepeat.n_1ms	 = 0;	  //定时变量置0
	NewRepeat.period_ms	= 200;
	NewRepeat.regaddr   = 0x00;
	NewRepeat.length    = 48;
	NewRepeat.objAddr	= 0x03;
	NewRepeat.pUSART    = pUSART;
	MdfRepeatArray(&NewRepeat);			
}




#endif