
#include "main.h"

/**
  * @简  述  CAN初始化。
  * @参  数  tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq - CAN_SJW_4tq 
  *          tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq - CAN_BS2_8tq;
  *          tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq  - CAN_BS1_16tq
  *          brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
  *          mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
  * @说  明  波特率 = Fpclk1/((tbs1+1+tbs2+1+1)*brp);
  *          Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
  *          则波特率为:42M/((6+7+1)*6)=500Kbps
  * @返回值  无
  */
void CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)
{

		GPIO_InitTypeDef GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;

		//使能相关时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORT时钟	                   											 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

		//初始化GPIO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		//引脚复用映射配置
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); 

		//CAN单元设置
		CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
		CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
		CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
		CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
		CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
		CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
		CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
		CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
		CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
		CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
		CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
		CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 

		//配置过滤器
		CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
		CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化	
} 

/**
  * @简  述  CAN发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
  * @参  数  msg:数据指针,最大为8个字节
  *          num:数据长度(最大为8)	
  * @返回值  0,成功；其他,失败；
  */
uint8_t CAN_SendMsg(uint8_t* msg, uint8_t num)
{	
  uint8_t mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
	
  //数据封装
  TxMessage.StdId=0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=num;	  // 发送两帧信息
  for(i=0; i<num; i++)
	TxMessage.Data[i]=msg[i]; 
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
	
  //等待发送结束
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
	  i++;	
  
  //返回状态
  if(i>=0XFFF)
	  return 1;
  return 0;		

}

/**
  * @简  述  CAN接收数据查询
  * @参  数  msg:接收数据缓存区	
  * @返回值  0,无数据被收到；  其他,接收的数据长度；
  */
uint8_t CAN_RecvMsg(uint8_t *msg)
{		   		   
 	uint32_t i;
	CanRxMsg RxMessage;
	
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
		return 0;		//没有接收到数据,直接退出
	
	//读取数据	
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    for(i=0; i<RxMessage.DLC; i++)
	    msg[i]=RxMessage.Data[i];  
	
	return RxMessage.DLC;	
}

/******************* (C) 版权 2022 XTARK **************************************/
