//
//JIAlonglong 2023.2.17
//
#include "elmo.h"
#include "stm32f4xx_hal.h"
////GDUT_JIAlonglong m3508拓展板
///*******************************控制驱动器命令************************************/
///**
//* @brief  Elmo驱动器初始化
//* @param  CANx：所使用的CAN通道编号
//* @author ACTION
//*/
//void ElmoInit(CAN_HandleTypeDef* hcan)
//{
//    uint32_t data[1][2] = {0x00000001, 0x00000000};
//    uint32_t TxMailbox;
//    CAN_TxHeaderTypeDef TxMessage;
//    uint8_t TxData[8];
//		
//    TxMessage.StdId = ELMO_BROADCAST_ID;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 8;

//    TxData[0] = *(unsigned long*)&data[0][0] & 0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0] >> 8) & 0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0] >> 16) & 0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0] >> 24) & 0xff;
//    TxData[4] = *(unsigned long*)&data[0][1] & 0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1] >> 8) & 0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1] >> 16) & 0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1] >> 24) & 0xff;

//    if (HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
//        // Failed to add message to the transmit mailbox
//    }
//}


///**
//* @brief  电机使能（通电）
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @author ACTION
//* @note ELMO驱动器默认初始状态为电机失能，使用电机时需要对其进行使能
//*       部分驱动器参数需要在电机失能状态下才可以配置
//*/
//void MotorOn(CAN_HandleTypeDef* hcan, uint8_t ElmoNum)
//{
//	//第一个数发送MO命令，第二个数发送1给电机使能（通电）
//	uint32_t data[1][2] = {
//		{ 0x00004F4D, 0x00000001 },
//	};
//						 
//	CAN_TxHeaderTypeDef TxHeader;
//	uint8_t TxData[8];
//	TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;	// standard identifier=0
//	TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;	// extended identifier=StdId
//	TxHeader.IDE = CAN_ID_STD;						// type of identifier for the message is Standard
//	TxHeader.RTR = CAN_RTR_DATA;						// the type of frame for the message that will be transmitted
//	TxHeader.DLC = 8;

//	uint32_t TxMailbox;
//	
//	//填充数据到TxHeader.Data数组
//	TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//	TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//	TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//	TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//	TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//	TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//	TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//	TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//	//发送数据
//	if(HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//	{
//		//发送失败，处理错误
//	}

//	//等待发送成功
//	uint16_t timeout = 0;
//	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && timeout < 60000)
//	{
//		timeout++;
//	}
//	if (timeout >= 60000) {
//		//发送超时，处理错误
//	}
//}

///**
//* @brief  电机失能（断电）
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @author ACTION GDUT-JIAlonglong
//*/
//void MotorOff(CAN_HandleTypeDef* hcan, uint8_t ElmoNum) {
//    //第一个数据发送MO命令，第二个数据发送0给电机失能（断电）
//    uint32_t data[1][2] = {
//        0x00004F4D,0x00000000,  //MO  0
//    };
//    uint32_t mbox;
//    CAN_TxHeaderTypeDef TxHeader;
//    uint8_t TxData[8];
//                     
//    TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;   // standard identifier=0
//    TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;   // extended identifier=StdId
//    TxHeader.IDE = CAN_ID_STD;                      // type of identifier for the message is Standard
//    TxHeader.RTR = CAN_RTR_DATA;                     // the type of frame for the message that will be transmitted
//    TxHeader.DLC = 8;

//    TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//    TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//    //发送数据
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mbox) != HAL_OK) {
//        // 发送失败处理
//    }
//    
//    //等待发送成功
//    uint16_t timeout = 0;
//    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3) {
//        timeout++;
//        if (timeout > 60000) {
//            // 处理超时
//            break;
//        }
//    }
//}

///**
//* @brief  驱动器速度环初始化
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  acc：加速度，单位：脉冲每二次方秒
//* @param  dec：减速度，单位：脉冲每二次方秒
//* @author ACTION
//* @note 在速度环初始化后才可以使能电机！！
//*/
//void VelLoopCfg(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
//{
//  SetUnitMode(hcan, ElmoNum, SPEED_CONTROL_MODE);
//  SetAccAndDec(hcan, ElmoNum, acc, dec);
//}

///**
//* @brief  配置加速度与减速度
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  acc：加速度，单位：脉冲每二次方秒
//* @param  dec：减速度，单位：脉冲每二次方秒
//* @author ACTION
//* @note
//*/
//void SetAccAndDec(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
//{
//    //第一个数据发送AC\DC命令，第二个数据发送命令值
//    uint32_t data[2][2]={
//                            0x00004341,0x00000000,    //AC
//                            0x00004344,0x00000000    //DC
//    };
//    uint32_t TxMailbox;
//    CAN_TxHeaderTypeDef TxMessage;
//		uint8_t TxData[8];

//    TxMessage.StdId = ELMO_DEVICE_BASEID + ElmoNum;       // standard identifier=0
//    TxMessage.ExtId = ELMO_DEVICE_BASEID + ElmoNum;       // extended identifier=StdId
//    TxMessage.IDE = CAN_ID_STD;                          // type of identifier for the message is Standard
//    TxMessage.RTR = CAN_RTR_DATA;                         // the type of frame for the message that will be transmitted
//    TxMessage.DLC = 8;

//    data[0][1] = acc;
//    data[1][1] = dec;

//    for(uint8_t i = 0; i < 2; i++)
//    {
//				TxData[0] = *(unsigned long*)&data[i][0]&0xff;
//				TxData[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
//				TxData[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
//				TxData[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
//				TxData[4] = *(unsigned long*)&data[i][1]&0xff;
//				TxData[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
//				TxData[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
//				TxData[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;

//        if (HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
//            // 在这里应加入异常处理
//        }
//        
//        //等待发送成功
//        uint16_t timeout = 0;
//        while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//        {
//            timeout++;
//            if(timeout > 60000)
//            {
//                // 在这里应加入异常处理
//            }
//        }
//    }
//}


///**
//* @brief  驱动器位置环初始化
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  acc：加速度，单位：脉冲每二次方秒
//* @param  dec：减速度，单位：脉冲每二次方秒
//* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
//* @author ACTION
//* @note 在位置环初始化后才可以使能电机！！
//*/
//void PosLoopCfg(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel)
//{
//    // set unit mode to position control mode
//    SetUnitMode(hcan, ElmoNum, POSITION_CONTROL_MODE);

//    // set acceleration and deceleration
//    SetAccAndDec(hcan, ElmoNum, acc, dec);

//    // set velocity for position loop
//    SetPosLoopVel(hcan, ElmoNum, vel);
//}


///**
//* @brief  配置位置环运行最大速度
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
//* @author ACTION
//* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
//*/
//void SetPosLoopVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, int32_t vel)
//{
//	//第一个数据发送SP命令，第二个数据发送命令值
//	uint32_t data[1][2]={
//		0x00005053,0x00000000,		//SP
//	};
//	
//	CAN_TxHeaderTypeDef TxHeader;
//	uint8_t TxData[8];
//	uint32_t TxMailbox;
//	
//	TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;
//	TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 8;
//	
//	data[0][1] = vel;
//	
//	TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//	TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//	TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//	TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//	TxData[4] =  *(unsigned long*)&data[0][1]&0xff;
//	TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//	TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//	TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//	HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
//	
//	//等待发送成功
//	uint16_t timeout = 0;
//	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//	{
//		timeout++;
//		if(timeout > 60000)
//		{
//			//在这里应加入异常处理
//		
//		}
//	}
//}


///**
//* @brief  电机速度控制
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
//* @author ACTION
//*/
//void VelCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, int32_t vel)
//{
//	SetJoggingVel(hcan, ElmoNum, vel);
//}

///**
//* @brief  电机位置控制
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  posMode: 位置环运行模式，范围：
//				ABSOLUTE_MODE: 绝对位置模式
//				RELATIVE_MODE: 相对位置模式
//* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
//* @author ACTION
//*/
//void PosCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,uint8_t posMode,int32_t pos)
//{
//	SendPosCmd(hcan, ElmoNum, posMode, pos);
//}

///**
//* @brief  配置驱动器工作模式
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  unitMode：驱动器工作模式，范围：
//			TORQUE_CONTROL_MODE：力矩控制模式，在该模式下可以执行TC电流命令
//			SPEED_CONTROL_MODE：速度控制模式，在该模式下通过设置JV值控制速度
//			MICRO_STEPPER_MODE：直流电机不能使用该模式
//			DUAL_POSITION_MODE：双位置闭环模式
//			SINGLE_POSITION_MODE：单位置闭环模式，在该模式下可以配置PA、PR、JV、PT或PVT运动
//* @author ACTION
//* @note 只有在电机失能时可以配置该参数
//*/

//void SetUnitMode(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint8_t unitMode)
//{
//	//第一个数据发送UM命令，第二个数据发送模式
//	uint32_t data[1][2]={
//						0x00004D55,0x00000000,      //UM
//					 };
//	uint32_t mailbox;
//	CAN_TxHeaderTypeDef TxHeader;
//	TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
//	TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;		// extended identifier=StdId
//	TxHeader.IDE = CAN_ID_STD;							// type of identifier for the message is Standard
//	TxHeader.RTR = CAN_RTR_DATA;							// the type of frame for the message that will be transmitted
//	TxHeader.DLC = 8;
//	uint8_t TxData[8];
//	data[0][1] = unitMode;
//	TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//	TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//	TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//	TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//	TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//	TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//	TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//	TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
//	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mailbox);
//	
//	//等待发送成功
//	uint16_t timeout = 0;
//	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//	{
//		timeout++;
//		if(timeout > 60000)
//		{
//			//超时处理
//			break;
//		}
//	}
//}



///**
//* @brief  配置运行速度
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
//* @author ACTION
//* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
//*/
//void SetJoggingVel(CAN_HandleTypeDef *hcan, uint8_t ElmoNum, int32_t vel)
//{
//  // 第一个数据发送JV命令，第二个数据发送命令值
//  uint32_t data[1][2] = {
//      0x0000564A, 0x00000000,  // JV
//  };
//  uint32_t mbox;
//  CAN_TxHeaderTypeDef TxHeader;
//  uint8_t TxData[8];

//  TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;   // standard identifier=0
//  TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;   // extended identifier=StdId
//  TxHeader.IDE = CAN_ID_STD;                      // type of identifier for the message is Standard
//  TxHeader.RTR = CAN_RTR_DATA;                     // the type of frame for the message that will be transmitted
//  TxHeader.DLC = 8;

//  data[0][1] = vel;

//  TxData[0] = *(unsigned long *)&data[0][0] & 0xff;
//  TxData[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
//  TxData[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
//  TxData[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
//  TxData[4] = *(unsigned long *)&data[0][1] & 0xff;
//  TxData[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
//  TxData[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
//  TxData[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

//  HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mbox);

//  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//  {
//  }
//}


///**
//* @brief  配置位置环命令
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @param  posMode: 位置环运行模式，范围：
//				ABSOLUTE_MODE: 绝对位置模式
//				RELATIVE_MODE: 相对位置模式
//* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
//* @author ACTION
//* @note：位置正负号代表旋转的方向，大于零为正方向，小于零为负方向
//*/
//void SendPosCmd(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint8_t posMode, int32_t pos)
//{
//    uint32_t data[1][2] = {0x00000000, 0x00000000};  //PA
//    uint32_t mailbox;
//    CAN_TxHeaderTypeDef TxHeader;
//    uint8_t TxData[8];

//    TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.DLC = 8;

//    if (posMode == ABSOLUTE_MODE)
//    {
//        data[0][0] = 0x00004150;  //绝对
//    }

//    data[0][1] = pos;

//    TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//    TxData[4] =  *(unsigned long*)&data[0][1]&0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//		
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mailbox) != HAL_OK)
//    {
//        // 网络错误处理
//    }

//    uint16_t timeout = 0;
//    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)  // 等待消息发送完成
//    {
//        timeout++;
//        if (timeout > 60000)
//        {
//            // 超时处理
//        }
//    }
//}



///**********************************读取驱动器数据命令*************************************/

///**
//* @brief  读取电机位置
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @author ACTION
// * @note：接收标识符为：0x00005850
//*/
//void ReadActualPos(CAN_HandleTypeDef* hcan, uint8_t ElmoNum)
//{
//    uint32_t data[1][2] = {
//        0x40005850, 0x00000000 //PX
//    };
//    uint32_t mailbox;
//    CAN_TxHeaderTypeDef TxMessage;
//		uint8_t TxData[8];
//    
//    TxMessage.StdId = ELMO_DEVICE_BASEID + ElmoNum;   // standard identifier=0
//    TxMessage.ExtId = ELMO_DEVICE_BASEID + ElmoNum;   // extended identifier=StdId
//    TxMessage.IDE = CAN_ID_STD;                     // type of identifier for the message is Standard
//    TxMessage.RTR = CAN_RTR_DATA;                    // the type of frame for the message that will be transmitted
//    TxMessage.DLC = 8;

//    TxData[0] = *(unsigned long*)&data[0][0] & 0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0] >> 8) & 0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0] >> 16) & 0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0] >> 24) & 0xff;
//    TxData[4] = *(unsigned long*)&data[0][1] & 0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1] >> 8) & 0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1] >> 16) & 0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1] >> 24) & 0xff;

//    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &mailbox);;

//    uint16_t timeout = 0;
//    while (status != HAL_OK)
//    {
//        timeout++;
//        if (timeout > 60000)
//        {
//            // handle timeout error
//        }
//        status = HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &mailbox);
//    }
//}


///**
//* @brief  读取电机速度
//* @param  CANx：所使用的CAN通道编号
//* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
//* @author ACTION
// * @note：接收标识符为：0x00005856
//*/
//void ReadActualVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum)
//{
//    uint32_t data[1][2] = {
//        0x40005856, 0x00000000,      // VX
//    };
//    CAN_TxHeaderTypeDef TxMessage;
//    uint8_t TxData[8];
//		uint32_t TxMailbox;
//    
//    TxMessage.StdId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxMessage.ExtId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 8;
//    
//    TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//    TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
//    
//    HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &TxMailbox);
//    
//    uint16_t timeout = 0;
//    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3) {
//        timeout++;
//        if(timeout > 60000) {
//            // handle timeout
//        }
//    }
//}
