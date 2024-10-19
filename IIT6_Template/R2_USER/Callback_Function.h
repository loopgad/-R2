#ifndef Callback_Function
#define Callback_Function

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "stm32h7xx_hal.h"
#include "crc_util.h"
#include "ROS.h"
#include "Global_Namespace.h"
#include <cstdint>
#include <string.h>
#include <stdbool.h>

using namespace Xbox_Namespace; 
using namespace Action_Namespace;

//xbox手柄接收帧定义
#define FRAME_HEAD_0_RC9 0xFC
#define FRAME_HEAD_1_RC9 0xFB
#define FRAME_END_0_RC9 0xFD
#define FRAME_END_1_RC9 0xFE
#define MAX_DATA_LENGTH_RC9 64
//
#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0
#define PI 3.14159265358979323846

#define   robot_start 0
#define   robot_load_left 1
#define	  robot_shoot 2
#define   robot_load_right 3

#define y  		0
#define x 		1
#define w		2


// 数据帧结构体
typedef struct serial_frame_mat
{
    uint8_t data_length; // 数据载荷的字节数
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint8_t rx_temp_data_mat[MAX_DATA_LENGTH_RC9]; // 临时接收数据缓冲区
    uint8_t frame_end[2];
    union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2]; // CRC 校验的字节形式
    } check_code;
    uint16_t crc_calculated;
} serial_frame_mat_t;

// 全局变量定义
serial_frame_mat_t rx_frame_mat;        // 接收数据帧
uint8_t rxIndex_ = 0;                   // 当前接收的字节索引


// 状态机状态（枚举量）
enum rxState
{
    WAITING_FOR_HEADER_0,
    WAITING_FOR_HEADER_1,
    WAITING_FOR_ID,
    WAITING_FOR_LENGTH,
    WAITING_FOR_DATA,
    WAITING_FOR_CRC_0,
    WAITING_FOR_CRC_1,
    WAITING_FOR_END_0,
    WAITING_FOR_END_1
} state_ = WAITING_FOR_HEADER_0;

//串口结构体指针
extern UART_HandleTypeDef huart1;//手柄使用
extern UART_HandleTypeDef huart2;//ros使用
extern UART_HandleTypeDef huart3;//action使用


inline void Handle_Receive_Data(uint8_t byte);//数据接收处理函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
inline void Update_Action(float value[6]);

//#ifdef __cplusplus
//}
//#endif

#endif // Callback_Function