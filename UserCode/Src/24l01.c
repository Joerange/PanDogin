#include "24l01.h"
#include "main.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "usart.h"


/* 扩展变量 ------------------------------------------------------------------*/

#define CE_LOW 	 HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_RESET)
#define CE_HIGH  HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_SET)
#define CS_LOW 	 HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)
#define CS_HIGH  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)

#define IRQ_READ HAL_GPIO_ReadPin(SPI1_IQRN_GPIO_Port, SPI1_IQRN_Pin)

//uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xE1,0xE2,0xE3,0xE4,0xE5}; //发送地址
//uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xE1,0xE2,0xE3,0xE4,0xE5};

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x19,0x73,0x47,0x42,0x84}; //发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x19,0x73,0x47,0x42,0x84}; //发送地址


/**
 * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
 * 输入参数: byte：待发送数据
 * 返 回 值: uint8_t：接收到的数据
 * 说    明：无
 */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte)
{
    uint8_t d_read, d_send = byte;
    if (HAL_SPI_TransmitReceive(&hspi4, &d_send, &d_read, 1, 0xFFF) != HAL_OK)
    {
        d_read = 0xFF;
    }
    return d_read;
}

/**
 * 函数功能: 检测24L01是否存在
 * 输入参数: 无
 * 返 回 值: 0，成功;1，失败
 * 说    明：无
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;

    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); //写入5个字节的地址.
    NRF24L01_Read_Buf(TX_ADDR, buf, 5);                  //读出写入的地址
    for (i = 0; i < 5; i++)
        if (buf[i] != 0XA5)
            break;
    if (i != 5)
        return 1; //检测24L01错误
    return 0;     //检测到24L01
}

/**
 * 函数功能: SPI写寄存器
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：reg:指定寄存器地址
 *
 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    CS_LOW;                                   //使能SPI传输
    status = SPIx_ReadWriteByte(&hspi4, reg); //发送寄存器号
    SPIx_ReadWriteByte(&hspi4, value);        //写入寄存器的值
    CS_HIGH;                                  //禁止SPI传输
    return (status);                          //返回状态值
}

/**
 * 函数功能: 读取SPI寄存器值
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：reg:要读的寄存器
 *
 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    CS_LOW;                                     //使能SPI传输
    SPIx_ReadWriteByte(&hspi4, reg);            //发送寄存器号
    reg_val = SPIx_ReadWriteByte(&hspi4, 0XFF); //读取寄存器内容
    CS_HIGH;                                    //禁止SPI传输
    return (reg_val);                           //返回状态值
}

/**
 * 函数功能: 在指定位置读出指定长度的数据
 * 输入参数: 无
 * 返 回 值: 此次读到的状态寄存器值
 * 说    明：无
 *
 */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;

    CS_LOW;                                   //使能SPI传输
    status = SPIx_ReadWriteByte(&hspi4, reg); //发送寄存器值(位置),并读取状态值
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        pBuf[uint8_t_ctr] = SPIx_ReadWriteByte(&hspi4, 0XFF); //读出数据
    }
    CS_HIGH;       //关闭SPI传输
    return status; //返回读到的状态值
}

/**
 * 函数功能: 在指定位置写指定长度的数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
 *
 */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;
    CS_LOW;                                   //使能SPI传输
    status = SPIx_ReadWriteByte(&hspi4, reg); //发送寄存器值(位置),并读取状态值
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        SPIx_ReadWriteByte(&hspi4, *pBuf++); //写入数据
    }
    CS_HIGH;       //关闭SPI传输
    return status; //返回读到的状态值
}

/**
 * 函数功能: 启动NRF24L01发送一次数据
 * 输入参数: 无
 * 返 回 值: 发送完成状况
 * 说    明：txbuf:待发送数据首地址
 *
 */
//uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
//{
//    uint8_t sta;
//    CE_LOW;
//    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //写数据到TX BUF  9个字节
//    CE_HIGH;                                                //启动发送
//
//    while (IRQ_READ != 0)
//        ; //等待发送完成
//
//    sta = NRF24L01_Read_Reg(STATUS);                 //读取状态寄存器的值
//    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
//    if (sta & MAX_TX)                                //达到最大重发次数
//    {
//        NRF24L01_Write_Reg(FLUSH_TX, 0xff); //清除TX FIFO寄存器
//        return MAX_TX;
//    }
//    if (sta & TX_OK) //发送完成
//    {
//        return TX_OK;
//    }
//    return 0xff; //其他原因发送失败
//}

/**
 * 函数功能:启动NRF24L01接收一次数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 *
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;
    sta = NRF24L01_Read_Reg(STATUS);                 //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
    if (sta & RX_OK)                                 //接收到数据
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); //读取数据
        NRF24L01_Write_Reg(FLUSH_RX, 0xff);                    //清除RX FIFO寄存器
        return 0;
    }
    return 1; //没收到任何数据
}

/**
 * 函数功能: 该函数初始化NRF24L01到RX模式
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 *
 */
void NRF24L01_RX_Mode(void)
{
    uint8_t sta;
    CE_LOW;
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);     //使能通道0的自动应答

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01); //使能通道0的接收地址

    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);       //设置RF通信频率

    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //选择通道0的有效数据宽度

    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启

    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC

    CE_HIGH; // CE为高,进入接收模式
    //HAL_Delay(1);

    sta = NRF24L01_Read_Reg(STATUS);                 //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志

//    for(uint8_t i = 0; i < 8; i++)
//    {
//        Remote.key[i] =  1000;
//    }
}

/**
 * 函数功能: 该函数初始化NRF24L01到TX模式
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 *
 */
void NRF24L01_TX_Mode(void)
{
    CE_LOW;
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);    //写TX节点地址
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);      //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  //使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0xff); //设置自动重发间隔时间:4000us + 86us;最大自动重发次数:15次
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);        //设置RF通道为40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    CE_HIGH;                                              // CE为高,10us后启动发送
    //HAL_Delay(1);
}

/*声明一些queue和task，会用到的哦*/
//extern osMessageQId remote_keyHandle;
//extern osMessageQId remote_dataHandle;
//
//extern osThreadId ShootTaskHandle;

/**
 * @brief 这个是外部中断的回调函数，nrf的中断引脚会在收到消息时拉低，提醒你收一下消息
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //中断触发接收
{


//    if(NRF24L01_RxPacket(rc_data) == 0)         //接收完成
//    {
//        if(Only_StandUp_flag)
//        {
//            if(rc_data[40] == 17) {         //按下按钮1接收到17
//                gpstate = Stand_Up_Pos;
//                Only_StandUp_flag = 0;
//            } else{
//
//            }
//        }
//        else if (!Only_StandUp_flag)
//        {
//            switch (rc_data[40]) {
//                case 17:              //button1
//                    gpstate = Stand_Up_Pos;
//                    break;
//                case 33:              //button2
//                    gpstate = trot_Pos;
//                    break;
//                case 49:              //button3
//                    gpstate = Stop_Pos;
//                    break;
//                case 65:              //button4
//                    gpstate = Jump_Pos;
//                    break;
//                default:
//                    break;
//            }
//        }
//        else         //虽然啥也没有，但就是想在elseif后面写一个else,防止跑到不知名的地方去，安全，安全，还是tmd安全
//        {
//
//        }
//        launch_sp_kp = (float )((rc_data[11]<<8)|rc_data[10])/100;
//        fall_sp_kp = (float )((rc_data[13]<<8)|rc_data[12])/100;
//        strech_sp_kp = (float )((rc_data[15]<<8)|rc_data[14])/100;
////        usart_printf("%f,%f,%f\r\n",launch_sp_kp,fall_sp_kp,strech_sp_kp);
//    }

}
