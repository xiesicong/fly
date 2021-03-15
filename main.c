#include "stm32f10x.h"
#include "systick.h"
#include "led.h"
#include "spi.h"
#include "nrf24l01.h"
#include "usart2.h"
#include "iic.h"
#include "mpu6050.h"
#include "timer.h"
#include "nvic.h"
#include "imu.h"
#include "ist8310.h"
#include "spl06.h"
#include "pwm.h"
#include "adc.h"
#include "flash.h"
#include "pid.h"
#include "parse_packet.h"
#include "gyro_cal.h"
#include "acc_cal.h"
#include "usart3.h"
#include "flow.h"
#include "fc_status.h"
#include "pair_freq.h"

/**********************************注意************************************/
/********************飞行前请务必先校准陀螺仪和加速度计********************/
/******如果不校准陀螺仪零偏，飞行时会自旋，而且还会影响姿态解算精度********/
/*******************如果不校准加速度计，飞行时定高不稳*********************/


int main(void)
{
    SystemInit();                       //系统初始化    
    systick_init();                     //系统滴答定时器初始化
    
    led_init();                         //状态灯初始化
    get_chip_id();
    
    usart2_init(115200);                //串口  
    usart3_init(115200);                //光流串口接收初始化
    pwm_init();                         //pwm初始
        
    delay_ms(100);                      //延时处理，以便开机进行零偏采集，防止瞬间抖动

    Spi_Init();
 	NRF24L01_Init();    		
	while(NRF24L01_Check())             //无线在位检测
	{ 
		delay_ms(50);
        led_toggle(2);
	}
    NRF24L01_RX_Mode();   				//接收模式
    
    wait_pairing();
    
    delay_ms(100);
    IIC_Init();                         
    mpu6050_init();                     
    while(get_mpu_id()!=0x68)           //mpu6050在位检测    
    {                                   
        led_toggle(1);                  
        delay_ms(50);                   
    }

    spl_init();                         //气压计初始化    
                                        
    get_iir_factor(&Mpu.att_acc_factor,0.005f,15);   	//姿态解算时加速度低通系数 
	get_iir_factor(&Mpu.fix_acc_factor,0.005f,2);		//高度融合时加速度低通系数
    get_iir_factor(&flow.vel_lpf_factor,0.005f,1);		//光流速度低通系数
    

    
    adc_init();   
    all_pid_init();                     //pid参数初始化
    
    read_cal_dat();						//flash校准数据读取
 
    delay_ms(100);
    led_off_all();                      //关闭状态灯
    
    timer_init();                       //定时器初始化    
    NVIC_config();                      //中断配置初始化
    
    while(1)
    {
		gyro_cal(&gyro_raw_f);          //陀螺仪零偏校准 
        acc_cal(&acc_att_lpf);  		//加速度计校准
        
        parse_key_info();				//按键数据解析
        		
		voltage_detection();            //低电压检测
    }
}










/*

Code:    存在ROM(FLASH)里面的指令，这是在程序运行过程中不变的量，是指令
RO_data：只读数据(Read only data)，是指令的操作数
RW_data: 已经初始化的可读写变量的大小，这个存在两个地方，初始化量存在ROM(FLASH)，
         由于还要对其进行"写"操作，所以RAM中也要占有相应空间
ZI_data: 程序中未初始化的变量大小(Zero Initialize)，存在RAM中

    ROM(FLASH) SIZE:  Code + RO-data + RW-data 
    FLASH SIZE = 25230 + 1130 + 208 = 26.568KB
    
    FLASH起始地址：0x08000000，所以在设置FLASH进行保存的时候，要大于本代码所占用的扇区地址
    代码扇区大小计算：0x08000000~0x08003fff  ：16*16*16*4=16KB
    所以在设置FLASH地址的时候要在代码flash之外
*/





