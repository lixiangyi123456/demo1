
#include "led.h"
#include "rtthread.h"

/************************************************
 正点原子 探索者F407开发板	RT-Thread实验18
 特色组件shell - 库函数版本
 
 RT-Thread官方微信公众号：RTThread
 RT-Thread技术论坛：http://www.rt-thread.org
 
 正点原子官方微信公众号：正点原子
 正点原子技术论坛：www.openedv.com
 正点原子淘宝店铺：http://eboard.taobao.com 
 
 作者： RT-Thread	&	正点原子
************************************************/
//lxy

static struct rt_thread led0_thread;//线程控制块
static struct rt_thread led1_thread;//线程控制块
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rt_led0_thread_stack[1024];//线程栈
static rt_uint8_t rt_led1_thread_stack[1024];//线程栈
  
//线程LED0
static void led0_thread_entry(void* parameter)
{
    rt_uint32_t count = 0;
    while (1)
    {
        LED0=0; //注意：F7不支持位带操作，LED操作请参考代码修改
        rt_thread_delay(500);   //延时500个tick
        
        LED0=1;     
        rt_thread_delay(500);  //延时500个tick
				 		
    }
}
  
//线程LED1
static void led1_thread_entry(void* parameter)
{
    rt_err_t err;
		rt_uint32_t count = 0;
    while (1)
    {
         LED1=0;   
         rt_thread_delay(100);  //延时100个tick
        
         LED1=1;    
         rt_thread_delay(100);   //延时100个tick			 				 
    }
}
 
int main(void)
{	
    // 创建静态线程
    rt_thread_init(&led0_thread,                //线程控制块
                   "led0",                      //线程名字，在shell里面可以看到
                   led0_thread_entry,           //线程入口函数
                   RT_NULL,                     //线程入口函数参数
                   &rt_led0_thread_stack[0],    //线程栈起始地址
                   sizeof(rt_led0_thread_stack),//线程栈大小
                   3,                           //线程的优先级
                   20);                         //线程时间片
                               
    rt_thread_startup(&led0_thread);            //启动线程led0_thread，开启调度
									 
    // 创建静态线程                          
	  rt_thread_init(&led1_thread,                //线程控制块
							  "led1",                         //线程名字，在shell里面可以看到
							  led1_thread_entry,              //线程入口函数
							  RT_NULL,                        //线程入口函数参数
							  &rt_led1_thread_stack[0],       //线程栈起始地址
							  sizeof(rt_led1_thread_stack),   //线程栈大小
							  3,                              //线程的优先级
							  20);                                  
  
    rt_thread_startup(&led1_thread);            //启动线程led1_thread，开启调度
    
}




 



