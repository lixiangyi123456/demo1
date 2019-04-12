
#include "led.h"
#include "rtthread.h"

/************************************************
 ����ԭ�� ̽����F407������	RT-Threadʵ��18
 ��ɫ���shell - �⺯���汾
 
 RT-Thread�ٷ�΢�Ź��ںţ�RTThread
 RT-Thread������̳��http://www.rt-thread.org
 
 ����ԭ�ӹٷ�΢�Ź��ںţ�����ԭ��
 ����ԭ�Ӽ�����̳��www.openedv.com
 ����ԭ���Ա����̣�http://eboard.taobao.com 
 
 ���ߣ� RT-Thread	&	����ԭ��
************************************************/
//lxy

static struct rt_thread led0_thread;//�߳̿��ƿ�
static struct rt_thread led1_thread;//�߳̿��ƿ�
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rt_led0_thread_stack[1024];//�߳�ջ
static rt_uint8_t rt_led1_thread_stack[1024];//�߳�ջ
  
//�߳�LED0
static void led0_thread_entry(void* parameter)
{
    rt_uint32_t count = 0;
    while (1)
    {
        LED0=0; //ע�⣺F7��֧��λ��������LED������ο������޸�
        rt_thread_delay(500);   //��ʱ500��tick
        
        LED0=1;     
        rt_thread_delay(500);  //��ʱ500��tick
				 		
    }
}
  
//�߳�LED1
static void led1_thread_entry(void* parameter)
{
    rt_err_t err;
		rt_uint32_t count = 0;
    while (1)
    {
         LED1=0;   
         rt_thread_delay(100);  //��ʱ100��tick
        
         LED1=1;    
         rt_thread_delay(100);   //��ʱ100��tick			 				 
    }
}
 
int main(void)
{	
    // ������̬�߳�
    rt_thread_init(&led0_thread,                //�߳̿��ƿ�
                   "led0",                      //�߳����֣���shell������Կ���
                   led0_thread_entry,           //�߳���ں���
                   RT_NULL,                     //�߳���ں�������
                   &rt_led0_thread_stack[0],    //�߳�ջ��ʼ��ַ
                   sizeof(rt_led0_thread_stack),//�߳�ջ��С
                   3,                           //�̵߳����ȼ�
                   20);                         //�߳�ʱ��Ƭ
                               
    rt_thread_startup(&led0_thread);            //�����߳�led0_thread����������
									 
    // ������̬�߳�                          
	  rt_thread_init(&led1_thread,                //�߳̿��ƿ�
							  "led1",                         //�߳����֣���shell������Կ���
							  led1_thread_entry,              //�߳���ں���
							  RT_NULL,                        //�߳���ں�������
							  &rt_led1_thread_stack[0],       //�߳�ջ��ʼ��ַ
							  sizeof(rt_led1_thread_stack),   //�߳�ջ��С
							  3,                              //�̵߳����ȼ�
							  20);                                  
  
    rt_thread_startup(&led1_thread);            //�����߳�led1_thread����������
    
}




 



