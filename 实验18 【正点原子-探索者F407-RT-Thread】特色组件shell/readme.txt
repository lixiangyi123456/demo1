������123//
RT-Thread-�й���������������ϵͳ
�Ϻ�����µ��ӿƼ����޹�˾
�绰��021-58995663
QQȺ��87088566
΢�źţ�RTThread
��˾��վ��http://www.rt-thread.org/
������̳��http://www.rt-thread.org/phpBB3/

ʵ������:
	̽����STM32F4������
	
ʵ��Ŀ��:
	RT-Thread��ɫ���shell���Ƽ�ʹ���ն����putty����SecureCRT
        ʹ������ԭ��XCOM��������Ҳ���ԣ�����version,free,list_thread�Ȳ������
msh >version

 \ | /
- RT -     Thread Operating System
 / | \     2.1.0 build Nov 18 2017
 2006 - 2017 Copyright by rt-thread team
msh >list_thread
thread pri  status      sp     stack size max used left tick  error
------ ---  ------- ---------- ----------  ------  ---------- ---
led1     3  suspend 0x00000078 0x00000400    11%   0x00000014 000
led0     3  suspend 0x00000078 0x00000400    11%   0x00000014 000
tshell   7  ready   0x00000134 0x00001000    08%   0x00000007 000
tidle    7  ready   0x00000058 0x00000100    34%   0x00000020 000
msh >free
total memory: 113160
used memory : 536
maximum allocated memory: 1704
msh >

	
Ӳ����Դ:
	1,DS0(������PF9) 
	2,DS1(������PF10)
	3.rt_kprintf��ӳ�䵽����1
		
ʵ������:
�նˣ�115200-N-1����ӡ��Ϣ


ע������:
1.RTEֻ�蹴ѡRTT RTOS��kernel
2.�����ں�LED��ʼ��������ӵ�void rt_hw_board_init()����

����ԭ��@ALIENTEK
2014-10-24
������������ӿƼ����޹�˾
�绰��020-38271790
���棺020-36773971
����http://shop62103354.taobao.com
http://shop62057469.taobao.com
��˾��վ��www.alientek.com
������̳��www.openedv.com
              