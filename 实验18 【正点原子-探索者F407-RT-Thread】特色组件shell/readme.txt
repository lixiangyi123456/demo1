测试用123//
RT-Thread-中国自主物联网操作系统
上海睿赛德电子科技有限公司
电话：021-58995663
QQ群：87088566
微信号：RTThread
公司网站：http://www.rt-thread.org/
技术论坛：http://www.rt-thread.org/phpBB3/

实验器材:
	探索者STM32F4开发板
	
实验目的:
	RT-Thread特色组件shell，推荐使用终端软件putty或者SecureCRT
        使用正点原子XCOM调试助手也可以，输入version,free,list_thread等测试命令。
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

	
硬件资源:
	1,DS0(连接在PF9) 
	2,DS1(连接在PF10)
	3.rt_kprintf重映射到串口1
		
实验现象:
终端（115200-N-1）打印信息


注意事项:
1.RTE只需勾选RTT RTOS的kernel
2.将串口和LED初始化函数添加到void rt_hw_board_init()里面

正点原子@ALIENTEK
2014-10-24
广州市星翼电子科技有限公司
电话：020-38271790
传真：020-36773971
购买：http://shop62103354.taobao.com
http://shop62057469.taobao.com
公司网站：www.alientek.com
技术论坛：www.openedv.com
              