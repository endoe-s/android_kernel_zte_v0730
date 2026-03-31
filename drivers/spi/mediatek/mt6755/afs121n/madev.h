/* MicroArray Fprint
 * madev.h
 * date: 2015-11-02
 * version: v2.3
 * Author: czl
 */

#ifndef MADEV_H
#define MADEV_H

#define SPI_SPEED 	(6*1000000) 	//120/121:10M, 80/81:6M

//表面类型
#define	COVER_T		1
#define COVER_N		2
#define COVER_M		3
#define COVER_NUM	COVER_N

//指纹类型
#define AFS120	0x78
//#define AFS80 	0x50

#if defined(AFS120)
	#define W   	120   //宽
	#define H   	120   //高
	#define WBUF	121
	#define FBUF  	(1024*15)	//读写长度
#elif defined(AFS80)
	#define W   	80    //宽
	#define H   	192   //高
	#define WBUF	81
	#define FIMG	(W*H)
	#define FBUF  	(1024*16)	//读写长度
#endif

//接口命令
#define IOCTL_DEBUG			0x100	//调试信息
#define IOCTL_IRQ_ENABLE	0x101	//中断使能
#define IOCTL_SPI_SPEED   	0x102	//SPI速度
#define IOCTL_READ_FLEN		0x103	//读帧长度(保留)
#define IOCTL_LINK_DEV		0x104	//连接设备(保留)
#define IOCTL_COVER_NUM		0x105	//材料编号
#define IOCTL_GET_VDATE		0x106	//版本日期

#define IOCTL_CLR_INTF		0x110	//清除中断标志
#define IOCTL_GET_INTF		0x111	//获取中断标志
#define IOCTL_REPORT_FLAG	0x112 	//上报标志
#define IOCTL_REPORT_KEY	0x113	//上报键值
#define IOCTL_SET_WORK		0x114	//设置工作
#define IOCTL_GET_WORK		0x115	//获取工作
#define IOCTL_SET_VALUE		0x116	//设值
#define IOCTL_GET_VALUE		0x117	//取值
#define IOCTL_TRIGGER		0x118	//唤醒

#define IOCTL_WAKE_LOCK         0x119
#define IOCTL_WAKE_UNLOCK       0x120

#define IOCTL_SCREEN_ON 	0x121
#define IOCTL_KEY_DOWN 		0x121
#define IOCTL_KEY_UP 		0x122
#define IOCTL_SET_X			0x123
#define IOCTL_SET_Y			0x124
#define IOCTL_KEY_TAP		0x125
#define IOCTL_KEY_DTAP		0x126
#define IOCTL_KEY_LTAP 		0x127

#define IOCTL_KEY_TAP_ZERO  0x130
#define IOCTL_KEY_TAP_ONE   0x131
#define IOCTL_KEY_LEFT		0x132
#define IOCTL_KEY_RIGHT		0x133


#define TRUE 	1
#define FALSE 	0

#endif /* MADEV_H */



