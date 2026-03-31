#ifndef __CS_NAVIGATIN_H__
#define __CS_NAVIGATIN_H__

//#ifdef __cplusplus
//extern "C" {
//#endif  // __cplusplus


/*
 *@func:通过寄存器reg48的值，判断手指滑动方向
 *@param[in]:reg48_arr存放寄存器reg48采样数组
 *@param[in]:length是数组长度
 *@param[in]:mode选择导航模式，0表示兼顾上下左右滑动、1表示只做上下滑动、2表示只做左右滑动
 *@return：返回手指滑动方向，例如上(1)、下(2)、左(3)、右(4)、UNKOWN（-1）
 */

//导航12个感应点宏开关
//lww add 2016.08.19
#define NAV_USE_12_POINTS 1

//设置防抖动阈值（灵敏度）,在这里直接赋值:理论值乘以一个系数，系数可调
//其中min_threahold(8_point) = 2;max_threshold(8_point)=180
//min_threahold(12_point) = 1;max_threshold(8_point)=190

#if NAV_USE_12_POINTS
#define THRESHOLD  (1*100)
#else
#define THRESHOLD  (2*25)	
#endif

int cs_finger_navigation(int *reg48_arr,int length,int mode);

//#ifdef __cplusplus
//}  // extern "C"
//#endif  // __cplusplus

#endif