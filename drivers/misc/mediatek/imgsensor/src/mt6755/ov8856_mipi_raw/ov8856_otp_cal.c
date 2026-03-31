#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <linux/xlog.h>
//#include <asm/system.h>

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"


#include "ov8856mipiraw_Sensor.h"
#include "ov8856_otp.h"

#if 1
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

//#define OV8856_R2A_write_i2c(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV8856MIPI_WRITE_ID)
#define PFX "OV8856_R2A_OTP"
#define LOG_INF(format, args...)	 //xlog_printk(ANDROID_LOG_ERROR   , PFX, "[%s] " format, __FUNCTION__, ##args)


#define Delay(ms)  mdelay(ms)
static unsigned char OV8856MIPI_WRITE_ID = 0X00;

kal_uint16 OV8856_R2A_read_i2c(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV8856MIPI_WRITE_ID);
    return get_byte;
}

kal_uint16 OV8856_R2A_write_i2c(int addr,int  para)
{
		iWriteReg((u16) addr , (u32) para , 1, OV8856MIPI_WRITE_ID);
		return 1;
}
#endif
static struct ov8856_otp_struct current_otp;
static bool firstOpenCamer = true;
void ov8856_otp_cali(unsigned char writeid)
{
	//struct ov8856_otp_struct current_otp;
	OV8856MIPI_WRITE_ID = writeid;
	
	
	if(firstOpenCamer)
		{
		memset(&current_otp, 0, sizeof(struct ov8856_otp_struct));
	ov8856_read_otp(&current_otp);
	printk("this is first time enter ,need to read otp zlx!\n");
	firstOpenCamer=false;
		}
		
	ov8856_apply_otp(&current_otp);
		
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc/invalid otp lenc, 1 valid otp lenc
int ov8856_read_otp(struct ov8856_otp_struct *otp_ptr)
{
int otp_flag, addr, temp, i;
//set 0x5001[3] to ¡°0¡±
int temp1;
int checksum2=0;
temp1 = OV8856_R2A_read_i2c(0x5001);
OV8856_R2A_write_i2c(0x5001, (0x00 & 0x08) | (temp1 & (~0x08)));
// read OTP into buffer
OV8856_R2A_write_i2c(0x3d84, 0xC0);
OV8856_R2A_write_i2c(0x3d88, 0x70); // OTP start address
OV8856_R2A_write_i2c(0x3d89, 0x10);
OV8856_R2A_write_i2c(0x3d8A, 0x72); // OTP end address
OV8856_R2A_write_i2c(0x3d8B, 0x0a);
OV8856_R2A_write_i2c(0x3d81, 0x01); // load otp into buffer
	mdelay(10);
// OTP base information and WB calibration data
otp_flag = OV8856_R2A_read_i2c(0x7010);
addr = 0;
if((otp_flag & 0xc0) == 0x40) {
addr = 0x7011; // base address of info group 1
}
else if((otp_flag & 0x30) == 0x10) {
addr = 0x7019; // base address of info group 2
}
if(addr != 0) {
(*otp_ptr).flag = 0xC0; // valid info and AWB in OTP
(*otp_ptr).module_integrator_id = OV8856_R2A_read_i2c(addr);
(*otp_ptr).lens_id = OV8856_R2A_read_i2c( addr + 1);
(*otp_ptr).production_year = OV8856_R2A_read_i2c( addr + 2);
(*otp_ptr).production_month = OV8856_R2A_read_i2c( addr + 3);
(*otp_ptr).production_day = OV8856_R2A_read_i2c(addr + 4);
temp = OV8856_R2A_read_i2c(addr + 7);
(*otp_ptr).rg_ratio = (OV8856_R2A_read_i2c(addr + 5)<<2) + ((temp>>6) & 0x03);
(*otp_ptr).bg_ratio = (OV8856_R2A_read_i2c(addr + 6)<<2) + ((temp>>4) & 0x03);
printk("---zlx---ov8856_read_otp: rg_ratio: %d, bg_ratio: %d\n", (*otp_ptr).rg_ratio,(*otp_ptr).bg_ratio);
}
else {
(*otp_ptr).flag = 0x00; // not info and AWB in OTP
(*otp_ptr).module_integrator_id = 0;
(*otp_ptr).lens_id = 0;
(*otp_ptr).production_year = 0;
(*otp_ptr).production_month = 0;
(*otp_ptr).production_day = 0;
(*otp_ptr).rg_ratio = 0;
(*otp_ptr).bg_ratio = 0;
}
// OTP VCM Calibration
otp_flag = OV8856_R2A_read_i2c(0x7021);
addr = 0;
if((otp_flag & 0xc0) == 0x40) {
addr = 0x7022; // base address of VCM Calibration group 1
}
else if((otp_flag & 0x30) == 0x10) {
addr = 0x7025; // base address of VCM Calibration group 2
}
if(addr != 0) {
(*otp_ptr).flag |= 0x20;
temp = OV8856_R2A_read_i2c(addr + 2);
(* otp_ptr).VCM_start = (OV8856_R2A_read_i2c(addr)<<2) | ((temp>>6) & 0x03);
(* otp_ptr).VCM_end = (OV8856_R2A_read_i2c(addr + 1) << 2) | ((temp>>4) & 0x03);
(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
}
else {
(* otp_ptr).VCM_start = 0;
(* otp_ptr).VCM_end = 0;
(* otp_ptr).VCM_dir = 0;

}
//just for debug
/*
int val=0;
for(i=0x7010;i<=0x720a;i++) 
{
val=OV8856_R2A_read_i2c( i);
printk("---zlx---read  otp: [0x%x] = %d\n", i, val);
}
*/

// OTP Lenc Calibration
otp_flag = OV8856_R2A_read_i2c(0x7028);
addr = 0;
if((otp_flag & 0xc0) == 0x40) {
addr = 0x7029; // base address of Lenc Calibration group 1
}
else if((otp_flag & 0x30) == 0x10) {
addr = 0x711a; // base address of Lenc Calibration group 2
}
if(addr != 0) {
for(i=0;i<240;i++) {
(* otp_ptr).lenc[i]=OV8856_R2A_read_i2c(addr + i);
//printk("---zlx---read lens otp: lenc[%d] = %d\n", i, (* otp_ptr).lenc[i]);
checksum2 += (* otp_ptr).lenc[i];
}
checksum2 = (checksum2)%255 +1;
(* otp_ptr).checksum = OV8856_R2A_read_i2c(addr + 240);
if((* otp_ptr).checksum == checksum2){
(*otp_ptr).flag |= 0x10;
}
}
else {
for(i=0;i<240;i++) {
(* otp_ptr).lenc[i]=0;
}
}
for(i=0x7010;i<=0x720a;i++) {
OV8856_R2A_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
}
//set 0x5001[3] to ¡°1¡±
temp1 = OV8856_R2A_read_i2c(0x5001);
OV8856_R2A_write_i2c(0x5001, (0x08 & 0x08) | (temp1 & (~0x08)));
return (*otp_ptr).flag;
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int ov8856_apply_otp(struct ov8856_otp_struct *otp_ptr)
{
int RG_Ratio_Typical = 0x12c, BG_Ratio_Typical = 0x157;
int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
// apply OTP WB Calibration
if ((*otp_ptr).flag & 0x40) {
rg = (*otp_ptr).rg_ratio;
bg = (*otp_ptr).bg_ratio;


//calculate G gain
R_gain = (RG_Ratio_Typical*1000) / rg;
B_gain = (BG_Ratio_Typical*1000) / bg;
G_gain = 1000;
if (R_gain < 1000 || B_gain < 1000)
{
if (R_gain < B_gain)
Base_gain = R_gain;
else
Base_gain = B_gain;
}
else
{
Base_gain = G_gain;
}
R_gain = 0x400 * R_gain / (Base_gain);
B_gain = 0x400 * B_gain / (Base_gain);
G_gain = 0x400 * G_gain / (Base_gain);
printk("---zlx---ov8856_apply_otp: R_gain: 0x%x, B_gain: 0x%x, G_gain: 0x%x\n", R_gain,B_gain,G_gain);
// update sensor WB gain
if (R_gain>0x400) {
OV8856_R2A_write_i2c(0x5019, R_gain>>8);
OV8856_R2A_write_i2c(0x501a, R_gain & 0x00ff);
}
if (G_gain>0x400) {
OV8856_R2A_write_i2c(0x501b, G_gain>>8);
OV8856_R2A_write_i2c(0x501c, G_gain & 0x00ff);
}
if (B_gain>0x400) {
OV8856_R2A_write_i2c(0x501d, B_gain>>8);
OV8856_R2A_write_i2c(0x501e, B_gain & 0x00ff);
}
}
// apply OTP Lenc Calibration
if ((*otp_ptr).flag & 0x10) {
temp = OV8856_R2A_read_i2c(0x5000);
temp = 0x20 | temp;
OV8856_R2A_write_i2c(0x5000, temp);
for(i=0;i<240;i++) {
OV8856_R2A_write_i2c(0x5900 + i, (*otp_ptr).lenc[i]);
}
}
//printk("---zlx---apply otp: (*otp_ptr).flag= %d\n", (*otp_ptr).flag);
return (*otp_ptr).flag;
}
