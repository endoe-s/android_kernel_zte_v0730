#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/errno.h>
//#include <linux/xlog.h> 
//#include <asm/system.h>

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"


#include "ov13853mipiraw_Sensor.h"
/****************************Modify Following Strings for Debug****************************/
//#define PFX "OV13853_camera_pdaf"
//#define LOG_1 LOG_INF("OV13853,MIPI 4LANE\n")
//#define LOG_2 LOG_INF("preview 2096*1552@30fps,640Mbps/lane; video 4192*3104@30fps,1.2Gbps/lane; capture 13M@30fps,1.2Gbps/lane\n")
/****************************   Modify end    *******************************************/

//#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define TAG_NAME "OV13853_PDAF_READ"
#define LOG_INF(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

//#include "ov13853_otp.h"
struct otp_pdaf_struct {
unsigned char pdaf_flag; //bit[7]--0:empty; 1:Valid
unsigned char data1[496];//output data1
unsigned char data2[806];//output data2
unsigned char data3[102];//output data3
unsigned char pdaf_checksum;//checksum of pd, SUM(0x0801~0x0D7C)%255+1

};




extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define EEPROM_READ_ID    0xB1
#define EEPROM_WRITE_ID   0xB0
#define I2C_SPEED        400  //CAT24C512 can support 1Mhz

#define START_OFFSET     0x0000

#define Delay(ms)  mdelay(ms)
//static unsigned char OV13853MIPI_WRITE_ID = (0xA0 >> 1);
//#define EEPROM_READ_ID  0xA3
//#define EEPROM_WRITE_ID   0xA2
#define MAX_OFFSET       0xcd8
#define DATA_SIZE 4096
//BYTE eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static bool OV13853_selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > MAX_OFFSET)
        return false;
	kdSetI2CSpeed(I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, EEPROM_WRITE_ID)<0)
		return false;
    return true;
}


static bool OV13853_read_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	unsigned char flag1=0;
	//unsigned char flag1_year=0;
	//unsigned char flag1_mouth=0;
	//unsigned char flag1_day=0;
	
	int offset =0;

    //OV13853_selective_read_eeprom(0x0001, &flag1_year);
	//OV13853_selective_read_eeprom(0x0002, &flag1_mouth);
	//OV13853_selective_read_eeprom(0x0003, &flag1_day);

	//LOG_INF("--lihy--OV13853_read_eeprom: year nouth day = %d - %d - %d\n", flag1_year, flag1_mouth, flag1_day);
	
	OV13853_selective_read_eeprom(0x0400, &flag1);
	//LOG_INF("--lihy--OV13853_read_eeprom: flag1=0x%x\n", flag1);
	offset = 0x0401;
	if((flag1 == 0xe0)||(flag1 == 0xff))
	{
		for(i = 0; i < 496; i++) {
			if(!OV13853_selective_read_eeprom(offset, &data[i])){
				//LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
				return false;
			}
			if(i==5)
			{
				//LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
				data[i] = 0x5c;
			}
			//LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
			offset++;
		}
	
		offset=0x0802;
		for(i = 496; i <1372; i++) {  //0x0002 --- 0x0327 page 2
			if(!OV13853_selective_read_eeprom(offset, &data[i])){
				//LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
				return false;
			}
			if(i == 501)
			{
				//LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
				data[i] = 0x5c;
			}
			//LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
			offset++;
		}
        #if 0
        offset=0x0B28;
		for(i = 1302; i < 1372; i++) {  //0x0328--0x036e  page 2
			if(!OV13853_selective_read_eeprom(offset, &data[i])){
				return false;
			}
			offset++;
		}
		#endif
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_otp_pdaf_data( kal_uint16 addr, BYTE* data, kal_uint32 size){
	
	LOG_INF("read_otp_pdaf_data enter");
	if(!get_done || last_size != size || last_offset != addr) {
		//if(!_read_eeprom(addr, eeprom_data, size)){
		if(!OV13853_read_eeprom(addr, data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			//LOG_INF("read_otp_pdaf_data fail");
			return false;
		}
	}
	//memcpy(data, eeprom_data, size);
	//LOG_INF("read_otp_pdaf_data end");
    return true;
}

//
