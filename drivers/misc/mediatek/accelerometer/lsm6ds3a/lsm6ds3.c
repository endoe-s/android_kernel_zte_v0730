/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* ST LSM6DS3 Accelerometer and Gyroscope sensor driver on MTK platform
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <cust_acc.h>
#include "lsm6ds3.h"
#include <accel.h>
#include <hwmsensor.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h> 


static DEFINE_MUTEX(lsm6ds3_i2c_mutex);
/* Maintain  cust info here */
struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

/* For  driver get cust info */
struct acc_hw *get_cust_acc(void)
{
	return &accel_cust;
}


/*---------------------------------------------------------------------------*/
#define DEBUG 									1
/*----------------------------------------------------------------------------*/
#define CONFIG_LSM6DS3_LOWPASS   				1 /*apply low pass filter on output */  
#define LSM6DS3_STEP_COUNTER   					1 
//#define LSM6DS3_TILT_FUNCTION   				1 /*dependency on  LSM6DS3_STEP_COUNTER */
//#define LSM6DS3_SIGNIFICANT_MOTION   			1 /*dependency on  LSM6DS3_STEP_COUNTER */

/*----------------------------------------------------------------------------*/
#define LSM6DS3_AXIS_X          	0
#define LSM6DS3_AXIS_Y          	1
#define LSM6DS3_AXIS_Z          	2
#define LSM6DS3_ACC_AXES_NUM        3
#define LSM6DS3_GYRO_AXES_NUM       3
#define LSM6DS3_ACC_DATA_LEN        6   
#define LSM6DS3_GYRO_DATA_LEN       6   
#define LSM6DS3_ACC_DEV_NAME        "lsm6ds3a"
/*----------------------------------------------------------------------------*/
#if defined(LSM6DS3_STEP_COUNTER)
#include <step_counter.h>
#endif
#if defined(LSM6DS3_TILT_FUNCTION)
#include <tilt_detector.h>
#endif

static const struct i2c_device_id lsm6ds3_i2c_id[] = {{LSM6DS3_ACC_DEV_NAME,0},{}};
/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lsm6ds3_i2c_remove(struct i2c_client *client);
static int LSM6DS3_init_client(struct i2c_client *client, bool enable);
static int LSM6DS3_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DS3_ACC_AXES_NUM]);
static int lsm6ds3_acc_suspend(struct i2c_client *client, pm_message_t msg);
static int lsm6ds3_acc_resume(struct i2c_client *client);
static int LSM6DS3_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate);

#if defined(LSM6DS3_STEP_COUNTER)
static int LSM6DS3_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable);
static int LSM6DS3_acc_Enable_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_FUNC_EN_t newValue);
static int LSM6DS3_Write_PedoThreshold(struct i2c_client *client, u8 newValue);
static int LSM6DS3_Reset_Pedo_Data(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue);
static irqreturn_t lsm6ds3_eint_func(int irq, void *info);

#if defined(LSM6DS3_TILT_FUNCTION)
static int LSM6DS3_Enable_Tilt_Func(struct i2c_client *client, bool enable);
static int LSM6DS3_Enable_Tilt_Func_On_Int(struct i2c_client *client, LSM6DS3_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable);
#endif
#if defined(LSM6DS3_SIGNIFICANT_MOTION)
static int LSM6DS3_Set_SigMotion_Threshold(struct i2c_client *client, u8 SigMotion_Threshold);
static int LSM6DS3_Enable_SigMotion_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_SIGN_MOT_t newValue);
static int LSM6DS3_Int_Ctrl(struct i2c_client *client, LSM6DS3_ACC_GYRO_INT_ACTIVE_t int_act, LSM6DS3_ACC_GYRO_INT_LATCH_CTL_t int_latch);
static int LSM6DS3_Enable_SigMotion_Func_On_Int(struct i2c_client *client, bool enable);
#endif
#endif
static int LSM6DS3_acc_SetFullScale(struct i2c_client *client, u8 acc_fs);
static int LSM6DS3_Get_Pedo_DataReg(struct i2c_client *client, u16 *Value);


/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
    GYRO_TRC_FILTER  = 0x01,
    GYRO_TRC_RAWDATA = 0x02,
    GYRO_TRC_IOCTL   = 0x04,
    GYRO_TRC_CALI	= 0X08,
    GYRO_TRC_INFO	= 0X10,
    GYRO_TRC_DATA	= 0X20,
} GYRO_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_ACC_AXES_NUM];
    int sum[LSM6DS3_ACC_AXES_NUM];
    int num;
    int idx;
};
struct gyro_data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_GYRO_AXES_NUM];
    int sum[LSM6DS3_GYRO_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct lsm6ds3_i2c_data {
    struct i2c_client *client;
	struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t 				layout;
    /*misc*/
    //struct data_resolution *reso;
    struct delayed_work   eint_work;				
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[LSM6DS3_GYRO_AXES_NUM+1];

    /*data*/
	s8                      offset[LSM6DS3_ACC_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[LSM6DS3_ACC_AXES_NUM+1];
	
	int 					sensitivity;

#if defined(CONFIG_LSM6DS3_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif     
    u32 current_step;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif

static struct i2c_driver lsm6ds3_i2c_driver = {
    .driver = {
        .name           = LSM6DS3_ACC_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = accel_of_match,
#endif  
    },
	.probe      		= lsm6ds3_i2c_probe,
	.remove    			= lsm6ds3_i2c_remove,
    .suspend            = lsm6ds3_acc_suspend,
    .resume             = lsm6ds3_acc_resume,
	.id_table = lsm6ds3_i2c_id,
};

static unsigned int stepC_irq;
static int of_get_LSM6DS3_platform_data(struct device *dev);

static int lsm6ds3_local_init(void);
static int lsm6ds3_local_uninit(void);

static int lsm6ds3_acc_init_flag = -1;

static struct acc_init_info  lsm6ds3_init_info = {
	.name   = LSM6DS3_ACC_DEV_NAME,
	.init   = lsm6ds3_local_init,
	.uninit = lsm6ds3_local_uninit,
};

#if defined(LSM6DS3_STEP_COUNTER)
static int lsm6ds3_step_c_local_init(void);
static int lsm6ds3_step_c_local_uninit(void);

static struct step_c_init_info  lsm6ds3_step_c_init_info = {
	.name   = "lsm6ds3_step_counter",
	.init   = lsm6ds3_step_c_local_init,
	.uninit = lsm6ds3_step_c_local_uninit,
	//.platform_diver_addr = &lsm6ds3_pedometer_driver,
};
#endif
#if defined(LSM6DS3_TILT_FUNCTION)
static int lsm6ds3_tilt_local_init(void);
static int lsm6ds3_tilt_local_uninit(void);
static struct tilt_init_info  lsm6ds3_tilt_init_info = {
	.name   = "LSM6DS3_TILT",
	.init   = lsm6ds3_tilt_local_init,
	.uninit = lsm6ds3_tilt_local_uninit,
};
#endif


/*----------------------------------------------------------------------------*/
static struct i2c_client *lsm6ds3_i2c_client = NULL;
static struct lsm6ds3_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static bool enable_status = false;
static bool pedo_enable_status = false;
static struct wake_lock lsm6ds3_stepc_wake_lock; //send data to framework before sleep


static DEFINE_MUTEX(lsm6ds3_mutex);
static DEFINE_MUTEX(current_step_mutex);
/*----------------------------------------------------------------------------*/

#define GSE_TAG                  "[LSM6DS3-ACCEL] "
#define GSE_FUN(f)               printk( GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk( GSE_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static void LSM6DS3_dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x0C;
  u8 regdata=0;
  for(i=0; i<35 ; i++)
  {
    //dump all
    hwmsen_read_byte(client,addr,&regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;	
  }

  addr = 0x49;

  for(i=0x49; i<0x60; i++)
  {
    //dump all
    hwmsen_read_byte(client,addr,&regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;
  }
}

static void LSM6DS3_power(struct acc_hw *hw, unsigned int on) 
{
#if 0 // According to MTK porting guide, so marked the following codes.
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LSM6DS3"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "LSM6DS3"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on; 
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_write_rel_calibration(struct lsm6ds3_i2c_data *obj, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    obj->cali_sw[LSM6DS3_AXIS_X] = obj->cvt.sign[LSM6DS3_AXIS_X]*dat[obj->cvt.map[LSM6DS3_AXIS_X]];
    obj->cali_sw[LSM6DS3_AXIS_Y] = obj->cvt.sign[LSM6DS3_AXIS_Y]*dat[obj->cvt.map[LSM6DS3_AXIS_Y]];
    obj->cali_sw[LSM6DS3_AXIS_Z] = obj->cvt.sign[LSM6DS3_AXIS_Z]*dat[obj->cvt.map[LSM6DS3_AXIS_Z]];
#if DEBUG		
		if(atomic_read(&obj->trace) & GYRO_TRC_CALI)
		{
			GSE_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n", 
				obj->cvt.sign[LSM6DS3_AXIS_X],obj->cvt.sign[LSM6DS3_AXIS_Y],obj->cvt.sign[LSM6DS3_AXIS_Z],
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				obj->cvt.map[LSM6DS3_AXIS_X],obj->cvt.map[LSM6DS3_AXIS_Y],obj->cvt.map[LSM6DS3_AXIS_Z]);
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)\n", 
				obj->cali_sw[LSM6DS3_AXIS_X],obj->cali_sw[LSM6DS3_AXIS_Y],obj->cali_sw[LSM6DS3_AXIS_Z]);
		}
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_ResetCalibration(struct i2c_client *client)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;    
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_ReadCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->cali_sw[LSM6DS3_AXIS_X];
    dat[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->cali_sw[LSM6DS3_AXIS_Y];
    dat[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->cali_sw[LSM6DS3_AXIS_Z];

#if DEBUG		
		if(atomic_read(&obj->trace) & GYRO_TRC_CALI)
		{
			GSE_LOG("Read gyro calibration data  (%5d, %5d, %5d)\n", 
				dat[LSM6DS3_AXIS_X],dat[LSM6DS3_AXIS_Y],dat[LSM6DS3_AXIS_Z]);
		}
#endif
                                       
    return 0;
}


static int LSM6DS3_acc_WriteCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[LSM6DS3_GYRO_AXES_NUM];


	GSE_FUN();
	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        		
		cali[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->cali_sw[LSM6DS3_AXIS_X];
		cali[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->cali_sw[LSM6DS3_AXIS_Y];
		cali[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->cali_sw[LSM6DS3_AXIS_Z]; 
		cali[LSM6DS3_AXIS_X] += dat[LSM6DS3_AXIS_X];
		cali[LSM6DS3_AXIS_Y] += dat[LSM6DS3_AXIS_Y];
		cali[LSM6DS3_AXIS_Z] += dat[LSM6DS3_AXIS_Z];
#if DEBUG		
		if(atomic_read(&obj->trace) & GYRO_TRC_CALI)
		{
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n", 
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				cali[LSM6DS3_AXIS_X],cali[LSM6DS3_AXIS_Y],cali[LSM6DS3_AXIS_Z]);
		}
#endif
		return LSM6DS3_acc_write_rel_calibration(obj, cali);
	} 

	return err;
}
/*----------------------------------------------------------------------------*/
static int LSM6DS3_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	//databuf[0] = LSM6DS3_FIXED_DEVID;    

	res = hwmsen_read_byte(client,LSM6DS3_WHO_AM_I,databuf);
    GSE_LOG(" LSM6DS3  id=%x, res=%d\n",databuf[0],res);
	if(databuf[0]!=LSM6DS3_FIXED_DEVID)
	{
		return LSM6DS3_ERR_IDENTIFICATION;
	}

	if (res < 0)
	{
		return LSM6DS3_ERR_I2C;
	}
	
	return LSM6DS3_SUCCESS;
}


#if defined(LSM6DS3_TILT_FUNCTION)
static int LSM6DS3_enable_tilt(struct i2c_client *client, bool enable)
{
	int res = 0;
	
	if(enable)
	{
		//set ODR to 26 hz		
		res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_26HZ);
		if(LSM6DS3_SUCCESS == res)
		{
			GSE_LOG(" %s set 26hz odr to acc\n", __func__);
		}			
		
		res = LSM6DS3_Enable_Tilt_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_ERR(" LSM6DS3_Enable_Tilt_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		
		res = LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_ERR(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}		
		
		res = LSM6DS3_Enable_Tilt_Func_On_Int(client, LSM6DS3_ACC_GYRO_INT1, true);  //default route to INT1 	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_ERR(" LSM6DS3_Enable_Tilt_Func_On_Int failed!\n");
			return LSM6DS3_ERR_STATUS;
		}	
		enable_irq(stepC_irq);
	}
	else
	{
		res = LSM6DS3_Enable_Tilt_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Enable_Tilt_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		enable_irq(stepC_irq);
	}
	
	return LSM6DS3_SUCCESS;
}
#endif


#if defined(LSM6DS3_STEP_COUNTER)
static int LSM6DS3_enable_pedo(struct i2c_client *client, bool enable)
{
	//u8 databuf[2] = {0};    
	int res = 0;

	if(true == enable)
	{	
		//software reset
		//set ODR to 26 hz		
		res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_26HZ);
		if(LSM6DS3_SUCCESS == res)
		{
			GSE_LOG(" %s set 26hz odr to acc\n", __func__);
		}
		//enable tilt feature and pedometer feature
		res = LSM6DS3_acc_Enable_Pedometer_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Pedometer_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
	    #if 1 //????	
		res = LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}	
		#endif
		res = LSM6DS3_acc_SetFullScale(client,LSM6DS3_ACC_RANGE_4g);//step_counter
	    if(res != LSM6DS3_SUCCESS) 
	    {
		    return res;
	    }
		res = LSM6DS3_Write_PedoThreshold(client, 0x8E);// set threshold to a certain value here
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Write_PedoThreshold failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		res = LSM6DS3_Reset_Pedo_Data(client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
		
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Reset_Pedo_Data failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
	}
	else
	{	 
	    res = LSM6DS3_Reset_Pedo_Data(client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Reset_Pedo_Data failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		res = LSM6DS3_acc_Enable_Pedometer_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed at disable pedo!\n");
			return LSM6DS3_ERR_STATUS;
		}
		//do not turn off the func
	}
	return LSM6DS3_SUCCESS;
}
#endif

static int LSM6DS3_acc_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;

	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return LSM6DS3_SUCCESS;
	}

	res = hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf);
	if(res)		
	{
		GSE_ERR("read lsm6ds3 power ctl register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	GSE_LOG("LSM6DS3_CTRL1_XL:databuf[0] =  %x!\n", databuf[0]);


	if(true == enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;//clear lsm6ds3 gyro ODR bits
		databuf[0] |= LSM6DS3_ACC_ODR_104HZ; //default set 100HZ for LSM6DS3 acc
	}
	else
	{
		// do nothing
		if (false == pedo_enable_status)
		{
		    databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;
			databuf[0] |= LSM6DS3_ACC_ODR_POWER_DOWN;
		}
		else
		{
			databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;//clear lsm6ds3 gyro ODR bits
			databuf[0] |= LSM6DS3_ACC_ODR_26HZ; //default set 26HZ for LSM6DS3 acc
		}
	}
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL;    
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_LOG("LSM6DS3 set power mode: ODR 100hz failed!\n");
		return LSM6DS3_ERR_I2C;
	}	
	else
	{
		GSE_LOG("set LSM6DS3 gyro power mode:ODR 100HZ ok %d!\n", enable);
	}	

	sensor_power = enable;
	
	return LSM6DS3_SUCCESS;    
}


/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	GSE_FUN();     
	
	res = hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf);
	if(res)		
	{
		GSE_ERR("read LSM6DS3_CTRL1_XL err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_CTRL1_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_RANGE_MASK;//clear 
	databuf[0] |= acc_fs;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL; 
	
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	switch(acc_fs)
	{
		case LSM6DS3_ACC_RANGE_2g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_2G;
			break;
		case LSM6DS3_ACC_RANGE_4g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_4G;
			break;
		case LSM6DS3_ACC_RANGE_8g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_8G;
			break;
		case LSM6DS3_ACC_RANGE_16g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_16G;
			break;
		default:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_2G;
			break;
	}

	res = hwmsen_read_byte(client, LSM6DS3_CTRL9_XL, databuf);
	if(res) 
	{
		GSE_ERR("read LSM6DS3_CTRL1_XL err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_CTRL9_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_ENABLE_AXIS_MASK;//clear 
	databuf[0] |= LSM6DS3_ACC_ENABLE_AXIS_X | LSM6DS3_ACC_ENABLE_AXIS_Y| LSM6DS3_ACC_ENABLE_AXIS_Z;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL9_XL; 
	
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
// set the acc sample rate
static int LSM6DS3_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    

	res = hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf);
	if(res) 
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;//clear 
	databuf[0] |= sample_rate;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL; 
	
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write sample rate register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
        GSE_LOG("sample rate(ODR) set to: 0x%x\n", databuf[1]);
	}

	return LSM6DS3_SUCCESS;    
}

#if defined(LSM6DS3_TILT_FUNCTION)
static int LSM6DS3_Enable_Tilt_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    
	
	if(hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LSM6DS3_TILT_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_TILT_EN_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_TILT_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_TILT_EN_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_TAP_CFG; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS; 
}

static int LSM6DS3_Enable_Tilt_Func_On_Int(struct i2c_client *client, LSM6DS3_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
	GSE_FUN();    
	
	if(LSM6DS3_ACC_GYRO_INT1 == tilt_int)
	{
		op_reg = LSM6DS3_MD1_CFG;
	}
	else if(LSM6DS3_ACC_GYRO_INT2 == tilt_int)
	{
		op_reg = LSM6DS3_MD2_CFG;
	}
	
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	
	if(enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_TILT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_TILT_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_TILT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_TILT_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_LOW, LSM6DS3_ACC_GYRO_INT_LATCH);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS; 
}
#endif

#if defined(LSM6DS3_SIGNIFICANT_MOTION)
static int LSM6DS3_Set_SigMotion_Threshold(struct i2c_client *client, u8 SigMotion_Threshold)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    
	
	if(hwmsen_read_byte(client, LSM6DS3_FUNC_CFG_ACCESS, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] = 0x80;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_FUNC_CFG_ACCESS; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	
	databuf[1] = SigMotion_Threshold;
	databuf[0] = LSM6DS3_SM_THS; 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	
	databuf[1] = 0x00;
	databuf[0] = LSM6DS3_FUNC_CFG_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}	
	return LSM6DS3_SUCCESS;    
}

static int LSM6DS3_Enable_SigMotion_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_SIGN_MOT_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_ACC_GYRO_SIGN_MOT_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}


static int LSM6DS3_Int_Ctrl(struct i2c_client *client, LSM6DS3_ACC_GYRO_INT_ACTIVE_t int_act, LSM6DS3_ACC_GYRO_INT_LATCH_CTL_t int_latch)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;

	
	GSE_FUN();    

	//config latch int or no latch
	op_reg = LSM6DS3_TAP_CFG;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	
	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_LATCH_CTL_MASK;//clear 
	databuf[0] |= int_latch;			
	
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	// config high or low active
	op_reg = LSM6DS3_CTRL3_C;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	
	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_ACTIVE_MASK;//clear 
	databuf[0] |= int_act;			
	
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
    //dinggaoshan20160202 -- set all interrupt signals available on INT1 pad enable.
    op_reg = LSM6DS3_CTRL4_C;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
		
	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT2_ON_INT1_MASK;//clear 
	databuf[0] |= LSM6DS3_ACC_GYRO_INT2_ON_INT1_ENABLE;			
	
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write enable int2 on int1 func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	//dinggaoshan20160202 -- set all interrupt signals available on INT1 pad enable end.

	return LSM6DS3_SUCCESS; 
}


static int LSM6DS3_Enable_SigMotion_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
	
	LSM6DS3_ACC_GYRO_FUNC_EN_t func_enable;
	LSM6DS3_ACC_GYRO_SIGN_MOT_t sigm_enable;
	GSE_FUN();    
	
	if(enable)
	{
		func_enable = LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED;
		sigm_enable = LSM6DS3_ACC_GYRO_SIGN_MOT_ENABLED;
		
		res = LSM6DS3_acc_Enable_Func(client, func_enable);	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}	
	}
	else
	{
		//func_enable = LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED;
		sigm_enable = LSM6DS3_ACC_GYRO_SIGN_MOT_DISABLED;
	}		
	
	res = LSM6DS3_Enable_SigMotion_Func(client, sigm_enable);	
	if(res != LSM6DS3_SUCCESS)
	{
		GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
		return LSM6DS3_ERR_STATUS;
	}	
	
	//Config interrupt for significant motion

	op_reg = LSM6DS3_INT1_CTRL;
	
	
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	
	if(enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_SIGN_MOT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_SIGN_MOT_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_SIGN_MOT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_SIGN_MOT_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}	
	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_LOW, LSM6DS3_ACC_GYRO_INT_LATCH);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}	
	return LSM6DS3_SUCCESS; 
}
#endif
#if defined(LSM6DS3_STEP_COUNTER)
static int LSM6DS3_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    
	
	if(hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LSM6DS3_PEDO_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_PEDO_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_PEDO_EN_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_TAP_CFG; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable pedometer func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

    // dinggaoshan20160202---------enable step overflow interrupt.
    if(hwmsen_read_byte(client, LSM6DS3_INT2_CTRL, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	
	if(enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_STEP_OV_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_STEP_OV_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_STEP_OV_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_STEP_OV_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_INT2_CTRL; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable step overflow interrupt register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	// dinggaoshan20160202---------enable step overflow interrupt end.
	return LSM6DS3_SUCCESS;    
}  
static int LSM6DS3_acc_Enable_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_FUNC_EN_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_ACC_GYRO_FUNC_EN_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}
static int LSM6DS3_W_Open_RAM_Page(struct i2c_client *client, LSM6DS3_ACC_GYRO_RAM_PAGE_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    
	
	if(hwmsen_read_byte(client, LSM6DS3_RAM_ACCESS, databuf))
	{
		GSE_ERR("%s read LSM6DS3_RAM_ACCESS register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_RAM_PAGE_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_RAM_ACCESS; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_RAM_ACCESS register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}
static int LSM6DS3_Write_PedoThreshold(struct i2c_client *client, u8 newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();    

	// enable access to the embeded functions.
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED);
	if(LSM6DS3_SUCCESS != res)
	{
		return res;
	}
	// threshold
	if(hwmsen_read_byte(client, LSM6DS3_CONFIG_PEDO_THS_MIN, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
		
	}
	
	databuf[0] &= ~0x9F; 
	databuf[0] |= (newValue & 0x9F);
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CONFIG_PEDO_THS_MIN; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	// debounce time
	databuf[0] = 0x14;
	databuf[1] = 0x5F; 
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	// disable access to the embedded functions reg.
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED);
	if(LSM6DS3_SUCCESS != res)
	{
		GSE_ERR("%s write LSM6DS3_W_Open_RAM_Page failed!\n", __func__);
		return res;
	}
	
	return LSM6DS3_SUCCESS; 
}

static int LSM6DS3_clear_Pedo_Reg(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN(); 
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc LSM6DS3_CTRL10_C data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_PEDO_RST_STEP_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	msleep(50);//sleep for regdata to setto 0.	

	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Reset_Pedo_Data(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue)
{
	int res = LSM6DS3_SUCCESS;
	u16 pedo_data = 0;
	GSE_FUN(); 
	
	LSM6DS3_Get_Pedo_DataReg(client, &pedo_data);
	if (pedo_data > 0)
	{
	    mutex_lock(&current_step_mutex);
		obj_i2c_data->current_step += pedo_data;
		mutex_unlock(&current_step_mutex);
		GSE_LOG("%s: current_step = %d\n", __FUNCTION__,obj_i2c_data->current_step);

		res = LSM6DS3_clear_Pedo_Reg(client, newValue);
	}
	return res;
}
static int LSM6DS3_Get_Pedo_DataReg(struct i2c_client *client, u16 *Value)
{
	u8 databuf[2] = {0}; 
	
	mutex_lock(&lsm6ds3_mutex);
	if(hwmsen_read_block(client, LSM6DS3_STEP_COUNTER_L, databuf, 2))
	{
	    mutex_unlock(&lsm6ds3_mutex);
		GSE_ERR("LSM6DS3 read stepC data  error\n");
		return -2;
	} 
	*Value = (databuf[1]<<8)|databuf[0];	
	mutex_unlock(&lsm6ds3_mutex);
    GSE_LOG("%s: raw_step=%d\n", __FUNCTION__, *Value);

	return LSM6DS3_SUCCESS;
}
#endif

/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadRawData(struct i2c_client *client, char *buf)
{
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data *)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return -EINVAL;

	if (atomic_read(&obj->suspend))
		return -EIO;

	res = LSM6DS3_ReadAccRawData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}
	sprintf(buf, "%04x %04x %04x", obj->data[LSM6DS3_AXIS_X],
	obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z]);

	return 0;
}


/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadAccData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LSM6DS3_ACC_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = LSM6DS3_acc_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on lsm6ds3 error %d!\n", res);
		}
		msleep(20);
	}

    res = LSM6DS3_ReadAccRawData(client, obj->data);
	if(res)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
	#if 1
		obj->data[LSM6DS3_AXIS_X] = (s64)(obj->data[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
		obj->data[LSM6DS3_AXIS_Y] = (s64)(obj->data[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		obj->data[LSM6DS3_AXIS_Z] = (s64)(obj->data[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		
		obj->data[LSM6DS3_AXIS_X] += obj->cali_sw[LSM6DS3_AXIS_X];
		obj->data[LSM6DS3_AXIS_Y] += obj->cali_sw[LSM6DS3_AXIS_Y];
		obj->data[LSM6DS3_AXIS_Z] += obj->cali_sw[LSM6DS3_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->data[LSM6DS3_AXIS_X];
		acc[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->data[LSM6DS3_AXIS_Y];
		acc[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->data[LSM6DS3_AXIS_Z];

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);

		//Out put the mg
		/*
		acc[LSM6DS3_AXIS_X] = acc[LSM6DS3_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LSM6DS3_AXIS_Y] = acc[LSM6DS3_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LSM6DS3_AXIS_Z] = acc[LSM6DS3_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		*/
	#endif


		sprintf(buf, "%04x %04x %04x", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
		//GSE_LOG("obj->data:%04x %04x %04x\n", obj->data[LSM6DS3_AXIS_X], obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z]);
		//GSE_LOG("acc:%04x %04x %04x\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
		
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			GSE_LOG("gsensor data: %s!\n", buf);
			//LSM6DS3_dumpReg(client);
		}
	}
	
	return 0;
}
static int LSM6DS3_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DS3_ACC_AXES_NUM])
{
	//struct lsm6ds3_i2c_data *priv = i2c_get_clientdata(client);      

	int err = 0;
	char databuf[6] = {0};

	if(NULL == client)
	{
		err = -EINVAL;
	}	
	else
	{
		if(hwmsen_read_block(client, LSM6DS3_OUTX_L_XL, databuf, 6))
		{
			GSE_ERR("LSM6DS3 read acc data  error\n");
			return -2;
		}
		else
		{
	
			data[LSM6DS3_AXIS_X] = (s16)((databuf[LSM6DS3_AXIS_X*2+1] << 8) | (databuf[LSM6DS3_AXIS_X*2]));
			data[LSM6DS3_AXIS_Y] = (s16)((databuf[LSM6DS3_AXIS_Y*2+1] << 8) | (databuf[LSM6DS3_AXIS_Y*2]));
			data[LSM6DS3_AXIS_Z] = (s16)((databuf[LSM6DS3_AXIS_Z*2+1] << 8) | (databuf[LSM6DS3_AXIS_Z*2]));	
		}      
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LSM6DS3 Chip");
	return 0;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LSM6DS3_ReadChipInfo(client, strbuf, LSM6DS3_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	int x,y,z;
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
	sscanf(strbuf, "%x %x %x", &x, &y, &z);	
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x,y,z);            
}
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	//char strbuf[LSM6DS3_BUFSIZE];
	s16 data[LSM6DS3_ACC_AXES_NUM] = {0};
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	//LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
	LSM6DS3_ReadAccRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0],data[1],data[2]);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	
	return count;    
}
static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	//int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	LSM6DS3_init_client(obj->client, true);
	LSM6DS3_dumpReg(obj->client);
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->sensitivity, obj->hw->power_id, obj->hw->power_vol);   
	LSM6DS3_dumpReg(obj->client);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{

	struct lsm6ds3_i2c_data *data = i2c_get_clientdata(lsm6ds3_i2c_client);

	if(NULL == lsm6ds3_i2c_client)
	{
		GSE_ERR("this_client IS NULL !\n");
		return -1;
	}


	if(NULL == data)
	{
		printk(KERN_ERR "lsm6ds3_i2c_data is null!!\n");
		return -1;
	}


	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct lsm6ds3_i2c_data *data = i2c_get_clientdata(lsm6ds3_i2c_client);

	if(NULL == lsm6ds3_i2c_client)
	{
		GSE_ERR("this_client IS NULL !\n");
		return -1;
	}

	if(NULL == data)
	{
		printk(KERN_ERR "lsm6ds3_i2c_data is null!!\n");
		return -1;
	}

	

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
/*---add step counter sys for debug-----20160509-dinggaoshan-----------------------*/
#ifdef LSM6DS3_STEP_COUNTER
static ssize_t show_pedometer_enable(struct device_driver *ddri, char *buf)
{
    return sprintf(buf, "%d\n", pedo_enable_status ? 1 : 0);
}

static ssize_t show_pedometer_step(struct device_driver *ddri, char *buf)
{
    u16 steps = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	LSM6DS3_Get_Pedo_DataReg(priv->client, &steps);
	if (steps < 0)
	{
        steps = 0;
	}
	return sprintf(buf, "%d\n", steps);
}
#endif

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensorrawdata,           S_IRUGO, show_sensorrawdata_value,    NULL);

static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,  store_trace_value);
static DRIVER_ATTR(chipinit,      S_IWUSR | S_IRUGO, show_chipinit_value,   store_chipinit_value);

static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(layout,      S_IWUSR | S_IRUGO, show_layout_value, store_layout_value);
/*---add step counter sys for debug-----20160509-dinggaoshan------------*/
#ifdef LSM6DS3_STEP_COUNTER
static DRIVER_ATTR(pedo_enable, S_IRUGO|S_IWUSR, show_pedometer_enable, NULL);
static DRIVER_ATTR(step_counter, S_IRUGO|S_IWUSR, show_pedometer_step, NULL);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute *LSM6DS3_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/	
	&driver_attr_sensorrawdata,   /*dump sensor raw data*/	
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,  
	&driver_attr_chipinit,
	&driver_attr_layout,
	/*---add step counter sys for debug-----20160509-dinggaoshan-----*/
	#ifdef LSM6DS3_STEP_COUNTER
    &driver_attr_pedo_enable,
    &driver_attr_step_counter,
	#endif
};


/*----------------------------------------------------------------------------*/
static int lsm6ds3_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DS3_attr_list)/sizeof(LSM6DS3_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver,  LSM6DS3_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n",  LSM6DS3_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof( LSM6DS3_attr_list)/sizeof( LSM6DS3_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver,  LSM6DS3_attr_list[idx]);
	}
	return err;
}
static int LSM6DS3_Set_RegInc(struct i2c_client *client, bool inc)
{
	u8 databuf[2] = {0};    
	int res = 0;
	//GSE_FUN();     
	
	res = hwmsen_read_byte(client, LSM6DS3_CTRL3_C, databuf);
	if(res) 
	{
		GSE_ERR("read LSM6DS3_CTRL1_XL err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_CTRL3_C register: 0x%x\n", databuf[0]);
	}
	if(inc)
	{
		databuf[0] |= LSM6DS3_CTRL3_C_IFINC;
		
		databuf[1] = databuf[0];
		databuf[0] = LSM6DS3_CTRL3_C; 
		
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("write full scale register err!\n");
			return LSM6DS3_ERR_I2C;
		}
	}

	return LSM6DS3_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_init_client(struct i2c_client *client, bool enable)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();	
    GSE_LOG(" lsm6ds3 addr %x!\n",client->addr);
	res = LSM6DS3_CheckDeviceID(client);
	if(res != LSM6DS3_SUCCESS)
	{
	    GSE_ERR("LSM6DS3_CheckDeviceID err, res=%d",res);
		return res;
	}

	res = LSM6DS3_acc_SetPowerMode(client, enable);
	if(res != LSM6DS3_SUCCESS)
	{
	    GSE_ERR("LSM6DS3_acc_SetPowerMode err, res=%d\n",res);
		return res;
	}
	
	res = LSM6DS3_Set_RegInc(client, true);
	if(res != LSM6DS3_SUCCESS) 
	{
		return res;
	}

	res = LSM6DS3_acc_SetFullScale(client,LSM6DS3_ACC_RANGE_2g);//we have only this choice
	if(res != LSM6DS3_SUCCESS) 
	{
		return res;
	}

	// enable ACC and ODR set to 104Hz
	res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_104HZ);
	if(res != LSM6DS3_SUCCESS ) 
	{
		return res;
	}

	GSE_LOG("LSM6DS3_init_client OK!\n");	
	
#ifdef CONFIG_LSM6DS3_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/

static int lsm6ds3_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
	
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lsm6ds3_enable_nodata(int en)
{
    //int res =0;
	int value = en;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if(value == 1)
	{
		enable_status = true;
	}
	else
	{
		enable_status = false;
	}
	GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
	
	if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
	{
		GSE_LOG("Gsensor device have updated!\n");
	}
	else
	{
		
		err = LSM6DS3_acc_SetPowerMode( priv->client, enable_status);					
	}

    GSE_LOG("mc3xxx_enable_nodata OK!\n");
    return err;
}

static int lsm6ds3_set_delay(u64 ns)
{
    int value =0;
	int err = 0;
	int sample_delay=0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	
    value = (int)ns/1000/1000;

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	
					
	if(value <= 5)
	{
		sample_delay = LSM6DS3_ACC_ODR_208HZ;
	}
	else if(value <= 10)
	{
		sample_delay = LSM6DS3_ACC_ODR_104HZ;
	}
	else
	{
		sample_delay = LSM6DS3_ACC_ODR_52HZ;
	}
				
	err = LSM6DS3_acc_SetSampleRate(priv->client, sample_delay);
	if(err != LSM6DS3_SUCCESS ) 
	{
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50)
	{
		atomic_set(&priv->filter, 0);
	}
	else
	{					
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[LSM6DS3_AXIS_X] = 0;
		priv->fir.sum[LSM6DS3_AXIS_Y] = 0;
		priv->fir.sum[LSM6DS3_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}

    GSE_LOG("lsm6ds3_ACC_set_delay (%d), ODR = 0x%x\n",value, sample_delay);
    return 0;
}

static int lsm6ds3_get_data(int* x ,int* y,int* z, int* status)
{
    char buff[LSM6DS3_BUFSIZE];
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	
//	GSE_LOG("%s (%d),  \n",__FUNCTION__,__LINE__);

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	memset(buff, 0, sizeof(buff));
	LSM6DS3_ReadAccData(priv->client, buff, LSM6DS3_BUFSIZE);
	
	sscanf(buff, "%x %x %x", x, y, z);				
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;				

    return 0;
}

#if defined(LSM6DS3_TILT_FUNCTION)
static int lsm6ds3_tilt_open_report_data(int open)
{
	int res = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	
	if(1 == open)
	{
		res = LSM6DS3_enable_tilt(priv->client, true);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("%s run LSM6DS3_enable_tilt to true failed!\n", __func__);
		}
	}
	else if(0 == open)
	{
		res = LSM6DS3_enable_tilt(priv->client, false);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("%s run LSM6DS3_enable_tilt to false failed!\n", __func__);
		}
	}
	
	return res;
}
#endif


#if defined(LSM6DS3_SIGNIFICANT_MOTION)
static int lsm6ds3_step_c_enable_significant(int en)
{
	int res =0;
	//int value = en;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	
	if(1 == en)
	{
		res = LSM6DS3_Set_SigMotion_Threshold(priv->client, 0x08);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("%s run LSM6DS3_Set_SigMotion_Threshold to fail!\n", __func__);
		}
		res = LSM6DS3_acc_SetSampleRate(priv->client, LSM6DS3_ACC_ODR_26HZ);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("%s run LSM6DS3_Set_SigMotion_Threshold to fail!\n", __func__);
		}
		res = LSM6DS3_Enable_SigMotion_Func_On_Int(priv->client, true); //default route to INT2
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}
		//
		res = LSM6DS3_acc_SetFullScale(priv->client, LSM6DS3_ACC_RANGE_2g);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}
		enable_irq(stepC_irq);
	}
	else if(0 == en)
	{
		res = LSM6DS3_Enable_SigMotion_Func_On_Int(priv->client, false);
		{
			GSE_LOG("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}
		enable_irq(stepC_irq);
	}
	
	return res;
}
#else
static int lsm6ds3_step_c_enable_significant(int en)
{
	return 0;
}
#endif

#if defined(LSM6DS3_STEP_COUNTER)
static int lsm6ds3_step_c_open_report_data(int open)
{
	return LSM6DS3_SUCCESS;
}
static int lsm6ds3_step_c_enable_nodata(int en)
{
	int res =0;
	int value = en;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	GSE_FUN();   

	if(priv == NULL)
	{
		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	if(value == 1)
	{
		res = LSM6DS3_enable_pedo(priv->client, true);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("LSM6DS3_enable_pedo failed at open action!\n");
			return res;
		}
		pedo_enable_status = true;
		//err = LSM6DS3_acc_SetPowerMode( priv->client, enable_status);
	}
	else
	{
		res = LSM6DS3_enable_pedo(priv->client, false);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("LSM6DS3_enable_pedo failed at close action!\n");
			return res;
		}

		pedo_enable_status = false;
	}
	
	GSE_LOG("lsm6ds3_step_c_enable_nodata %d OK!\n", value);
    return err;
}
static int lsm6ds3_step_c_enable_step_detect(int en)
{
	return 0;
}

static int lsm6ds3_step_c_set_delay(u64 delay)
{
	//Null
	return 0;
}
static int lsm6ds3_step_c_get_data(uint32_t *value, int *status)
{
	int err = 0;
	u16 regdata=0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	err = LSM6DS3_Get_Pedo_DataReg(priv->client, &regdata);
	
	GSE_LOG("lsm6ds3_step_c_getdata,current_step=%d, datainchip=%d, report_step=%d\n", 
		priv->current_step, regdata, regdata + priv->current_step);

	wake_lock(&lsm6ds3_stepc_wake_lock); 
	
	*value = regdata + priv->current_step;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	wake_unlock(&lsm6ds3_stepc_wake_lock); 
	//send data to framework before sleep end

	return err;
}
static int lsm6ds3_step_c_get_data_step_d(uint32_t *value, int *status)
{
	return 0;
}
static int lsm6ds3_step_c_get_data_significant(uint32_t *value, int *status)
{
	return 0;
}
#endif

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int lsm6ds3_open(struct inode *inode, struct file *file)
{
	file->private_data = lsm6ds3_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long lsm6ds3_acc_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[LSM6DS3_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			
			LSM6DS3_ReadChipInfo(client, strbuf, LSM6DS3_BUFSIZE);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
	#if 0			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}	
	#endif
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
	#if 0		
			if(copy_to_user(data, &gsensor_offset, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}	
	#endif
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			LSM6DS3_ReadRawData(client, strbuf);
			//LSM6DS3_ReadAccRawData(client, strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
		#if 0
			cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000); //NTC
			cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
			cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
		#else
			cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x);
			cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y);	
			cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z);	
		#endif
				err = LSM6DS3_acc_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = LSM6DS3_acc_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

		     err = LSM6DS3_acc_ReadCalibration(client, cali);
			 if (err)
			 {
			     break;
		     }
					
		#if 0
			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		#else
			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]);
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]);
		#endif
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static struct file_operations lsm6ds3_acc_fops = {
	.owner = THIS_MODULE,
	.open = lsm6ds3_open,
	.release = lsm6ds3_release,
	.unlocked_ioctl = lsm6ds3_acc_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice lsm6ds3_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lsm6ds3_acc_fops,
};

static int lsm6ds3_acc_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);     
	int err = 0;
	
	GSE_FUN();   	
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		//err = LSM6DS3_gyro_SetPowerMode(obj->client, false);
		err = LSM6DS3_acc_SetPowerMode(obj->client, false);
		if(err)
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		
		sensor_power = false;
		
		LSM6DS3_power(obj->hw, 0);

	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_resume(struct i2c_client *client)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -1;
	}

	LSM6DS3_power(obj->hw, 1);
	
	err = LSM6DS3_acc_SetPowerMode(obj->client, enable_status);
	if(err)
	{
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return err ;        
	}
	atomic_set(&obj->suspend, 0);  

	return 0;
}

static int of_get_LSM6DS3_platform_data(struct device *dev)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek, GSE_1-eint");
	if (node) {
		stepC_irq = irq_of_parse_and_map(node, 0);
		if (stepC_irq < 0) {
			GSE_ERR("stepC request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		GSE_ERR("stepC_irq : %d\n", stepC_irq);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/

static void lsm6ds3_eint_work(struct work_struct *work)
{
	//
	u8 databuf[2] = {0}; 
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	
	GSE_FUN();    
		
	if(hwmsen_read_byte(obj->client, LSM6DS3_FUNC_SRC, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		enable_irq(stepC_irq);
		return;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);		
	}
#ifdef LSM6DS3_STEP_COUNTER
    if (LSM6DS3_STEP_OVERFLOW_INT_STATUS & databuf[0])
    {
        mutex_lock(&current_step_mutex);
        obj->current_step += 65535;
	mutex_unlock(&current_step_mutex);
    }
	else if(LSM6DS3_SIGNICANT_MOTION_INT_STATUS & databuf[0])
	{
		//add the action when receive the significant motion
		step_notify(TYPE_SIGNIFICANT);
	}
	else if(LSM6DS3_STEP_DETECT_INT_STATUS & databuf[0])
	{
		//add the action when receive step detection interrupt
		step_notify(TYPE_STEP_DETECTOR);
	}
#endif
#ifdef LSM6DS3_TILT_FUNC
	else if(LSM6DS3_TILT_INT_STATUS & databuf[0])
	{
		//add the action when receive the tilt interrupt
		tilt_notify();
	}
#endif 
	enable_irq(stepC_irq);

	return;
	
}
static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lsm6ds3_i2c_data *obj;
	struct acc_control_path ctl={0};
    struct acc_data_path data={0};
	
	int err = 0;
	GSE_FUN();

	of_get_LSM6DS3_platform_data( &client->dev);
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct lsm6ds3_i2c_data));

	obj->hw = hw; 
	
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
    GSE_LOG("The direction of gsenosor is (%d).\n",obj->hw->direction);
	
	if(err)
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
	lsm6ds3_i2c_client = new_client;	

	if (request_irq(stepC_irq, lsm6ds3_eint_func, IRQF_TRIGGER_NONE,
		LSM6DS3_ACC_DEV_NAME, (void *)client)) {
		GSE_ERR("%s Could not allocate lsm6ds3_int !\n", __func__);
	
		goto exit_kfree;
	}
	irq_set_irq_wake(client->irq, 1);
	// interrupt
	INIT_DELAYED_WORK(&obj->eint_work, lsm6ds3_eint_work);

	err = LSM6DS3_init_client(new_client, false);
	if(err)
	{
	    GSE_ERR("LSM6DS3_init_client err, err= %d\n", err);
		goto exit_init_failed;
	}

#if 1	// gyro ???
	err = misc_register(&lsm6ds3_acc_device);
	if(err)
	{
		GSE_ERR("lsm6ds3_gyro_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
#endif

	err = lsm6ds3_create_attr(&(lsm6ds3_init_info.platform_diver_addr->driver));
	if(err)
	{
		GSE_ERR("lsm6ds3 create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}	

    ctl.open_report_data= lsm6ds3_open_report_data;
    ctl.enable_nodata = lsm6ds3_enable_nodata;
    ctl.set_delay  = lsm6ds3_set_delay;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;

    err = acc_register_control_path(&ctl);
    if(err)
    {
         GSE_ERR("register acc control path err\n");
        goto exit_kfree;
    }

    data.get_data = lsm6ds3_get_data;
    data.vender_div = 1000;
    err = acc_register_data_path(&data);
    if(err)
    {
        GSE_ERR("register acc data path err= %d\n", err);
        goto exit_kfree;
    }

	lsm6ds3_acc_init_flag = 0;
	//dinggaoshan 20160129 send data to framework before sleep(reference n32-kxcnl.c)
	wake_lock_init(&lsm6ds3_stepc_wake_lock, WAKE_LOCK_SUSPEND, "lsm6ds3_stepc_wake_locks");

	LSM6DS3_clear_Pedo_Reg(client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);

	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	//misc_deregister(&lsm6ds3_gyro_device);
	misc_deregister(&lsm6ds3_acc_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	lsm6ds3_acc_init_flag = -1;

	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

	err = lsm6ds3_delete_attr(&(lsm6ds3_init_info.platform_diver_addr->driver));
	if(err)
	{
		GSE_ERR("lsm6ds3_i2c_remove fail: %d\n", err);
	}
#if 1
	err = misc_deregister(&lsm6ds3_acc_device);
	if(err)
	{
		GSE_ERR("misc_deregister lsm6ds3_gyro_device fail: %d\n", err);
	}
#endif
	lsm6ds3_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	
	return 0;
}


#if defined(LSM6DS3_TILT_FUNCTION)
static int lsm6ds3_tilt_get_data(int *value, int *status)
{
	return 0;
}

static int lsm6ds3_tilt_local_init(void)
{
	int res = 0;

	struct tilt_control_path tilt_ctl={0};
	struct tilt_data_path tilt_data={0};


	if(lsm6ds3_acc_init_flag == -1)
	{
	}
	else
	{
		res = lsm6ds3_setup_eint();
		tilt_ctl.open_report_data= lsm6ds3_tilt_open_report_data;	
		res = tilt_register_control_path(&tilt_ctl);

		tilt_data.get_data = lsm6ds3_tilt_get_data;
		res = tilt_register_data_path(&tilt_data);
	}
	return 0;
}
static int lsm6ds3_tilt_local_uninit(void)
{
    return 0;
}
#endif

#if defined(LSM6DS3_STEP_COUNTER)
static irqreturn_t lsm6ds3_eint_func(int irq, void *info)
{
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	GSE_FUN();

	schedule_delayed_work(&priv->eint_work, 1*HZ/1000); //10ms
	disable_irq_nosync(stepC_irq);

	return IRQ_HANDLED;
}

static int lsm6ds3_step_c_local_init(void)
{
	int err = 0;

	//Adapt step counter, tilt and significant detection
	//struct step_c_init_info *step_obj;
	struct step_c_control_path step_ctl={0};
	struct step_c_data_path step_data={0};	

	GSE_FUN();

	if(lsm6ds3_acc_init_flag == -1)
	{
		GSE_LOG("lsm6ds3_step_c_local_init,lsm6ds3_acc_init_flag == -1.\n");
		err = -1;
	}
	else
	{
		step_ctl.open_report_data= lsm6ds3_step_c_open_report_data;
		step_ctl.enable_nodata = lsm6ds3_step_c_enable_nodata;
		step_ctl.enable_step_detect  = lsm6ds3_step_c_enable_step_detect;
		step_ctl.set_delay = lsm6ds3_step_c_set_delay;
		step_ctl.is_report_input_direct = false;
		step_ctl.is_support_batch = false;		
		step_ctl.enable_significant = lsm6ds3_step_c_enable_significant;

		err = step_c_register_control_path(&step_ctl);
		if(err)
		{
			 GSE_ERR("register step counter control path err\n");
			//goto exit_kfree;
		}
	
		step_data.get_data = lsm6ds3_step_c_get_data;
		step_data.get_data_step_d = lsm6ds3_step_c_get_data_step_d;
		step_data.get_data_significant = lsm6ds3_step_c_get_data_significant;
		
		step_data.vender_div = 1;
		err = step_c_register_data_path(&step_data);
		if(err)
		{
			GSE_ERR("register step counter data path err= %d\n", err);
			//goto exit_kfree;
		}
	}
	
	return err;
}
static int lsm6ds3_step_c_local_uninit(void)
{
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/

static int lsm6ds3_local_init(void)
{
	GSE_FUN();
	LSM6DS3_power(hw, 1);
	
	if(i2c_add_driver(&lsm6ds3_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(lsm6ds3_acc_init_flag == -1)
	{
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}
	return 0;
}

static int lsm6ds3_local_uninit(void)
{
    GSE_FUN();    
    LSM6DS3_power(hw, 0);  	
    i2c_del_driver(&lsm6ds3_i2c_driver);
	
    return 0;
}



/*----------------------------------------------------------------------------*/
static int __init lsm6ds3a_init(void)
{
	const char *name = "mediatek,lsm6ds3a";

	GSE_FUN();
	hw = get_accel_dts_func(name, hw);

	if (!hw)
		GSE_ERR("get dts info fail\n");

	acc_driver_add(&lsm6ds3_init_info);	

#if defined(LSM6DS3_STEP_COUNTER)
	step_c_driver_add(&lsm6ds3_step_c_init_info); //step counter
#endif
#if defined(LSM6DS3_TILT_FUNCTION)
	tilt_driver_add(&lsm6ds3_tilt_init_info);
#endif

	return 0;    
}

static void __exit lsm6ds3a_exit(void)
{
	GSE_FUN();

}
/*----------------------------------------------------------------------------*/
module_init(lsm6ds3a_init);
module_exit(lsm6ds3a_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM6DS3 Accelerometer and gyroscope driver");
MODULE_AUTHOR("xj.wang@mediatek.com, darren.han@st.com");
