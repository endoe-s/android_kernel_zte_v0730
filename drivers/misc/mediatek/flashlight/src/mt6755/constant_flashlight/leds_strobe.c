/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
//weiguohua 
#if !defined(CONFIG_MTK_LEGACY)
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#else
#include <mt-plat/mt_gpio.h>
#endif


#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <asm-generic/gpio.h>
//weiguohua
/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
//#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define PK_DBG_FUNC(fmt, arg...)    pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;


/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */




static u8 gIsTorch[18] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//static int glm3646LedmaxDuty[17] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 13};
//static int glm3646Led1Duty[17] = { 3,  7,   11,  15,  19,  23,   27,  31,  35,   39,  43,  47,  51,   51,  55,   55,  55};
static u8 glm3646LedmaxDuty[17] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x06, 0x07, 
                                   0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0d, 0x0e, 0x0e};
static u8 glm3646Led1Duty[17] =   {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x25, 0x2c, 0x32, 
	                               0x38, 0x3e, 0x44, 0x4a, 0x5d, 0x64, 0x64, 0x6b, 0x6b};
/* current(mA) 50,94,141,188,281,375,469,563,656,750,844,938,1031,1125,1220,1313,1406,1500 */



/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *RT4505_i2c_client;




struct RT4505_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct RT4505_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct RT4505_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int RT4505_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

static int RT4505_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}



/* ========================= */


#define GPIO_FLASH_EN_PIN  8
int FL_Init(void);
static int RT4505_chip_init(struct RT4505_chip_data *chip)
{
//weiguohua 

	gpio_direction_output(GPIO_FLASH_EN_PIN, 1);
	gpio_set_value(GPIO_FLASH_EN_PIN, 1);
		
	FL_Init();
	PK_DBG(" zte RT4505_chip_init line=%d\n", __LINE__);
//weiguohua	
	return 0;
}

static int RT4505_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct RT4505_chip_data *chip;
	struct RT4505_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("RT4505_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("RT4505 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct RT4505_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("RT4505 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct RT4505_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;


	RT4505_i2c_client = client;

	if (RT4505_chip_init(chip) < 0)
		goto err_chip_init;
	PK_DBG("RT4505 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("RT4505 probe is failed\n");
	return -ENODEV;
}

static int RT4505_remove(struct i2c_client *client)
{
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define RT4505_NAME "leds-RT4505"
static const struct i2c_device_id RT4505_id[] = {
	{RT4505_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id RT4505_of_match[] = {
	//{.compatible = "mediatek,strobe_main"},
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver RT4505_i2c_driver = {
	.driver = {
		   .name = RT4505_NAME,
#ifdef CONFIG_OF
		   .of_match_table = RT4505_of_match,
#endif
		   },
	.probe = RT4505_probe,
	.remove = RT4505_remove,
	.id_table = RT4505_id,
};

static int __init RT4505_init(void)
{
	PK_DBG("RT4505_init\n");
	return i2c_add_driver(&RT4505_i2c_driver);
}

static void __exit RT4505_exit(void)
{
	i2c_del_driver(&RT4505_i2c_driver);
}


module_init(RT4505_init);
module_exit(RT4505_exit);

MODULE_DESCRIPTION("Flash driver for RT4505");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;

	val = RT4505_read_reg(RT4505_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{
	int buf[2];

#ifdef CONFIG_P650A31_FLASHLIHGT_LM3646
	if (gIsTorch[gDuty] == 1)
	{
	    buf[0] = 0x01;
		buf[1] = 0xE6;
	    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);  // set time-out  in flash mode
	}
	else
	{
		buf[0] = 0x04;
	    buf[1] = 0x47;
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);  // set time-out  in flash mode
		buf[0] = 0x01;
		buf[1] = 0xE7;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_Enable line=%d\n", __LINE__);
	}
#else
	buf[0] = 10;
	if (gIsTorch[gDuty] == 1)
		buf[1] = 0x71;
	else
		buf[1] = 0x77;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_Enable line=%d\n", __LINE__);
#endif
	return 0;
}



int FL_Disable(void)
{
	int buf[2];

	#ifdef CONFIG_P650A31_FLASHLIHGT_LM3646
	buf[0] = 01;
	buf[1] = 00;
    #else
	buf[0] = 10;
	buf[1] = 0x70;
	#endif
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	int buf[2];

	if (duty >= 17)
		duty = 16;
	if (duty < 0)
		duty = 0;
	if((duty == 8)||(duty == 9)||(duty == 10))
	{
		duty = 13;  //lihy for f31
	}
	gDuty = duty;
	#ifdef CONFIG_P650A31_FLASHLIHGT_LM3646
	buf[0] = 0x05;
	buf[1] = 0x50 | glm3646LedmaxDuty[gDuty];  // toal current
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	buf[0] = 0x06;
	buf[1] = glm3646Led1Duty[gDuty];   //led 1 current
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	buf[0] = 0x07;
	buf[1] = 0x7a;  //torch current
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	#else
	buf[0] = 9;
	buf[1] = gLedDuty[duty];
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	#endif
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	return 0;
}




int FL_Init(void)
{
	int buf[2];

    #ifdef CONFIG_P650A31_FLASHLIHGT_LM3646
	buf[0]= 0x09;
	buf[1]= 0x70;
	#else
	buf[0] = 0;
	buf[1] = 0x80;
	#endif
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    #ifdef CONFIG_P650A31_FLASHLIHGT_LM3646
	#else
	buf[0] = 8;
	buf[1] = 0x7;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	#endif

	PK_DBG(" FL_Init line=%d\n", __LINE__);
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;

int FL_ftm_Enable(void)
{
	int buf[2];
    
	    buf[0] = 0x07;	//led 1 torch 
		buf[1] = 0x7a;
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
		buf[0] = 0x01;
		buf[1] = 0xE6;
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	
		mdelay(1000);
	    FL_Disable();
	
	    mdelay(1000);
	    buf[0] = 0x07;	//led 1 torch  disable
		buf[1] = 0x00;
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
		buf[0] = 0x01;
		buf[1] = 0xE6;
		RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
		mdelay(1000);
	    FL_Disable();
	
	return 0;
}

	
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} 
		else 
	    {
		       if (arg == 3)
		       {
					FL_ftm_Enable();
			   }
			   else if (arg == 2)
			   {
		        	//hrtimer_cancel(&g_ftm_timeOutTimer);
					FL_Disable();
			   }
			   else
			   {
			   		hrtimer_cancel(&g_timeOutTimer);
				    FL_Disable();
			   }
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
		//ftm_timerInit(); //lihy ftm timer
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}

//lihy
static kal_bool eng_flash_onoff_flag = KAL_TRUE;
int FL_ENG_Enable(void)
{
	int buf[2];
	
	if (eng_flash_onoff_flag == KAL_FALSE)
	{
		return 0;
	}

    buf[0] = 0x07;	//led 1 torch 
	buf[1] = 0x7a;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	buf[0] = 0x01;
	buf[1] = 0xE6;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);

	mdelay(1000);
	if (eng_flash_onoff_flag == KAL_FALSE)
	{
		return 0;
	}
	
    FL_Disable();
	
    mdelay(1000);
	
    buf[0] = 0x07;	//led 1 torch  disable
	buf[1] = 0x00;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	buf[0] = 0x01;
	buf[1] = 0xE6;
	if (eng_flash_onoff_flag == KAL_FALSE)
	{
		return 0;
	}
	
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
	mdelay(1000);
    FL_Disable();
	//mdelay(1000);
    return 0;
}

void eng_flash_enable(void)
{
    eng_flash_onoff_flag = KAL_TRUE;
	FL_Init();
	FL_ENG_Enable();
}
void eng_flash_disable(void)
{
    eng_flash_onoff_flag = KAL_FALSE;
    FL_Disable();
}
//end
FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
