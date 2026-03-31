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

/*
 * DW9814AF voice coil motor driver
 *
 *
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9767AF_DRV"
//#define AF_I2C_SLAVE_ADDR        0x18

//#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static u8 is_initial = 0;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;


static int i2c_read(u8 a_u2Addr , u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)(a_u2Addr)};
	
	 //g_pstAF_I2Cclient->addr = 0x0c;
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 2) {
	    LOG_INF(" I2C write failed!! \n");
	    return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
	    LOG_INF(" I2C read failed!! \n");
	    return -1;
	}
   
    return 0;
}


static u8 read_data(u8 addr)
{
	u8 get_byte = 0;
	i2c_read(addr, &get_byte);
	//DW9767AFDB("[DW9767AF]  get_byte %d\n", get_byte);
	return get_byte;
}

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{

	*a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04) & 0xff);
	LOG_INF("I2C s4AF_ReadReg failed!!\n");
	return 0;
}


static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	//g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	//g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
	
    //g_pstAF_I2Cclient->addr = 0x0c;
	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
	LOG_INF("LIHY  puSendCmd[012]: %d,  %d,  %d\n", puSendCmd[0], puSendCmd[1], puSendCmd[2]);
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}
static void set_work_mode(void)
{
	int ret = 0;
	char puSendCmd1[2] = {0x02, 0x01};
	char puSendCmd2[2] = {0x02, 0x00};
	char puSendCmd3[2] = {0x1A, 0x01};
	char puSendCmd4[2] = {0x1B, 0x00};
	char puSendCmd5[2] = {0x06, 0x0A};
	char puSendCmd6[2] = {0x07, 0x01};
	char puSendCmd7[2] = {0x08, 0x43};
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);	
	msleep(2);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd5, 2);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd6, 2);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd7, 2);
}
static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}
	if (is_initial == 0)
	{
		set_work_mode();
		is_initial = 1;
	}
	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4AF_ReadReg(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

    LOG_INF("lihy a_u4Position Pos %ld\n", a_u4Position);

	LOG_INF("lihy g_u4CurrPosition Pos %ld\n", g_u4CurrPosition);
	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */


	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9767AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9767AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		char puSendCmd[2];

		s4AF_WriteReg(200);
		msleep(40);
		s4AF_WriteReg(80);
		msleep(30);
		s4AF_WriteReg(20);
		msleep(20);
		puSendCmd[0] = (char)(0x02);
		puSendCmd[1] = (char)(0x01);
		i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
		LOG_INF("Wait\n");
		
		msleep(20);
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

    is_initial = 0;
	LOG_INF("End\n");

	return 0;
}

void DW9767AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}

void DW9767AF_SetI2Cclent_first(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
int DW9767AF_Release_First(void)
{
    int ret = 0;
	char puSendCmd1[2] = {0x02, 0x00};
	char puSendCmd2[2] = {0x02, 0x01};

	LOG_INF("Start\n");
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);
	msleep(30);
	ret = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	LOG_INF("end\n");
	return 0;
}