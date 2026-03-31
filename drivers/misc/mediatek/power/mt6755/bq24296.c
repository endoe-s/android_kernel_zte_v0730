/*
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
//#include <cust_acc.h>
//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
//#include <linux/hwmsen_helper.h>
//#include <linux/xlog.h>

//#include <mt-plat/mt_typedefs.h>
#include <mt-plat/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>

#include "bq24296.h"
//#include "cust_charging.h"
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>

#if defined(CONFIG_MTK_FPGA)
#else
#include <cust_i2c.h>
#endif

//#include "mt_i2c.h"
*/
#if 0
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <linux/xlog.h>

#include <mt-plat/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include "bq24296.h"
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>

#if defined(CONFIG_MTK_FPGA)
#else
#ifdef CONFIG_OF
#else
#include <cust_i2c.h>
#endif
#endif
#endif
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#ifdef CONFIG_OF
#else
#include <mt_i2c.h>
#endif
#include "bq24296.h"

/*----------------------------------------------------------------------------*/
#define CHARGER_TAG                  "[bq24296] "
#define CHARGER_FUN(f)               printk(KERN_ERR BLS_TAG"%s\n", __FUNCTION__)
#define CHARGER_ERR(fmt, args...)    printk(KERN_ERR BLS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CHARGER_LOG(fmt, args...)    printk(KERN_ERR BLS_TAG fmt, ##args)
//#define pr_info pr_notice
//#define pr_info printk
/*----------------------------------------------------------------------------*/



/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#ifdef CONFIG_OF
#else
#define bq24296_SLAVE_ADDR_WRITE   0xD6
#define bq24296_SLAVE_ADDR_READ    0xD7


//#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define bq24296_BUSNUM I2C3
//#else
//#define bq24296_BUSNUM 0
//#endif
#endif

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id bq24296_i2c_id[] = {{"bq24296",0},{}};   
kal_bool chargin_hw_init_done = KAL_FALSE;
static int bq24296_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);



#ifdef CONFIG_OF
static const struct of_device_id bq24296_of_match[] = {
	{ .compatible = "mediatek,SWITHING_CHARGER" },
	{},
};
MODULE_DEVICE_TABLE(of, bq24296_of_match);
#else
static struct i2c_board_info __initdata i2c_bq24296={ I2C_BOARD_INFO("bq24296", 0xD7>>1)};
#endif


static struct i2c_driver bq24296_driver = {
    .driver = {
        .name    = "bq24296",
#ifdef CONFIG_OF 
        .of_match_table = bq24296_of_match,
#endif
    },
    .probe       = bq24296_driver_probe,
    .id_table    = bq24296_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
kal_uint8 bq24296_reg[bq24296_REG_NUM] = {0};

static DEFINE_MUTEX(bq24296_i2c_access);

int g_bq24296_hw_exist=0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24296] 
  *
  *********************************************************/
  #if 0
	int bq24296_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
	{
		char	 cmd_buf = 0;
		char	 readData = 0;
		int 	 ret=0;
	
		//BLS_FUN();
		mutex_lock(&bq24296_i2c_access);	  
	
		cmd_buf = cmd;

	   
	    pr_info("bq24296_read_byte: new_client->addr:0x%x\n",new_client->addr);
		new_client->addr = (new_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
		ret = i2c_master_send(new_client, &cmd_buf, (1<<8 | 1));
		if (ret < 0) 
		{	 
			pr_info("send command error!!\n");
	
			mutex_unlock(&bq24296_i2c_access);
			return ret;
		}
		
		readData = cmd_buf;
		*returnData = readData;
	
		new_client->addr = new_client->addr & I2C_MASK_FLAG;
	
		mutex_unlock(&bq24296_i2c_access);	
		return 0;
	}


#else
int bq24296_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&bq24296_i2c_access);

   // new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
		pr_info("send command error!!\n");
        new_client->ext_flag=0;
        mutex_unlock(&bq24296_i2c_access);
		return ret;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    //new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
	
    mutex_unlock(&bq24296_i2c_access);    
    return 1;
}
#endif

#if 0
int bq24296_write_byte(kal_uint8 cmd, kal_uint8 writeData)
{
    char    write_data[2] = {0};
    int     ret=0;

	//BLS_FUN();
    mutex_lock(&bq24296_i2c_access);
	pr_info("bq24296_write_byte: new_client->addr:0x%x\n",new_client->addr);
	
    write_data[0] = cmd;
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
	   pr_info("send command error!!\n");
       mutex_unlock(&bq24296_i2c_access);
       return ret;
    }
    mutex_unlock(&bq24296_i2c_access);
    return 0;
}

#else
int bq24296_write_byte(kal_uint8 cmd, kal_uint8 writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&bq24296_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;

    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
	
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
		pr_info("send command error!!\n");
        new_client->ext_flag=0;    
        mutex_unlock(&bq24296_i2c_access);
       return ret;
    }

    new_client->ext_flag=0;    
    mutex_unlock(&bq24296_i2c_access);
    return 1;
}
#endif
/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
kal_uint32 bq24296_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24296_reg = 0;
    int ret = 0;

    pr_info("--------------------------------------------------\n");

    ret = bq24296_read_byte(RegNum, &bq24296_reg);

    pr_info("[bq24296_read_interface] Reg[%x]=0x%x\n", RegNum, bq24296_reg);
    
    bq24296_reg &= (MASK << SHIFT);
    *val = (bq24296_reg >> SHIFT);    
	
    pr_info("[bq24296_read_interface] val=0x%x\n", *val);
	
    return ret;
}

kal_uint32 bq24296_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24296_reg = 0;
    int ret = 0;

    pr_info("--------------------------------------------------\n");

    ret = bq24296_read_byte(RegNum, &bq24296_reg);
	if(ret < 0){	
		pr_info("[bq24296_config_interface] read Reg[%x]=0x%x err\n", RegNum, bq24296_reg);
	}
    
	pr_info("[bq24296_config_interface] Reg[%x]=0x%x\n", RegNum, bq24296_reg);
	
    bq24296_reg &= ~(MASK << SHIFT);
    bq24296_reg |= (val << SHIFT);


    ret = bq24296_write_byte(RegNum, bq24296_reg);
	if(ret < 0){
		pr_info("[bq24296_config_interface] write Reg[%x]=0x%x err\n", RegNum, bq24296_reg);
	}
    pr_info("[bq24296_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24296_reg);


    // Check
    //ret = bq24296_read_byte(RegNum, &bq24296_reg);
	//if(ret < 0){
	//	pr_info("[bq24296_config_interface] read Reg[%x]=0x%x err\n", RegNum, bq24296_reg);
	//}
   // pr_info("[bq24296_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24296_reg);

    return ret;
}

//write one register directly
kal_uint32 bq24296_reg_config_interface (kal_uint8 RegNum, kal_uint8 val)
{   
    kal_uint32 ret = 0;
    
    ret = bq24296_write_byte(RegNum, val);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//CON0----------------------------------------------------

void bq24296_set_en_hiz(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_EN_HIZ_MASK),
                                    (kal_uint8)(CON0_EN_HIZ_SHIFT)
                                    );
}

void bq24296_set_vindpm(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_VINDPM_MASK),
                                    (kal_uint8)(CON0_VINDPM_SHIFT)
                                    );
}

void bq24296_set_iinlim(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_IINLIM_MASK),
                                    (kal_uint8)(CON0_IINLIM_SHIFT)
                                    );
}

//CON1----------------------------------------------------

void bq24296_set_reg_rst(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_REG_RST_MASK),
                                    (kal_uint8)(CON1_REG_RST_SHIFT)
                                    );
}

void bq24296_set_wdt_rst(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_WDT_RST_MASK),
                                    (kal_uint8)(CON1_WDT_RST_SHIFT)
                                    );
}

void bq24296_set_otg_config(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_OTG_CONFIG_MASK),
                                    (kal_uint8)(CON1_OTG_CONFIG_SHIFT)
                                    );
}

void bq24296_set_chg_config(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_CHG_CONFIG_MASK),
                                    (kal_uint8)(CON1_CHG_CONFIG_SHIFT)
                                    );
}

void bq24296_set_sys_min(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_SYS_MIN_MASK),
                                    (kal_uint8)(CON1_SYS_MIN_SHIFT)
                                    );
}

void bq24296_set_boost_lim(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_BOOST_LIM_MASK),
                                    (kal_uint8)(CON1_BOOST_LIM_SHIFT)
                                    );
}

//CON2----------------------------------------------------

void bq24296_set_ichg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_ICHG_MASK),
                                    (kal_uint8)(CON2_ICHG_SHIFT)
                                    );
}

void bq24296_set_bcold(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_BCOLD_MASK),
                                    (kal_uint8)(CON2_BCOLD_SHIFT)
                                    );
}
void bq24296_set_force_20pct(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_FORCE_20PCT_MASK),
                                    (kal_uint8)(CON2_FORCE_20PCT_SHIFT)
                                    );
}

//CON3----------------------------------------------------

void bq24296_set_iprechg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON3), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON3_IPRECHG_MASK),
                                    (kal_uint8)(CON3_IPRECHG_SHIFT)
                                    );
}

void bq24296_set_iterm(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON3), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON3_ITERM_MASK),
                                    (kal_uint8)(CON3_ITERM_SHIFT)
                                    );
}

//CON4----------------------------------------------------

void bq24296_set_vreg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_VREG_MASK),
                                    (kal_uint8)(CON4_VREG_SHIFT)
                                    );
}

void bq24296_set_batlowv(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_BATLOWV_MASK),
                                    (kal_uint8)(CON4_BATLOWV_SHIFT)
                                    );
}

void bq24296_set_vrechg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_VRECHG_MASK),
                                    (kal_uint8)(CON4_VRECHG_SHIFT)
                                    );
}

//CON5----------------------------------------------------

void bq24296_set_en_term(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_EN_TERM_MASK),
                                    (kal_uint8)(CON5_EN_TERM_SHIFT)
                                    );
}

void bq24296_batfet_rst_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_BATFET_RST_EN_MASK),
                                    (kal_uint8)(CON5_BATFET_RST_EN_SHIFT)
                                    );
}


void bq24296_set_watchdog(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_WATCHDOG_MASK),
                                    (kal_uint8)(CON5_WATCHDOG_SHIFT)
                                    );
}

void bq24296_set_en_timer(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_EN_TIMER_MASK),
                                    (kal_uint8)(CON5_EN_TIMER_SHIFT)
                                    );
}

void bq24296_set_chg_timer(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_CHG_TIMER_MASK),
                                    (kal_uint8)(CON5_CHG_TIMER_SHIFT)
                                    );
}

//CON6----------------------------------------------------

void bq24296_set_treg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_TREG_MASK),
                                    (kal_uint8)(CON6_TREG_SHIFT)
                                    );
}

void bq24296_set_boostv(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_BOOSTV_MASK),
                                    (kal_uint8)(CON6_BOOSTV_SHIFT)
                                    );
}

void bq24296_set_bhot(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_BHOT_MASK),
                                    (kal_uint8)(CON6_BHOT_SHIFT)
                                    );
}

//CON7----------------------------------------------------

void bq24296_set_tmr2x_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON7), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON7_TMR2X_EN_MASK),
                                    (kal_uint8)(CON7_TMR2X_EN_SHIFT)
                                    );
}

void bq24296_set_batfet_disable(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON7), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON7_BATFET_Disable_MASK),
                                    (kal_uint8)(CON7_BATFET_Disable_SHIFT)
                                    );
}

void bq24296_set_int_mask(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=bq24296_config_interface(   (kal_uint8)(bq24296_CON7), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON7_INT_MASK_MASK),
                                    (kal_uint8)(CON7_INT_MASK_SHIFT)
                                    );
}

//CON8----------------------------------------------------

kal_uint32 bq24296_get_system_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24296_read_interface(     (kal_uint8)(bq24296_CON8), 
                                    (&val),
                                    (kal_uint8)(0xFF),
                                    (kal_uint8)(0x0)
                                    );
    return val;
}

kal_uint32 bq24296_get_vbus_stat(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24296_read_interface(     (kal_uint8)(bq24296_CON8), 
                                    (&val),
                                    (kal_uint8)(CON8_VBUS_STAT_MASK),
                                    (kal_uint8)(CON8_VBUS_STAT_SHIFT)
                                    );
    return val;
}

kal_uint32 bq24296_get_chrg_stat(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24296_read_interface(     (kal_uint8)(bq24296_CON8), 
                                    (&val),
                                    (kal_uint8)(CON8_CHRG_STAT_MASK),
                                    (kal_uint8)(CON8_CHRG_STAT_SHIFT)
                                    );
    return val;
}

kal_uint32 bq24296_get_vsys_stat(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24296_read_interface(     (kal_uint8)(bq24296_CON8), 
                                    (&val),
                                    (kal_uint8)(CON8_VSYS_STAT_MASK),
                                    (kal_uint8)(CON8_VSYS_STAT_SHIFT)
                                    );
    return val;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void bq24296_hw_component_detect(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=bq24296_read_interface(0x0A, &val, 0xFF, 0x0);
    
    if(val == 0)
        g_bq24296_hw_exist=0;
    else
        g_bq24296_hw_exist=1;

    printk("[bq24296_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n", 
        g_bq24296_hw_exist, val);
}

int is_bq24296_exist(void)
{
    printk("[is_bq24296_exist] g_bq24296_hw_exist=%d\n", g_bq24296_hw_exist);
    
    return g_bq24296_hw_exist;
}

int  bq24296_is_dpm_mode(void)
{
	if(bq24296_reg[bq24296_CON8]&(1<<CON8_DPM_STAT_SHIFT))
	{
		return 1;
	}
	else
		return 0;
}
void bq24296_dump_register(void)
{
    int i=0;
    printk("[bq24296] ");
    for (i=0;i<bq24296_REG_NUM;i++)
    {
        bq24296_read_byte(i, &bq24296_reg[i]);
        printk("[0x%x]=0x%x ", i, bq24296_reg[i]);        
    }
    printk("\n");
}

static int bq24296_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 

    pr_notice("[bq24296_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;	
		pr_notice("[bq24296_driver_probe] \n");
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    //---------------------
    bq24296_hw_component_detect();
    bq24296_dump_register();
    chargin_hw_init_done = KAL_TRUE;

    return 0;                                                                                       

exit:
    return err;

}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
kal_uint8 g_reg_value_bq24296=0;
static ssize_t show_bq24296_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    pr_notice("[show_bq24296_access] 0x%x\n", g_reg_value_bq24296);
    return sprintf(buf, "%u\n", g_reg_value_bq24296);
}
static ssize_t store_bq24296_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    pr_notice("[store_bq24296_access] \n");
    
    if(buf != NULL && size != 0)
    {
        pr_notice("[store_bq24296_access] buf is %s and size is %zu \n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(size > 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            pr_notice("[store_bq24296_access] write bq24296 reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=bq24296_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=bq24296_read_interface(reg_address, &g_reg_value_bq24296, 0xFF, 0x0);
            pr_notice("[store_bq24296_access] read bq24296 reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_bq24296);
            pr_notice("[store_bq24296_access] Please use \"cat bq24296_access\" to get value\r\n");
        }        
    }    
    return size;
}
static DEVICE_ATTR(bq24296_access, 0664, show_bq24296_access, store_bq24296_access); //664

static void bq24296_shutdown(struct platform_device *dev)
{
    bq24296_set_otg_config(0x00);  
}

static int bq24296_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    pr_notice("******** bq24296_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24296_access);
    
    return 0;
}

struct platform_device bq24296_user_space_device = {
    .name   = "bq24296-user",
    .id     = -1,
};

static struct platform_driver bq24296_user_space_driver = {
    .probe      = bq24296_user_space_probe,
    .shutdown   = bq24296_shutdown,
    .driver     = {
        .name = "bq24296-user",
    },
};

static int __init bq24296_subsys_init(void)
{    
    int ret=0;
    
	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "[bq24296_init] init start with i2c DTS");
#else
    pr_notice("[bq24296_init] init start. ch=%d\n", bq24296_BUSNUM);
	
	i2c_register_board_info(bq24296_BUSNUM, &i2c_bq24296, 1);
#endif
    if(i2c_add_driver(&bq24296_driver)!=0)
    {
        pr_notice("[bq24296_init] failed to register bq24296 i2c driver.\n");
    }
    else
    {
        pr_notice("[bq24296_init] Success to register bq24296 i2c driver.\n");
    }

    // bq24296 user space access interface
    ret = platform_device_register(&bq24296_user_space_device);
    if (ret) {
        pr_notice("****[bq24296_init] Unable to device register(%d)\n", ret);
        return ret;
    }
    ret = platform_driver_register(&bq24296_user_space_driver);
    if (ret) {
        pr_notice("****[bq24296_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    
    return 0;        
}

static void __exit bq24296_exit(void)
{
    i2c_del_driver(&bq24296_driver);
}

//module_init(bq24296_init);
//module_exit(bq24296_exit);
subsys_initcall(bq24296_subsys_init);
module_exit(bq24296_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24296 Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");
