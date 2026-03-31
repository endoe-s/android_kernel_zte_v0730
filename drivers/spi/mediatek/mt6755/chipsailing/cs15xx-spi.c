/*
 *
 * synchronous driver interface to SPI finger print sensor
 *
 *	Copyright (c) 2015  ChipSailing Technology.
 *	All rights reserved.
***********************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include "../mt_spi.h"
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/rtpm_prio.h>
#include <linux/wakelock.h>

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/of_irq.h>
#include <linux/input.h>
#include "cs15xx-spi.h"
#include "cs_navigation.h"
#include "../fingerprint_core.h"
#include <linux/proc_fs.h>
#include <linux/workqueue.h>

#if AUTOGAIN
#include "autogain.h"
#endif

#define LONGTOUCHTIME   70         //700ms   ;  Seconds  100=1S <----长按时间可以修改这个宏
#define NAVDECTIME      30         //300ms   ;  Seconds  100=1S <----NAV超时时间可以修改这个宏
#define DOUBELDECTIME   40         //300ms   ;  Seconds  100=1S <----双击间隙超时时间可以修改这个宏

#define WAITDOWNTIME    75         //75ms   ;手指按下时间间隔

static int  KEY_MODE_FLAG = 1;     //0:off 1:on   ;default 1
static int  NAV_MODE_FLAG = 1;     //0:off 1:on   ;default 1
static int  SPI_IOC_READER_FLAG;   //0:Exit 1:Enter

#define DRIVER_VERSION   "V1.9.4_20161106"

#define DRIVER_NAME      "cs_spi"



//SPI parameter definition
#define SPI_CLOCK_SPEED    7000000
#define CS_FP_MAJOR		   155	    /* assigned 153*/
#define N_SPI_MINORS	   33//32	/* ... up to 256 */



//image parameter definition
#define READ_IMAGE_SIZE 10*1024     //10K Bytes 
#define IMAGE_SIZE      112*88
#define IMAGE_DUMMY     4

//global parameter
static u16 g_cs_id;
static u32 g_cs_bufsiz = 4096;
static int g_irg_flag = 0;
static volatile int g_isnav=0 ;
static volatile int g_longtouch =0;
static int  host_deep_sleep =-1;   //1:主机已经进入深度休眠   0：主机未进入深度休眠   -1：默认值
//nav image parameter definition
static int g_nav_reg_val[10] = {0}; //每一位代表0x48寄存器记录下来的8个区域的数值 ，如：值位0xff则表示0~7的位置全有数据1
static int g_nav_valid_num = 0;
static volatile u8  g_finger_status =0;  //g_finger_status=1：按下状态  g_finger_status=0：抬起	 
//static int g_timer_on = 0;             //0:cs_timer is off        1:cs_timer is on
static int g_timer_kick_calc = 0;        //g_timer_kick_calc统计手指kick数量
//static int g_work_func_finish;         //中断线程开始与结束标志

//If the navigation calculation is completed, we will use the thread switch chip mode.   20161006
static  struct workqueue_struct *force_mode_wq;
void force_mode_func(struct work_struct *work);
//struct work_struct force_mode_work;
struct delayed_work force_mode_work;  

#if AUTOGAIN
int g_new_gain_value = DEFAULT_GAIN_VALUE;
int g_new_base_value = DEFAULT_BASE_VALUE;
#endif

//global struct parameter
static struct task_struct   *cs_irq_thread = NULL;
static struct fasync_struct *async;
static struct input_dev     *cs15xx_inputdev = NULL;
static struct cs_fp_data    *cs15xx;
static struct timer_list	cs_timer;
static struct timer_list	cs_timer_finger_detec;
static struct timer_list	cs_timer_kick;
static struct timer_list	cs_timer_long_louch;
//static struct wake_lock     cs_lock; //for wakelock

static int cs_create_inputdev(void);
static int  cs_timer_init(void);
static void cs_force_mode(int mod);
void cs_report_key(int key_type,int key_status);

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static DEFINE_MUTEX(sfr_write_lock);
static DEFINE_MUTEX(cs_spi_lock);

//extern  int cs_finger_navigation(int pos[],int len);

struct cs_fp_data
{
    dev_t	             devt;
    spinlock_t           spi_lock;
    struct spi_device	 *spi;
    unsigned int         irq;
    int 								 clk_enabled;
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend early_fp;
#endif
    unsigned int         gpio_irq;
    unsigned int         gpio_reset;
    struct list_head	 device_entry;
    struct  mutex	     buf_lock;
    unsigned		     users;
    u8	                *buffer;    /* buffer is NULL unless this device is open (users > 0) */
};

static struct mt_chip_conf spi_conf =
{
    .setuptime = 7,//15,cs
    .holdtime = 7,//15, cs
    .high_time = 25,//6 sck
    .low_time =  25,//6  sck
    .cs_idletime = 3,//20,
    //.ulthgh_thrsh = 0,
    .cpol = 0,
    .cpha = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = DMA_TRANSFER,
    //.com_mod = FIFO_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};

static void cs_fp_complete(void *arg)
{
    complete(arg);
}

static ssize_t cs_fp_sync(struct spi_message *message)
{
    DECLARE_COMPLETION_ONSTACK(done);
    int status;

    message->complete = cs_fp_complete;
    message->context = &done;

    spin_lock_irq(&cs15xx->spi_lock);
    if (cs15xx->spi == NULL)
        status = -ESHUTDOWN;
    else
        status = spi_async(cs15xx->spi, message);
    spin_unlock_irq(&cs15xx->spi_lock);

    if (status == 0)
    {
        wait_for_completion(&done);
        status = message->status;
        if (status == 0)
            status = message->actual_length;
    }
    return status;
}

static inline ssize_t cs_fp_wrbyte(u8 *buff,int length)
{
    struct spi_message m;

    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .bits_per_word = 8,
        .tx_buf  = buff,
        .len	 = length,
    };
    FUNC_ENTRY();
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    FUNC_EXIT();
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_fp_sync( &m) return = %zx\n",cs_fp_sync( &m));
    return cs_fp_sync( &m);
}

static int cs_read_sfr( u8 addr,u8 *data,u8 rd_count)
{
    int ret ;
    struct spi_message m;

    u8 rx[4] = {0};
    u8 tx[4] = {0};                        //only rd 2bytes
    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .bits_per_word = 8,
        .tx_buf =tx,
        .rx_buf =rx,
        .len = 2+rd_count,
    };
    //FUNC_ENTRY();
    tx[0] = 0xDD;
    tx[1] = addr;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret= cs_fp_sync(&m);
    if(ret < 0)
        return ret;
    for(ret = 0; ret < rd_count; ret++)
    {
        data[ret] = rx[2+ret];
    }

    //FUNC_EXIT();
    return data[0];
}

static inline void cs_write_sfr( u8 addr, u8 value )
{
	

    
    u8 send_buf[3]= {0};
    //u8 test[6] = {0};
    //cs_fp_wrbyte(send_buf,3);
    struct spi_message m;

    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .bits_per_word = 8,
        .tx_buf  = send_buf,
        .len	 = 3,
    };
    mutex_lock(&sfr_write_lock);
    send_buf[0] = 0xCC;
    send_buf[1] = addr;
    send_buf[2] = value;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    cs_fp_sync(&m);
	mutex_unlock(&sfr_write_lock);
    //cs_read_sfr(addr,test,1);
   
}

static void cs_write_sram(u16 addr,u16 value )
{
    u8 send_buf[5]= {0};
    //cs_fp_wrbyte(send_buf,5);
    struct spi_message m;
    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .bits_per_word = 8,
        .tx_buf  = send_buf,
        .len	 = 5,
    };
    //FUNC_ENTRY();
    send_buf[0] = 0xAA;
    send_buf[1] = (addr&0xff00)>>8;    //addr_h
    send_buf[2] = addr&0x00ff;         //addr_l
    send_buf[3] = value&0x00ff;
    send_buf[4] = (value&0xff00)>>8;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    cs_fp_sync(&m);
    //FUNC_EXIT();
}

static u16 cs_read_sram(u16 addr,u8 *data,u8 rd_count)
{
    u16 ret ;
    struct spi_message m;
    u8 rx[6] = {0};
    u8 tx[6] = {0};                         //only rd 2bytes

    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .tx_buf =tx,
        .rx_buf =rx,
        .len = 3+rd_count+1,                 //first nop
    };

    tx[0] = 0xBB;
    tx[1] = addr>>8;
    tx[2] = addr;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret= cs_fp_sync(&m);
    if(ret < 0)
        return ret;
    for (ret = 0; ret < rd_count; ret++)
    {
        data[ret] = rx[4+ret];                 //first nop
    }

    return  (data[1]&0x00ff)<<8|data[0];
}


static void cs_write_sram_bit(u16 addr,u16 bit,bool vu)
{
    u8 buff[2] = {0};
    u16   temp = 0;

    temp = cs_read_sram(addr,buff,2);

    if(vu)
    {
        temp = temp | (1<<bit);
    }
    else
    {
        temp = temp & (~(1<<bit));
    }

    cs_write_sram(addr ,temp);
}


static inline ssize_t cs_fp_rd_id(u8 *buffer)
{
    int ret;
    ret = cs_read_sfr(0x3E,buffer,2);
    return ret;
}

static void cs_fp_reset(unsigned int delay_ms)
{
    FUNC_ENTRY();
	/*
	gpio_set_value(GPIO_FINGER_RST, 0);
	mdelay(delay_ms);
	gpio_set_value(GPIO_FINGER_RST, 1);
	*/
	pin_select(rst_low);
	msleep(5);
	pin_select(rst_high);
	mdelay(5);

    FUNC_EXIT();
}

static void cs_dump_sensor_reg(void)
{
    u16 data;
    u8 rx[6] = {0};
    u8 index = 0;
    u16 reg_addr = 0xFC40; //sensor addr0_end

    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]dump reg start=========================\n ");
    for(index = 0; index < 24; index++)
    {
        data=cs_read_sram(reg_addr,rx,2);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]reg_addr(0x%04x) = 0x%04x\n", reg_addr, data);
        reg_addr =reg_addr + 2;
    }
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]dump reg end===========================\n ");
}

#if NAV_USE_12_POINTS
/*
新12点布局，根据布局图配置地址寄存器
[   8  9   ]
[2  0  1  3]
[6  4  5  7]
[   A  B   ]
*/
static void cs_config_area_reg(void)
{
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_config_area_reg++\n");

    //检测面积扩大
    cs_write_sram(0xFC40, 0x0528);// Area 0 H
    cs_write_sram(0xFC42, 0x0521);// Area 0 L

    cs_write_sram(0xFC44, 0x0828);// Area 1 H
    cs_write_sram(0xFC46, 0x0821);// Area 1 L

    cs_write_sram(0xFC48, 0x0224);// Area 2 H
    cs_write_sram(0xFC4A, 0x021D);// Area 2 L

    cs_write_sram(0xFC4C, 0x0B24);// Area 3 H
    cs_write_sram(0xFC4E, 0x0B1D);// Area 3 L

    cs_write_sram(0xFC50, 0x053C);// Area 4 H
    cs_write_sram(0xFC52, 0x0535);// Area 4 L

    cs_write_sram(0xFC54, 0x083C);// Area 5 H
    cs_write_sram(0xFC56, 0x0835);// Area 5 L

    cs_write_sram(0xFC58, 0x023C);// Area 6 H
    cs_write_sram(0xFC5A, 0x0235);// Area 6 L

    cs_write_sram(0xFC5C, 0x0B3C);// Area 7 H
    cs_write_sram(0xFC5E, 0x0B35);// Area 7 L

    cs_write_sram(0xFC60, 0x0510);// Area 8 H
    cs_write_sram(0xFC62, 0x0509);// Area 8 L

    cs_write_sram(0xFC64, 0x0810);// Area 9 H
    cs_write_sram(0xFC66, 0x0809);// Area 9 L

    cs_write_sram(0xFC68, 0x0550);// Area A H
    cs_write_sram(0xFC6A, 0x0549);// Area A L

    cs_write_sram(0xFC6C, 0x0850);// Area B H
    cs_write_sram(0xFC6E, 0x0849);// Area B L
}
#endif

int cs_fp_config(void)
{
    //reset IC
    FUNC_ENTRY();
    cs_fp_reset(5);

    cs_write_sfr(0x0F, 0x01);
    cs_write_sfr(0x1C, 0x1D);
    cs_write_sfr(0x1F, 0x0A);
    cs_write_sfr(0x42, 0xAA);
    cs_write_sfr(0x60, 0x08);
    cs_write_sfr(0x63, 0x60);
    //cs_write_sfr(0x47, 0x60);//add 20160712
    //cs_write_sfr(0x13, 0x31);//add 20160712
    //cs_write_sram(0xFC1E, 0x0);//add 20160712

    /*****for 3.3V********/
    cs_write_sfr(0x22, 0x07);
    cs_write_sram(0xFC8C, 0x0001);
    cs_write_sram(0xFC90, 0x0001);
    /*********************/

    cs_write_sram(0xFC02, 0x0420);
    cs_write_sram(0xFC1A, 0x0C30);

#if NAV_USE_12_POINTS
    cs_write_sram(0xFC22, 0x086C);//打开Normal下12个敏感区域
#else
    cs_write_sram(0xFC22, 0x085C);//cs_write_sram(0xFC22, 0x0848);打开Normal下8个敏感区域
#endif

//2016.08.23
#if AUTOGAIN
    cs_write_sram(0xFC2E, DEFAULT_GAIN_VALUE);
    cs_write_sram(0xFC30, DEFAULT_BASE_VALUE);
#else
    cs_write_sram(0xFC2E, 0x00F9);//cs_write_sram(0xFC2E, 0x008F);	//cs_write_sram(0xFC2E, 0x00F6); 20160624
    cs_write_sram(0xFC30, 0x0270);//cs_write_sram(0xFC30, 0x0260);	//cs_write_sram(0xFC30, 0x0300);	// 20160624
#endif

    cs_write_sram(0xFC06, 0x0039);
    cs_write_sram(0xFC08, 0x0008);//add  times    20160624
    cs_write_sram(0xFC0A, 0x0016);
    cs_write_sram(0xFC0C, 0x0022);
    cs_write_sram(0xFC12, 0x002A);
    cs_write_sram(0xFC14, 0x0035);
    cs_write_sram(0xFC16, 0x002B);
    cs_write_sram(0xFC18, 0x0039);
    cs_write_sram(0xFC28, 0x002E);
    cs_write_sram(0xFC2A, 0x0018);
    cs_write_sram(0xFC26, 0x282D);
    cs_write_sram(0xFC82, 0x01FF);
    cs_write_sram(0xFC84, 0x0007);
    cs_write_sram(0xFC86, 0x0001);
    cs_write_sram_bit(0xFC80, 12,0);
    cs_write_sram_bit(0xFC88, 9, 0);
    cs_write_sram_bit(0xFC8A, 2, 0);
    cs_write_sram(0xFC8E, 0x3354);

#if NAV_USE_12_POINTS
    //设置新的感应点分布地址
    cs_config_area_reg();
#endif

    FUNC_EXIT();
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]IC data load succeed !\n");
    return 0;
}

static u16 cs_read_info(void)
{
    u16 hwid,data;
    u8 rx[6] = {0};

    cs_debug(DEFAULT_DEBUG,"[ChipSailing]Fingerprint driver version 1:%s \n",DRIVER_VERSION);
    //read IC ID
    cs_read_sfr(0x3F,rx,1);
    hwid = rx[0];
    hwid = hwid<<8;
    cs_read_sfr(0x3E,rx,1);
    hwid = hwid | rx[0];
    cs_debug(DEFAULT_DEBUG,"[ChipSailing]read ID : 0x%x \n",hwid);

    //read ic mode
    cs_read_sfr(0x46,rx,1);
    cs_debug(DEFAULT_DEBUG,"[ChipSailing]read mode : %x \n",rx[0]);
    
	cs_read_sfr(0x50,rx,1);
    cs_debug(DEFAULT_DEBUG,"[ChipSailing]read irqstatus : %x \n",rx[0]);
    //read gain
    data=cs_read_sram(0xFC2E,rx,2);
    cs_debug(DEFAULT_DEBUG,"[ChipSailing]read gain : %x \n",data);
	//read base
    data=cs_read_sram(0xFC30,rx,2);
    cs_debug(DEFAULT_DEBUG,"[ChipSailing]read base : %x \n",data);
	
    return hwid;
}

//CS chip init
static int cs_init(void)
{
    int status;
    status = cs_fp_config();
    if(status < 0)
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing] %s : config cs chip data error !!\n ",__func__);
        return -1;
    }
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing] %s : config cs chip data success !! \n",__func__);
    status = cs_read_info();
    if(status < 0)
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing] %s : read cs chip info error !! \n",__func__);
        return -1;
    }
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing] %s : read cs chip info  success !!\n ",__func__);

    return 0;
}

static int cs_fp_spi_transfer(u8 *tx_cmd,u8 *rx_data,int length)
{
    int ret ;
    struct spi_message m;
    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .tx_buf =tx_cmd,
        .rx_buf =rx_data,
        .len = length,
    };
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret= cs_fp_sync( &m);

    return ret;
}

static int cs_read_image(u8 *buffer)
{
    int ret ;
//    int totalSize= IMAGE_SIZE+IMAGE_DUMMY;
    u8 *image;
    u8 tx[6];
    u16 num_count;

#if AUTOGAIN
    bool autogain_ret = false;
    u8 rx_autogain[6] = {0};
    int *pold_autogain_params;
    int *pnew_autogain_params;
#endif

    FUNC_ENTRY();

    image = kmalloc(READ_IMAGE_SIZE, GFP_KERNEL);

    if (!image) 
    {
        cs_debug(DEFAULT_DEBUG,"[ChipSailing] cs_read_image  kmalloc failed!\n");
        return -ENOMEM;
    }

    tx[0] = 0xBB;
    tx[1] = 0xff;
    tx[2] = 0xff;

    num_count=0;

    disable_irq(cs15xx->irq);

    //cs_write_sfr(0x60,0x08);
    cs_write_sram(0xFC00,0x0003);
    if(gpio_get_value(cs15xx->gpio_irq))   //cs15xx->irq
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]gpio_get_value =1: %s \n", __func__);
    }
    else
    {   
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]gpio_get_value =0：%s \n", __func__);
    }
    cs_fp_spi_transfer(tx,image,READ_IMAGE_SIZE);

#if AUTOGAIN
    pold_autogain_params = kmalloc(3*sizeof(int),GFP_KERNEL);
    pnew_autogain_params = kmalloc(3*sizeof(int),GFP_KERNEL);
    //1、根据采集的图像数据image得到新的值
    pold_autogain_params[0] = cs_read_sram(0xFC2E,rx_autogain,2);//default value 0x00F9 ,cs_fp_config set //gain
    pold_autogain_params[1] = cs_read_sram(0xFC30,rx_autogain,2);//default value 0x0270 ,cs_fp_config set //base
    pold_autogain_params[2] = cs_read_sram(0xFC32,rx_autogain,2);//default value 0x0200 ,cs_fp_config NO set this value //gray
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]pold_autogain_params[0]::gain = 0x%04x, pold_autogain_params[1]::base = 0x%04x, pold_autogain_params[2]::gray = 0x%04x\n", pold_autogain_params[0], pold_autogain_params[1], pold_autogain_params[2]);

    pnew_autogain_params[0] = pold_autogain_params[0];
    pnew_autogain_params[1] = pold_autogain_params[1];
    pnew_autogain_params[2] = pold_autogain_params[2];

    autogain_ret = ChipSailing_AutoGain(image+4, 112, 88, pold_autogain_params, pnew_autogain_params);

    if(autogain_ret)
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]pnew_autogain_params[0]::gain = 0x%04x, pnew_autogain_params[1]::base = 0x%04x, pnew_autogain_params[2]::gray = 0x%04x\n", pnew_autogain_params[0], pnew_autogain_params[1], pnew_autogain_params[2]);
        //记录新的值，便于IC reset后写为此值
        g_new_gain_value = pnew_autogain_params[0];
        g_new_base_value = pnew_autogain_params[1];
        //2、重新设置gain和base
        cs_write_sram(0xFC2E, pnew_autogain_params[0]);
        cs_write_sram(0xFC30, pnew_autogain_params[1]);

        //3、重新采集图像
        cs_write_sram(0xFC00,0x0003);
        if(gpio_get_value(cs15xx->gpio_irq))   //cs15xx->irq
        {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]gpio_get_value =1: %s \n", __func__);
        }
        else
        {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]gpio_get_value =0：%s \n", __func__);
        }

        memset(image, 0x00, READ_IMAGE_SIZE);
        //重新采图
        cs_fp_spi_transfer(tx, image, READ_IMAGE_SIZE);
    }
    else //gain无变化
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]no need to change gain/base param\n");
    }
    kfree(pold_autogain_params);
    kfree(pnew_autogain_params);
#endif

    ret = __copy_to_user((u8 __user *)buffer, image+4, IMAGE_SIZE);                   //copy to user
    if (ret != 0)
    {
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]SPI SENSOR: %s: error copying to user\n", __func__);
        ret = -1;
    }

    enable_irq(cs15xx->irq);

    kfree(image);
    FUNC_EXIT();
    return ret;
}

static inline ssize_t cs_fp_sync_write( size_t len)
{
    struct spi_transfer	t =
    {
        .cs_change   = 0,
        .delay_usecs = 0,
        .speed_hz    = SPI_CLOCK_SPEED,
        .tx_buf		   = cs15xx->buffer,
        .len	     	 = len,
    };
    struct spi_message	m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return cs_fp_sync( &m);
}

static inline ssize_t cs_fp_sync_read(size_t len)
{
    struct spi_transfer	t =
    {
        .cs_change   = 0,
        .delay_usecs = 0,
        .speed_hz    = SPI_CLOCK_SPEED,
        .rx_buf		   = cs15xx->buffer,
        .len		     = len,
    };
    struct spi_message	m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return cs_fp_sync(&m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
cs_fp_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{

    ssize_t			status = 0;

    /* chipselect only toggles at start or end of operation */
    if (count > g_cs_bufsiz)
        return -EMSGSIZE;

    cs15xx = filp->private_data;

    mutex_lock(&cs15xx->buf_lock);
    status = cs_fp_sync_read(count);
    if (status > 0)
    {
        unsigned long	missing;

        missing = copy_to_user(buf, cs15xx->buffer, status);
        if (missing == status)
            status = -EFAULT;
        else
            status = status - missing;
    }
    mutex_unlock(&cs15xx->buf_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t cs_fp_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t			status = 0;
    unsigned long		missing;
    unsigned char buffer[128] = {0};

    /* chipselect only toggles at start or end of operation */
    if (count > g_cs_bufsiz)
        return -EMSGSIZE;

    cs15xx = filp->private_data;

//mutex_lock(&cs15xx->buf_lock);
    missing = copy_from_user(buffer, buf, count-1);
    if (missing == 0)
    {
        //cs_debug(DEFAULT_DEBUG, "[ChipSailing]%s, copy data from user space, success\n", __func__);
        status = -EFAULT;
    }

    if(strcmp(buffer,"info") == 0)
    {
        cs_read_info();
        cs_debug(DEFAULT_DEBUG, "[ChipSailing]NAV_MODE_FLAG=%d KEY_MODE_FLAG=%d\n", NAV_MODE_FLAG,KEY_MODE_FLAG);

#if NAV_USE_12_POINTS
        //cs_debug(DEFAULT_DEBUG, "[ChipSailing]NAV_USE_12_POINTS define!\n");
#endif

    }   
    if(strcmp(buffer,"reset") == 0)
    {
        cs_fp_config();
        cs_force_mode(2);  //set to sleep
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]echo reset sensor to sleep ok l\n");
    }
    if(strcmp(buffer,"poweron") == 0) 
    {
        cs_fp_reset(5);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]g_cs_id= %x \n",g_cs_id);
    }

    //注意在sleep模式无法读取正确值
    if(strcmp(buffer,"dump") == 0)
    {
        cs_dump_sensor_reg();
    }

    //mutex_unlock(&cs15xx->buf_lock);

    return -1;
}

static int cs_fp_message(struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
    struct spi_message	msg;
    struct spi_transfer	*k_xfers;
    struct spi_transfer	*k_tmp;
    struct spi_ioc_transfer *u_tmp;
    unsigned		n, total;
    u8			*buf;
    int			status = -EFAULT;

    spi_message_init(&msg);
    k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
    if (k_xfers == NULL)
        return -ENOMEM;

    /* Construct spi_message, copying any tx data to bounce buffer.
     * We walk the array of user-provided transfers, using each one
     * to initialize a kernel version of the same transfer.
     */
    buf = cs15xx->buffer;
    total = 0;
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers; n; n--, k_tmp++, u_tmp++)
    {
        k_tmp->len = u_tmp->len;

        total += k_tmp->len;
        if (total > g_cs_bufsiz)
        {
            status = -EMSGSIZE;
            goto done;
        }

        if (u_tmp->rx_buf)
        {
            k_tmp->rx_buf = buf;
            if (!access_ok(VERIFY_WRITE, (u8 __user *)(uintptr_t) u_tmp->rx_buf,u_tmp->len))
                goto done;
        }
        if (u_tmp->tx_buf)
        {
            k_tmp->tx_buf = buf;
            if (copy_from_user(buf, (const u8 __user *)(uintptr_t) u_tmp->tx_buf,u_tmp->len))
                goto done;
        }
        buf += k_tmp->len;

        k_tmp->cs_change = !!u_tmp->cs_change;
        k_tmp->bits_per_word = u_tmp->bits_per_word;
        k_tmp->delay_usecs = u_tmp->delay_usecs;
        k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
        dev_dbg(&cs15xx->spi->dev,
                "  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
                u_tmp->len,
                u_tmp->rx_buf ? "rx " : "",
                u_tmp->tx_buf ? "tx " : "",
                u_tmp->cs_change ? "cs " : "",
                u_tmp->bits_per_word ? : cs15xx->spi->bits_per_word,
                u_tmp->delay_usecs,
                u_tmp->speed_hz ? : cs15xx->spi->max_speed_hz);
#endif
        spi_message_add_tail(k_tmp, &msg);
    }

    status = cs_fp_sync(&msg);
    if (status < 0)
        goto done;

    /* copy any rx data out of bounce buffer */
    buf = cs15xx->buffer;
    for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++)
    {
        if (u_tmp->rx_buf)
        {
            if (__copy_to_user((u8 __user *)(uintptr_t) u_tmp->rx_buf, buf, u_tmp->len))
            {
                status = -EFAULT;
                goto done;
            }
        }
        buf += u_tmp->len;
    }
    status = total;

done:
    kfree(k_xfers);
    return status;
}

static long cs_fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int			err = 0;
    int			retval = 0;

    struct spi_device	*spi;
    u32			tmp;
    unsigned		n_ioc;
    struct spi_ioc_transfer	*ioc;
    /* Check type and command number */
    //FUNC_ENTRY();
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,(void __user *)arg, _IOC_SIZE(cmd));

    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,(void __user *)arg, _IOC_SIZE(cmd));

    if (err)
        return -EFAULT;

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    cs15xx = filp->private_data;
    spin_lock_irq(&cs15xx->spi_lock);
    spi = spi_dev_get(cs15xx->spi);
    spin_unlock_irq(&cs15xx->spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    //mutex_lock(&cs15xx->buf_lock);
    mutex_lock(&cs_spi_lock);
    switch (cmd & 0xff00ffff)
    {
    case (SPI_IOC_SCAN_IMAGE & 0xff00ffff):       //(%c)'k'=(%x)4B=(%d)75
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd====== scan image \n");
        retval = cs_read_image((u8 __user*)arg);
		del_timer(&cs_timer_kick);
		g_timer_kick_calc=0;
        break;

    case (SPI_IOC_SENSOR_RESET & 0xff00ffff):
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======reset \n");
        cs_fp_config();
        cs_force_mode(2);  //set to sleep
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]IOCTL reset sensor to sleep ok\n");
        break;

    case (SPI_IOC_KEYMODE_ON & 0xff00ffff):
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======SPI_IOC_KEYMODE_ON \n");
        KEY_MODE_FLAG=1;
        break;
    case (SPI_IOC_KEYMODE_OFF & 0xff00ffff):
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======SPI_IOC_KEYMODE_OFF \n");
        KEY_MODE_FLAG=0;
        break;
    case (SPI_IOC_NAV_ON & 0xff00ffff):
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======SPI_IOC_NAV_ON \n");
        NAV_MODE_FLAG=1;
        break;
    case (SPI_IOC_NAV_OFF & 0xff00ffff):
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======SPI_IOC_NAV_OFF \n");
        NAV_MODE_FLAG=0;
        break;
    case (SPI_IOC_FINGER_STATUS & 0xff00ffff):
        __put_user(g_finger_status, (__u32 __user *)arg);
        //cs_debug(DEFAULT_DEBUG,"[chipsailing]cmd======SPI_IOC_FINGER_STATUS g_finger_status=%d  arg=%ld \n",g_finger_status,arg);
        break;

    case (SPI_IOC_READER_ENTER & 0xff00ffff):
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======SPI_IOC_READER_ENTER \n");
        SPI_IOC_READER_FLAG=1;
        break;
    case (SPI_IOC_READER_EXIT & 0xff00ffff):
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd======SPI_IOC_READER_EXIT \n");
        SPI_IOC_READER_FLAG=0;
        break;

    default:
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cmd====== unknow \n");
        /* segmented and/or full-duplex I/O request */
        if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))|| _IOC_DIR(cmd) != _IOC_WRITE)
        {
            retval = -ENOTTY;
            //cs_debug(DEFAULT_DEBUG,"[ChipSailing]IOCTL retval = -ENOTTY 1;\n");
            break;
        }

        tmp = _IOC_SIZE(cmd);
        if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
        {
            retval = -EINVAL;
            //cs_debug(DEFAULT_DEBUG,"[ChipSailing]IOCTL retval = -EINVAL 2;\n");
            break;
        }

        n_ioc = tmp / sizeof(struct spi_ioc_transfer);
        if (n_ioc == 0)
        {
            //cs_debug(DEFAULT_DEBUG,"[ChipSailing]IOCTL n_ioc =0;\n");
            break;
        }
        /* copy into scratch area */
        ioc = kmalloc(tmp, GFP_KERNEL);
        if (!ioc)
        {
            retval = -ENOMEM;
            //cs_debug(DEFAULT_DEBUG,"[ChipSailing]retval = -ENOMEM; 3\n");
            break;
        }
        if (__copy_from_user(ioc, (void __user *)arg, tmp))
        {
            kfree(ioc);
            retval = -EFAULT;
            //cs_debug(DEFAULT_DEBUG,"[ChipSailing]retval = -EFAULT; 4\n");
            break;
        }

        /* translate to spi_message, execute */
        retval = cs_fp_message(ioc, n_ioc);
        kfree(ioc);
        break;
    }
    mutex_unlock(&cs_spi_lock);
   // mutex_unlock(&cs15xx->buf_lock);
    spi_dev_put(spi);

//FUNC_EXIT();
    return retval;
}

static long cs_fp_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return cs_fp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}

static irqreturn_t cs_irq_handler(int irq, void *dev_id)
{
	  g_irg_flag = 1;
    //wake_lock_timeout(&cs_lock, 5*HZ);
    disable_irq_nosync(cs15xx->irq);
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

u64  cs_get_current_ms(void)
{
    if((get_jiffies_64()*10)<0)
        return 0;
    else
        return (get_jiffies_64()*10);
}


void cs_report_key(int key_type,int key_status)
{

    switch(key_type)
    {
    case 0:
        input_report_key(cs15xx_inputdev,KEY_HOME,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key KEY_HOME \n");
        break;

    case 1:
        input_report_key(cs15xx_inputdev,KEY_MENU,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key KEY_MENU \n");
        break;

    case 2:
        input_report_key(cs15xx_inputdev,KEY_BACK,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key KEY_BACK \n");
        break;

    case 3:
        input_report_key(cs15xx_inputdev,KEY_POWER,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key KEY_POWER \n");
        break;

    case 4:
        input_report_key(cs15xx_inputdev,KEY_F11,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key KEY_F11 \n");
        break;

    case 5:
        input_report_key(cs15xx_inputdev,KEY_F5,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key  KEY_F5\n");
        break;

    case 6:
        input_report_key(cs15xx_inputdev,KEY_F6,key_status);
        input_sync(cs15xx_inputdev);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs input_report_key  KEY_F6\n");
        break;

    default:
        break;

    }
}

//cs_bit_count如果>=7个区域有数据则返回1，否则返回0
int cs_bit_count(unsigned int n)
{
    unsigned int cnt = 0;

    while(n>0)
    {
        if((n&1)==1)
            cnt++;
        n>>=1;
    }

    return cnt>7?1:0;       //7为可改变的数据敏感点数量
}
void wakeup(void)
{   
    u8 send_buf[3]= {0};
    struct spi_message m;

    struct spi_transfer t =
    {
        .cs_change = 0,
        .delay_usecs = 1,
        .speed_hz = SPI_CLOCK_SPEED,
        .bits_per_word = 8,
        .tx_buf  = send_buf,
        .len	 = 1,
    };
    mutex_lock(&sfr_write_lock);
    send_buf[0] = 0xd5;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    cs_fp_sync(&m);
	mutex_unlock(&sfr_write_lock);
}

static int cs_force_to_idle(void)
{
    int ret = 0;
    int count = 0;
    u8 status = 0x00;
    u8 mode = 0x00;
	
    switch ( cs_read_sfr(0x46, &mode, 1) )
    {
    case 0x70:
        break;
    case 0x71:
        do
        {
            cs_write_sfr(0x46, 0x70);
            if (0x70 == cs_read_sfr(0x46, &mode, 1))
            {
                break;
            }
        }
        while (++count < 5);
        if (count == 5)
            ret = -1;

        count = 0;
        do
        {
            if (cs_read_sfr(0x50, &status, 1) & 0x01)
            {
                cs_write_sfr(0x50, 0x01);
                break;
            }
        }
        while (++count < 5);//while (++count < 1);
        printk("ChipSailing count=%d",count);
        break;
    default:
        do
        {
            wakeup();
            cs_write_sfr(0x46, 0x70);
            if (0x70 == cs_read_sfr(0x46, &mode, 1))
            {
                break;
            }
        }
        while (++count < 5);
        if (count == 5)
            ret = -2;

        count = 0;
        do
        {
            if (cs_read_sfr(0x50, &status, 1) & 0x01)
            {
                cs_write_sfr(0x50, 0x01);
                break;
            }
        }
        while (++count < 5);
        break;
    }

    return 0;
}

static void cs_force_mode(int mod)
{
    cs_force_to_idle();
    switch(mod)
    {
    case 4:  //deep sleep
        cs_write_sfr(0x46, 0x76);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]force to deep mode \n");
        break;
    case 3:  //normal
        cs_write_sfr(0x46, 0x71);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]force to normal \n");
        break;
    case 2:  //sleep
        cs_write_sfr(0x46, 0x72);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]force to sleep \n");
        break;
    case 1:  //idle
        cs_write_sfr(0x46, 0x70);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]force to idle \n");
        break;
    default:
        break;
    }
	

   
}

#if 1
void force_mode_func(struct work_struct *work)
{  
   
	 disable_irq_nosync(cs15xx->irq);
	 cs_force_mode(2);
	 cs_debug(DEFAULT_DEBUG,"[ChipSailing]force to sleep mode in the queue\n");
	 enable_irq(cs15xx->irq);
}
#endif 

void  cs_timer_func(unsigned long arg)
{
    int ret=0;  
	//u8 index = 0;

       if(NAV_MODE_FLAG == 0)
    {   
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing] %s : NAV_TIMER run ,but NAV_MODE_FLAG=%d !!\n ",__func__,NAV_MODE_FLAG);
        return;
    }

    //cs_debug(DEFAULT_DEBUG, "[ChipSailing]cs_timer start, Num1=%lld Num2=%lld,g_finger_status=%d,KEY_MODE_FLAG=%d\n",num1,num2,g_finger_status,KEY_MODE_FLAG);
#if 0
    for(index = 0; index < g_nav_valid_num; index++)
    {
        cs_debug(DEFAULT_DEBUG, "[ChipSailing]g_nav_reg_val[%d] = 0x%04x\n", index, g_nav_reg_val[index]);
    }
#endif

    ret = cs_finger_navigation(g_nav_reg_val, g_nav_valid_num,0);   //ret: 1-down  2-up 3-left  4-right  -1:unknow
    if(ret >0){
		//cs_debug(DEFAULT_DEBUG,"[ChipSailing] cs input_report_key KEY_KICK  DEL KEY_LONGTOUCH !,ret=%d; \n", ret);
		del_timer(&cs_timer_long_louch);
		g_isnav =1;
	}
    if(ret==4)
    {
        input_report_key(cs15xx_inputdev,NAV_DOWN,1);
        input_sync(cs15xx_inputdev);
        input_report_key(cs15xx_inputdev,NAV_DOWN,0);
        input_sync(cs15xx_inputdev);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]Direction:down------------------\n");
    }
    else if(ret==3)
    {
        input_report_key(cs15xx_inputdev,NAV_UP,1);
        input_sync(cs15xx_inputdev);
        input_report_key(cs15xx_inputdev,NAV_UP,0);
        input_sync(cs15xx_inputdev);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]Direction:up--------------------\n");
    }
    else if(ret==1)
    {
        if(SPI_IOC_READER_FLAG==0)
        {
            input_report_key(cs15xx_inputdev,NAV_LEFT,1);
            input_sync(cs15xx_inputdev);
            input_report_key(cs15xx_inputdev,NAV_LEFT,0);
            input_sync(cs15xx_inputdev);
        }
        else
        {
            input_report_key(cs15xx_inputdev,KEY_VOLUMEDOWN,1);
            input_sync(cs15xx_inputdev);
            input_report_key(cs15xx_inputdev,KEY_VOLUMEDOWN,0);
            input_sync(cs15xx_inputdev);
        }

        cs_debug(DEFAULT_DEBUG,"[ChipSailing]Direction:left-------------------\n");
    }
    else if(ret==2)
    {
        if(SPI_IOC_READER_FLAG==0)
        {
            input_report_key(cs15xx_inputdev,NAV_RIGHT,1);
            input_sync(cs15xx_inputdev);
            input_report_key(cs15xx_inputdev,NAV_RIGHT,0);
            input_sync(cs15xx_inputdev);
        }
        else
        {
            input_report_key(cs15xx_inputdev,KEY_VOLUMEUP,1);
            input_sync(cs15xx_inputdev);
            input_report_key(cs15xx_inputdev,KEY_VOLUMEUP,0);
            input_sync(cs15xx_inputdev);
        }
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]Direction:right------------------\n");
    }
    else
    {
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]Direction:UNKNOW-----------------\n");
    }

    //计算完成后g_nav_valid_num和数组值清空
    memset(g_nav_reg_val, 0, sizeof(int)*10);
    g_nav_valid_num = 0;
}


void  cs_timer_long_touch_func(unsigned long arg)
{
    //手指已经长按下1S后且未离开则执行长按事件上报
    if(g_finger_status == 1)
    {
        g_longtouch =1;
        input_report_key(cs15xx_inputdev,KEY_LONGTOUCH,1);  //KEY_LONGTOUCH
        input_sync(cs15xx_inputdev);
        input_report_key(cs15xx_inputdev,KEY_LONGTOUCH,0); //KEY_LONGTOUCH
        input_sync(cs15xx_inputdev);
		
    }
    cs_debug(DEFAULT_DEBUG,"[ChipSailing] cs input_report_key KEY_KICK KEY_LONGTOUCH !\n");
}

//超时后检测手指是那种kick方式
void cs_timer_kick_func(unsigned long arg)
{
    //static int g_timer_kick_calc = 0;   //g_timer_kick_calc统计kick总数
   //cs_debug(DEFAULT_DEBUG,"[ChipSailing]g_timer_kick_calc=%d \n",g_timer_kick_calc);
    if(KEY_MODE_FLAG == 0 || g_longtouch ==1  ){
	   //if(KEY_MODE_FLAG == 0 || g_longtouch ==1 || g_isnav ==1 ){
		g_longtouch=0;
		g_isnav =0;
		g_timer_kick_calc=0;
		return ;
	}
    switch(g_timer_kick_calc)
    {
    case 1: 
        //input_report_key(cs15xx_inputdev,KEY_KICK1S,1);
        //input_sync(cs15xx_inputdev);
        //input_report_key(cs15xx_inputdev,KEY_KICK1S,0);
        //input_sync(cs15xx_inputdev);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing] cs input_report_key KEY_KICK once  !\n");
        break;
    case 2:
        input_report_key(cs15xx_inputdev,KEY_KICK2S,1);
        input_sync(cs15xx_inputdev);
        input_report_key(cs15xx_inputdev,KEY_KICK2S,0);
        input_sync(cs15xx_inputdev);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing] cs input_report_key KEY_KICK twice  !\n");
        break;
    case 3:
        //<---
        //do something

        //--->
        cs_debug(DEFAULT_DEBUG,"[ChipSailing] cs input_report_key KEY_KICK 3 times  !\n");
        break;

    default:
        break;

    }

    g_timer_kick_calc=0;
}

//超时后检测手指是否离开
void cs_finger_detec_timer_func(unsigned long arg)
{

     //cs_debug(DEFAULT_DEBUG,"[ChipSailing]g_finger_status = %d !\n",g_finger_status);
	if( g_finger_status == 1 )
	{
		cs_debug(DEFAULT_DEBUG,"[ChipSailing]set g_finger status = 2, finger leave\n");
		input_report_key(cs15xx_inputdev,KEY_F19,0);
		input_sync(cs15xx_inputdev);
		g_finger_status = 0;
		g_timer_kick_calc++;
#if 1		
            // !在定时器中添加与spi操作相关的代码会导致手机系统重启，所以使用队列的方式实现模式切换
		    if( (NAV_MODE_FLAG == 1) && (g_finger_status == 0)){
		      //cs_debug(DEFAULT_DEBUG,"ChipSailing  start to sleep\n");
			  //如果手指抬起则启动线程切换模式  20161006
		       //queue_work(force_mode_wq, &force_mode_work);
			   cancel_delayed_work(&force_mode_work);
			   queue_delayed_work(force_mode_wq, &force_mode_work, 500);     //5s
	        }
#endif	
 
		del_timer(&cs_timer_long_louch);	//如果手指抬起则删除长按定时器
		
		del_timer(&cs_timer_kick);   
		//cs_debug(DEFAULT_DEBUG,"ChipSailing del cs_timer_kick \n");
		add_timer(&cs_timer_kick);                //添加定时器，定时器开始生效
		mod_timer(&cs_timer_kick, jiffies + DOUBELDECTIME);  //重新设定超时时间100~1S  //300ms
	 
	}     

}

static int cs_work_func(void *data)
{
    u8 mode = 0x00;
    u8 status = 0x00;
    u8 status1 = 0x00;
	int i=0;
    int bitcount_reg48 = 0;
    //struct sched_param param = {.sched_priority = REG_RT_PRIO(1)};

#if NAV_USE_12_POINTS
    u8 reg49_val[2] = {0};
    u16 reg49_48_val = 0;
#endif

    u8 reg48_val[2] = {0};
    u8 val_index = 0;      
    u64 currentMs1 = 0;
    u64 currentMs2 = 0;
    u64 waitms = 0;

    //sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        currentMs1 = cs_get_current_ms();//ktime_to_us(ktime_get());
        //set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, g_irg_flag != 0);
    g_irg_flag = 0;
        currentMs2= cs_get_current_ms(); //ktime_to_us(ktime_get());
        waitms = currentMs2 - currentMs1;
    //深度休眠等待spi总线恢复
    mutex_lock(&cs_spi_lock);
		if(host_deep_sleep==1)  //深度休眠后中断线程先于resume运行
	    {  
          do{ 
            if((cs_read_sfr(0x46, &mode, 1)==0x72)||(cs_read_sfr(0x46, &mode, 1)==0x71))   
               {
			//	printk("[ChipSailing]cs_report_key  power \n");
				break;
				}
				 mdelay(5); 
			 }while(1);  
		 } 
        status = cs_read_sfr(0x50, &status, 1);
        mode   = cs_read_sfr(0x46, &mode, 1);

        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_work_func in --- val_index =%d, mode =0x%02x , status =0x%02x  \n", val_index,mode,status);

/****启动导航定时器,长按定时器**/

		if((status != 0x01)&&(status != 0x00)&& (waitms > WAITDOWNTIME)&& (g_finger_status == 0))    //(reg48_val[0] != 0)
		{
			
			g_finger_status = 1;
			cs_debug(DEFAULT_DEBUG,"[ChipSailing]set g_finger_status = 1, finger putdown\n");
            if(KEY_MODE_FLAG ==1){
				input_report_key(cs15xx_inputdev,KEY_F19,1);
				input_sync(cs15xx_inputdev);
				del_timer(&cs_timer_long_louch);
				add_timer(&cs_timer_long_louch);                           //添加定时器，定时器开始生效
				mod_timer(&cs_timer_long_louch, jiffies + LONGTOUCHTIME);  //重新设定超时时间100~1S  
				//////cs_debug(DEFAULT_DEBUG,"[ChipSailing]add_timer(&cs_timer for longtouch)...NAV_MODE_FLAG =%d\n",NAV_MODE_FLAG);

			}
			if(NAV_MODE_FLAG == 1){
				del_timer(&cs_timer);
				add_timer(&cs_timer);                             //添加定时器，定时器开始生效
				mod_timer(&cs_timer, jiffies + NAVDECTIME);     
				//////cs_debug(DEFAULT_DEBUG,"[ChipSailing]add_timer(&cs_timer for nav)...\n");
				
				memset(g_nav_reg_val, 0, sizeof(int)*10);         //刚启动定时器时给记录导航数据的数组清零
				val_index = 0;
				g_nav_valid_num = 0;
				
			}
		}
		
/*********采集数据**********/	
        if((NAV_MODE_FLAG == 1) && (val_index <= 9) && (mode == 0x71)&&(status == 0x04))
        {
			
			 cs_read_sfr(0x48, reg48_val, 1);
            //////cs_debug(DEFAULT_DEBUG,"[ChipSailing]NAV 0x48 value=0x%02x\n", reg48_val[0]);	
#if NAV_USE_12_POINTS
             cs_read_sfr(0x49, reg49_val, 1);
             reg49_48_val = ((u16)(reg49_val[0] << 8) + reg48_val[0]) & 0x0FFF;  //数据合并为12位			            
			////cs_debug(DEFAULT_DEBUG,"[ChipSailing]NAV 0x49 value=0x%02x\n", reg49_val[0]);
			////cs_debug(DEFAULT_DEBUG,"[ChipSailing]reg49_48_val = 0x%04x\n", reg49_48_val);
#endif       
            if(g_nav_valid_num != 0)   //去重复数据
            {
#if NAV_USE_12_POINTS				
                if((reg49_48_val != g_nav_reg_val[val_index-1] ))
                {
                    g_nav_reg_val[val_index] = reg49_48_val;
                    val_index++;
                    g_nav_valid_num = val_index;
                    ////cs_debug(DEFAULT_DEBUG, "[ChipSailing]B:store g_nav_reg_val[%d] = 0x%04x, g_nav_valid_num = %d\n", (val_index-1), g_nav_reg_val[val_index-1], g_nav_valid_num);
                }
#else				
				if((reg48_val[0] != g_nav_reg_val[val_index-1]))
                {
                    g_nav_reg_val[val_index] = reg48_val[0];
                    val_index++;
                    g_nav_valid_num = val_index;
                    ////cs_debug(DEFAULT_DEBUG,"[ChipSailing]B:store g_nav_reg_val[%d] = 0x%04x, g_nav_valid_num = %d\n", (val_index-1), g_nav_reg_val[val_index-1], g_nav_valid_num);
                }
#endif				
                else
                {
                    ////cs_debug(DEFAULT_DEBUG, "[ChipSailing]nav data equal, discard!\n");
                }
            }
            else//第一次存储数据
            {
                val_index = 0;// debug
#if NAV_USE_12_POINTS	
                g_nav_reg_val[val_index] = reg49_48_val;
#else
				g_nav_reg_val[val_index] = reg48_val[0];
#endif
                val_index++;
                g_nav_valid_num = val_index;
                //cs_debug(DEFAULT_DEBUG,"[ChipSailing]A:store g_nav_reg_val[%d] = 0x%04x, g_nav_valid_num = %d\n", (val_index-1), g_nav_reg_val[val_index-1], g_nav_valid_num);
            }
        }
		
		
/*****切模式及异步信号****/
        if(mode == 0x71 && status == 0x04)
        {  
			cs_read_sfr(0x48, reg48_val, 1);
			
            cs_write_sfr(0x46, 0x70);
            for ( i = 0; i<10; i++) {
		       if (( cs_read_sfr(0x50, &status1, 1) ==0x01 )) {
				    cs_write_sfr(0x50, 0x01);  //clear irq
				    break;
			    }
		    }
			
            //cs_bit_count如果>=7个区域有数据则返回1，否则返回0
            bitcount_reg48 = cs_bit_count(reg48_val[0]);
            ////cs_debug(DEFAULT_DEBUG,"[ChipSailing]bitcount_reg48 =%d \n", bitcount_reg48);
          
            if(NAV_MODE_FLAG == 1)
            {
                //kill_fasync(&async, SIGIO, POLL_IN); //如果在导航模式下产生的中断则直接上报信号 0928
				cs_force_mode(3);                    //cs_write_sfr(0x46, 0x71); 
                ////cs_debug(DEFAULT_DEBUG,"[ChipSailing]NAV normal kill_fasync,into normal ..... \n");
            }
            else
            {
                if(bitcount_reg48 == 1)
                {
                    kill_fasync(&async, SIGIO, POLL_IN);
                    ////cs_debug(DEFAULT_DEBUG,"[ChipSailing]normal kill_fasync ..... \n");
                }
                else
                {
                    cs_force_mode(3);
                    ////cs_debug(DEFAULT_DEBUG, "[ChipSailing]entering area too small, force to normal ..... \n");
                }
            }
        }
        else if(mode == 0x72 && status == 0x08)
        {
            cs_write_sfr(0x46, 0x70);
            for ( i = 0; i<10; i++) {
		       if (( cs_read_sfr(0x50, &status1, 1) ==0x01 )) {
				    cs_write_sfr(0x50, 0x01);  //clear irq
				    break;
			    }
		    }
			if(NAV_MODE_FLAG == 1){
				cs_force_mode(3);
			}
			else{
				kill_fasync(&async, SIGIO, POLL_IN);
				////cs_debug(DEFAULT_DEBUG,"[ChipSailing]sleep kill_fasync ..... \n");
		    }
        }
		else 
        {
            cs_write_sfr(0x50, 0x01);
            ////cs_debug(DEFAULT_DEBUG,"[ChipSailing]idle kill_fasync ..... \n");
        }
		
/******超时定时器*****/		
		if( status != 0x01){
			del_timer(&cs_timer_finger_detec);
			add_timer(&cs_timer_finger_detec);              // 超时检测手指抬起
			mod_timer(&cs_timer_finger_detec, jiffies + 15); // 50ms	   //50->150加长抬起检测时间
		}	
		
#if 0
        mode   = cs_read_sfr(0x46, &mode, 1);
        status = cs_read_sfr(0x50, &status, 1);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_work_func end---val_index=%d, mode=0x%02x, status=0x%02x\n",val_index, mode, status);
#endif

		mutex_unlock(&cs_spi_lock);
        enable_irq(cs15xx->irq);
    }
    while(!kthread_should_stop());
    return 0;
}

static int cs_fp_fasync(int fd, struct file *filp, int mode)
{
    int ret;
    ret = fasync_helper(fd, filp, mode, &async);
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]fasync_helper .... \n");
    return ret;
}

static int cs_fp_open(struct inode *inode, struct file *filp)
{
    int status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(cs15xx, &device_list, device_entry)
    {
        if (cs15xx->devt == inode->i_rdev)
        {
            status = 0;
            break;
        }
    }
    if (status == 0)
    {
        if (!cs15xx->buffer)
        {
            cs15xx->buffer = kmalloc(g_cs_bufsiz, GFP_KERNEL);
            if (!cs15xx->buffer)
            {
                dev_dbg(&cs15xx->spi->dev, "open/ENOMEM\n");
                status = -ENOMEM;
            }
        }
        if (status == 0)
        {
            cs15xx->users++;
            filp->private_data = cs15xx;
            nonseekable_open(inode, filp);
        }
    }
    else
        pr_debug("[ChipSailing]: nothing for minor %d\n", iminor(inode));

    mutex_unlock(&device_list_lock);

    return status;
}

// Register interrupt device
static int cs_create_inputdev(void)
{
    cs15xx_inputdev = input_allocate_device();
    if (!cs15xx_inputdev)
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs15xx_inputdev create faile!\n");
        return -ENOMEM;
    }
   
    __set_bit(EV_KEY,cs15xx_inputdev->evbit);
    __set_bit(KEY_KICK1S,cs15xx_inputdev->keybit);
    __set_bit(KEY_KICK2S,cs15xx_inputdev->keybit);
    __set_bit(KEY_LONGTOUCH,cs15xx_inputdev->keybit);
    __set_bit(KEY_POWER,cs15xx_inputdev->keybit);

    __set_bit(KEY_HOME,cs15xx_inputdev->keybit);
    __set_bit(KEY_BACK,cs15xx_inputdev->keybit);
    __set_bit(KEY_MENU,cs15xx_inputdev->keybit);
    __set_bit(KEY_F11,cs15xx_inputdev->keybit);
    __set_bit(KEY_F6,cs15xx_inputdev->keybit);
    __set_bit(KEY_VOLUMEDOWN,cs15xx_inputdev->keybit);
    __set_bit(KEY_VOLUMEUP,cs15xx_inputdev->keybit);

    __set_bit(KEY_F5,cs15xx_inputdev->keybit);

    __set_bit(NAV_UP,cs15xx_inputdev->keybit);
    __set_bit(NAV_DOWN,cs15xx_inputdev->keybit);
    __set_bit(NAV_LEFT,cs15xx_inputdev->keybit);
    __set_bit(NAV_RIGHT,cs15xx_inputdev->keybit);
    __set_bit(KEY_F19,cs15xx_inputdev->keybit);

    cs15xx_inputdev->id.bustype = BUS_HOST;
    cs15xx_inputdev->name = "cs15xx_inputdev";
    cs15xx_inputdev->id.version = 1;


    if (input_register_device(cs15xx_inputdev))
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]register inputdev failed\n");
        input_free_device(cs15xx_inputdev);
        return -ENOMEM;
    }
    else
    {
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]register inputdev success!\n");
        return 0;
    }
}

static int cs_fp_release(struct inode *inode, struct file *filp)
{
    int status = 0;

    mutex_lock(&device_list_lock);
    cs15xx = filp->private_data;
    filp->private_data = NULL;

    /* last close? */
    cs15xx->users--;
    if (!cs15xx->users)
    {
        int		dofree;

        kfree(cs15xx->buffer);
        cs15xx->buffer = NULL;

        /* ... after we unbound from the underlying device? */
        spin_lock_irq(&cs15xx->spi_lock);
        dofree = (cs15xx->spi == NULL);
        spin_unlock_irq(&cs15xx->spi_lock);

        if (dofree)
            kfree(cs15xx);
    }
    mutex_unlock(&device_list_lock);

    return status;
}

/* REVISIT switch to aio primitives, so that userspace
 * gets more complete API coverage.  It'll simplify things
 * too, except for the locking.
 */
static const struct file_operations cs_fp_fops =
{
    .owner =	THIS_MODULE,
    .write =	cs_fp_write,
    .read =		cs_fp_read,
    .unlocked_ioctl = cs_fp_ioctl,
    .compat_ioctl = cs_fp_compat_ioctl,
    .open =		cs_fp_open,
    .release =	cs_fp_release,
    .llseek =	no_llseek,
    .fasync =	cs_fp_fasync,
};

static ssize_t fp_chipid_show(struct device *dev,   
		    struct device_attribute *attr, char *buf)
{   
	return sprintf(buf, "chipsailing\n"); 
}
static DEVICE_ATTR(chipid, S_IRUGO, fp_chipid_show, NULL);

static struct attribute *fp_debug_attrs[] = {
	&dev_attr_chipid.attr,
    NULL
};

static const struct attribute_group fp_debug_attr_group = {
    .attrs = fp_debug_attrs,
    .name = "debug"
};

static ssize_t fp_proc_read_val(struct file *file,
		char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0; 
	char data[80];

	len += sprintf(data + len, "IC:chipsailing.\n");

	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t fp_proc_write_val(struct file *filp,
		const char *buff, size_t len, 
		loff_t * off) 
{
	return len; 
}

static struct file_operations fp_proc_ops = {
	.read = fp_proc_read_val,
	.write = fp_proc_write_val,
};

static void create_fp_proc_entry(void)
{
	struct proc_dir_entry *fp_proc_entry;
	fp_proc_entry = proc_create("driver/fingerprint_id", 0644, NULL, &fp_proc_ops);
	if (fp_proc_entry) {
		printk(KERN_INFO "create fpsensor proc file sucess!\n");
	} else 
		printk(KERN_INFO "create fpsensor proc file failed!\n");
}
static struct class *cs_fp_class;


/*-------------------------------------------------------------------------*/
static int cs_fp_probe(struct spi_device *spi)
{
    //int 		err;
    int			status;
    unsigned long		minor;

    //u32 ints[2]= {0};
    FUNC_ENTRY();
    /* Allocate driver data */
    cs15xx = kzalloc(sizeof(*cs15xx), GFP_KERNEL);
    if (!cs15xx)
        return -ENOMEM;

    spin_lock_init(&cs15xx->spi_lock);
    mutex_init(&cs15xx->buf_lock);
    mutex_lock(&device_list_lock);
    INIT_LIST_HEAD(&cs15xx->device_entry);

    /*Initialize  SPI parameters.*/
    cs15xx->spi = spi;
    cs15xx->spi->mode = SPI_MODE_0;
    cs15xx->spi->bits_per_word = 8;
    cs15xx->spi->chip_select   = 0;
	cs15xx->spi->controller_data = (void*)&spi_conf;
    if(spi_setup(cs15xx->spi) < 0 )
    {
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs15xx setup failed\n");
		kfree(cs15xx);
        return -ENODEV;
    }
	
	#if 1
	//创建一个名为force_mode_queue的工作队列并把工作队列的入口地址赋给声明的指针20161006
	force_mode_wq = create_workqueue("force_mode_queue");
	//INIT_WORK(&force_mode_work,force_mode_func);
	INIT_DELAYED_WORK(&force_mode_work,force_mode_func);
	#endif
	

	pin_select(pwr_on);
    //hw_reset
    cs_fp_reset(5);
	
    //read id
    g_cs_id = cs_read_info();
    //if read id right then go on probe
    if(cs_read_info() == 0xa062)
    {
		pin_select(irq_en);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]read chip id is: %x  	\n",g_cs_id);
        /* Initialize the driver data */
        BUILD_BUG_ON(N_SPI_MINORS > 256);
        status = register_chrdev(CS_FP_MAJOR, "cs_spi", &cs_fp_fops);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_fp_init register_chrdev state=%d\n ",status);
        if (status < 0)
            return status;

        cs_fp_class = class_create(THIS_MODULE, "cs_spi");
        if (IS_ERR(cs_fp_class))
        {
            unregister_chrdev(CS_FP_MAJOR, DRIVER_NAME );
            return PTR_ERR(cs_fp_class);
        }
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]create class success!\n");
        minor = find_first_zero_bit(minors, N_SPI_MINORS);
        if (minor < N_SPI_MINORS)
        {
            struct device *dev;

            cs15xx->devt = MKDEV(CS_FP_MAJOR, minor);
            dev = device_create(cs_fp_class, &spi->dev, cs15xx->devt,cs15xx, "cs_spi");
            status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
        }
        else
        {
            dev_dbg(&spi->dev, "no minor number available!\n");
            status = -ENODEV;
        }
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing] Create cs_spi node success!\n");
        if (status == 0)
        {
            set_bit(minor, minors);
            list_add(&cs15xx->device_entry, &device_list);
        }

        //init device file
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]init device file !\n");
        if (status == 0)
            spi_set_drvdata(spi, cs15xx);
        else
            kfree(cs15xx);

        cs15xx->irq = get_gpio(num);
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs15xx->irq num=%d\n",cs15xx->irq);
            if(request_irq(cs15xx->irq,cs_irq_handler,IRQF_TRIGGER_RISING | IRQF_ONESHOT ,"finger_print",NULL))
        {
            cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs finger sensor ---IRQ LINE NOT AVAILABLE!!\n");

            //goto err_free_input;
        }
        enable_irq_wake(cs15xx->irq);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]init device file success!\n");
        //for IC Initialize
        cs_create_inputdev();
        SPI_IOC_READER_FLAG=0;
        //wake_lock_init(&cs_lock, WAKE_LOCK_SUSPEND, "cs_wake_lock");
        cs_init();
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_init success !\n");

        mutex_unlock(&device_list_lock);
        //cs_debug(DEFAULT_DEBUG,"[ChipSailing]Enter creating thread...!\n");
        cs_irq_thread =kthread_run(cs_work_func,NULL,"cs_thread");
        if(IS_ERR(cs_irq_thread))
        {
            //cs_debug(DEFAULT_DEBUG,"[ChipSailing] Failed to create kernel thread: %ld\n", PTR_ERR(cs_irq_thread));
        }
        cs_timer_init();
#if 0//defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
        cs15xx->early_fp.level		= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
        //cs15xx->early_fp.suspend	= cs_suspend,
       // cs15xx->early_fp.resume		= cs_resume,
        register_early_suspend(&cs15xx->early_fp);
#endif
		status = sysfs_create_group(&spi->dev.kobj,&fp_debug_attr_group);
		if(status){
			cs_debug(DEFAULT_DEBUG,"Failed to create chipid file.\n");
		}
		create_fp_proc_entry();
    }
    else
    {
        cs_debug(DEFAULT_DEBUG, "[ChipSailing]read chip id FAIL !\n" );
		kfree(cs15xx);
		pin_select(pwr_off);
        status = -ENODEV;
    }

    FUNC_EXIT();
    return status;
}

static int  cs_fp_remove(struct spi_device *spi)
{
    struct cs_fp_data	*cs15xx = spi_get_drvdata(spi);

	//删除队列  20161006
	 destroy_workqueue(force_mode_wq);
	
    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&cs15xx->spi_lock);
    cs15xx->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&cs15xx->spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);

    list_del(&cs15xx->device_entry);
    device_destroy(cs_fp_class, cs15xx->devt);
    //wake_lock_destroy(&cs_lock);
    clear_bit(MINOR(cs15xx->devt), minors);
#if 0//defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    unregister_early_suspend(&cs15xx->early_fp);
#endif
    if (cs15xx->users == 0)
        kfree(cs15xx);
    mutex_unlock(&device_list_lock);

    return 0;
}
/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/cs15xxB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

#if 1//defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
static int cs_clk_enable(struct cs_fp_data *spidev)
{

#ifdef CONFIG_MTK_CLKMGR
    enable_clock(MT_CG_PERI_SPI0, "spi");
#else
    /* changed after MT6797 platform */
    // struct mt_spi_t *ms = NULL;

    // ms = spi_master_get_devdata(spidev->spi->master);
    //mt_spi_enable_clk(ms);    // FOR MT6797
#endif
    spidev->clk_enabled = 1;

    return 0;
}

static int cs_resume(struct spi_device *spi)
{
    struct cs_fp_data *spi_dev;
    spi_dev = spi_get_drvdata(spi);
    // Initialize the driver data
    spi_dev->spi = spi;
    spi_dev->spi->bits_per_word = 8;
    spi_dev->spi->mode = SPI_MODE_0;
    spi_setup(spi_dev->spi);
    cs_clk_enable(spi_dev);
 
    host_deep_sleep=0;

    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]2016cs_resume_test.\n");
    return 0;
}

static int cs_suspend(struct spi_device *spi, pm_message_t mesg)
{
   host_deep_sleep=1;
   return 0;
}
#endif

static struct spi_driver cs_fp_spi_driver =
{
    .driver = {
        .name   =	FINGERPRINT_DEV_NAME,
        .owner  =	THIS_MODULE,
    },
    .probe =	cs_fp_probe,
    .remove =	cs_fp_remove,
    .suspend = cs_suspend,
    .resume = cs_resume,
};

static int  cs_timer_init(void)
{
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_timer_init+\n");
    init_timer(&cs_timer);               //初始化定时器
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing][timer]init timer finish!\n");
    cs_timer.expires = jiffies+1;        //设定超时时间，100代表1秒
    cs_timer.data = 60;                  //传递给定时器超时函数的值
    cs_timer.function = cs_timer_func;   //设置定时器超时函数

    init_timer(&cs_timer_finger_detec);  //初始化定时器
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing][timer]init cs_timer_finger_detec finish!\n");
    cs_timer_finger_detec.expires = jiffies+1;
    cs_timer_finger_detec.data = 60;    
    cs_timer_finger_detec.function = cs_finger_detec_timer_func; //设置定时器超时函数

    init_timer(&cs_timer_kick);           //初始化定时器
    //cs_debug(DEFAULT_DEBUG,"[timer]init cs_timer_kick finish!\n");
    cs_timer_kick.expires = jiffies+1;
    cs_timer_kick.data = 60;    
    cs_timer_kick.function = cs_timer_kick_func;//设置定时器超时函数

    init_timer(&cs_timer_long_louch);     //初始化定时器
    //cs_debug(DEFAULT_DEBUG,"[timer]init cs_timer_long_louch finish!\n");
    cs_timer_long_louch.expires = jiffies+LONGTOUCHTIME;
    cs_timer_long_louch.data = 60;    
    cs_timer_long_louch.function = cs_timer_long_touch_func; //设置定时器超时函数
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing][timer]add timer success!\n");
    return 0;
}

static void  cs_timer_exit (void)
{
    del_timer(&cs_timer);                //卸载模块时，删除定时器
    del_timer(&cs_timer_finger_detec);
    del_timer(&cs_timer_kick);
    del_timer(&cs_timer_long_louch);
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_timer_exit+\n");
}

static int __init cs_fp_init(void)
{
    int status;
    FUNC_ENTRY();
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]spi_register_board_info finished\n");
    status = spi_register_driver(&cs_fp_spi_driver);
    //cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_fp_init spi_register_driver state=%d\n ",status);
    if ((status < 0) || ( g_cs_id != 0xa062))
    {
        cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs register================\n");
        class_destroy(cs_fp_class);
        unregister_chrdev(CS_FP_MAJOR, DRIVER_NAME );
		if ( g_cs_id != 0xa062) {
		   printk("[ChipSailing] id not match, unregister driver\n");
		   spi_unregister_driver(&cs_fp_spi_driver);
		}
		return status;
    }
    cs_debug(DEFAULT_DEBUG,"[ChipSailing]cs_fp_init success\n");
    FUNC_EXIT();
    return status;
}

static void __exit cs_fp_exit(void)
{
    FUNC_ENTRY();
    spi_unregister_driver(&cs_fp_spi_driver);
    class_destroy(cs_fp_class);
    unregister_chrdev(CS_FP_MAJOR, DRIVER_NAME );

    free_irq(cs15xx->irq, cs15xx->spi);
    //gpio_free(cs15xx->gpio_reset);
    //gpio_free(cs15xx->gpio_irq);
    cs_timer_exit();

    FUNC_EXIT();
}

module_init(cs_fp_init);
module_exit(cs_fp_exit);
MODULE_AUTHOR("ChipSailing Technology");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cs15xx");
module_param(g_cs_bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(g_cs_bufsiz, "data bytes in biggest supported SPI message");

