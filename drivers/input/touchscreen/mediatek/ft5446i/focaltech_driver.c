#include <linux/interrupt.h>
//#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
//dma
#include <linux/dma-mapping.h>

#include "tpd.h"

#ifdef CONFIG_MTK_BOOT
#include "mt_boot_common.h"
#endif

#include "focaltech_driver.h"
#ifdef CONFIG_CREATE_SYS_INTERFACE
#include "tpd_sys.h"
#endif
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12
//register define

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...) 			printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag			= 0;
static u8 tpd_proximity_flag_one		= 0; //add for tpd_proximity by wangdongfang
static u8 tpd_proximity_detect		= 1;//0-->close ; 1--> far away
#endif

#ifdef FOCAL_FTS_BLACK_GESTURE
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP			0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O			0x30
#define GESTURE_W			0x31
#define GESTURE_M			0x32
#define GESTURE_E			0x33
#define GESTURE_C			0x34
#define GESTURE_L			0x44
#define GESTURE_S			0x46
#define GESTURE_V			0x54
#define GESTURE_Z			0x41

#include "ft_gesture_lib.h"

#define FOCAL_FTS_BLACK_GESTURE_POINTS 255
#define FOCAL_FTS_BLACK_GESTURE_POINTS_ONETIME	62
#define FOCAL_FTS_BLACK_GESTURE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif

#ifdef FOCAL_TPD_ESD_PROTECT
#define TPD_ESD_CHECK_CIRCLE        						5000
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
//add for esd
static int count_irq = 0;
static unsigned long esd_check_circle = TPD_ESD_CHECK_CIRCLE;
static u8 run_check_91_register = 0;
static int g_esd_wq_enable = 0;
#endif
#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;

//for touch virtual key debug
static int touch_key_down[TPD_KEY_COUNT];
#endif

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct touch_info {
	int y[10];
	int x[10];
	int p[10];
	int id[10];
	int count;
};

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

extern struct tpd_device *tpd;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_rw_mutex);

struct Upgrade_Info fts_updateinfo[] =
{
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
};
struct Upgrade_Info fts_updateinfo_curr;

struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
static int tpd_flag = 0;
static int tpd_halt=0;
static int point_num = 0;
static int p_point_num = 0;
static int g_tpd_debug_point = 0;

#ifdef FOCAL_MSG_DMA_MODE
u8 *g_dma_buff_va = NULL;	 //
dma_addr_t g_dma_buff_pa = 0;	 //modify by mike.li for kernel-3.10 [2015.1.14]

static void msg_dma_release(void){
	if(g_dma_buff_va){
		dma_free_coherent(&tpd->dev->dev, 4096, g_dma_buff_va, g_dma_buff_pa);
		g_dma_buff_va = NULL;
		g_dma_buff_pa = 0;
		TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
}
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]	= TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]	   = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

static const struct i2c_device_id focal_tpd_id[] = {{"focaltech", 0}, {} };
static const struct of_device_id focal_dt_match[] = {
	{.compatible = "mediatek,focal_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(focal_dt_match),
		.name = "focaltech_drv",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = focal_tpd_id,
	.detect = tpd_detect,
};

int fts_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_mutex);

	// for DMA I2c transfer
	if(writelen!=0) {
		//DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		client->timing = 300;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen) {
			dev_err(&client->dev, "tpd %s i2c write failed. \n", __func__);
		}
		client->addr = client->addr & I2C_MASK_FLAG & (~ I2C_DMA_FLAG);
	}

	//DMA Read 
	if(readlen!=0) {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		client->timing = 300;
		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);
		memcpy(readbuf, g_dma_buff_va, readlen);
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}

	mutex_unlock(&i2c_rw_mutex);
	return ret;

}

/*write data by i2c*/
int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	//int i = 0;

	mutex_lock(&i2c_rw_mutex);

	//ret = i2c_master_send(client, writebuf, writelen);
	memcpy(g_dma_buff_va, writebuf, writelen);
	
	client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
	client->timing = 300;
	if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen) {
		dev_err(&client->dev, "tpd %s i2c write failed. \n", __func__);
	}

	client->addr = client->addr & I2C_MASK_FLAG & (~ I2C_DMA_FLAG);

	mutex_unlock(&i2c_rw_mutex);
	return ret;
}

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_Write(client, buf, sizeof(buf));
}

int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);
}
/*
static int tpd_get_tplcd_res(struct focal_chip_data *focal_data)
{    
	int tp_resx = 0, tp_resy = 0;

	//if(focal_data->tpd_fwversion < 0xc && focal_data->tpd_vendorid == 0x57) {
	//	tp_resx = 719;
	//	tp_resy = 1279;
	//} else {
	//	tp_resx = 1079;
	//	tp_resy = 1919;
	//}
	tp_resx = TPD_RES_X;
	tp_resy = TPD_RES_Y;
	focal_data->report_button = false;
	
	printk("tpd %s tp (%d, %d) lcd(%d, %d).\n", __func__, tp_resx, tp_resy, TPD_RES_X, TPD_RES_Y);
	focal_data->tp_resx = tp_resx;
	focal_data->tp_resy = tp_resy;
	focal_data->lcd_resx = TPD_RES_X;
	focal_data->lcd_resy = TPD_RES_Y;
	return 0;
}*/
#ifdef CONFIG_CREATE_SYS_INTERFACE
#ifdef CONFIG_PROC_FS
#define	TOUCH_PROC_FILE	"driver/tpd_touch"
static struct proc_dir_entry *tpd_debug_proc_entry;

static ssize_t proc_read_val(struct file *file,
	char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0;
	char data[800];
	//unsigned char uc_reg_value = 0;
	struct focal_chip_data *ts_data = i2c_get_clientdata(i2c_client);
	//int rst_gpio_dir, rst_gpio_pullen, rst_gpio_pullsel, rst_gpio_inver, rst_gpio_outval, rst_gpio_inval, rst_gpio_mode;
	//int int_gpio_dir, int_gpio_pullen, int_gpio_pullsel, int_gpio_inver, int_gpio_outval, int_gpio_inval, int_gpio_mode;

	len += sprintf(data + len, "Focal Touchscreen 20151012.\n");
	len += sprintf(data + len, "tpd fw version:0x%x manufacturer:0x%x.\n", ts_data->tpd_fwversion, ts_data->tpd_vendorid);
	len += sprintf(data + len, "tpd is suspend:%d , need stay awake:%d , bsg enable? :%d .\n", ts_data->tpd_suspend, ts_data->need_stay_awake, 
		ts_data->enable_wakeup_gesture);
	len += sprintf(data + len, "tp (%d, %d) lcd(%d, %d) report button (%d).\n", ts_data->tp_resx,  ts_data->tp_resy, 
		ts_data->lcd_resx, ts_data->lcd_resy, ts_data->report_button);
	len += sprintf(data + len, "tp (rst pin:%d, int pin:%d) (int irq:%d)", ts_data->tpd_rst_gpio_number, ts_data->tpd_int_gpio_number, 
		ts_data->touch_irq);
	/*
	rst_gpio_dir = mt_get_gpio_dir(GPIO_CTP_RST_PIN);
	rst_gpio_pullen = mt_get_gpio_pull_enable(GPIO_CTP_RST_PIN);
	rst_gpio_pullsel = mt_get_gpio_pull_select(GPIO_CTP_RST_PIN);
	rst_gpio_inver = mt_get_gpio_inversion(GPIO_CTP_RST_PIN);
	rst_gpio_outval = mt_get_gpio_out(GPIO_CTP_RST_PIN);
	rst_gpio_inval = mt_get_gpio_in(GPIO_CTP_RST_PIN);
	rst_gpio_mode = mt_get_gpio_dir(GPIO_CTP_RST_PIN);
	
	int_gpio_dir = mt_get_gpio_dir(GPIO_CTP_EINT_PIN);
	int_gpio_pullen = mt_get_gpio_pull_enable(GPIO_CTP_EINT_PIN);
	int_gpio_pullsel = mt_get_gpio_pull_select(GPIO_CTP_EINT_PIN);
	int_gpio_inver = mt_get_gpio_inversion(GPIO_CTP_EINT_PIN);
	int_gpio_outval = mt_get_gpio_out(GPIO_CTP_EINT_PIN);
	int_gpio_inval = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
	int_gpio_mode = mt_get_gpio_dir(GPIO_CTP_EINT_PIN);
	
	len += sprintf(data + len, "rst pin 0x%x, mode:%d, dir:%d, pullen:%d, sel:%d, inver:%d, outval:%d, in:%d.\n",
		GPIO_CTP_RST_PIN, rst_gpio_mode, rst_gpio_dir, rst_gpio_pullen, rst_gpio_pullsel, rst_gpio_inver, rst_gpio_outval, rst_gpio_inval);
	len += sprintf(data + len, "int pin 0x%x, mode:%d, dir:%d, pullen:%d, sel:%d, inver:%d, outval:%d, in:%d.\n",
		GPIO_CTP_EINT_PIN, int_gpio_mode, int_gpio_dir, int_gpio_pullen, int_gpio_pullsel, int_gpio_inver, int_gpio_outval, int_gpio_inval);
	*/
	//fts_read_reg(0xa5, &uc_reg_value);
	//len += sprintf(data + len, "Ft5x06 Touchscreen, workmode:%d.\n", uc_reg_value);
	
	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t proc_write_val(struct file *filp,
					 const char *buff, size_t len,
					 loff_t * off)
{
	return len;
}

static struct file_operations tpd_touch_proc_ops = {
	.read = proc_read_val,
	.write = proc_write_val,
};

static void create_tpd_debug_proc_entry(void *data)
{
	tpd_debug_proc_entry = proc_create(TOUCH_PROC_FILE, 0644, NULL, &tpd_touch_proc_ops);
	if (tpd_debug_proc_entry) {
		//tpd_debug_proc_entry->data = (void*)data;
		//g_data = data;
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
}
#endif
extern int tpd_fts_ctpm_firmware_upgrade(struct i2c_client *client, unsigned char * pbt_buf, unsigned int size, unsigned int force_upg);
extern int tpd_get_chipfw_version_etc(struct i2c_client *i2c_client, int * chipid_in_chip, 
	int *fwver_in_chip, int *vendorid_in_chip);
static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
	int fwver_in_chip = 0, vendorid_in_chip = 0, chipid_in_chip = 0;
	struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
	
	tpd_get_chipfw_version_etc(data->i2c_client, &chipid_in_chip, &fwver_in_chip, &vendorid_in_chip);
	
	data->tpd_fwversion = fwver_in_chip;
	data->tpd_vendorid = vendorid_in_chip;

	TPD_DMESG("%s Type:0x%x, Partner:0x%x, FwVersion:0x%x.\n", __func__, 
		chipid_in_chip, vendorid_in_chip, fwver_in_chip);

	strcpy(cdev->ic_tpinfo.tp_name, "Focal");
	cdev->ic_tpinfo.chip_model_id = 4;
	
	cdev->ic_tpinfo.chip_part_id= chipid_in_chip;
	cdev->ic_tpinfo.module_id= vendorid_in_chip;
	cdev->ic_tpinfo.chip_ver = 0;
	cdev->ic_tpinfo.firmware_ver= fwver_in_chip;
	cdev->ic_tpinfo.i2c_type = 0;
	cdev->ic_tpinfo.i2c_addr = i2c_client->addr;

	//tpd_get_tplcd_res(data);

	return 0;
}

static int tpd_compare_tp(struct tpd_classdev_t *cdev, unsigned char *buf)
{
	u8 uc_host_fm_ver;
	u8 uc_tp_fm_ver;
	int i_ret;
	int fwver_in_chip = 0, vendorid_in_chip = 0, chipid_in_chip = 0;
	u8 fwver_in_file = 0, vendorid_in_file = 0;
	struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;

	tpd_get_chipfw_version_etc(data->i2c_client, &chipid_in_chip, &fwver_in_chip, &vendorid_in_chip);

	fwver_in_file = buf[cdev->tp_fw.data_len- 2];
	vendorid_in_file = buf[cdev->tp_fw.data_len- 1];

	uc_tp_fm_ver = fwver_in_file;
	uc_host_fm_ver = fwver_in_chip;

	if((vendorid_in_chip == vendorid_in_file || vendorid_in_chip == FTS_REG_VENDOR_ID) && 
	(fwver_in_chip == FTS_REG_FW_VER || fwver_in_chip < fwver_in_file) ) {
		TPD_DMESG("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
		uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = 0;
	} else {
		TPD_DMESG("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
		uc_tp_fm_ver, uc_host_fm_ver);
		TPD_DMESG("[FTS] Not need upgrade firmware\n");
		i_ret = 0xff;
	}

	strcpy(tpd_fw_cdev.file_tpinfo.tp_name, tpd_fw_cdev.ic_tpinfo.tp_name);
	tpd_fw_cdev.file_tpinfo.module_id = vendorid_in_file;
	tpd_fw_cdev.file_tpinfo.firmware_ver = fwver_in_file;

	return i_ret;
}

static int tpd_flash_firmware(struct tpd_classdev_t *cdev, unsigned char * data, unsigned int size, int force_upg)
{
	char ret = -1;
	
#ifdef CONFIG_SYS_FIRMWARE_UPGRADE
	struct focal_chip_data *focal_data = (struct focal_chip_data*) cdev->private;

	printk("tpd %s function in. \n", __func__);
#ifdef FOCAL_TPD_ESD_PROTECT
	if(g_esd_wq_enable != 0) {
		cancel_delayed_work_sync(&gtp_esd_check_work);
	}
#endif
	focal_data->need_stay_awake = true;
	if(focal_data->tpd_suspend) {
		printk("tpd %s ts in suspend mode, wait 5 sec \n", __func__);
		msleep(5000);
		if(focal_data->tpd_suspend) {
			printk("tpd %s ts still in suspend mode, return. \n", __func__);
			ret = 0;
			goto out;
		}
	}
	printk("%s curr chipid:0x%x, chip part id:0x%x.\n", __func__, fts_updateinfo_curr.CHIP_ID, cdev->ic_tpinfo.chip_part_id);
	
	if(cdev->ic_tpinfo.chip_part_id != fts_updateinfo_curr.CHIP_ID) {
		int i = 0;
		for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
		{
			if(cdev->ic_tpinfo.chip_part_id == fts_updateinfo[i].CHIP_ID)
			{
				memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
				break;
			}
		}
	}
	disable_irq(focal_data->touch_irq);
	/*update firmware*/
	if(focal_data->tpd_fwversion == 0x00) {
		ret = tpd_fts_ctpm_firmware_upgrade(focal_data->i2c_client, data, size, 1);
	} else {
		ret = tpd_fts_ctpm_firmware_upgrade(focal_data->i2c_client, data, size, force_upg);
	}
	if( 0 != ret ) {
		printk("tpd: Update firmware failed!\n");
	} else {
		printk("tpd: Update firmware success!\n");
		ret = 0;
	}
	enable_irq(focal_data->touch_irq); 
	msleep(100);

	tpd_init_tpinfo(cdev);
#ifdef FOCAL_TPD_ESD_PROTECT
	if(g_esd_wq_enable != 0) {
		count_irq = 0;
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(TPD_ESD_CHECK_CIRCLE));
	}
#endif

out:
	focal_data->need_stay_awake = false;

	return ret;
#else
	ret = 0;

	return ret;
#endif
}

static int tpd_read_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	int i = 0;
	struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
	unsigned char regaddr = addr;

	ret = fts_i2c_Read(data->i2c_client, &regaddr, 1, buf, len);
	TPD_DMESG("Read from addr:0x%x val=", addr);
	for(i = 0; i < (len < 8? len : 8); i++)
	{
		TPD_DMESG("0x%x ", buf[i]);
	}
	TPD_DMESG("\n");
	
	return ret;
}

static int tpd_write_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
	unsigned char buffer[256];

	buffer[0] = (u8) addr;
	memcpy(&buffer[1], buf, len);
	ret = fts_i2c_Write(data->i2c_client, buffer, len + 1);
	
	return ret;
}

static int tpd_get_wakegesture(struct tpd_classdev_t *cdev)
{
	//struct synaptics_rmi4_data *ts = (struct synaptics_rmi4_data*) cdev->private;
	//TPD_DMESG("%s wakeup_gesture %d.\n", __func__, ts->wakeup_gesture);
	
	return 0;
}
static int tpd_enable_wakegesture(struct tpd_classdev_t *cdev, int enable)
{
	struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
	TPD_DMESG("%s previous val is:%d, current val is:%d.\n", __func__, data->enable_wakeup_gesture, enable);

	data->enable_wakeup_gesture = enable;
	
	return enable;
}

static int tpd_register_fw_class(struct focal_chip_data *data)
{
	tpd_fw_cdev.name = "touchscreen";
	tpd_fw_cdev.private = (void*)data;
	tpd_fw_cdev.flash_fw = tpd_flash_firmware;
	tpd_fw_cdev.read_block = tpd_read_block;
	tpd_fw_cdev.write_block = tpd_write_block;
	tpd_fw_cdev.compare_tp_version = tpd_compare_tp;
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
	//for black wakeup gesture.
	tpd_fw_cdev.get_gesture = tpd_get_wakegesture;
	tpd_fw_cdev.wake_gesture = tpd_enable_wakegesture;
	
	tpd_classdev_register(&(data->i2c_client->dev), &tpd_fw_cdev);
	tpd_init_tpinfo(&tpd_fw_cdev);

#ifdef CONFIG_PROC_FS
	create_tpd_debug_proc_entry(data);
#endif
	return 0;
}
#endif
void focaltech_get_upgrade_array(void)
{
	u8 chip_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(i2c_client,FTS_REG_CHIP_ID,1,&chip_id);

	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

static void tpd_down(int x, int y, int p)
{
	struct focal_chip_data *ts_data = i2c_get_clientdata(i2c_client);
	
	//if(x > TPD_RES_X) {
	//	TPD_DEBUG("warning: IC have sampled wrong value.\n");;
	//	return;
	//}
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//printk("tpd:D[%4d %4d %4d] ", x, y, p);
	/* track id Start 0 */
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	input_mt_sync(tpd->dev);
#ifdef TPD_HAVE_BUTTON 
#ifdef CONFIG_MTK_BOOT
	if (ts_data->report_button || FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {	 
		tpd_button(x, y, 1);  
	}
	
	//if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	//{
	//	//msleep(50);
	//	printk("D virtual key \n");
	//}
#endif
	//for touch virtual key debug
	if(y > TPD_RES_Y) {
		int i = 0;
		for(i = 0; i < TPD_KEY_COUNT; i ++) {
			if(touch_key_down[i] != 1 && (x >= tpd_keys_dim_local[i][0] - (tpd_keys_dim_local[i][2] / 2) &&
			    x <= tpd_keys_dim_local[i][0] + (tpd_keys_dim_local[i][2] / 2) ) ){
				printk("tpd virtual key : %d (%d, %d) down.\n", i, x, y);
				touch_key_down[i] = 1;
			}
		}
	}
#endif
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }
 
static void tpd_up(int x, int y,int *count)
{
	struct focal_chip_data *ts_data = i2c_get_clientdata(i2c_client);
	
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	//printk("U[%4d %4d %4d] ", x, y, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);
#ifdef CONFIG_MTK_BOOT
	if (tpd_dts_data.use_tpd_button) {
		if (ts_data->report_button ||FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {	
			tpd_button(x, y, 0); 
		}
	}
#ifdef TPD_HAVE_BUTTON 
	//for touch virtual key debug
	{
		int i = 0;
		for(i = 0; i < TPD_KEY_COUNT; i ++) {
			if(touch_key_down[i] == 1 ){
				printk("tpd virtual key : %d (%d, %d) up.\n", i, x, y);
				touch_key_down[i] = 0;
			}
		}
	}
#endif
#endif
 }
/*static void to_lcd_point(int *x, int *y)
{
	struct focal_chip_data *data = i2c_get_clientdata(i2c_client);
	if(0 == data->tp_resx || 0 == data->tp_resy) {
		return;
	}
	if(data->tp_resy < 1300 && data->lcd_resy < 1300 && *y > 1300) {
		*y = 2000;
	} else if(data->tp_resy < 1300 && data->lcd_resy > 1300 && *y > 1300) {
		*y = 2000;
	} else if(data->tp_resy > 1300 && data->lcd_resy < 1300 && *y > 1930) {
		*y = 2000;
	} else if(data->tp_resy > 1300 && data->lcd_resy > 1300 && *y > 1930) {
		*y = 2000;
	}  else {
		*x = (*x) * data->lcd_resx / data->tp_resx;
		*y = (*y) * data->lcd_resy / data->tp_resy;
	}
}*/
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[128] = {0};
	u16 high_byte,low_byte;
	u8 report_rate = 0, reg = 0;
	
	p_point_num = point_num;
	
	if (tpd_halt) {
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}

	report_rate = 0x00;
	reg = 0x00;
	//fts_i2c_Read(i2c_client, &reg, 1, data, 64);
	fts_i2c_Read(i2c_client, &reg, 1, data, 9);
	/*get the number of the touch points*/

	point_num= data[2] & 0x0f;
	if(point_num > 1) {
		reg = 0x9;
		fts_i2c_Read(i2c_client, &reg, 1, &data[9], 6 * point_num);
	}
	
	for(i = 0; i < point_num; i++) {
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
		cinfo->id[i] = data[3+6*i+2]>>4; //touch id
		/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;	
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;
		//to_lcd_point(&cinfo->x[i], &cinfo->y[i]);
		if((g_tpd_debug_point - 1) % 160 == 0) {
			printk("tpd:%d%d %d,%d\n", cinfo->id[i], cinfo->p[i], cinfo->x[i], cinfo->y[i]);
		}
	}

	//printk(" MIKE: tpd cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);
	return true;

};

#ifdef MT_PROTOCOL_B
/*
*report the point information
*/
static int fts_read_Touchdata(struct ts_event *pinfo)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	if (tpd_halt) {
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}

	ret = fts_i2c_Read(i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",__func__);
		return ret;
	}
	memset(pinfo, 0, sizeof(struct ts_event));
	
	pinfo->touch_point = 0;
	//printk("tpd  fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d \n", fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID);
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			pinfo->touch_point++;
		pinfo->au16_x[i] =
			(s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
			8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		pinfo->au16_y[i] =
			(s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
			8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		pinfo->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		pinfo->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	
	return 0;
}

 /*
 *report the point information
 */
static void fts_report_value(struct ts_event *data)
 {
	 struct ts_event *event = data;
	 int i = 0;
	 int up_point = 0;
 
	 for (i = 0; i < event->touch_point; i++) {
		 input_mt_slot(tpd->dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE,0x3f);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,0x05);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X,event->au16_x[i]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y,event->au16_y[i]);
			//printk("tpd D x[%d] =%d,y[%d]= %d",i,event->au16_x[i],i,event->au16_y[i]);
		} else {
			up_point++;
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
		}
	 }
 
	 if(event->touch_point == up_point)
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
	 else
		 input_report_key(tpd->dev, BTN_TOUCH, 1);
 
	 input_sync(tpd->dev);
	//printk("tpd D x =%d,y= %d",event->au16_x[0],event->au16_y[0]);
 }
#endif

#ifdef TPD_PROXIMITY
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;	 
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;
	
	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	printk("[proxi_fts]read: 999 0xb0's value is 0x%02X\n", state);

	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");	
	} else {
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
	}
	
	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_fts]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,

		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_fts]command = 0x%02X\n", command);		
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}					
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	return err;	
}
#endif

#ifdef FOCAL_FTS_BLACK_GESTURE
static void check_gesture(int gesture_id)
{
	unsigned int keycode = KEY_RESERVED;
	
	printk("tpd fts gesture_id==0x%x\n ", gesture_id);
	
	switch(gesture_id) {
	case GESTURE_LEFT:
		keycode = KEY_GESTURE_LEFT;
		printk("tpd fts gesture_id==0x%x, bsg slide left.\n ", gesture_id);
		break;
	case GESTURE_RIGHT:
		keycode = KEY_GESTURE_RIGHT;
		printk("tpd fts gesture_id==0x%x, bsg slide right.\n ", gesture_id);
		break;
	case GESTURE_UP:
		keycode = KEY_GESTURE_UP;
		printk("tpd fts gesture_id==0x%x, bsg slide up.\n ", gesture_id);
		break;
	case GESTURE_DOWN:
		keycode = KEY_GESTURE_DOWN;
		printk("tpd fts gesture_id==0x%x, bsg slide down.\n ", gesture_id);
		break;
	case GESTURE_DOUBLECLICK:
		keycode = KEY_GESTURE_DOUBLE_CLICK;
		printk("tpd fts gesture_id==0x%x, bsg double click.\n ", gesture_id);
		break;
	case GESTURE_O:
		keycode = KEY_GESTURE_O;
		printk("tpd fts gesture_id==0x%x, bsg 'o'.\n ", gesture_id);
		break;
	case GESTURE_W:
		keycode = KEY_GESTURE_W;
		printk("tpd fts gesture_id==0x%x, bsg 'w'.\n ", gesture_id);
		break;
	case GESTURE_M:
		keycode = KEY_GESTURE_M;
		printk("tpd fts gesture_id==0x%x, bsg 'm'.\n ", gesture_id);
		break;
	case GESTURE_C:
		keycode = KEY_GESTURE_C;
		printk("tpd fts gesture_id==0x%x, bsg 'c'.\n ", gesture_id);
		break;
	case GESTURE_E:
		keycode = KEY_GESTURE_E;
		printk("tpd fts gesture_id==0x%x, bsg 'e'.\n ", gesture_id);
		break;
	case GESTURE_L:
		keycode = KEY_GESTURE_L;
		printk("tpd fts gesture_id==0x%x, bsg 'l'.\n ", gesture_id);
		break;
	case GESTURE_S:
		keycode = KEY_GESTURE_S;
		printk("tpd fts gesture_id==0x%x, bsg 's'.\n ", gesture_id);
		break;
	case GESTURE_V:
		keycode = KEY_GESTURE_V;
		printk("tpd fts gesture_id==0x%x, bsg 'v'.\n ", gesture_id);
		break;
	case GESTURE_Z:
		keycode = KEY_GESTURE_Z;
		printk("tpd fts gesture_id==0x%x, bsg 'z'.\n ", gesture_id);
		break;
	default:
		keycode = KEY_RESERVED;
		printk("tpd fts gesture_id==0x%x, bsg is not defined.\n ", gesture_id);
		break;
	}
	input_report_key(tpd->dev, keycode, 1);
	input_sync(tpd->dev);
	input_report_key(tpd->dev, keycode, 0);
	input_sync(tpd->dev);
}

static int fts_read_Gestruedata(void)
{
	unsigned char buf[FOCAL_FTS_BLACK_GESTURE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	int gestrue_id = 0;
	short pointnum = 0;

	buf[0] = 0xd3;

	printk("tpd fts %s fun in .\n", __func__);

	pointnum = 0;
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, FOCAL_FTS_BLACK_GESTURE_POINTS_HEADER);
	//printk( "tpd read FOCAL_FTS_BLACK_GESTURE_POINTS_HEADER.\n");
	if (ret < 0) {
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	/* FW */
	 if (fts_updateinfo_curr.CHIP_ID==0x54) {
		gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;
	 
		 if((pointnum * 4 + 8)<255) {
			ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
			printk("tpd fts %s value 0x%x 0x%x .\n", __func__, buf[0], buf[1]);
		 } else {
			ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
			printk("tpd fts %s value 0x%x 0x%x .\n", __func__, buf[0], buf[1]);
			ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
			printk("tpd fts %s value 0x%x 0x%x .\n", __func__, buf[0], buf[1]);
		 }
		 if (ret < 0) {
			   printk( "%s read touchdata failed.\n", __func__);
			   return ret;
		 }
		 check_gesture(gestrue_id);
		 for(i = 0;i < pointnum;i++)	{
			coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
				8 | (((s16) buf[1 + (4 * i)])& 0xFF);
			coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
				8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
		 }
		return -1;
	 }

	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		check_gesture(gestrue_id);
		printk( "tpd %d check_gesture gestrue_id.\n", gestrue_id);
		return -1;
	}
	
	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;
	if((pointnum * 4 + 8)<255) {
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	} else {
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
		ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	}
	if (ret < 0) {
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	//gestrue_id = fetch_object_sample(buf, pointnum);
	//check_gesture(gestrue_id);
	printk( "tpd %d read gestrue_id.\n", gestrue_id);

	for(i = 0;i < pointnum;i++) {
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -1;
}
#endif

 static int touch_event_handler(void *unused)
 {
	struct touch_info cinfo, pinfo;
	int i=0;
#ifdef FOCAL_FTS_BLACK_GESTURE
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);
	u8 state;
#endif
#ifdef MT_PROTOCOL_B
	struct ts_event pevent;
	int ret = 0;
#endif
#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
#endif

	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
 
	do {
		//printk("MIKE: tpd touch_event_handler start !kthread_should_stop() = %d\n", !kthread_should_stop());
		//enable_irq(focal_data->touch_irq); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter, tpd_flag != 0);
		//printk("MIKE: tpd touch_event_handler step 2\n");				 
		tpd_flag = 0;

		set_current_state(TASK_RUNNING);
#ifdef FOCAL_FTS_BLACK_GESTURE
		if(focal_data->enable_wakeup_gesture && focal_data->tpd_suspend) {
			i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
			//printk("tpd fts_read_Gestruedata state=%d\n",state);
			if(state ==1)	{
				fts_read_Gestruedata();
				continue;
			}
		}
#endif
#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)	{

			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			TPD_PROXIMITY_DEBUG("proxi_fts 0xB0 state value is 1131 0x%02X\n", state);
			if(!(state&0x01))
			{
			tpd_enable_ps(1);
			}
			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			TPD_PROXIMITY_DEBUG("proxi_fts 0x01 value is 1139 0x%02X\n", proximity_status);
			if (proximity_status == 0xC0)
			{
			tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
			tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);
			if ((err = tpd_read_ps()))
			{
			TPD_PROXIMITY_DMESG("proxi_fts read ps data 1156: %d\n", err);	
			}
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			//{
			//	TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
			//}
		}  
#endif
	//printk("MIKE: tpd touch_event_handler step 3\n");

#ifdef MT_PROTOCOL_B
		{
			ret = fts_read_Touchdata(&pevent);
			if (ret == 0)
			fts_report_value(&pevent);
		}
#else
#endif
		{
			if (tpd_touchinfo(&cinfo, &pinfo)) 
			{
				//printk("MIKE: tpd point_num = %d step 3\n",point_num);
				//TPD_DEBUG_SET_TIME;
				if(point_num >0) 
				{
					for(i =0; i<point_num; i++)//only support 3 point
					{
						tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
					}
					input_sync(tpd->dev);
				} else {
					tpd_up(cinfo.x[0], cinfo.y[0],&cinfo.id[0]);
					//TPD_DEBUG("release --->\n");			   
					input_sync(tpd->dev);
				}
			}
		}
	}while(!kthread_should_stop());
	 return 0;
 }
 
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	//TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
#ifdef FOCAL_TPD_ESD_PROTECT
	count_irq ++;
#endif
	g_tpd_debug_point ++;
	wake_up_interruptible(&waiter);
	//printk("MIKE:TPD interrupt has been triggered flag = %d\n", tpd_flag);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);
	//struct device_node *node = NULL;
	int ret = 0;

	//node = of_find_matching_node(NULL, touch_of_match);
	if (focal_data->touch_irq) {
		/*focal_data->touch_irq = gpio_to_irq(focal_data->tpd_int_gpio_number);*/
		//focal_data->touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(focal_data->touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
			if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}

static int of_get_focal_platform_data(struct device *dev)
{
	int retval = -1;
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);
	
	/*int ret, num;*/
	if (dev->of_node) {
		const struct of_device_id *match;

		TPD_DMESG("warnning %s: of_node null, found it.\n", __func__);

		match = of_match_device(of_match_ptr(focal_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
	focal_data->avdd_regulator = regulator_get(dev, "avdd_regulator");
	retval = regulator_set_voltage(focal_data->avdd_regulator, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp1 voltage: %d\n", retval);
		return -1;
	}
	focal_data->touch_irq = irq_of_parse_and_map(dev->of_node, 0);
	focal_data->tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
	focal_data->tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);

	TPD_DMESG("rst_gpio_number %d\n", focal_data->tpd_rst_gpio_number);
	TPD_DMESG("int_gpio_number %d\n", focal_data->tpd_int_gpio_number);
	TPD_DMESG("touch_irq %d\n", focal_data->touch_irq);
	return 0;
}
 
void fts_reset_tp(int HighOrLow)
{
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);
	
	if(HighOrLow) {
		gpio_direction_output(focal_data->tpd_rst_gpio_number, 1);
	} else {
		gpio_direction_output(focal_data->tpd_rst_gpio_number, 0);
	}
}

#ifdef FOCAL_TPD_ESD_PROTECT
/************************************************************************
* Name: force_reset_guitar
* Brief: reset
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void force_reset_guitar(void)
{
	//s32 i;
	//s32 ret;
	int retval = -1;
	bool b_success = false; 
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);

	TPD_DMESG("%s fun in.\n", __func__);

	gpio_direction_output(focal_data->tpd_rst_gpio_number, 0);
	retval = regulator_disable(focal_data->avdd_regulator);
	if (retval != 0) {
		TPD_DMESG("Failed to disable reg-vgp1: %d try again.\n", retval);
		regulator_disable(focal_data->avdd_regulator);
	}
	msleep(15);
	retval = regulator_enable(focal_data->avdd_regulator);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp1: %d\n", retval);
	msleep(15);
	gpio_direction_output(focal_data->tpd_rst_gpio_number, 1);
	msleep(300);
	
#ifdef TPD_PROXIMITY
	if (FT_PROXIMITY_ENABLE == tpd_proximity_flag) 
	{
		tpd_enable_ps(FT_PROXIMITY_ENABLE);
	}
#endif
}
//0 for no apk upgrade, 1 for apk upgrade
extern int apk_debug_flag; 
#define A3_REG_VALUE								0x54
#define RESET_91_REGVALUE_SAMECOUNT 				5
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count = 0;
/************************************************************************
* Name: gtp_esd_check_func
* Brief: esd check function
* Input: struct work_struct
* Output: no
* Return: 0
***********************************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	int i;
	int ret = -1;
	u8 data, data_old;
	u8 flag_error = 0;
	int reset_flag = 0;
	u8 check_91_reg_flag = 0;

	TPD_DMESG("fts esd check fun in.\n");

	if (tpd_halt ) 
	{
		return;
	}
	if(apk_debug_flag) 
	{
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(esd_check_circle));
		return;
	}

	run_check_91_register = 0;
	for (i = 0; i < 3; i++) 
	{
		//ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
		ret = fts_read_reg(i2c_client, 0xA3,&data);
		if (ret<0) 
		{
			printk("FTS_ESD [Touch] read value fail");
			//return ret;
		}
		if (ret==1 && A3_REG_VALUE==data)
		{
		    break;
		}
	}

	if (i >= 3) 
	{
		force_reset_guitar();
		printk("FTS_ESD --tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
		reset_flag = 1;
		goto FOCAL_RESET_A3_REGISTER;
	}

	//esd check for count
  	//ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x8F, 1, &data);
	ret = fts_read_reg(i2c_client, 0x8F,&data);
	if (ret<0) 
	{
		printk("FTS_ESD [Touch] read value fail");
		//return ret;
	}
	//printk("FTS_ESD 0x8F:%d, count_irq is %d\n", data, count_irq);
			
	flag_error = 0;
	if((count_irq - data) > 10) 
	{
		if((data+200) > (count_irq+10) )
		{
			flag_error = 1;
		}
	}
	
	if((data - count_irq ) > 10) 
	{
		flag_error = 1;		
	}
		
	if(1 == flag_error) 
	{	
		printk("FTS_ESD --tpd reset.1 == flag_error...data=%d	count_irq %d . \n ", data, count_irq);
	    	force_reset_guitar();
		reset_flag = 1;
		goto FOCAL_RESET_INT;
	}

	run_check_91_register = 1;
	//ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0x91, 1, &data);
	ret = fts_read_reg(i2c_client, 0x91,&data);
	if (ret<0) 
	{
		printk("FTS_ESD [Touch] read value fail");
		//return ret;
	}
	//printk("FTS_ESD ---------91 register value = 0x%02x	old value = 0x%02x\n",	data, g_old_91_Reg_Value);
	if(0x01 == g_first_read_91) 
	{
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	} 
	else 
	{
		if(g_old_91_Reg_Value == data)
		{
			g_91value_same_count++;
			printk("FTS_ESD  91 value ==============, g_91value_same_count=%d\n", g_91value_same_count);
			if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) 
			{
				force_reset_guitar();
				printk("FTS_ESD --tpd reset. g_91value_same_count = 5\n");
				g_91value_same_count = 0;
				reset_flag = 1;
			}
			
			//run_check_91_register = 1;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
			g_old_91_Reg_Value = data;
		} 
		else 
		{
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
			//run_check_91_register = 0;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE;
		}
	}
FOCAL_RESET_INT:
FOCAL_RESET_A3_REGISTER:
	count_irq=0;
	data=0;
	//fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
	ret = fts_write_reg(i2c_client, 0x8F,data);
	if (ret<0) 
	{
		printk("FTS_ESD [Touch] write value fail");
		//return ret;
	}

	ret = fts_read_reg(i2c_client, 0x8F,&data);
	//printk("FTS_ESD [Touch] read 0x8F value : 0x%x \n", data);
	
	if(0 == run_check_91_register)
	{
		g_91value_same_count = 0;
	}
#ifdef TPD_PROXIMITY
	if( (1 == reset_flag) && ( FT_PROXIMITY_ENABLE == tpd_proximity_flag) )
	{
		if((tpd_enable_ps(FT_PROXIMITY_ENABLE) != 0))
		{
			APS_ERR("enable ps fail\n"); 
			return -1;
		}
	}
	//end esd check for count
#endif
    	if (!tpd_halt)
    	{
        	//queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(TPD_ESD_CHECK_CIRCLE));
        	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(esd_check_circle));
    	}

    	return;
}
#endif
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	strcpy(info->type, TPD_DEVICE);	
	return 0;
 }

 static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	struct focal_chip_data *focal_data;
	
#ifdef TPD_PROXIMITY
	int err;
	struct hwmsen_object obj_ps;
#endif
	printk("%s: in.\n", __func__);

	client->addr = 0x38;

	focal_data = kzalloc(sizeof(*focal_data), GFP_KERNEL);
	if (!focal_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for focal_data\n",
				__func__);
		return -ENOMEM;
	}
	focal_data->i2c_client = client;
	i2c_set_clientdata(client, focal_data);
	i2c_client = client;

#ifdef FOCAL_MSG_DMA_MODE
	//add by mike.li for kernel-3.10 [2015.1.15]
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 4096, &g_dma_buff_pa, GFP_KERNEL);
	if(!g_dma_buff_va){
		TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	}
	memset(g_dma_buff_va, 0, 4096);
#endif

	of_get_focal_platform_data(&client->dev);

	/* configure the gpio pins */
	retval = gpio_request_one(focal_data->tpd_rst_gpio_number, GPIOF_OUT_INIT_LOW,
				 "touchp_reset");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio reset_pin\n");
		return -1;
	}
	retval = gpio_request_one(focal_data->tpd_int_gpio_number, GPIOF_IN,
				 "tpd_int");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio int_pin\n");
		gpio_free(focal_data->tpd_rst_gpio_number);
		return -1;
	}
	
	gpio_direction_output(focal_data->tpd_rst_gpio_number, 0);
	mdelay(5);
	if(focal_data->avdd_regulator) {
		retval = regulator_enable(focal_data->avdd_regulator);
		if (retval != 0)
			TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	}
	mdelay(5);
	gpio_direction_output(focal_data->tpd_rst_gpio_number, 1);
	
	/* set INT mode */
	gpio_direction_input(focal_data->tpd_int_gpio_number);
	msleep(200);
	
	uc_reg_addr = FTS_REG_POINT_RATE;				
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	printk("MIKE: mtk_tpd[FTS] report rate is %dHz.\n",uc_reg_value * 10);

	uc_reg_addr = FTS_REG_FW_VER;
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FTS_REG_CHIP_ID;
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	retval=fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] chip id is %d.\n",uc_reg_value);
	if(retval < 0) {
		printk("mtk_tpd[FTS] Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
		msg_dma_release();
		goto error_i2c_failed;
	}

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) { 
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	tpd_irq_registration();

	tpd_load_status = 1;

#ifdef CONFIG_CREATE_SYS_INTERFACE
	tpd_register_fw_class(focal_data);
#endif
	//tpd_get_tplcd_res(focal_data);

	focaltech_get_upgrade_array();
#ifdef SYSFS_DEBUG
	fts_create_sysfs(i2c_client);
#endif
#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
			__func__);
#endif

#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(i2c_client);
#endif

#ifdef TPD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(i2c_client);
#endif
#ifdef CONFIG_WIND_DEVICE_INFO
	{
		extern u16 g_ctp_fwvr; 
		extern u16 g_ctp_vendor;
		extern char g_ctp_id_str[21];

		uc_reg_addr = FTS_REG_FW_VER;
		fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
		fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
		g_ctp_fwvr = uc_reg_value;
		memcpy(&g_ctp_id_str, &fts_updateinfo_curr.FTS_NAME, 20);
	}
#endif
#ifdef TPD_PROXIMITY
	{
		obj_ps.polling = 1; //0--interrupt mode;1--polling mode;
		obj_ps.sensor_operate = tpd_ps_operate;
		if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps))) {
			TPD_DEBUG("hwmsen attach fail, return:%d.", err);
		}
	}
#endif
#ifdef FOCAL_TPD_ESD_PROTECT
	uc_reg_addr = 0x90;				
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	g_esd_wq_enable = 0;
	if(uc_reg_value == 0xA5) {
		printk("tpd: %s esd check fun start.\n", __func__);
		g_esd_wq_enable = 1;
		INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
		gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(TPD_ESD_CHECK_CIRCLE));
	} else {
		printk("tpd: %s not need esd check fun.\n", __func__);
	}
#endif
#ifdef FOCAL_FTS_BLACK_GESTURE
	//init_para(480,854,60,0,0);
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);    //FOR wakeup TEST
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_C); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_Z);
		
	__set_bit(KEY_GESTURE_RIGHT, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_UP, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_U, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_O, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_E, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_M, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_W, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_L, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_S, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_V, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_Z, tpd->dev->keybit);

	focal_data->need_stay_awake = false;
	focal_data->enable_wakeup_gesture = false;
	focal_data->tpd_suspend = false;
#endif
#ifdef MT_PROTOCOL_B
	input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

	printk("MIKE: fts Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

	tpd_load_status = 1;
	
	return 0;

error_i2c_failed:
	kfree(focal_data);

	return -1;
}

 static int tpd_remove(struct i2c_client *client)
 {
	 msg_dma_release();

#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	fts_release_sysfs(client);
#endif
#ifdef FOCAL_TPD_ESD_PROTECT
	if(g_esd_wq_enable != 0) {
		destroy_workqueue(gtp_esd_check_workqueue);
	}
#endif
#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif

	 TPD_DEBUG("TPD removed\n");
 
   return 0;
}

static int tpd_local_init(void)
{
	//TPD_DMESG("Focaltech fts I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	TPD_DMESG("Focaltech fts I2C Touchscreen Driver %s.\n", __func__);

	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("fts unable to add i2c driver.\n");
		return -1;
	}
	if(tpd_load_status == 0) {
		TPD_DMESG("fts add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	
#ifdef TPD_HAVE_BUTTON	   
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))	  
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif	
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);	
	tpd_type_cap = 1;
	
	return 0; 
}

static void tpd_resume(struct device *h)
{
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);
	
	TPD_DMESG("TPD wake up\n");
	if(focal_data->need_stay_awake) {
		printk("%s need_stay_awake return.\n", __func__);
		focal_data->tpd_suspend = false;
		return;
	}
	focal_data->tpd_suspend = false;
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1) {
		if(tpd_proximity_flag_one == 1) {
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	

#ifdef FOCAL_FTS_BLACK_GESTURE
	if(focal_data->enable_wakeup_gesture) {
		TPD_DMESG("TPD exit black gesture wakeup mode. \n");
		fts_write_reg(i2c_client,0xD0,0x00);
	}
#endif

#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	regulator_enable(tpd->reg);
	msleep(30);
	enable_irq(focal_data->touch_irq);
#else
	gpio_direction_output(focal_data->tpd_rst_gpio_number, 0);
	//tpd_gpio_output(0, 0);
	msleep(15);
	//tpd_gpio_output(0, 1);
	gpio_direction_output(focal_data->tpd_rst_gpio_number, 1);
	msleep(30);
	enable_irq(focal_data->touch_irq);
#endif

	tpd_halt = 0;
	
#ifdef FOCAL_TPD_ESD_PROTECT
	count_irq = 0;
	if(g_esd_wq_enable != 0) {
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(TPD_ESD_CHECK_CIRCLE));
	}
#endif

	TPD_DMESG("TPD wake up done\n");
}

static void tpd_suspend(struct device *h)
{
	unsigned char data = 0x3;
	struct focal_chip_data *focal_data = i2c_get_clientdata(i2c_client);

	TPD_DMESG("TPD enter sleep\n");

	if(focal_data->need_stay_awake) {
		printk("%s need_stay_awake return.\n", __func__);
		return;
	}
	focal_data->tpd_suspend = true;
	
#ifdef FOCAL_TPD_ESD_PROTECT
	if(g_esd_wq_enable != 0) {
		cancel_delayed_work_sync(&gtp_esd_check_work);
	}
#endif

#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif
#ifdef FOCAL_FTS_BLACK_GESTURE
	if(focal_data->enable_wakeup_gesture) {
		TPD_DMESG("TPD enter black gesture wakeup mode. \n");
		fts_write_reg(i2c_client, 0xd0, 0x01);
		if (fts_updateinfo_curr.CHIP_ID==0x54) {
			fts_write_reg(i2c_client, 0xd1, 0xff);
			fts_write_reg(i2c_client, 0xd2, 0xff);
			fts_write_reg(i2c_client, 0xd5, 0xff);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			fts_write_reg(i2c_client, 0xd7, 0xff);
			fts_write_reg(i2c_client, 0xd8, 0xff);
		}
		return;
	}
#endif

	tpd_halt = 1;

	disable_irq(focal_data->touch_irq);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	regulator_disable(tpd->reg);
#else
	data = 0x3;
	fts_write_reg(i2c_client, 0xA5, data);    //tp enter sleep mode
#endif
	TPD_DMESG("TPD enter sleep done\n");
} 

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "focaltech",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif	
};
 
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	TPD_DMESG("MediaTek focaltech touch panel driver init\n");

	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add focaltech driver failed\n");

	return 0;
}
/* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek focaltech touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}
 
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

