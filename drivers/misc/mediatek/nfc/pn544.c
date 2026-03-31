/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */ 

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

//#include <../mach/mt6735/include/mach/eint.h>
//#include <mach/mt_pm_ldo.h>
//#include <cust_eint.h>
//#include <cust_i2c.h>
//#include <cust_eint_md1.h>
//#include <mach/eint.h>
#include <linux/interrupt.h>

#include <linux/proc_fs.h> 
#include <pn544.h>

#define NFC_I2C_BUSNUM  		    3
#define I2C_NFC_SLAVE_7_BIT_ADDR	0x28
#define PN544_DRVNAME		"pn544"
static const struct i2c_device_id pn544_id[] = { { "pn544", 0 }, {} };

#define NFC_TAG                  "[PN544] "
#define NFC_FUN(f)               printk(NFC_TAG"%s\n", __func__)
#define NFC_ERR(fmt, args...)    printk(NFC_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define NFC_LOG(fmt, args...)    printk(NFC_TAG fmt, ##args)
#define NFC_DBG(fmt, args...)    printk(NFC_TAG fmt, ##args)

#define MAX_BUFFER_SIZE		256
//static struct i2c_board_info __initdata pn544_i2c_nfc = { I2C_BOARD_INFO(PN544_DRVNAME, I2C_NFC_SLAVE_7_BIT_ADDR)};

/******************************************************************************
 * extern functions
 *******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void); 

struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
    unsigned int 		ven_gpio;
    unsigned int 		firm_gpio;
    unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
};

static struct pn544_dev *p_pn544_dev = NULL;

static struct pn544_i2c_platform_data pn544_platform_data = {
    .irq_gpio = 56,   /* GPIO56 */
    .ven_gpio = 87,   /* GPIO87 */
    .firm_gpio = 86,  /* GPIO86 */
};

//For DMA
static char *I2CDMAWriteBuf = NULL;
static unsigned int I2CDMAWriteBuf_pa;
static char *I2CDMAReadBuf = NULL;
static unsigned int I2CDMAReadBuf_pa;

struct wake_lock pn544_timer_lock;
static unsigned int nfc_irq;

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	NFC_FUN();
	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		disable_irq_nosync(nfc_irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *info)
{
	struct pn544_dev *pn544_dev = p_pn544_dev;

	NFC_FUN();
	
	if (!gpio_get_value(pn544_platform_data.irq_gpio)) 
	{
		NFC_DBG( "NO IRQ\n");		
		return IRQ_NONE;
	}

	pn544_disable_irq(pn544_dev);

	wake_up_interruptible(&pn544_dev->read_wq);
	wake_lock_timeout(&pn544_timer_lock, 2*HZ);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	int ret;
	//char tmp[MAX_BUFFER_SIZE];

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	NFC_DBG("pn544 %s : reading %zu bytes.\n", __func__, count);
	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_platform_data.irq_gpio)) 
	{        
		NFC_DBG("pn544 read no event\n");		
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}
		
		NFC_DBG("pn544 read wait event\n");		
		pn544_dev->irq_enabled = true;
		enable_irq(nfc_irq);
	
		ret = wait_event_interruptible(pn544_dev->read_wq, gpio_get_value(pn544_platform_data.irq_gpio));


		if (ret) 
		{
			NFC_DBG("pn544 read wait event error\n");
			goto fail;
		}
		pn544_disable_irq(pn544_dev);
	}

	/* Read data */	
  ret = i2c_master_recv(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf_pa, count);
	   
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) 
	{
		pr_err("pn544 %s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		pr_err("pn544 %s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	
   if (copy_to_user(buf, I2CDMAReadBuf, ret)) 
   {
      NFC_DBG("%s : failed to copy to user space\n", __func__);
      return -EFAULT;
   }

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev;
	int ret,idx = 0;
	//char tmp[MAX_BUFFER_SIZE];

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*255)], count)) 
	{
		NFC_DBG("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}
	
	NFC_DBG("pn544 %s : writing %zu bytes.\n", __func__, count);
	/* Write data */
  ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
	
	if (ret != count) 
	{
		usleep_range(6000,10000);
        ret = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);
		NFC_DBG("[PN544_DEBUG],I2C Writer Retry,%d",ret);
	}
	if (ret != count) {
		NFC_DBG("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
//	printk("pn544 PC->IFD:");
//	for(i = 0; i < count; i++) 
//	{
//		printk(" %02X\n", I2CDMAWriteBuf[i]);
//	}
//	printk("\n");

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);

	filp->private_data = pn544_dev;
	
	pr_debug("pn544 %s : %d,%d\n", __func__, imajor(inode), iminor(inode));
    //hwPowerOn(MT6323_POWER_LDO_VCAMD,VOL_1800,"kd_camera_hw");
    //printk("pn544 power on vcamd_1800\n");  

	return 0;
}
static int pn544_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    NFC_FUN();
    //hwPowerDown(MT6323_POWER_LDO_VCAMD,"kd_camera_hw");
    //printk("pn544 power off vcamd_1800\n");		
    return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) 
	{
		case PN544_SET_PWR:
			if (arg == 2) {
				/* power on with firmware download (requires hw reset) */
				NFC_DBG("pn544 %s power on with firmware\n", __func__);
				gpio_set_value(pn544_platform_data.ven_gpio, 1);
				gpio_set_value(pn544_platform_data.firm_gpio, 1);				

				msleep(20);
				gpio_set_value(pn544_platform_data.ven_gpio, 0);
				msleep(60);
				gpio_set_value(pn544_platform_data.ven_gpio, 1);
				msleep(20);
			} else if (arg == 1) {
				/* power on */
				NFC_DBG("pn544 %s power on\n", __func__);
				gpio_set_value(pn544_platform_data.firm_gpio, 0);	
				gpio_set_value(pn544_platform_data.ven_gpio, 1);
				msleep(20);
			} else  if (arg == 0) {
				/* power off */
				NFC_DBG("pn544 %s power off\n", __func__);
				gpio_set_value(pn544_platform_data.firm_gpio, 0);	
				gpio_set_value(pn544_platform_data.ven_gpio, 0);
				msleep(60);
			} else {
				NFC_DBG("pn544 %s bad arg %lu\n", __func__, arg);
				return -EINVAL;
			}
			break;
		default:
			NFC_DBG("pn544 %s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = 
{
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.release = pn544_release,
	.unlocked_ioctl  = pn544_dev_ioctl,
	.compat_ioctl = pn544_dev_ioctl,
};

static int of_get_PN544_platform_data(struct device *dev)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek, NFC-eint");
	if (node) {
		//alsps_int_gpio_number = of_get_named_gpio(node, "int-gpio", 0);
		nfc_irq = irq_of_parse_and_map(node, 0);
		if (nfc_irq < 0) {
			NFC_DBG("nfc request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		NFC_DBG("nfc_irq : %d\n", nfc_irq);
	}
	return 0;
}

static ssize_t power_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int val;
	val=0;
	sscanf(buf,"%d",&val);
	NFC_DBG("input pn544 value:%d\n",val );
	if( val > 0 )
	{
		//hwPowerOn(MT6323_POWER_LDO_VCAMD,VOL_1800,"kd_camera_hw");
		//printk("pn544 power on vcamd_1800\n");	
	}
	else
	{
		//hwPowerDown(MT6323_POWER_LDO_VCAMD,"kd_camera_hw");
		//printk("pn544 power off vcamd_1800\n");		
	}
	return -EINVAL;
}

static struct file_operations p_fops= {
    .read = NULL,
    .write = power_write,
};

static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct pn544_dev *pn544_dev;

	NFC_DBG("pn544 nfc probe step01 is ok\n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("pn544 %s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	NFC_DBG("pn544 nfc probe step02 is ok\n");

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) 
	{
		dev_err(&client->dev, "pn544 failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	memset(pn544_dev, 0, sizeof(struct pn544_dev));
	p_pn544_dev = pn544_dev;

	NFC_DBG("pn544 nfc probe step03 is ok\n");

	of_get_PN544_platform_data(&client->dev);
	
	client->addr = (client->addr & I2C_MASK_FLAG);
	client->addr = (client->addr | I2C_DMA_FLAG);
	client->addr = (client->addr | I2C_DIRECTION_FLAG);
	client->timing = 400;
	pn544_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	wake_lock_init(&pn544_timer_lock, WAKE_LOCK_SUSPEND, "pn544 timer wakelock");
	
	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_DRVNAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	pn544_dev->firm_gpio = pn544_platform_data.firm_gpio;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) 
	{
		pr_err("pn544 %s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}
    
	NFC_DBG("pn544 nfc probe step04 is ok\n");
	
	/* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */    
#ifdef CONFIG_64BIT    
    I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
#else
    I2CDMAWriteBuf = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) 
	{
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		goto err_request_irq_failed;
	}
	
#ifdef CONFIG_64BIT 	
    I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
#else
    I2CDMAReadBuf = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) 
	{
		pr_err("%s : failed to allocate dma buffer\n", __func__);
		goto err_request_irq_failed;
	}
  NFC_DBG("%s :I2CDMAWriteBuf_pa %d, I2CDMAReadBuf_pa,%d\n", __func__, I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);

   /* EINT_gpio */
   	client->irq = nfc_irq;
	if (request_irq(nfc_irq, pn544_dev_irq_handler, IRQF_TRIGGER_NONE,
		PN544_DRVNAME, (void *)client)) {
		NFC_DBG("%s Could not allocate APDS9950_INT !\n", __func__);
	
		goto err_dma_alloc;
	}
	irq_set_irq_wake(client->irq, 1);
	//disable_irq_nosync(data->irq);

	NFC_DBG("pn544 nfc probe step05 is ok\n");

	pn544_dev->irq_enabled = true;

	NFC_DBG("%s : requesting IRQ %d\n", __func__, client->irq);
	i2c_set_clientdata(client, pn544_dev);
	
	NFC_DBG("pn544 nfc probe step06 is ok\n");
	proc_create("pn544", 0666, NULL, &p_fops);

    return 0;

err_dma_alloc:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
	p_pn544_dev = NULL;
err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);   
	return ret;
} 

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	if (I2CDMAWriteBuf)
	{
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
		I2CDMAWriteBuf = NULL;
		I2CDMAWriteBuf_pa = 0;
	}

	if (I2CDMAReadBuf)
	{
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
		I2CDMAReadBuf = NULL;
		I2CDMAReadBuf_pa = 0;
	}

	pn544_dev = i2c_get_clientdata(client);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
	p_pn544_dev = NULL;
	
	return 0;
}

//#ifdef CONFIG_OF
static const struct of_device_id nfc_of_match[] = {
	{ .compatible = "mediatek,NFC", },
	{},
};
//#endif

static struct i2c_driver pn544_driver = 
{
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= 
	{
		.owner	= THIS_MODULE,
		.name	= "pn544",
//#ifdef CONFIG_OF
		.of_match_table = nfc_of_match,
//#endif
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn544_dev_init(void)
{
	NFC_DBG("pn544 Loading pn544 driver\n");
	
	//i2c_register_board_info(NFC_I2C_BUSNUM, &pn544_i2c_nfc, 1);
	
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	NFC_DBG("pn544 Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
