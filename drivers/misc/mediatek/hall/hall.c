#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/proc_fs.h>
#include <hwmsensor.h>

#define HALL_NAME 			"Hall"

#define HALL_TAG                  "[Hall] "
#define HALL_FUN(f)               printk(HALL_TAG"%s\n", __func__)
#define HALL_ERR(fmt, args...)    printk(HALL_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define HALL_LOG(fmt, args...)    printk(HALL_TAG fmt, ##args)

static unsigned int hall_irq=293;
static unsigned int hall_int_gpio=0;

static int hall_probe(struct platform_device *pdev);
static int hall_remove(struct platform_device *pdev);
    
#define HALL_CHIP_DETECT_META  _IOW('h', 0x01, int)
#define HALL_CHIP_GET_DATA    1   //_IOR('h', 0x02, int)

struct hall_data {
	struct platform_device *pdev;
	};
	
static struct wake_lock hall_wake_lock;
struct delayed_work hall_delayed_work;
static struct input_dev *hall_input_dev = NULL;

void hall_delayed_work_func(struct work_struct *work)
{
	int val= gpio_get_value(hall_int_gpio);
	
	HALL_LOG("%s(), val:%d(0:cover, 1:uncover)\n", __FUNCTION__, val);// 0 close, 1 open
	if(val == 1)
	{
		input_report_abs(hall_input_dev, ABS_DISTANCE, val+1);
		input_sync(hall_input_dev);
		irq_set_irq_type(hall_irq, IRQF_TRIGGER_LOW);
	}
	else if (val==0)
	{
		wake_lock_timeout(&hall_wake_lock, 2*HZ);
		input_report_abs(hall_input_dev, ABS_DISTANCE, val+1);
		input_sync(hall_input_dev);
		irq_set_irq_type(hall_irq, IRQF_TRIGGER_HIGH);
	}

	enable_irq(hall_irq);

	return;
}

static irqreturn_t hall_irq_handler(int irq, void *info)
{
	HALL_FUN();
	schedule_delayed_work(&hall_delayed_work, msecs_to_jiffies(10));
	disable_irq_nosync(hall_irq);
		
	return IRQ_HANDLED;
}

/*Device ATTR*/
static ssize_t hall_show_get_status(struct device_driver *ddri, char *buf)
{	
	int val;
	val = gpio_get_value(hall_int_gpio);

	return sprintf(buf, "%d\n", val);
;	
}
static DRIVER_ATTR(get_status, S_IRUGO,
		  hall_show_get_status, NULL);

static ssize_t hall_show_sensordevnum(struct device_driver *ddri, char *buf)
{	
	int ret=0, devnum=0;
	const char *devname = NULL;

	devname = dev_name(&hall_input_dev->dev);

	ret = sscanf(devname+5, "%d", &devnum);
	return sprintf(buf, "%d\n", devnum);
}
static DRIVER_ATTR(sensordevnum, S_IRUGO, 
	hall_show_sensordevnum, NULL);

static ssize_t hall_store_test(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = simple_strtoul(buf, NULL, 2);
	HALL_LOG("%s: test val = %d\n", __FUNCTION__, val);

	if(val==1){
		wake_lock_timeout(&hall_wake_lock, 2*HZ);
		input_report_abs(hall_input_dev, ABS_DISTANCE, val);
		input_sync(hall_input_dev);
	}   
	else if(val==0){
		input_report_abs(hall_input_dev, ABS_DISTANCE, val);
		input_sync(hall_input_dev);
	}		
	return count;
}
static DRIVER_ATTR(test, S_IWUSR | S_IRUGO,
	NULL, hall_store_test);

static struct driver_attribute *hall_attr_list[] = {
	&driver_attr_get_status,
	&driver_attr_sensordevnum,
	&driver_attr_test,
};
 
static int hall_create_attr(struct device_driver  *driver)
{
	int idx, err=0;
	int num = (int)(sizeof(hall_attr_list)/sizeof(hall_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for (idx=0;idx<num;idx++)
	{
		err = driver_create_file(driver, hall_attr_list[idx]);
		if (err<0)
		{
			HALL_ERR("driver_create_file(%s) fail, err= %d\n", hall_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return 0;
}
static int hall_open(struct inode *inode, struct file *file)
{
	HALL_FUN();
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int hall_release(struct inode *inode, struct file *file)
{
	HALL_FUN();
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long hall_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	char gpio_level=0;
	int ret = 0;

    switch(cmd)
    {
        case HALL_CHIP_GET_DATA:
            gpio_level = gpio_get_value(hall_int_gpio);

            HALL_LOG("%s, hall_get_data, gpio_level = %d\n",__func__, gpio_level);     

		if (copy_to_user((void __user *)arg, (const void *)&gpio_level, 1))
		{
			HALL_ERR("hall_get_data: copy_to_user failed\n");
			return -EFAULT;
		}
			
            break;

        default:
            HALL_ERR("%d ,hall_unlocked_ioctl cmd error\n",cmd);
            break;
    }

    return ret;
}
/*----------------------------------------------------------------------------*/
static struct file_operations hall_fops = {
	.owner = THIS_MODULE,
	.open = hall_open,
	.release = hall_release,
	.unlocked_ioctl = hall_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice hall_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HALL_NAME,
	.fops = &hall_fops,
};

static const struct of_device_id hall_match_table[] = {
	{ .compatible = "mediatek,hall",},
};
static struct platform_driver hall_driver = {
	.probe		= hall_probe,
	.remove		= hall_remove,
	.driver		= {
		.name	= HALL_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = hall_match_table,
	},
};
static int of_get_Hall_platform_data(void)
{
	struct device_node *node = NULL;
	u32 ints[2]={0,0};
	unsigned int debounce=0;
	

	node = of_find_compatible_node(NULL, NULL, "mediatek, HALL_1-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, 2);
		hall_int_gpio = ints[0];
		debounce = ints[1];
		gpio_set_debounce(hall_int_gpio, debounce);
		
		hall_irq = irq_of_parse_and_map(node, 0);
		if (hall_irq < 0) {
			HALL_ERR("hall request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		HALL_LOG("hall_irq:%d, hall_int_gpio=%d, debounce=%d\n", hall_irq, hall_int_gpio, debounce);
	}
	return 0;
}


static int hall_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct hall_data *data;
	
	HALL_LOG("begin:%s\n",__func__);

	data = kzalloc(sizeof(struct hall_data), GFP_KERNEL);
	if (!data) {
		HALL_ERR("Failed to allocate memory\n");
		ret = -ENOMEM;
		goto exit;
	}
	
	data->pdev=pdev;

     	HALL_LOG("~1:%s\n",__func__);

	of_get_Hall_platform_data();

	if (request_irq(hall_irq, hall_irq_handler, IRQF_TRIGGER_LOW,
		HALL_NAME, (void *)data->pdev)) {
		HALL_ERR("%s Could not allocate Hall_INT !\n", __func__);
	
		goto exit_kfree;
	}
	irq_set_irq_wake(hall_irq, 1);
	INIT_DELAYED_WORK(&hall_delayed_work, hall_delayed_work_func);
	
	HALL_LOG("~2:%s\n",__func__);
	
	ret = misc_register(&hall_device);
	if (ret) {
		HALL_ERR("hall misc_register fail\n");
		ret = -ENOMEM;
		goto exit_free_irq;
	}
	HALL_LOG("~3:%s\n",__func__);
	wake_lock_init(&hall_wake_lock, WAKE_LOCK_SUSPEND, HALL_NAME);

	HALL_LOG("~4:%s\n",__func__);

       hall_input_dev = input_allocate_device();
	if (hall_input_dev == NULL)
	{
		HALL_ERR("%s() input_allocate_device fail\n", __FUNCTION__);
		goto exit_unregister_misc;
	}
	
	hall_input_dev->name = HALL_NAME;
	hall_input_dev->id.bustype = BUS_VIRTUAL;
	set_bit(EV_ABS, hall_input_dev->evbit);
	input_set_capability(hall_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(hall_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	//input_set_drvdata(dev, cxt);	
	
	ret = input_register_device(hall_input_dev);
	if (ret)
	{
		HALL_LOG("%s(), input_register_device fail\n", __FUNCTION__);
		goto err_free_input_device;
	}

	HALL_LOG("~5:%s\n",__func__);

	ret = hall_create_attr(&(hall_driver.driver));
	if (ret)
		goto err_free_input_device;

	HALL_LOG("%s ,ok\n",__func__);
	return 0;

err_free_input_device:
	input_free_device(hall_input_dev);
exit_unregister_misc:
	misc_deregister(&hall_device);	
	wake_lock_destroy(&hall_wake_lock);
exit_free_irq:
	free_irq(hall_irq, pdev);	
exit_kfree:
	kfree(data);
exit:
	return ret;
}

static int hall_remove(struct platform_device *pdev)
{

	input_free_device(hall_input_dev);
	misc_deregister(&hall_device);	
	free_irq(hall_irq, pdev);	
	wake_lock_destroy(&hall_wake_lock);
	return 0;
}

#ifndef CONFIG_OF
static struct platform_device hall_platform_device = {
	.name = HALL_NAME,
	.id = -1
};
#endif

static int __init hall_init(void)
{
	HALL_FUN();

    #ifndef CONFIG_OF
    int ret=0;
	ret = platform_device_register(&hall_platform_device);
	if (ret)
		HALL_ERR("platform_device_register error %d\n", ret);

    #endif

	platform_driver_register(&hall_driver);
	return 0;
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);

