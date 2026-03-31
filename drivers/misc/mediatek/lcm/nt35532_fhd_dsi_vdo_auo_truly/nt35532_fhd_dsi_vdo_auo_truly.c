#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_gpio.h>
#endif
//#include <cust_gpio_usage.h>
//#include <cust_i2c.h>


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)

#ifndef CONFIG_FPGA_EARLY_PORTING
//#define GPIO_65132_EN_P   GPIO_LCD_BIAS_ENP_PIN  //GPIO12      
//#define GPIO_65132_EN_N   GPIO_LCD_BIAS_ENN_PIN  //GPIO25		  

//#define GPIO_65132_EN GPIO_NFC_RST_PIN        
#endif


#define REGFLAG_DELAY             								0xFC
#define REGFLAG_UDELAY             								0xFB

#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW       								0xFE
#define REGFLAG_RESET_HIGH      								0xFF

//#define REGFLAG_PORT_SWAP									 0xFFFC
//#define REGFLAG_END_OF_TABLE      							 0xFFFD   // END OF REGISTERS MARKER



#define LCM_DSI_CMD_MODE									 0
#define LCM_ID_NT35532                                       0x80

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) \
		lcm_util.set_gpio_lcd_enn_bias(cmd)		


//Gate IC Driver


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
/*
//basic sequence
static struct LCM_setting_table lcm_initialization_setting1[] = {
	{0xFF,1,{0x00}},
	{0xD3,1,{0x08}},
	{0xD4,1,{0x0E}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 100, {}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 40, {}},
};
//video mode input (MIPI)=Normal Video mode input "60Hz<=>1Hz"
static struct LCM_setting_table lcm_initialization_setting2[] = {
	{0xFF,1,{0x05}},
	{0xFB,1,{0x01}},
	{0xAF,1,{0xC0}},
	{0xB0,1,{0x3A}},
	{0xB1,1,{0x2E}},
	{0xB2,1,{0x70}},
	{0xFF,1,{0x00}},
	{0xFB,1,{0x01}},
	{0xB0,1,{0x09}},
	{0xD3,1,{0x05}},
	{0xD4,1,0x0E},
	{0x11,0,{}},
	{REGFLAG_DELAY, 100, {}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 40, {}},
};
//30Hz<=>60Hz Input (MIPI) = 1 Frame(1/60sec)+1/60sec interval
static struct LCM_setting_table lcm_initialization_setting3[] = {
	{0xFF,1,{0x00}},//used to select page
	{0xFB,1,{0x01}},//used to select the control value of CMD2 Page0
	{0xAF,1,{0x40}},//3GAMMA_BLUE_NEGATIVE
	{0xB0,1,{0x3A}},//
	{0xB1,1,{0x16}},
	{0xB2,1,{0x70}},
	{0xFF,1,{0x00}},
	{0xFB,1,{0x01}},
	{0xB0,1,{0x19}},
	{0xD3,1,{0x06}},//0x08 //0x05
	{0xD4,1,{0x0E}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 100, {}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 40, {}},
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 17, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 100, {}},
	{0xFF,1,{0x05}},
	{0xFB,1,{0x01}},
	{0xD7,1,{0x30}},
	{0xD8,1,{0x70}},
};
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				if(table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;
				
			case REGFLAG_UDELAY :
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}
*/
#ifdef BUILD_LK
#define GPIO_LCD_ID1 59 // 61|0x80000000
#define GPIO_PCD_ID0  60 //62|0x80000000
static unsigned int lcm_compare_id(void)
{
//	int pin_lcd_id0=0;	
//	int pin_lcd_id1=0;
	
 //      pin_lcd_id1 = mt_get_gpio_in(GPIO_LCD_ID1);
//	pin_lcd_id0= mt_get_gpio_in(GPIO_PCD_ID0);
	
//#ifdef BUILD_LK
//	printf("%s, BOE, pin_lcd_id0= %d, pin_lcd_id1 = %d \n", __func__, pin_lcd_id0, pin_lcd_id1);
//#endif
//	if (pin_lcd_id1 == 0 && pin_lcd_id0 == 0){
//		return  1;  
//	}else{
//		return 0;
//	}
   	return 1;
}
#endif
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE;//BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM		    = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_DSI_FORMAT_RGB888;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 19; 
	params->dsi.vertical_frontporch					= 16; //16
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 30; //30
	params->dsi.horizontal_backporch				= 30; //30
	params->dsi.horizontal_frontporch				= 100; //20
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 482; //this value must be in MTK suggested table

	params->dsi.cont_clock=0;
	params->dsi.clk_lp_per_line_enable = 1;    //xsz modify no clk continue mode
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;    //xsz modify esd check mode:TE
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
	
}
//xieshizhao add begin
static void NT35532_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
	unsigned int data_array[16];
	data_array[0]=((0x00001500) | (para << 24) | (cmd<<16));
	
	dsi_set_cmdq(data_array, 1, 1);
}
//xieshizhao add end
#define NT35532_DCS_write_1A_0P(cmd)		data_array[0]=(0x00000500 | (cmd<<16)); \
											dsi_set_cmdq(data_array, 1, 1);

#if 0
static void NT35532_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
	unsigned int data_array[16];

	data_array[0] = (0x00023902);
	data_array[1] = (0x00000000 | (para << 8) | (cmd));
	dsi_set_cmdq(data_array, 2, 1);
}


#define NT35532_DCS_write_1A_0P(cmd)		data_array[0]=(0x00000500 | (cmd<<16)); \
											dsi_set_cmdq(data_array, 1, 1);
#endif
#if 0
static void NT35532_DCS_read_1A_1P(unsigned char cmd, unsigned char para)
{
	unsigned int data_array[16];
	char  buffer[3];

	data_array[0] = (0x00013700);
	
	//data_array[1] = (0x00000000 | (para << 8) | (cmd));
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);
	
	read_reg_v2(cmd, buffer, 1);
	printk("[xsz] [cmd:0x%02x] = 0x%02x\n", cmd,buffer[0]);
	
}
#endif

static void init_lcm_registers(void) 
{ 
   unsigned int data_array[16]; 


//xieshizhao modify for OTP 
NT35532_DCS_write_1A_1P(0xFF,0xEE); 
NT35532_DCS_write_1A_1P(0x18,0x40); 
MDELAY(10);		
NT35532_DCS_write_1A_1P(0x18,0x00); 
MDELAY(10);		
NT35532_DCS_write_1A_1P(0xFF,0x00); 



NT35532_DCS_write_1A_1P(0xFF,0x01); 
NT35532_DCS_write_1A_1P(0xFB,0x01); 
NT35532_DCS_write_1A_1P(0x6E,0x00);




NT35532_DCS_write_1A_1P(0xFF,0x00); 
NT35532_DCS_write_1A_1P(0xFB,0x01); 
//NT35532_DCS_write_1A_1P(0xD3,0x10); 
NT35532_DCS_write_1A_1P(0xD3,0x15); 
NT35532_DCS_write_1A_1P(0xD4,0x10); 
   
//NT35532_DCS_write_1A_1P(0xD5,0x18); 
//NT35532_DCS_write_1A_1P(0xD6,0xB8); 
NT35532_DCS_write_1A_1P(0xD5,0x44);  //fps
NT35532_DCS_write_1A_1P(0xD6,0x14);  //fps

NT35532_DCS_write_1A_1P(0xD7,0x00); 

NT35532_DCS_write_1A_1P(0x35,0x00);  //open TE

                                   
NT35532_DCS_write_1A_0P(0x11); 
MDELAY(120); 
NT35532_DCS_write_1A_0P(0x29); 
MDELAY(20); 

} 


#if 0
//test init_lcm_registers()
static void init_lcm_registers(void)
{
unsigned int data_array[16];
	data_array[0] = 0x00ff1500;
dsi_set_cmdq(data_array, 1, 1);
 
data_array[0] = 0x01fb1500;
dsi_set_cmdq(data_array, 1, 1);
 
data_array[0] = 0x00023902;
data_array[1] = 0x0000ff51;
dsi_set_cmdq(data_array, 2, 1);
 
data_array[0] = 0x00023902;
data_array[1] = 0x00002c53;
dsi_set_cmdq(data_array, 2, 1);
 
data_array[0] = 0x00023902;
data_array[1] = 0x00000055;
dsi_set_cmdq(data_array, 2, 1);
 
data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(200);
 
data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(120);

}
#endif

//#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN

static void lcm_init(void)
{    

#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
#else
	set_gpio_lcd_enp(1);
	MDELAY(5);
	set_gpio_lcd_enn(1);
#endif

	//mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
	
	MDELAY(5);

	SET_RESET_PIN(0);     //xieshizhao time sequence modify 2016.06.01
	MDELAY(10);
	
	SET_RESET_PIN(1);
	MDELAY(10);
	NT35532_DCS_write_1A_1P(0x4F,0x01);  // 4F:deep standby mode
	MDELAY(120);   
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);

#ifdef BUILD_LK
	dprintf(0, "[LK]lcm_initialization_setting----\n");    	
#else
	printk("[KERNEL]lcm_initialization_setting----\n");
#endif
	init_lcm_registers();
    	
}


static void lcm_suspend(void)
{	
	unsigned int array[16]; 



	   NT35532_DCS_write_1A_1P(0xFF,0x01);    
          NT35532_DCS_write_1A_1P(0xFB,0x01);   
          NT35532_DCS_write_1A_1P(0x0F,0xE0);   	
          NT35532_DCS_write_1A_1P(0x6E,0x80); 	
	NT35532_DCS_write_1A_1P(0xFF,0x00);   //xsz 20161109
          
    	array[0]=0x00280500;   //28:display off
	dsi_set_cmdq(array, 1, 1);
	MDELAY(40);

	array[0]=0x00100500;  //10:sleep in
	dsi_set_cmdq(array, 1, 1);
   MDELAY(120);

   NT35532_DCS_write_1A_1P(0x4F,0x01); 	// 4F:deep standby mode
	 MDELAY(20);
 

       set_gpio_lcd_enn(0);
	MDELAY(5);
	set_gpio_lcd_enp(0);

}

static void lcm_resume(void)
{
	 lcm_init();
}



static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}



LCM_DRIVER nt35532_fhd_dsi_vdo_auo_truly_lcm_drv = 
{
    .name			= "nt35532_fhd_dsi_vdo_auo_truly_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.init_power     = lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
	.resume         = lcm_resume,
	.suspend        = lcm_suspend,
};

