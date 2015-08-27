#if 1
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
	
	#define Lcd_Log printf
#else
    #include <linux/string.h>
	#include <linux/kernel.h>
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
	
	#define Lcd_Log printk
#endif

#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      						0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID                                      0x980604
#define GPIO_LCD_ID_PIN GPIO_LCM_ID_PIN 

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
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                      lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
};


static struct LCM_setting_table lcm_initialization_setting[] = {

{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},   
{0x08,1,{0x10}},               
{0x21,1,{0x01}},               
{0x30,1,{0x02}},               
{0x31,1,{0x02}},               
{0x60,1,{0x07}},               
{0x61,1,{0x06}},               
{0x62,1,{0x06}},               
{0x63,1,{0x04}},               
{0x40,1,{0x14}},               
{0x41,1,{0x44}},               
{0x42,1,{0x01}},               
{0x43,1,{0x89}},               
{0x44,1,{0x89}},               
{0x45,1,{0x1B}},               
{0x46,1,{0x44}},               
{0x47,1,{0x44}},               
{0x50,1,{0x85}},               
{0x51,1,{0x85}},               
{0x52,1,{0x00}},               
{0x53,1,{0x64}},               
{0xA0,1,{0x00}}, 
{0xA1,1,{0x00}}, 
{0xA2,1,{0x03}}, 
{0xA3,1,{0x0E}}, 
{0xA4,1,{0x08}}, 
{0xA5,1,{0x1F}}, 
{0xA6,1,{0x0F}}, 
{0xA7,1,{0x0B}}, 
{0xA8,1,{0x03}}, 
{0xA9,1,{0x06}}, 
{0xAA,1,{0x05}}, 
{0xAB,1,{0x02}}, 
{0xAC,1,{0x0E}}, 
{0xAD,1,{0x25}}, 
{0xAE,1,{0x1D}}, 
{0xAF,1,{0x00}}, 
{0xC0,1,{0x00}}, 
{0xC1,1,{0x04}}, 
{0xC2,1,{0x0F}}, 
{0xC3,1,{0x10}}, 
{0xC4,1,{0x0B}}, 
{0xC5,1,{0x1E}}, 
{0xC6,1,{0x09}}, 
{0xC7,1,{0x0A}}, 
{0xC8,1,{0x00}}, 
{0xC9,1,{0x0A}}, 
{0xCA,1,{0x01}}, 
{0xCB,1,{0x06}}, 
{0xCC,1,{0x09}}, 
{0xCD,1,{0x2A}}, 
{0xCE,1,{0x28}},        
{0xCF,1,{0x00}},        
{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},                          
{0x00,1,{0xA0}},
{0x01,1,{0x05}},
{0x02,1,{0x00}},
{0x03,1,{0x00}},
{0x04,1,{0x01}},
{0x05,1,{0x01}},
{0x06,1,{0x88}},
{0x07,1,{0x04}},
{0x08,1,{0x01}},
{0x09,1,{0x90}},
{0x0A,1,{0x04}},
{0x0B,1,{0x01}},
{0x0C,1,{0x01}},
{0x0D,1,{0x01}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0x55}},
{0x11,1,{0x50}},
{0x12,1,{0x01}},
{0x13,1,{0x85}},
{0x14,1,{0x85}},
{0x15,1,{0xC0}},
{0x16,1,{0x0B}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x20,1,{0x01}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x01}},
{0x25,1,{0x23}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x02}},
{0x31,1,{0x22}},
{0x32,1,{0x11}},
{0x33,1,{0xAA}},
{0x34,1,{0xBB}},
{0x35,1,{0x66}},
{0x36,1,{0x00}},
{0x37,1,{0x22}},
{0x38,1,{0x22}},
{0x39,1,{0x22}},
{0x3A,1,{0x22}},
{0x3B,1,{0x22}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x3F,1,{0x22}},
{0x40,1,{0x22}},
{0x53,1,{0x1A}},
{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     
{0x18,1,{0x1D}},
{0x17,1,{0x12}},  
{0x02,1,{0x77}},
{0xE1,1,{0x79}},
{0x06,1,{0x13}},    
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},

       

//{0x35,1,{0x00}},//TE

{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,5,{}},

/*{0xFF, 5, {0xFF,0x98,0x06,0x04,0x01}},
	{0x08, 1, {0x10}},

	{0x21, 1, {0x01}},
	{0x30, 1, {0x02}},

	{0x31, 1, {0x02}},
	{0x40, 1, {0x15}},

	{0x41, 1, {0x44}},
	{0x42, 1, {0x03}},

	{0x43, 1, {0x89}},
	{0x44, 1, {0x82}},

	{0x50, 1, {0x78}},
	{0x51, 1, {0x78}},

	{0x52, 1, {0x00}},
	{0x53, 1, {0x59}},

	//{0x57, 1, {0x50}},
	{0x60, 1, {0x05}},

	{0x61, 1, {0x00}},
	{0x62, 1, {0x06}},

	{0x63, 1, {0x03}},
	{0xa0, 1, {0x00}},

	{0xa1, 1, {0x05}},
	{0xa2, 1, {0x0b}},

	{0xa3, 1, {0x10}},
	{0xa4, 1, {0x07}},

	{0xa5, 1, {0x1e}},
	{0xa6, 1, {0x0a}},

	{0xa7, 1, {0x09}},
	{0xa8, 1, {0x02}},

	{0xa9, 1, {0x09}},
	{0xaa, 1, {0x02}},

	{0xab, 1, {0x06}},
	{0xac, 1, {0x0d}},

	{0xad, 1, {0x35}},
	{0xae, 1, {0x31}},

	{0xaf, 1, {0x07}},
	{0xc0, 1, {0x00}},

	{0xc1, 1, {0x03}},
	{0xc2, 1, {0x0a}},

	{0xc3, 1, {0x0f}},
	{0xc4, 1, {0x08}},

	{0xc5, 1, {0x18}},
	{0xc6, 1, {0x0a}},

	{0xc7, 1, {0x09}},
	{0xc8, 1, {0x05}},

	{0xc9, 1, {0x09}},
	{0xca, 1, {0x06}},

	{0xcb, 1, {0x06}},
	{0xcc, 1, {0x0c}},

	{0xcd, 1, {0x28}},
	{0xce, 1, {0x24}},

	{0xcf, 1, {0x07}},
	{0xff, 5, {0xff,0x98,0x06,0x04,0x06}},

	{0x00, 1, {0x3c}},
	{0x01, 1, {0x06}},

	{0x02, 1, {0x00}},
	{0x03, 1, {0x00}},

	{0x04, 1, {0x0d}},
	{0x05, 1, {0x0d}},

	{0x06, 1, {0x80}},
	{0x07, 1, {0x04}},

	{0x08, 1, {0x03}},
	{0x09, 1, {0x00}},

	{0x0a, 1, {0x00}},
	{0x0b, 1, {0x00}},

	{0x0c, 1, {0x0d}},
	{0x0d, 1, {0x0d}},

	{0x0e, 1, {0x00}},
	{0x0f, 1, {0x00}},

	{0x10, 1, {0x50}},
	{0x11, 1, {0xd0}},

	{0x12, 1, {0x00}},
	{0x13, 1, {0x00}},

	{0x14, 1, {0x00}},
	{0x15, 1, {0xc0}},

	{0x16, 1, {0x08}},
	{0x17, 1, {0x00}},

	{0x18, 1, {0x00}},
	{0x19, 1, {0x00}},

	{0x1a, 1, {0x00}},
	{0x1b, 1, {0x00}},

	{0x1c, 1, {0x48}},
	{0x1d, 1, {0x00}},

	{0x20, 1, {0x10}},
	{0x21, 1, {0x23}},

	{0x22, 1, {0x45}},
	{0x23, 1, {0x67}},

	{0x24, 1, {0x01}},
	{0x25, 1, {0x23}},

	{0x26, 1, {0x45}},
	{0x27, 1, {0x67}},

	{0x30, 1, {0x13}},
	{0x31, 1, {0x22}},

	{0x32, 1, {0xdd}},
	{0x33, 1, {0xcc}},

	{0x34, 1, {0xbb}},
	{0x35, 1, {0xaa}},

	{0x36, 1, {0x22}},
	{0x37, 1, {0x26}},

	{0x38, 1, {0x72}},
	{0x39, 1, {0xff}},

	{0x3a, 1, {0x22}},
	{0x3b, 1, {0xee}},

	{0x3c, 1, {0x22}},
	{0x3d, 1, {0x22}},

	{0x3e, 1, {0x22}},
	{0x3f, 1, {0x22}},

	{0x40, 1, {0x22}},
	//{0x52, 1, {0x10}},
	//{0x53, 1, {0x10}},
	{0xff, 5, {0xff,0x98,0x06,0x04,0x07}},

	{0x17, 1, {0x22}},
	{0x06, 1, {0x06}},
	{0xff, 5, {0xff,0x98,0x06,0x04,0x00}},
	//{0x35, 1, {0x00}},//open te

	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 0, {0x00}},
        {REGFLAG_DELAY, 10, {}},*/
// Note
// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
// Setting ending by predefined flag
{REGFLAG_END_OF_TABLE,0x00,{}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
        {REGFLAG_DELAY, 120, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
        {REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


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

		//enable tearing-free
		params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED; 
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE 
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;//4
		params->dsi.vertical_backporch					= 20;//16
		params->dsi.vertical_frontporch 				= 20;//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		
		params->dsi.horizontal_sync_active			= 10;
		params->dsi.horizontal_backporch				= 60; //80
		params->dsi.horizontal_frontporch				= 200; //80
		params->dsi.horizontal_blanking_pixel		= 60; //
		params->dsi.horizontal_active_pixel 		= FRAME_WIDTH;

  	// Bit rate calculation
  	params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4  //1
  	params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4///1
  	params->dsi.fbk_div =15;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		//28
}

static void lcm_init(void)
{
		unsigned int data_array[64];

#if defined(BUILD_LK)
		upmu_set_rg_vgp2_vosel(5);
		upmu_set_rg_vgp2_en(1);
		
		upmu_set_rg_vgp3_vosel(3);
		upmu_set_rg_vgp3_en(1);	
#else
		hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
		hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
#endif

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(100);//Must > 120ms

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	Lcd_Log("junchao hengsheng LCD\n");
	lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}


static int get_lcd_id(void)
{
	mt_set_gpio_mode(GPIO_LCD_ID_PIN,0);
	mt_set_gpio_dir(GPIO_LCD_ID_PIN,0);
	mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN,1);
	mt_set_gpio_pull_select(GPIO_LCD_ID_PIN,0);
	MDELAY(1);
	
	return mt_get_gpio_in(GPIO_LCD_ID_PIN);
}

static unsigned int lcm_compare_id(void)
{
#if defined(BUILD_LK)
		upmu_set_rg_vgp2_vosel(5);
		upmu_set_rg_vgp2_en(1);
		
		upmu_set_rg_vgp3_vosel(3);
		upmu_set_rg_vgp3_en(1);	
#else
		hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
		hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
#endif
        int array[4];
        char buffer[1];
        char id_high=0;
        char id_midd=0;
        char id_low=0;
        int id=0;

        //Do reset here
        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(25);       
        SET_RESET_PIN(1);
        MDELAY(50);      
       
        array[0]=0x00063902;
        array[1]=0x0698ffff;
        array[2]=0x00000104;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);

        array[0]=0x00023700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.
    
        read_reg_v2(0x00, buffer,1);
        id_high = buffer[0]; ///////////////////////0x98

        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[0]; ///////////////////////0x06

        read_reg_v2(0x02, buffer,1);
        id_low = buffer[0]; ////////////////////////0x04

        id = (id_high << 16) | (id_midd << 8) | id_low;
        
        Lcd_Log("wqcat %s, 0x00=0x%x,0x01=0x%x,0x02=0x%x,id=0x%x\n", __func__, id_high,id_midd,id_low,id);
        return (LCM_ID == id)?1:0;
        //return (get_lcd_id()==1)?1:0;

        }


static unsigned int lcm_esd_recover()
{
	 lcm_init();
   //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

LCM_DRIVER ili9806e_hengsheng_HS040NH06D_BOE_dsi_2_vdo_wvga_lcm_drv = 
{
    .name			= "ili9806e_hengsheng_HS040NH06D_BOE_dsi_2_vdo_wvga",
	.set_util_funcs = lcm_set_util_funcs,
    .compare_id    = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
# else
#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
    #define LCM_DEBUG  printf
    #define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
    #define LCM_DEBUG  printk
    #define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (480)
#define FRAME_HEIGHT                                        (800)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_ili9806c	0x9806
#define LCM_DSI_CMD_MODE                                    0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static unsigned int lcm_exist_flag = FALSE; 
static unsigned int lcm_compare_id_flag = FALSE; 

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)            lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_initialization_setting[] = {

	{0xFF,5,{0xFF,98,06,04,01}},   
{0x08,1,{0x10}},               
{0x21,1,{0x01}},               
{0x30,1,{0x02}},               
{0x31,1,{0x02}},               
{0x60,1,{0x07}},               
{0x61,1,{0x06}},               
{0x62,1,{0x06}},               
{0x63,1,{0x04}},               
{0x40,1,{0x14}},               
{0x41,1,{0x44}},               
{0x42,1,{0x01}},               
{0x43,1,{0x89}},               
{0x44,1,{0x89}},               
{0x45,1,{0x1B}},               
{0x46,1,{0x44}},               
{0x47,1,{0x44}},               
{0x50,1,{0x85}},               
{0x51,1,{0x85}},               
{0x52,1,{0x00}},               
{0x53,1,{0x64}},               
{0xA0,1,{0x00}}, 
{0xA1,1,{0x00}}, 
{0xA2,1,{0x01}}, 
{0xA3,1,{0x0A}}, 
{0xA4,1,{0x03}}, 
{0xA5,1,{0x1B}}, 
{0xA6,1,{0x0C}}, 
{0xA7,1,{0x0A}}, 
{0xA8,1,{0x05}}, 
{0xA9,1,{0x06}}, 
{0xAA,1,{0x03}}, 
{0xAB,1,{0x00}}, 
{0xAC,1,{0x12}}, 
{0xAD,1,{0x21}}, 
{0xAE,1,{0x1A}}, 
{0xAF,1,{0x00}}, 
{0xC0,1,{0x00}}, 
{0xC1,1,{0x01}}, 
{0xC2,1,{0x08}}, 
{0xC3,1,{0x0B}}, 
{0xC4,1,{0x08}}, 
{0xC5,1,{0x1B}}, 
{0xC6,1,{0x0A}}, 
{0xC7,1,{0x0B}}, 
{0xC8,1,{0x00}}, 
{0xC9,1,{0x0C}}, 
{0xCA,1,{0x06}}, 
{0xCB,1,{0x06}}, 
{0xCC,1,{0x09}}, 
{0xCD,1,{0x28}}, 
{0xCE,1,{0x24}},        
{0xCF,1,{0x00}},        
{0xFF,5,{0xFF,98,06,04,06}},                          
{0x00,1,{0xA0}},
{0x01,1,{0x05}},
{0x02,1,{0x00}},
{0x03,1,{0x00}},
{0x04,1,{0x01}},
{0x05,1,{0x01}},
{0x06,1,{0x88}},
{0x07,1,{0x04}},
{0x08,1,{0x01}},
{0x09,1,{0x90}},
{0x0A,1,{0x04}},
{0x0B,1,{0x01}},
{0x0C,1,{0x01}},
{0x0D,1,{0x01}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0x55}},
{0x11,1,{0x50}},
{0x12,1,{0x01}},
{0x13,1,{0x85}},
{0x14,1,{0x85}},
{0x15,1,{0xC0}},
{0x16,1,{0x0B}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x20,1,{0x01}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x01}},
{0x25,1,{0x23}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x02}},
{0x31,1,{0x22}},
{0x32,1,{0x11}},
{0x33,1,{0xAA}},
{0x34,1,{0xBB}},
{0x35,1,{0x66}},
{0x36,1,{0x00}},
{0x37,1,{0x22}},
{0x38,1,{0x22}},
{0x39,1,{0x22}},
{0x3A,1,{0x22}},
{0x3B,1,{0x22}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x3F,1,{0x22}},
{0x40,1,{0x22}},
{0x53,1,{0x1A}},
{0xFF,5,{0xFF,98,06,04,07}},     
{0x18,1,{0x1D}},
{0x17,1,{0x12}},  
{0x02,1,{0x77}},
{0xE1,1,{0x79}},
{0x06,1,{0x13}},    
{0xFF,5,{0xFF,98,06,04,00}},

       

//{0x35,1,{0x00}},//TE

{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,5,{}},

        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
    {0xFF,  3,  {0x80, 0x09, 0x01}},
    //{REGFLAG_DELAY, 10, {}},

        {0x00,  1,  {0x80}},
    //{REGFLAG_DELAY, 10, {}},

        {0xFF,  2,  {0x80,0x09}},
    //{REGFLAG_DELAY, 10, {}},

        {0x00,  1,  {0x03}},
    //{REGFLAG_DELAY, 10, {}},

        {0xFF,  1,  {0x01}},
    //{REGFLAG_DELAY, 10, {}},

        // Sleep Mode On
        // {0xC3, 1, {0xFF}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                MDELAY(2);
       	}
    }
	
}


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
	 
	 params->type	= LCM_TYPE_DSI;
	 
	 params->width	= FRAME_WIDTH;
	 params->height = FRAME_HEIGHT;
	    // enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	 params->dsi.mode	=  SYNC_PULSE_VDO_MODE;  
	 // DSI
	 /* Command mode setting */
	 params->dsi.LANE_NUM				 = LCM_TWO_LANE;
	 
	 //The following defined the fomat for data coming from LCD engine.
	 params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	 params->dsi.data_format.trans_seq	 = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	 params->dsi.data_format.padding	 = LCM_DSI_PADDING_ON_LSB;
	 params->dsi.data_format.format    = LCM_DSI_FORMAT_RGB888;
	 
	 // Video mode setting		 
	 params->dsi.intermediat_buffer_num = 2;
	 
	 params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	 
	 params->dsi.word_count=480*3;	 //DSI CMD mode need set these two bellow params, different to 6577
	 params->dsi.vertical_active_line=800;
	
	 params->dsi.vertical_sync_active				 = 4;//4
	 params->dsi.vertical_backporch 				 = 16;//16
	 params->dsi.vertical_frontporch				 = 20;//20
	 params->dsi.vertical_active_line				 = FRAME_HEIGHT;
	 
	 params->dsi.horizontal_sync_active 			 = 10;//10
	 params->dsi.horizontal_backporch				 = 80;//70;50
	 params->dsi.horizontal_frontporch				 = 100;//80;60
	 params->dsi.horizontal_blanking_pixel				 = 120;//100;80
	 params->dsi.horizontal_active_pixel			 = FRAME_WIDTH;
	 
	 // Bit rate calculation
	 params->dsi.pll_div1=0;	 // div1=0,1,2,3;div1_real=1,2,4,4
	 params->dsi.pll_div2=1;	 // div2=0,1,2,3;div2_real=1,2,4,4
	 params->dsi.fbk_sel=1; 	  // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
	 params->dsi.fbk_div =12;//28	 //ox1c=28	 // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	 //18--28

	params->dsi.ssc_disable = 1;

}

//add by hyde for debug
static void hyde_degug()
{

    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(200);
    
    array[0] = 0x00053700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xa1, buffer, 5);

    id_high = buffer[2];
    id_low = buffer[3];
    id = (id_high<<8) | id_low;

    #if defined(BUILD_LK)
        printf("OTM8018B uboot hyde compare_id%s \n", __func__);
        printf("%s id = 0x%08x \n", __func__, id);
        printf("hyde id is %08x \n",id);
    #else
        printk("OTM8018B kernel hyde compare_id%s \n", __func__);
        printk("%s id = 0x%08x \n", __func__, id);
        printk("hyde id is %08x \n",id);
    #endif

}


static void lcm_init(void)
{
	
	#if defined(BUILD_LK)
		upmu_set_rg_vgp2_vosel(5);
		upmu_set_rg_vgp2_en(1);
		
		upmu_set_rg_vgp3_vosel(3);
		upmu_set_rg_vgp3_en(1);	
#else
		hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "Lance_LCM");
		hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Lance_LCM");
#endif
	
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20); //20
    SET_RESET_PIN(1);
    MDELAY(150);//150
	

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{

	/*SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);*/

	push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_esd_check_sleep_out(void);
static unsigned int lcm_esd_recover(void);

static void lcm_resume(void)
{
    // lcm_init();
#if 1
    if(lcm_esd_check_sleep_out())
		lcm_esd_recover();
	else
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);

}
#endif

//extern int IMM_GetOneChannelValueEX(int dwChannel, int deCount);
static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned int id1 = 0;
    unsigned char buffer[3];
	unsigned char buffer1[3];
    unsigned int array[16];
    
    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
   // return 1;

	array[0]=0x00063902;
	array[1]=0x9806FFFF;// page enable
	array[2]=0x00000104;
	dsi_set_cmdq(&array, 3, 1);
	MDELAY(10);

    array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x00, buffer, 3);
    id = buffer[0]; //we only need ID
    id1 = buffer[1];
#if defined(BUILD_LK)
    /*The Default Value should be 0x00,0x80,0x00*/
    printf("-------ili9806e_dsi_vdo_LJJ [soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#else
    printk("-------ili9806e_dsi_vdo_LJJ [soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);

#endif
    /*array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x00, buffer1[0], 1);

	array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x01, buffer1[1], 1);

	array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x02, buffer1[2], 1);

#if defined(BUILD_LK)
    //The Default Value should be 0x00,0x80,0x00
	printf("zhangkunzhangkun1\n");
    printf("-------ili9806_dsi_vdo_dijing1 [soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer1[0],buffer1[1],buffer1[2]);
#else
	printk("zhangkunzhangkun1\n");
    printk("-------ili9806_dsi_vdo_dijing1 [soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer1[0],buffer1[1],buffer1[2]);

#endif*/

    return ((0x98 == id)&&((0x06 == id1)))?1:0;
   // return 1;
}

static unsigned int lcm_compare_id_for_esd(void)
{   
    unsigned int id = 0;
    unsigned int id1 = 0;
	unsigned int ret = 0;
    unsigned char buffer[3];
    unsigned int array[16];
	int lcm_adc = 0;
	int data[4];

    if(lcm_compare_id_flag !=TRUE){
	    array[0] = 0x00033700;// read id return two byte,version and id
	    dsi_set_cmdq(array, 1, 1);
	    read_reg_v2(0x00, buffer, 3);
	    id = buffer[0]; //we only need ID
	    id1 = buffer[1];
	#if defined(BUILD_LK)
		printf("zhangkunzhangkun\n");
	    printf("+++++lcm_compare_id_for_esd: yashi+++++ [soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
	#else
		printk("zhangkunzhangkun\n");
	    printk("+++++lcm_compare_id_for_esd: yashi+++++ [soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);

	#endif
	    ret = (0x98 == id) && (0x06 == id1);

	    if (ret == 1){
			lcm_exist_flag = TRUE;
		}
		
		lcm_compare_id_flag = TRUE;
	    return lcm_exist_flag;
    }else{
    	return lcm_exist_flag;
    }
	
}
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
  printk("ili9608c_fwvga_dsi_vdo_yashi: lcm_exist_flag=%d,lcm_compare_id_for_esd = %d.\n",
     lcm_exist_flag,lcm_compare_id_for_esd());

  if((lcm_exist_flag == TRUE) || (lcm_exist_flag != TRUE && lcm_compare_id_for_esd())){//lcm exist
	 printk("lcm_esd enter");
	 static int err_count = 0,uncount = 0,count = 0;
	 unsigned char buffer_1[12];
	 unsigned char buffer_2[4];
	 unsigned int array[16];
	 int i;
	 unsigned char fResult; 
 
	 printk("lcm_esd_check<<<\n");
	 for(i = 0; i < 1; i++)
		 buffer_1[i] = 0x00;
 
	 for(i = 0; i < 1; i++)
		 buffer_2[i] = 0x00;
 
	 //---------------------------------
	 // Set Maximum Return Size
	 //---------------------------------	
	 array[0] = 0x00013700;
	 dsi_set_cmdq(array, 1, 1);
 
	 //---------------------------------
	 // Read [9Ch, 00h, ECC] + Error Report(4 Bytes)
	 //---------------------------------
	 read_reg_v2(0x0A, buffer_1, 1);
 
	 printk( "lcm_esd_check: read(0x0A)\n");
	 for(i = 0; i < 1; i++) 			   
		 printk( "buffer_1[%d]:0x%x \n", i, buffer_1[i]);
 
		
 if((buffer_1[0] != 0x9C) || (err_count >= 2))
	 {
		 err_count = 0;
		 uncount++;
 
		 printk( "lcm_esd_check abnormal uncount=%d\n", uncount);
		 printk("lcm_esd_check>>>\n");
 
		 fResult = 1; 
	 //return TRUE; 
	 }
	 else
	 {		  
		 count++;
		 printk( "lcm_esd_check normal count=%d\n", count);
		 printk("lcm_esd_check>>>\n");
 
		 fResult = 0; 
	 //return FALSE; 
	 }
 	printk("lcm_esd_check fResult =%d\n",fResult);
	 if(fResult) 
		 return TRUE; 
	 else 
		 return FALSE; 
   }else{ //lcm doesn't exist
       return NULL;
   }
#endif

}

static unsigned int lcm_esd_check_sleep_out(void)
{
#ifndef BUILD_LK
	 static int err_count = 0,uncount = 0,count = 0;
	 unsigned char buffer_1[12];
	 unsigned char buffer_2[4];
	 unsigned int array[16];
	 int i;
	 unsigned char fResult; 
 
	 printk("yashi lcm_esd_check_sleep_out enter<<<\n");
	 for(i = 0; i < 1; i++)
		 buffer_1[i] = 0x00;
 
	 for(i = 0; i < 1; i++)
		 buffer_2[i] = 0x00;
 
	 //---------------------------------
	 // Set Maximum Return Size
	 //---------------------------------	
	 array[0] = 0x00013700;
	 dsi_set_cmdq(array, 1, 1);
 
	 //---------------------------------
	 // Read [9Ch, 00h, ECC] + Error Report(4 Bytes)
	 //---------------------------------
	 read_reg_v2(0x0A, buffer_1, 1);
 
	 printk( "yashi lcm_esd_check_sleep_out: read(0x0A)\n");
	 for(i = 0; i < 1; i++) 			   
		 printk( "buffer_1[%d]:0x%x \n", i, buffer_1[i]);
 
		
 if((buffer_1[0] != 0x08) || (err_count >= 2))
	 {
		 err_count = 0;
		 uncount++;
		 printk( "yashi lcm_esd_check_sleep_out abnormal uncount=%d\n", uncount);
		 fResult = 1; 
	 }
	 else
	 {		  
		 count++;
		 printk( "yashi lcm_esd_check_sleep_out normal count=%d\n", count);
		 fResult = 0; 
	 }
	 if(fResult) 
		 return TRUE; 
	 else 
		 return FALSE; 
#endif
}

static unsigned int lcm_esd_recover(void)
{
    
	if((lcm_exist_flag == TRUE) || (lcm_exist_flag != TRUE && lcm_compare_id_for_esd())){//lcm exist
	    unsigned char para = 0; 
	    static int recount = 0;
	#ifndef BUILD_LK
	    printk( "lcm_esd_recover\n"); 
	#endif

    lcm_init(); 
    MDELAY(10); 

//    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1); 
//    dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE ynn add for esd
    MDELAY(10); 
    //lcm_setbacklight(200); 
    recount++; 

#ifndef BUILD_LK
    printk( "lcm_esd_recover recover recount=%d\n",recount); 
#endif
    return TRUE; 
	}else{//lcm doesn't exist
		return NULL;
	}
}

LCM_DRIVER ili9806e_hengsheng_HS040NH06D_BOE_dsi_2_vdo_wvga_lcm_drv = 
{
    .name			= "ili9806e_hengsheng_HS040NH06D_BOE_dsi_2_vdo_wvga",
	.set_util_funcs = lcm_set_util_funcs,
    .compare_id    = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

/*LCM_DRIVER ili9806e_dsi_vdo_wvga_LJJ_LT40256MIT04_lcm_drv= 
{
    .name           = "ili9806e_dsi_vdo_wvga_LJJ_LT40256MIT04",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    //.esd_check = lcm_esd_check,
    .esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };*/
#endif
