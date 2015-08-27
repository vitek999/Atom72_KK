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

{0XFF,5,{0XFF,0X98,0X06,0X04,0X01}},     // Change to Page 1
{0X08,1,{0X10}},                 // output SDA
{0X21,1,{0X01}},                 // DE = 1 Active
{0X30,1,{0X02}},                 // 480 X 800
{0X31,1,{0X02}},                 // Column Inversion
{0X60,1,{0X07}},                 // SDTI
{0X61,1,{0X06}},                // CRTI
{0X62,1,{0X06}},                 // EQTI
{0X63,1,{0X04}},                // PCTI
{0X40,1,{0X15}},                // BT  +2.5/-2.5 pump for DDVDH-L
{0X41,1,{0X22}},                 // DVDDH DVDDL clamp  
{0X42,1,{0X00}},                 // VGH/VGL 
{0X43,1,{0X0B}},                 // VGH/VGL 
{0X44,1,{0X0C}},                 // VGH/VGL 
{0X45,1,{0X1B}},                 // VGL_REG  -10V 
{0X50,1,{0X78}},                 // VGMP
{0X51,1,{0X78}},                 // VGMN
{0X52,1,{0X00}},                   
{0X53,1,{0X6F}},                 //Flicker4F
{0X57,1,{0X50}},              
{0XA0,1,{0X00}},  // Gamma 0 /255
{0XA1,1,{0X14}},  // Gamma 4 /251
{0XA2,1,{0X1A}},  // Gamma 8 /247
{0XA3,1,{0X06}},  // Gamma 16	/239
{0XA4,1,{0X02}},  // Gamma 24 /231
{0XA5,1,{0X04}},  // Gamma 52 / 203
{0XA6,1,{0X1C}},  // Gamma 80 / 175
{0XA7,1,{0X00}},  // Gamma 108 /147
{0XA8,1,{0X19}},  // Gamma 147 /108
{0XA9,1,{0X0C}},  // Gamma 175 / 80
{0XAA,1,{0X0E}},  // Gamma 203 / 52
{0XAB,1,{0X04}},  // Gamma 231 / 24
{0XAC,1,{0X07}},  // Gamma 239 / 16
{0XAD,1,{0X1A}},  // Gamma 247 / 8
{0XAE,1,{0X02}},  // Gamma 251 / 4
{0XAF,1,{0X00}},  // Gamma 255 / 0
   
{0XC0,1,{0X00}},  // Gamma 0 
{0XC1,1,{0X01}},  // Gamma 4
{0XC2,1,{0X0A}},  // Gamma 8
{0XC3,1,{0X10}},  // Gamma 16
{0XC4,1,{0X07}},  // Gamma 24
{0XC5,1,{0X10}},  // Gamma 52
{0XC6,1,{0X01}},  // Gamma 80
{0XC7,1,{0X0F}},  // Gamma 108
{0XC8,1,{0X06}},  // Gamma 147
{0XC9,1,{0X0A}},  // Gamma 175
{0XCA,1,{0X12}},  // Gamma 203
{0XCB,1,{0X0B}},  // Gamma 231
{0XCC,1,{0X14}},  // Gamma 239
{0XCD,1,{0X17}},  // Gamma 247
{0XCE,1,{0X16}},  // Gamma 251
{0XCF,1,{0X00}},  // Gamma 255
        
{0XFF,5,{0XFF,0X98,0X06,0X04,0X07}},     // Change to Page 7
{0X18,1,{0X1D}},
{0X17,1,{0X22}},  
{0XE1,1,{0X79}},
{0X02,1,{0X77}},
        
{0XFF,5,{0XFF,0X98,0X06,0X04,0X06}},     // Change to Page 6
{0X00,1,{0X20}},
{0X01,1,{0X05}},
{0X02,1,{0X00}},    
{0X03,1,{0X00}},
{0X04,1,{0X01}},
{0X05,1,{0X01}},
{0X06,1,{0X88}},    
{0X07,1,{0X04}},
{0X08,1,{0X01}},
{0X09,1,{0X90}},    
{0X0A,1,{0X04}},    
{0X0B,1,{0X01}},    
{0X0C,1,{0X01}},
{0X0D,1,{0X01}},
{0X0E,1,{0X00}},
{0X0F,1,{0X00}},
{0X10,1,{0X55}},
{0X11,1,{0X50}},
{0X12,1,{0X01}},
{0X13,1,{0X0C}},
{0X14,1,{0X0D}},
{0X15,1,{0X43}},
{0X16,1,{0X0B}},
{0X17,1,{0X00}},
{0X18,1,{0X00}},
{0X19,1,{0X00}},
{0X1A,1,{0X00}},
{0X1B,1,{0X00}},
{0X1C,1,{0X00}},
{0X1D,1,{0X00}},
{0X20,1,{0X01}},
{0X21,1,{0X23}},
{0X22,1,{0X45}},
{0X23,1,{0X67}},
{0X24,1,{0X01}},
{0X25,1,{0X23}},
{0X26,1,{0X45}},
{0X27,1,{0X67}},
{0X30,1,{0X02}},
{0X31,1,{0X22}},
{0X32,1,{0X11}},
{0X33,1,{0XAA}},
{0X34,1,{0XBB}},
{0X35,1,{0X66}},
{0X36,1,{0X00}},
{0X37,1,{0X22}},
{0X38,1,{0X22}},
{0X39,1,{0X22}},
{0X3A,1,{0X22}},
{0X3B,1,{0X22}},
{0X3C,1,{0X22}},
{0X3D,1,{0X22}},
{0X3E,1,{0X22}},
{0X3F,1,{0X22}},
{0X40,1,{0X22}},
{0X52,1,{0X10}},
{0X53,1,{0X10}},
        
{0XFF,5,{0XFF,0X98,0X06,0X04,0X00}},     // Change to Page 0
{0x35,1,{0x00}},
{0x3A,1,{0x77}},
{0x55,1,{0x80}},
       

//{0x35,1,{0x00}},//TE

{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},
{REGFLAG_DELAY,5,{}},

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
  	params->dsi.fbk_div =18; //15 pyj		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		//28
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

LCM_DRIVER ili9806e_HF_14025_0000_A0_BOE_dsi_2_vdo_wvga_lcm_drv = 
{
    .name			= "ili9806e_HF_14025_0000_A0_BOE_dsi_2_vdo_wvga",
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

