/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/



#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#define Lcd_Log printf
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
    #include <linux/string.h>
    #define Lcd_Log printk
   
#endif
#include "lcm_drv.h"

#define BIRD_ESD_CHECK
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER
static unsigned int lcm_esd_test = 0;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x9816)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_initialization_setting[] = {
// VCI=2.8V
//************* Reset LCD Driver ****************//
//LCD_nRESET = 1;
//delayms(1); // Delay 1ms
//LCD_nRESET = 0;
//delayms(10); // Delay 10ms // This delay time is necessary
//LCD_nRESET = 1;
//delayms(120); // Delay 120 ms
//************* Start Initial Sequence **********//
// EXTC Command Set enable register

/*{0xFF, 3, {0xFF,0x98,0x16}},
// SPI Interface Setting 
{0xBA,1,{0x60}},
// Interface Mode Control   
{0XB0,1,{0x01}},

{0xBC,18,{0x03,0x0D,0x61,0x69,0x16,0x16,0x1B,0x11,0x70,0x00,0x00,0x00,0x16,0x16,0x09,0x00,0xFF,0XF0}},
// GIP 2
{0xBD, 8,{0x01,0x45,0x45,0x67,0x01,0x23,0x45,0x67}},
// GIP 3
{0xBE,17,{0x13,0x22,0x11,0x00,0x66,0x77,0x22,0x22,0xBA,0xDC,0xCB,0xAD,0x22,0x22,0x22,0x22,0x22}},
// en_volt_reg measure VGMP  
{0xED,2,{0x7F,0x0F}},

{0xF3,1,{0x70}},              
// Display Inversion
{0XB4,1,{0x00}},
// Power Control 1
{0XC0,3,{0x0F,0x0B,0x0A}},
// Power Control 2
{0XC1,4,{0x17,0x8E,0x87,0x20}},
// VGLO Selection
{0XD8,1,{0x50}},
// VGLO Selection
{0XFC,1,{0x07}},
// Positive Gamma Control  
{0XE0,16,{0x00,0x10,0x1D,0x0F,0x11,0x19,0XC8,0x08,0x05,0x0B,0x04,0x0D,0x10,0x2D,0x29,0x00}},
 // Negative Gamma Control 
{0XE1,16,{0x00,0x0A,0x15,0x12,0x11,0x16,0X79,0x07,0x03,0x07,0x08,0x0C,0x09,0x25,0x20,0x00}},
// Source Timing Adjust
{0XD5, 8,{0x0F,0x09,0x08,0x0A,0xCB,0XA5,0x01,0x04}},
// Resolution  
{0XF7,1,{0x89}},
// VcomVcomVcom 
{0XC7,1,{0x47}},//43
// 00=>FW ; 03=>BW 00=>FW ;
{0X36, 1,{0x00}},

//{0X35, 1,{0x00}},
// Exit Sleep
{0X11,1,{0x00}},
{REGFLAG_DELAY, 120, {}},
{0X29,1,{0x00}},*/
#if 0
{0xFF,3, {0xFF, 0x98, 0x16}},

{0xBA,1, {0x60}},

{0xF9,1, {0x06}},
{0xB0,1, {0x01}},

{0xBC,18, {0x03, 0x0D, 0x63, 0x67, 0x01, 0x01, 0x1B, 0x0F, 0x6E, 0x63, 0xFF, 0xFF, 0x01, 0x01, 0x0F, 0x00, 0xFF, 0xF2}},

{0xBD,8, {0x01, 0x23, 0x45, 0x67, 0x01, 0x23, 0x45, 0x67}},

{0xBE,17, {0x02, 0x22, 0x22, 0x22, 0x77, 0x66, 0xAA, 0xBB, 0xCC, 0xDD, 0x88, 0x99, 0x22, 0x22, 0x22, 0x22, 0x22}},

{0xED,2, {0x7F, 0x0F}},
{0xF3,1, {0x70}},
{0xB4,1, {0x02}},
{0xD8,1, {0x50}},
{0xFC,1, {0x08}},
{0xC0,3, {0x0F, 0x0B, 0x0A}},
{0xC1,4, {0x17, 0x82, 0x82, 0x20}},

{0xE0,16, {0x00, 0x06, 0x16, 0x10, 0x10, 0x17, 0xCB, 0x08, 0x03, 0x07, 0x05, 0x10, 0x14, 0x2B, 0x27, 0x00}},

{0xE1,16, {0x00, 0x03, 0x05, 0x0D, 0x11, 0x17, 0x79, 0x09, 0x03, 0x08, 0x06, 0x0A, 0x09, 0x2A, 0x27, 0x00}},

{0xD5,8, {0x0D, 0x0A, 0x0A, 0x0A, 0xCB, 0xA5, 0x01, 0x04}},

{0xF7,1, {0x8a}},
{0xC7,1, {0x6D}},

//#if defined(BIRD_ESD_CHECK)
//{0xd6,8, {0xff,0xa0,0x88,0x14,0x04,0x64,0x28,0x1a}},//added for after add esd function ,the scree is flashing
//#endif

{0xF9,1, {0x04}},
{REGFLAG_DELAY, 200, {}},
{0x11,1, {0x00}},
{REGFLAG_DELAY, 200, {}},

#if defined(BIRD_ESD_CHECK)
{0xd6,8, {0xff,0xa0,0x88,0x14,0x04,0x64,0x28,0x1a}},//added for after add esd function ,the scree is flashing
{0xEE,9, {0x0A, 0x1B, 0x5F, 0x40, 0x00, 0x00, 0x10, 0x00, 0x58}},//added for esd read reg maliejun 20130823
#endif

{0x29,1, {0x00}},

#else
{0xFF,3,{0xFF,0x98,0x16}},

{0xBA,1,{0x60}},

{0xB0,1,{0x01}},

{0xBC,23,{0x03,0x0D,0x63,0x31,0x01,0x01,0x1B,0x10,0x37,0x13,0x10,0x37,0x32,0x32,0x19,0x00,0xFF,0xE2,0x01,0x05,0x05,0x43,0x0B}},

{0xBD,8,{0x01,0x23,0x45,0x67,0x01,0x23,0x45,0x67}},

{0xBE,17,{0x00,0x22,0x22,0x22,0x97,0x86,0xCA,0xDB,0xAC,0xBD,0x68,0x79,0x22,0x22,0x22,0x22,0x22}},

{0xED,2,{0x7F,0x0F}},

{0xF3,1,{0x70}},

{0xB4,1,{0x02}},

{0xC0,3,{0x9F,0x0B,0x08}},

{0xC1,4,{0x17,0x98,0x92,0x20}},

{0xD8,1,{0x50}},

{0xFC,1,{0x08}},

{0xE0,16,{0x00,0x0C,0x23,0x13,0x15,0x1F,0xCB,0x08,0x04,0x08,0x03,0x0F,0x13,0x26,0x24,0x00}},

{0xE1,16,{0x00,0x09,0x15,0x11,0x13,0x17,0x79,0x08,0x03,0x08,0x06,0x0B,0x08,0x27,0x21,0x00}},

{0xD5,8,{0x0D,0x0A,0x0A,0x0A,0xCB,0xA5,0x01,0x04}},

{0xF7,1,{0x8A}},
{0xC7,1,{0x87}},

{0x36,1,{0x00}},

//#if defined(BIRD_ESD_CHECK)
//{0xd6,8, {0xff,0xa0,0x88,0x14,0x04,0x64,0x28,0x3a}},//added for after add esd function ,the scree is flashing 1a
//#endif


{0x11,1,{0x00}},
{REGFLAG_DELAY, 220, {}},

#if defined(BIRD_ESD_CHECK)
{0xd6,8, {0xf0,0xa0,0x88,0x14,0x04,0x64,0x28,0x1a}},//added for after add esd function ,the scree is flashing 1a
{0xEE,9, {0x0A,0x1B,0x5F,0x40,0x00,0x00,0x10,0x00,0x58}},//added for esd read reg maliejun 20130823

//{0xF0,1,{0x49}},
//{0x21,1,{0x00}},
#endif

{0x29,1,{0x00}},
//{0x2c,1,{0x00}},

#endif
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
            /*
                if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                    //#if defined(BUILD_UBOOT)
                    //	printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                    //#endif
                    while(read_reg(cmd) != table[i].para_list[0]);		
                }
				*/
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
    
    // enable tearing-free
   // params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED;
  //  params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;//CMD_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
    params->dsi.intermediat_buffer_num = 2;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;
  
     params->dsi.vertical_sync_active				= 4;//2;
    params->dsi.vertical_backporch					= 16;//20;
    params->dsi.vertical_frontporch					= 20;//20;	
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 10;//10;
    params->dsi.horizontal_backporch				= 80;//60;
    params->dsi.horizontal_frontporch				= 80;//200;
    params->dsi.horizontal_blanking_pixel				= 60;
    //params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
    
    // Bit rate calculation

    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =17;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		 17

}


static void init_lcm_registers(void)
{
    unsigned int data_array[16];

    data_array[0]= 0x00043902;
    data_array[1]= 0x0698ffff;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000060ba;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000001b0;
    dsi_set_cmdq(data_array, 2, 1);
     
    data_array[0]= 0x00163902;
    data_array[1]= 0x030e03bc;
    data_array[2]= 0x1b010163;
    data_array[3]= 0x00006f12;
    data_array[4]= 0x03010100;
    data_array[5]= 0x01f2ff00;
	data_array[6]= 0x0000c000;
    dsi_set_cmdq(data_array, 7, 1);
    
    data_array[0]= 0x00093902;
    data_array[1]= 0x451302bd;
    data_array[2]= 0x01674567;
    data_array[3]= 0x00000023;
    dsi_set_cmdq(data_array, 4, 1);
	
	data_array[0]= 0x000a3902;
    data_array[1]= 0x222201be;
    data_array[2]= 0x2267badc;
    data_array[3]= 0x00002222;    
    dsi_set_cmdq(data_array, 4, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000057C7;
    dsi_set_cmdq(data_array, 2, 1); 
     
    data_array[0]= 0x00043902;
    data_array[1]= 0x000f7fed;
    dsi_set_cmdq(data_array, 2, 1);   
	
   data_array[0]= 0x00043902;
    data_array[1]= 0x0a0b03c0;
    dsi_set_cmdq(data_array, 2, 1); 

	data_array[0]= 0x00023902;
    data_array[1]= 0x000008FC;
    dsi_set_cmdq(data_array, 2, 1); 


    data_array[0]= 0x00073902;
    data_array[1]= 0x000000DF;
    data_array[2]= 0x00200000;
    dsi_set_cmdq(data_array, 3, 1); 
	
    data_array[0]= 0x00023902;
    data_array[1]= 0x000074f3;
    dsi_set_cmdq(data_array, 2, 1);
    
   data_array[0]= 0x00043902;
   data_array[1]= 0x020202b4;
   dsi_set_cmdq(data_array, 2, 1);    
 
   


	data_array[0]= 0x00023902;
    data_array[1]= 0x000081F7;
    dsi_set_cmdq(data_array, 2, 1); 

	data_array[0]= 0x00043902;
    data_array[1]= 0x131300B1;
    dsi_set_cmdq(data_array, 2, 1); 

	data_array[0]= 0x00053902;
       data_array[1]= 0x400280F2;
	data_array[2]= 0x00000028;
    dsi_set_cmdq(data_array, 3, 1); 
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x7B7817c1;
    data_array[2]= 0x00000020;
    dsi_set_cmdq(data_array, 3, 1);    
    
                 
    
    data_array[0]= 0x00113902;
    data_array[1]= 0x130600e0;
    data_array[2]= 0xCB181211;
    data_array[3]= 0x0506020A;
    data_array[4]= 0x292E0E0D;
    data_array[5]= 0x00000000;
    dsi_set_cmdq(data_array, 6, 1);    
	
data_array[0]= 0x00113902;
    data_array[1]= 0x0E0600e1;
    data_array[2]= 0x78161110;
    data_array[3]= 0x07070308;
    data_array[4]= 0x262B090C;
    data_array[5]= 0x00000000;
    dsi_set_cmdq(data_array, 6, 1);       

	data_array[0]= 0x00023902;
    data_array[1]= 0x00000035;
    dsi_set_cmdq(data_array, 2, 1); 
     
  // 	data_array[0]= 0x00023902;
 //   data_array[1]= 0x0000C036;
 //   dsi_set_cmdq(data_array, 2, 1);               
    
     data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    data_array[0]= 0x000A3902;//added for esd read reg maliejun 20130823
    data_array[1]= 0x5F1B0Aee;
    data_array[2]= 0x10000040;
    data_array[3]= 0x00005800;
    dsi_set_cmdq(data_array, 4, 1);
    
    
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);  
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

#ifdef BUILD_LK
	printf("MYCAT ILI9608 lk lcm_init\n");
#else
      printk("MYCAT ILI9608  kernel lcm_init\n");
#endif
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(180);
		push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    //init_lcm_registers();
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(180);

  

   // data_array[0] = 0x014F1500;
   // dsi_set_cmdq(data_array, 1, 1);
   // MDELAY(40);
}


static void lcm_resume(void)
{
#if 0
    unsigned int data_array[16];

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(180);

    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);
#endif
	//add by xia lei jie for LCD INIT   
    lcm_init();
}
         
#if 1
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
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}
#else
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

		if(0 != y)
		{
			data_array[0]=0x003c3909;
			data_array[1]=0x00351508; //HS short packet
			dsi_set_cmdq(&data_array, 2, 0);
		}
		else
		{
			data_array[0]=0x002c0508;
			data_array[1]=0x003c3909;
			data_array[2]=0x01351508; //HS short packet
			dsi_set_cmdq(&data_array, 3, 0);
		}
}
#endif


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0, id2 = 0;
    unsigned char buffer[3];
    unsigned int data_array[16];

#ifdef BUILD_LK
		  pmic_config_interface(0x0532,5,0x7,5);//add by libo for VGP2 POWER ON
		  pmic_config_interface(0x050C,1,0x1,15);
#else
		  hwPowerOn(MT6323_POWER_LDO_VGP2,VOL_2800,"LCM");
#endif
    MDELAY(100);

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(180);	
    
    /*	
    data_array[0] = 0x00110500;		// Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    */
    //*************Enable CMD2 Page1  *******************//
    data_array[0]=0x00043902;
    data_array[1]=0x1698FFFF;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(20); 
    
    data_array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20); 
    
    read_reg_v2(0xD3, buffer, 3);
    id = buffer[1]; //we only need ID
    id2= buffer[1] << 8 | buffer[2]; //we test buffer 1
    
 
    Lcd_Log("%s, id=0x%x,id2=0x%x\n", __func__, id,id2);
 
 
    return (LCM_ID == id2)?1:0;

	   // return (get_lcd_id()==1)?1:0;

}


#if defined (BIRD_ESD_CHECK)
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	static int ok_count = 0;
	 unsigned char buffer[12];
	 unsigned int array[16];
	 int i;
	 
	 
	if(lcm_esd_test)
  {
      lcm_esd_test = 0;
      return 1;
  }
	 	
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	//MDELAY(20);
	read_reg_v2(0x0A, buffer, 1);
	 
	printk("mycat$$ huarui ### esd check buffer[0] = 0x%x\r\n",buffer[0]);
	
	if(buffer[0] == 0x9C)//9C
		return 0;
	else
		return 1;
 			
#endif
}

static unsigned int lcm_esd_recover(void)
{
  lcm_init();
	
#ifndef BUILD_LK	
	printk("mycat huarui ### lcm_esd_recover \r\n");
#endif
	
	return 1;
}
#endif


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806_APEX_A40269N50VIA_dsi_longteng_wvga_vdo_lcm_drv = 
{
    .name			= "ili9806_APEX_A40269N50VIA_dsi_longteng_wvga_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,

    .compare_id    = lcm_compare_id,
#if defined (BIRD_ESD_CHECK)//rm esd resolve LCD  flick
	.esd_check = lcm_esd_check,
  .esd_recover = lcm_esd_recover,
#endif

};

