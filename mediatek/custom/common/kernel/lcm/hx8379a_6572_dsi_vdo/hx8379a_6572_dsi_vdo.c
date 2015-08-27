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

#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH 										(480)
#define FRAME_HEIGHT										(864)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

//set LCM IC ID
#define LCM_ID_HX8379		 (0x79)

//#define LCM_DEBUG


/*--------------------------LCD module explaination begin---------------------------------------*/

//LCD module explaination					//Project		Custom		W&H		shijiao		degree	data		HWversion

//LCD_TDT_450_HX8379A_CPT_0807			//YarisL					480*864		12 o'clock	0		130807	Proto 01
//LCD_TDT_450_HX8379A_CPT_0924			//YarisL					480*864		12 o'clock	0		130924	PIO-EP01
//LCD_TDT_450_HX8379A_CPT_0926			//YarisL					480*864		12 o'clock	0		130926	PIO-EP01
//LCD_TDT_450_HX8379A_CPT_1030			//YarisL					480*864   	12 o'clock	0		131030	PIO-EP01 
//LCD_TDT_450_HX8379A_CPT_1220			//YarisL					480*864   	12 o'clock	0		131220	PIO-EP01 
/*--------------------------LCD module explaination end----------------------------------------*/

//#define LCD_TDT_450_HX8379A_CPT_0807	//light on the LCD,130807
//#define LCD_TDT_450_HX8379A_CPT_0924	//optimize the flick,130924
//#define LCD_TDT_450_HX8379A_CPT_0926	//change inversion from line to two dot(1+2dot),130926
//#define LCD_TDT_450_HX8379A_CPT_1030  //TDT optimize two dot(1+2dot) initial parameters final version, 131030
#define LCD_TDT_450_HX8379A_CPT_1220  //TDT change to line inversion, 131220

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
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  

       

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{0x2c,  1, 	{0x00}},
  	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
    {REGFLAG_DELAY, 100, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_mode_setting[] = {
	{0x55, 1, {0x1}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
	{0xB9,  3,      {0xFF, 0x83,0x79}},

	//{0xBA,1,{0x51}},    // wanghe 2013-07-19 should not this!!

	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	//{0xC3, 1, {0xFF}},

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

	// enable tearing-free
 	//params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE; 
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
    
	// Video mode setting		
 	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    // params->dsi.vertical_active_line=800;

    params->dsi.vertical_sync_active				= 5;	//4; // 3;
    params->dsi.vertical_backporch					= 6;	//7; //7; // 10; // 16; // 20;   // zrl 2013-09-24  is important if up or down black/white line
    params->dsi.vertical_frontporch					= 5;	//16; // 20;	
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 10;
    params->dsi.horizontal_backporch				= 55;  // 50
    params->dsi.horizontal_frontporch				= 55;  // 50
    params->dsi.horizontal_blanking_pixel			= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    // params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's

    //params->dsi.LPX	 = 8;//4
   
    // Bit rate calculation
	//1 Every lane speed    
    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =30;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		 29

}

static void lcm_init(void)
{
	unsigned int data_array[16];

#ifdef BUILD_LK
	printf("HX8379a----->%s\n", __func__);
#else
	printk("HX8379a----->%s\n", __func__);
#endif

    SET_RESET_PIN(1);
  	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//return;

#if defined(LCD_TDT_450_HX8379A_CPT_0807)

	data_array[0]=0x00043902;//Enable external Command
	data_array[1]=0x7983FFB9; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);//5000

	data_array[0]=0x00203902;
	data_array[1]=0x445000B1;
	data_array[2]=0x110890EA;
	data_array[3]=0x30287111;
	data_array[4]=0x1B421A9A;
	data_array[5]=0xE600F176;
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x000680B4;
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x3F003707;
	data_array[6]=0x04303F08;
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
	
    // set gamma
	data_array[0]=0x00243902;
	data_array[1]=0x121279E0;
	data_array[2]=0x3F2B2914;
	data_array[3]=0x12064C40;
	data_array[4]=0x15161410;
	data_array[5]=0x12161216;
	data_array[6]=0x2B291412;
	data_array[7]=0x064C403F;
	data_array[8]=0x16141012;
	data_array[9]=0x16121615;
	// data_array[10]=0x16121515;
	dsi_set_cmdq(&data_array, 11, 1);
	MDELAY(1);

	data_array[0]=0x00053902;
	data_array[1]=0x007300B6;
	data_array[1]=0x00000077;
	dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Enable external Command
	data_array[1]=0x00000035; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);//3000

    data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(50);//5000

#elif defined(LCD_TDT_450_HX8379A_CPT_0924)

	data_array[0]=0x00043902;//Enable external Command
	data_array[1]=0x7983FFB9; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);//5000

	data_array[0]=0x00203902;
	data_array[1]=0x445000B1;
	data_array[2]=0x110890EA;
	data_array[3]=0x30287111;
	data_array[4]=0x1B421A9A;
	data_array[5]=0xE600F176;
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x000680B4;
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x3F003707;
	data_array[6]=0x04303F08;
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
	
    // set gamma
	data_array[0]=0x00243902;
	data_array[1]=0x121279E0;
	data_array[2]=0x3F2B2914;
	data_array[3]=0x12064C40;
	data_array[4]=0x15161410;
	data_array[5]=0x12161216;
	data_array[6]=0x2B291412;
	data_array[7]=0x064C403F;
	data_array[8]=0x16141012;
	data_array[9]=0x16121615;
	dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);

	data_array[0]=0x00053902;
	data_array[1]=0x007200B6;
	data_array[2]=0x00000072;
	dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Enable external Command
	data_array[1]=0x00000035; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);//3000

    data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(50);//5000

#elif defined(LCD_TDT_450_HX8379A_CPT_0926)

	data_array[0]=0x00043902;//Enable external Command
	data_array[1]=0x7983FFB9; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);//5000

	data_array[0]=0x00203902;
	data_array[1]=0x465000B1;	//445000B1
	data_array[2]=0x110810EA;    // 11 08 90 ea
	data_array[3]=0x3028710F;	//  30 28 71 0a 
	data_array[4]=0x0C621A9A;	//0C421A9A
	data_array[5]=0xE600F16A;	//0x..76
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x000682B4;	//0x..80B4
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x34003707;	//0x3F
	data_array[6]=0x173A3A1B;	//0x04303F08
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	// set gamma
	data_array[0]=0x00243902;
	data_array[1]=0x121279E0;
	data_array[2]=0x3F2B2914;
	data_array[3]=0x12064C40;
	data_array[4]=0x15161410;
	data_array[5]=0x12161216;
	data_array[6]=0x2B291412;
	data_array[7]=0x064C403F;
	data_array[8]=0x16141012;
	data_array[9]=0x16121615;
	dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);

	data_array[0]=0x00053902;
	data_array[1]=0x007000B6;	 
	data_array[2]=0x00000070;	 
	dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Enable external Command
	data_array[1]=0x00000035; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);//3000

	data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);//5000
	
#elif defined(LCD_TDT_450_HX8379A_CPT_1030)
	
	data_array[0]=0x00043902;//Enable external Command
	data_array[1]=0x7983FFB9; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x375000B1;	//445000B1
	data_array[2]=0x110851EE;    // 11 08 90 ea
	data_array[3]=0x2C247414;	//  30 28 71 0a 
	data_array[4]=0x0C421A9A;	//0C421A9A
	data_array[5]=0xE600F176;	//0x..76
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x000682B4;	//0x..80B4
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x3C003707;	//0x3F
	data_array[6]=0x083D3D08;	//0x04303F08
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00033902;
	data_array[1]=0x001743C0;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
	
	/*nqb merge new lcm gamma parameters @20131119*/
	// set gamma
	data_array[0]=0x00243902;
	data_array[1]=0x101079E0;		//0x121279E0
	data_array[2]=0x3F252412;		//0x3F252414
	data_array[3]=0x12064434;		//0x12064736
	data_array[4]=0x15171410;		//0x1416140F
	data_array[5]=0x10121016;		//0x12161115
	data_array[6]=0x25241210;		//0x25241412
	data_array[7]=0x0644343F;		//0x0647363F
	data_array[8]=0x17141012;		//0x16140F12
	data_array[9]=0x14121615;		//0x16111514
	dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);

	data_array[0]=0x00053902;
	data_array[1]=0x006D00B6;		//0x007200B6	 
	data_array[2]=0x00000077;		//0x00000072	 
	dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);
	/*nqb merge end @20131119*/

	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Enable external Command
	data_array[1]=0x00000035; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);//3000

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);//5000
	
	data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);//5000

#elif defined(LCD_TDT_450_HX8379A_CPT_1220)
	
	data_array[0]=0x00043902;//Enable external Command
	data_array[1]=0x7983FFB9; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x445000B1;	//445000B1
	data_array[2]=0x110899EE;    // 11 08 90 ea
	data_array[3]=0x2C247111;	//  30 28 71 0a 
	data_array[4]=0x0A223FBF;	//0C421A9A
	data_array[5]=0xE600F16A;	//0x..76
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x000680B4;	//0x..80B4
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x3C003707;	//0x3F
	data_array[6]=0x143D3D14;	//0x04303F08
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
	
	// set gamma
	data_array[0]=0x00243902;
	data_array[1]=0x100479E0;
	data_array[2]=0x2A161613;
	data_array[3]=0x130C382C;
	data_array[4]=0x15161410;
	data_array[5]=0x04121116;
	data_array[6]=0x16161310;
	data_array[7]=0x0C382C2A;
	data_array[8]=0x16141013;
	data_array[9]=0x12111615;
	dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);

	data_array[0]=0x00053902;
	data_array[1]=0x007100B6;
	data_array[2]=0x00000071;
	dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
//baoqiang.qin add to fix display issue
#if 1
	data_array[0]=0x00033902;
	data_array[1]=0x000200C6;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
#endif
//baoqiang.qin add end
	data_array[0]=0x00023902;//Enable external Command
	data_array[1]=0x00000035; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);//3000

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);//5000
	
	data_array[0] = 0x00290500; //0x29,display on,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);//5000

#endif

}


static void flicker_debug(void)
{
	unsigned int data_array[16];
	static int temp = 0x50;

    SET_RESET_PIN(1);
  	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//return;
	data_array[0]=0x00043902;//Enable external Command
	data_array[1]=0x7983FFB9; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x009351BA;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x375000B1;	//445000B1
	data_array[2]=0x110851EE;    // 11 08 90 ea
	data_array[3]=0x2C247414;	//  30 28 71 0a 
	data_array[4]=0x0C421A9A;	//0C421A9A
	data_array[5]=0xE600F176;	//0x..76
	data_array[6]=0xE6E6E6E6;
	data_array[7]=0x0A050400;
	data_array[8]=0x6F05040B;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x000E3902;
	data_array[1]=0xFE0000B2;
	data_array[2]=0x22190C08;
	data_array[3]=0x0C08FF00;
	data_array[4]=0x00002019;
	dsi_set_cmdq(&data_array, 5, 1);
	MDELAY(1);

	data_array[0]=0x00203902;
	data_array[1]=0x000682B4;	//0x..80B4
	data_array[2]=0x32031032;
	data_array[3]=0x10325F13;
	data_array[4]=0x28013508;
	data_array[5]=0x3C003707;	//0x3F
	data_array[6]=0x083D3D08;	//0x04303F08
	data_array[7]=0x28084000;
	data_array[8]=0x04303008;
	dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);

	data_array[0]=0x00303902;
	data_array[1]=0x0A0000D5;
	data_array[2]=0x00050100;
	data_array[3]=0x99881800;
	data_array[4]=0x88450188;
	data_array[5]=0x23450188;
	data_array[6]=0x88888867;
	data_array[7]=0x88888888;
	data_array[8]=0x88105499;
	data_array[9]=0x54327688;
	data_array[10]=0x88888810;
	data_array[11]=0x00008888;
	data_array[12]=0x00000000;
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x047005DE;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00033902;
	data_array[1]=0x001743C0;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
	
	// set gamma
	data_array[0]=0x00243902;
	data_array[1]=0x121279E0;
	data_array[2]=0x3F252414;
	data_array[3]=0x12064736;
	data_array[4]=0x1416140F;
	data_array[5]=0x12161115;
	data_array[6]=0x25241412;
	data_array[7]=0x0647363F;
	data_array[8]=0x16140F12;
	data_array[9]=0x16111514;
	dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);
/*
	data_array[0]=0x00053902;
	data_array[1]=0x007000B6;	 
	data_array[2]=0x00000070;	 
	dsi_set_cmdq(&data_array, 3, 1);
	*/
		data_array[0]=0x00053902;
	//data_array[1]=0x007300B6;
	//data_array[2]=0x00000077;
	
	data_array[1] = (0x00<<24)|(temp<<16)|(0x00<<8)|0xB6;
	data_array[2] = (temp);
	
	dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);


#ifdef BUILD_LK
	printf("[%s]zrl HX8379a,temp = 0x%x,data_array[1] = 0x%x,data_array[2] = 0x%x\n",__func__,temp,data_array[1],data_array[2]);
#else
	printk("[%s]zrl HX8379a,temp = 0x%x,data_array[1] = 0x%x,data_array[2] = 0x%x\n",__func__,temp,data_array[1],data_array[2]);
#endif
	temp+=2;
	
	data_array[0]=0x00023902;
	data_array[1]=0x000002CC;	
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;//Enable external Command
	data_array[1]=0x00000035; 
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);//3000

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);//5000
	
	data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);//5000
	
}


/*
static void lcm_suspend(void)
{
//	lcm_compare_id();
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
}
*/
/*
static void lcm_suspend(void)
{
	int array[4];
	 
	array[0] = 0x00002200;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);//5000

	array[0] = 0x00280500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);//5000

	array[0] = 0x00100500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(120);//5000
		 
	SET_RESET_PIN(0);
	MDELAY(50);

}
*/

static void lcm_suspend(void)
{
	int array[4];
	 
	array[0] = 0x00100500;
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(130);
}


#if 0

static void lcm_resume(void)
{
	flicker_debug();
}

#else

/*
static void lcm_resume(void)
{
	lcm_init();
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}
*/

static void lcm_resume(void)
{
	int array[4];
	 
	array[0] = 0x00110500;
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(130);
	 
	array[0] = 0x00290500;
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(10);
}

#endif


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


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 0;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_setpwm(unsigned int divider)
{
	// TBD
}

static void lcm_setbacklight_mode(unsigned int mode)
{
	lcm_backlight_mode_setting[0].para_list[0] = mode;
	push_table(lcm_backlight_mode_setting, sizeof(lcm_backlight_mode_setting) / sizeof(struct LCM_setting_table), 1);
}

//static void lcm_setpwm(unsigned int divider)
//{
	// TBD
//}

static unsigned int lcm_getpwm(unsigned int divider)
{
	// ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
	// pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
	unsigned int pwm_clk = 23706 / (1<<divider);	
	return pwm_clk;
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK

	char  buffer[5];
	int   array[4];

#if defined(LCM_DEBUG)
	printk("hx8379a: lcm_esd_check enter\n");
#endif
	 
	array[0] = 0x00043700;
	dsi_set_cmdq(array, 1, 1);
	 
	read_reg_v2(0x09, buffer, 4);

#if defined(LCM_DEBUG)
	printk("lcm_esd_check buffer[0]=0x%x\n",buffer[0]);
	printk("lcm_esd_check buffer[1]=0x%x\n",buffer[1]);
	printk("lcm_esd_check buffer[2]=0x%x\n",buffer[2]);
	printk("lcm_esd_check buffer[3]=0x%x\n",buffer[3]); 
#endif
	
	if((buffer[0]==0x80)&&(buffer[1]==0x73))
	{
		return 0;
	}
	else
	{ 
		return 1;
	}

#endif
}


static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK 

    printk("lcm_esd_recover hx8379 enter");
	lcm_init();
	return 1;

#endif
}


static unsigned int lcm_compare_id();
static unsigned int lcm_compare_id()
{
    unsigned int id = 0;
    unsigned char buffer[4];
    unsigned int array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0]=0x00043902;
	array[1]=0x7983FFB9;
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);

	array[0]=0x00033902;
	array[1]=0x009351BA;
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2); //read device id,command is 0xBF, return 0x01,0x22,0x14,0x08,0xe0
	//id = read_reg(0xF4);
	id = buffer[0];                     

#if defined(BUILD_LK)
	printf("<<<< [%s]zrl-lk,read HX8379a ID:0xF4 = 0x%x,0x%x\n", __func__,id,buffer[1]);
#else
	printk("<<<< [%s]zrl-kernel,read HX8379a ID:0xF4 = 0x%x,0x%x\n", __func__,buffer[0],id);
#endif

	return (id == LCM_ID_HX8379)?1:0;

}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx8379a_6572_dsi_vdo_lcm_drv= 
{
    .name = "hx8379a_6572_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif

	//.set_backlight	= lcm_setbacklight,
	//.set_backlight_mode = lcm_setbacklight_mode,
	//.set_pwm		  = lcm_setpwm,
	//.get_pwm		  = lcm_getpwm	
	.esd_check	 	= lcm_esd_check,
	.esd_recover	= lcm_esd_recover, 
	.compare_id    	= lcm_compare_id,
};

