
//===============================================================================//
// Project Name: EPD Driver for Linux System                                     //
// Engineer: Ming-Jong Jou                                                       //
// Create Date: 2009/07/02                                                       //
// Revision:                                                                     //
// Version: 0.1 Release Date: 2009/07/02                                         //
//          0.2 Release Date: 2009/07/24                                         //
//              Compatible with SDL/QT/MiniGUI/MPlay @ S3C2416                   //
//          0.3 Release Date: 2010/05/17                                         //
//              Support AUO-K1901                                                //
//          0.4 Release Date: 2010/05/24                                         //
//              fix auo_epd_fb_init                                              //
//          0.5 Release Date: 2010/05/27                                         //
//              MJ: Add ioctl for send/show image                                //
//          0.6 Release Date: 2010/06/15                                         //
//              Lisa: 1. Add ioctls                                              //
//                       IOCTL_CMD_SEND_IMG_FB : DMA (data from fb0)             //            
//                       IOCTL_CMD_SHOW_IMG_FB : DDMA (data from fb0)            //
//                       IOCTL_CMD_SET_REFRESH_MODE : set epd update mode        //
//                       IOCTL_CMD_SET_ROTATION_ANGLE : set rotation angle       //
//                    2. Remove the timer for monitoring fb0                     //
//          0.7 Release Date: 2010/07/26                                         //
//              MJ:   1. Add platform_device_unregister to support re-insmod     //
//===============================================================================//
// add SLP (GPH12) control

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/mxcfb.h>
#include <linux/mxcfb_epdc_kernel.h>
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/fsl_devices.h>

#include <mach/iomux-mx50.h>//&*&*&*J1_add
	
#include <linux/time.h>

#include "epdc_regs.h"


#include <linux/syscalls.h> 
#include "elcdif_regs.h"	
#include "mxcfb_auo.h"		
#include "rgbmap16.h"		


#include <linux/wait.h> 

#include "start_logo.h"	

wait_queue_head_t	g_wait_queue;


#define AUO_BOOTING_LOGO			1			
#define AUO_SUPPORT_MODE4			1			

extern int mc13892_swbst_enable(struct regulator_dev *reg);
extern int mc13892_swbst_disable(struct regulator_dev *reg);

extern int mc13892_vgen1_enable(struct regulator_dev *reg);
extern int mc13892_vgen1_disable(struct regulator_dev *reg);

extern int mc13892_vgen3_enable(struct regulator_dev *reg);
extern int mc13892_vgen3_disable(struct regulator_dev *reg);


extern int mc13892_vusb2_enable(struct regulator_dev *reg);
extern int mc13892_vusb2_disable(struct regulator_dev *reg);

unsigned long porta_get_time(void);
void booting_startup_logo(void);	
void auo_mode4_initial(void);		



/*
AUO driver sample pins define:
-----------------------------------------
S3C_SIFCCON0:8	--> CS
S3C_SIFCCON0:1	--> RS

S3C_GPBDAT:3
S3C_GPHDAT:12	-->SLEEP
S3C_GPFDAT:5	-->RESET

S3C_GPBDAT:1	
S3C_SIFCCON0:7	-->HRD
S3C_SIFCCON1	-->Data bus (write)
S3C_SIFCCON0:6	-->HWE
S3C_SIFCCON2:0xffff	-->Data bus (read)
*/

#define  __DEBUG_AUO_EPD__

#ifdef __DEBUG_AUO_EPD__
#define DBG(fmt,args...)	printk(fmt ,##args)
#else
#define DBG(fmt,args...)
#endif


#define JEF_DEBUG1		0	
#define JEF_DEBUG2		0	
#define JEF_DEBUG3		0	
#define JEF_DEBUG4		0	

#if JEF_DEBUG3
#define	jeffrey_tag()		printk( "Jeffrey .. %s(%d):%s\r\n"	   , __FILE__, __LINE__, __FUNCTION__ ) 
#define	jeffrey_head()		printk( "Jeffrey ..  %s(%d):%s +++\r\n", __FILE__, __LINE__, __FUNCTION__ )
#define	jeffrey_tail()		printk( "Jeffrey ..  %s(%d):%s ---\r\n", __FILE__, __LINE__, __FUNCTION__ ) 
#else
#define	jeffrey_tag()		
#define	jeffrey_head()		
#define	jeffrey_tail()		
#endif

#if JEF_DEBUG2
#define	jeffrey_debug()		printk( "Jeffrey ..  %s(%d):%s +++\r\n", __FILE__, __LINE__, __FUNCTION__ )
#else
#define	jeffrey_debug()		
#endif

#define JEFFREY_TEST_PANEL	0 

#define PANEL_MAX_WIDTH		1024
#define PANEL_WIDTH         768 
#define PANEL_HEIGHT        1024 
#define PANEL_BPP           16 
#define PALETTE_BUFF_CLEAR (0x80000000)	



//&*&*&*J1_add: for AUO
//-----------------------------------------------------------------------------------------------------------------
#define	ELCDIF_BASE_ADDR	(DEBUG_BASE_ADDR + 0x0100A000)
#define ELCDIF_BASE			(ELCDIF_BASE_ADDR)

#define	HW_ELCDIF_TRANSFTER_COUNT	0x0030




#define	MPU_COMMAND	0
#define	MPU_DATA	1


#define CTRL_READ_WRITEB	(1<<28)
#define CTRL_LCDIF_MASTER	(1<<5)
#define CTRL_MODE86 		(1<<1)	// 8080 -> 0, 6800 -> 1
#define	CTRL_BYPASS_COUNT	(1<<19) // MPU MODE --> 0
#define	CTRL_RUN			(1<<0)
#define	CTRL_DATA_SELECT	(1<<16)
#define	CTRL1_BUSY_ENABLE	(1<<2)	// not used --> 0



#define REG_RD(base, reg) \
	(*(volatile unsigned int *)((base) + (reg)))
#define REG_WR(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg))) = (value))
#define REG_SET(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _SET))) = (value))
#define REG_CLR(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _CLR))) = (value))
#define REG_TOG(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _TOG))) = (value))

#define REG_RD_ADDR(addr) \
	(*(volatile unsigned int *)((addr)))
#define REG_WR_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr))) = (value))
#define REG_SET_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr) + 0x4)) = (value))
#define REG_CLR_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr) + 0x8)) = (value))
#define REG_TOG_ADDR(addr, value) \
	((*(volatile unsigned int *)((addr) + 0xc)) = (value))
 

#define EPDC_D1		(2*32 + 1)	/*GPIO_3_1 */
#define DISP_RS		(1*32 + 17)	/*GPIO_2_17 */
#define DISP_BUSY	(1*32 + 18)	/*GPIO_2_18 */
#define DISP_RESET	(1*32 + 20)	/*GPIO_2_20 */


#define	MPU_nSLEEP_Assert()		gpio_direction_output(DISP_BUSY, 0)		
#define	MPU_nSLEEP_Deassert()	gpio_direction_output(DISP_BUSY, 1)		
#define	MPU_nRESET_Assert()		gpio_direction_output(DISP_RESET, 0)	
#define	MPU_nRESET_Deassert()	gpio_direction_output(DISP_RESET, 1)	
#define	Get_MPU_nBusy()			gpio_get_value(DISP_RS)					

#define	MPU_nCS_Assert()		gpio_direction_output(EPDC_D1, 0)
#define	MPU_nCS_Deassert()		gpio_direction_output(EPDC_D1, 1)		

unsigned int command_buff[1] __attribute__ ((aligned(32)));
unsigned int parameter_buff[8] __attribute__ ((aligned(32)));

// elcdif clock settings
//static void __iomem *elcdif_base;

//static void __iomem *epdc_base;
void __iomem *epdc_base;
struct mxc_epdc_fb_data *g_fb_data;


static struct device *g_elcdif_dev;
static bool g_elcdif_axi_clk_enable;
static bool g_elcdif_pix_clk_enable;
static struct clk *g_elcdif_axi_clk;
static struct clk *g_elcdif_pix_clk;

static void __iomem *elcdif_buffer;


void *lcd_console_address;	

u8 lcd_base[PANEL_MAX_WIDTH*PANEL_MAX_WIDTH/2]; 
u8 lcd_base_temp[PANEL_MAX_WIDTH*PANEL_MAX_WIDTH/2]; 


//Display control variable
//------------------------------------

/*
 * g_bSleep is 1 : TCON is sleep (a sleep command is sent before)
 * g_bSleep is 0 : TCON is wakeup
 * I80 read data timing is slower when TCON is sleep
 */
static unsigned int  g_bSleep = 0;

//static unsigned int  g_iRefresh_display=1;
static unsigned int  g_iAutoRefreshMode=0; 
static unsigned int  g_iAuoRuning=0;			

static unsigned int  g_iAuoFlash = 0;
static unsigned int	 g_iAuoRefreshMode = 0; 
static unsigned int  g_iAuoRotation = 0;		

static unsigned int  g_iAuoHPL = 0;		
static unsigned int  g_iAuoFBG = 0;		


#define GET_DISPLAY_OWNER			while(g_iAuoRuning) msleep(5); g_iAuoRuning = 1;		
#define FREE_DISPLAY_OWNER			g_iAuoRuning = 0;											

// for ap use
//----------------
static unsigned int g_iRotationAngle = 0;	

//------------------------------------------------------------------------------------------
static void	mpu_write_32( unsigned int is_data, unsigned char* buffer, unsigned int count_width, unsigned int count_height );
static void	mpu_write_16( unsigned int is_data, unsigned char* buffer, unsigned int count_width, unsigned int count_height );
static void	mpu_read_32( unsigned int is_data, unsigned char* buffer, unsigned int count_width, unsigned int count_height );

//------------------
static void	pt_k1901_blank_black(void);
static void	pt_k1901_blank_white(void);
static void pt_k1901_full_refresh(void);
static void pt_k1901_full_refresh_mode(unsigned int mode);
static void pt_k1901_full_reshow(unsigned int mode);
static void pt_k1901_part_refresh(unsigned int mode,unsigned int x, unsigned int y, unsigned int w, unsigned int h,unsigned char* buff);
static void pt_k1901_part_reshow(unsigned int mode,unsigned int x, unsigned int y, unsigned int w, unsigned int h);


//------------------
static void	pt_k1901_dma_start( unsigned int HPL, unsigned int FBG, 
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff );
static void pt_k1901_ddma_start( unsigned int flash, unsigned int mode, unsigned int rota_n,
								unsigned int x, unsigned int y, unsigned int w, unsigned int h );
								
static void	pt_k1901_pip_start( unsigned int HPL, unsigned int FBG, 
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff );
static void pt_k1901_dpip_start( unsigned int flash, unsigned int mode, unsigned int rota_n,
								unsigned int x, unsigned int y, unsigned int w, unsigned int h );
								
static void	pt_k1901_cursor_start( unsigned int CPL, unsigned int CS, 
		unsigned int w, unsigned int h,
		unsigned char* buff );
static void pt_k1901_dcursor_start( unsigned int flash, unsigned int mode, unsigned int rota_n, 
								unsigned int CS, unsigned int CSR_IPROT,
								unsigned int x, unsigned int y );
								
static void	pt_k1901_animation_start(unsigned int APL, unsigned int AMFrameNo,
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff );
static void pt_k1901_danimation_start( unsigned int flash, unsigned int mode, unsigned int rota_n,unsigned int AMFrameNo,
								unsigned int x, unsigned int y, unsigned int w, unsigned int h );
								
static void	pt_k1901_pre_display_start( unsigned int EPL, 
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff );
static void	pt_k1901_data_stop(void);
static void	pt_k1901_wait_not_busy(void);
static void	pt_k1901_hw_reset(void);



// Freescale EPDC functions
//---------------------------------------------------------
static void mxc_epdc_fb_deferred_io(struct fb_info *info,struct list_head *pagelist);
static void epdc_done_work_func(struct work_struct *work);
static void epdc_submit_work_func(struct work_struct *work);
static int mxc_epdc_fb_set_fix(struct fb_info *info);
static ssize_t store_update(struct device *device,struct device_attribute *attr,const char *buf, size_t count);
static void epdc_powerdown(struct mxc_epdc_fb_data *fb_data);
int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,struct fb_info *info);
void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,	struct fb_info *info);
int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info);
int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info);
int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info);
int mxc_epdc_fb_wait_update_complete(u32 update_marker, struct fb_info *info);
int mxc_epdc_fb_wait_update_complete(u32 update_marker, struct fb_info *info);
int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,struct fb_info *info);
int mxc_epdc_get_pwrdown_delay(struct fb_info *info);


// AUO panel functions
//---------------------------------------------------------
static int auo_epd_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);

static int auo_epd_fb_set_par(struct fb_info *info);
static int auo_epd_fb_setcolreg(unsigned regno, unsigned red, unsigned green,unsigned blue, unsigned transp,struct fb_info *info);
void auo_epd_fb_fillrect(struct fb_info *p, const struct fb_fillrect *rect);
void auo_epd_fb_copyarea(struct fb_info *p, const struct fb_copyarea *area) ;
void auo_epd_fb_imageblit(struct fb_info *p, const struct fb_image *image) ;
static int auo_epd_fb_blank(int blank_mode, struct fb_info *info);
static int auo_epd_fb_pan_display(struct fb_var_screeninfo *var,struct fb_info *info);
static int auo_epd_fb_mmap(struct fb_info *info, struct vm_area_struct *vma);
static int auo_epd_fb_open(struct fb_info *info, int user);

static void	pt_k1901_aging(void);
static void	pt_k1901_aging_exit(void);
static void pt_k1901_init_set(void);


static void pt_24bpp_to_16(void);
static void pt_16bpp_to_16(void);
static void pt_16bpp_to_16_rev(void);
static void pt_16bpp_to_16_rev1(unsigned int x, unsigned int y, unsigned int w, unsigned int h);
static void pt_16bpp_to_16_rev2(unsigned int x, unsigned int y, unsigned int w, unsigned int h);
static void pt_8pp_to_16(unsigned int width, unsigned int height,unsigned char* buff);



//=================================================================================================================

//#define AUO_FPGA
/*
#define S3C2416_GPBCON  (S3C2410_PA_GPIO+0x10)
static void __iomem *gpGPIO_GPB;
#define S3C_GPBCON (gpGPIO_GPB)
#define S3C_GPBDAT (gpGPIO_GPB+0x04)
#define S3C2416_GPCCON  (S3C2410_PA_GPIO+0x20)
static void __iomem *gpGPIO_GPC;
#define S3C_GPCCON (gpGPIO_GPC)
#define S3C_GPCDAT (gpGPIO_GPC+0x04)
#define S3C_GPCUPD (gpGPIO_GPC+0x08)
#define S3C2416_GPDCON  (S3C2410_PA_GPIO+0x30)
static void __iomem *gpGPIO_GPD;
#define S3C_GPDCON (gpGPIO_GPD)
#define S3C_GPDDAT (gpGPIO_GPD+0x04)
#define S3C_GPDUPD (gpGPIO_GPD+0x08)
#define S3C2416_GPECON  (S3C2410_PA_GPIO+0x40)
static void __iomem *gpGPIO_GPE;
#define S3C_GPECON (gpGPIO_GPE)
#define S3C_GPEDAT (gpGPIO_GPE+0x04)
#define S3C_GPEUPD (gpGPIO_GPE+0x08)
#define S3C2416_GPFCON  (S3C2410_PA_GPIO+0x50)
static void __iomem *gpGPIO_GPF;
#define S3C_GPFCON (gpGPIO_GPF)
#define S3C_GPFDAT (gpGPIO_GPF+0x04)
#define S3C_GPFUPD (gpGPIO_GPF+0x08)

#define S3C2416_GPGCON  (S3C2410_PA_GPIO+0x60)
static void __iomem *gpGPIO_GPG;
#define S3C_GPGCON (gpGPIO_GPG)
#define S3C_GPGDAT (gpGPIO_GPG+0x04)
#define S3C_GPGUPD (gpGPIO_GPG+0x08)

#define S3C2416_GPHCON  (S3C2410_PA_GPIO+0x70)
static void __iomem *gpGPIO_GPH;
#define S3C_GPHCON (gpGPIO_GPH)
#define S3C_GPHDAT (gpGPIO_GPH+0x04)
#define S3C_GPHUPD (gpGPIO_GPH+0x08)

#define VIDCON0_CLKVAL_F_SHIFT  (6)
*/

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

/*
 * rotation angle can be 0, 90, 180, 270 degrees.
 * EPD mode can be 0, 1, 2, 3, 4 (now only 0, 1, 4 are used).
 * Please refreence TCON spec
 */
//static unsigned int rotation_angle = 0;
//static unsigned int refresh_mode = 1;

/*
 * The driver will count continuous non-flash mode (1, 2) updates.
 * if autoFlash true, when non_flash_count exceeds non_flash_threshold, 
 * a full screen mode 0 update will be issued automatically.
 */
static unsigned int non_flash_count = 0;
static bool autoFlash = true;
static unsigned int non_flash_threshold = 16;

static struct fb_info * gFB_info=NULL;
static struct fb_info info;


void k1901_show_image(int dest, int x, int y, int w, int h, int epd_mode, int flip, int id,int csr_iprot);
void auo_epd_i80_update_screen(int dest, int x, int y, int w, int h, int rotate, int id, struct fb_info *p);
u8 halftone16_render(u8 gray, int x, int y);

/*
 * Set 2416 GPIO
 */
void auo_epd_i80_ctrl_reg_init(void);
void auo_sys_var_init(void);


static int __init auo_epd_fb_init(void);
static int auo_epd_fb_ioctl( struct fb_info *info, unsigned int cmd, unsigned long arg );

/*
 * For bootup logo
 */
int display_update_work_func(void);
//int display_update_work_timer(void);


int thread_id;
int thread_id1;
int g_thread_id=NULL;



//==============================================
//        Frame buffer operations               
//==============================================
//---------------------------------------------------------------------------------------------------

static struct fb_ops auo_epd_fb_ops = {
		.owner		= THIS_MODULE,
		.fb_check_var	= auo_epd_fb_check_var,
		.fb_set_par	= auo_epd_fb_set_par,
		.fb_setcolreg	= auo_epd_fb_setcolreg,
		.fb_ioctl		= auo_epd_fb_ioctl,
		.fb_fillrect	= cfb_fillrect, //auo_epd_fb_fillrect, 	/* Needed !!! */
		.fb_copyarea	= cfb_copyarea, //auo_epd_fb_copyarea,	/* Needed !!! */
		.fb_imageblit	= cfb_imageblit, //auo_epd_fb_imageblit,	/* Needed !!! */
		.fb_blank = auo_epd_fb_blank,				
		.fb_pan_display = auo_epd_fb_pan_display,	
		.fb_mmap = auo_epd_fb_mmap,	
};
/* ------------------------------------------------------------------------- */

static char drv_name[]="mxc_auo_fb";

#define NUM_SCREENS_MIN	2
#define EPDC_NUM_LUTS 16
#define EPDC_MAX_NUM_UPDATES 20
#define INVALID_LUT -1

#define DEFAULT_TEMP_INDEX	0
#define DEFAULT_TEMP		20 /* room temp in deg Celsius */

#define INIT_UPDATE_MARKER	0x12345678
#define PAN_UPDATE_MARKER	0x12345679

#define POWER_STATE_OFF	0
#define POWER_STATE_ON	1


static unsigned long default_bpp = PANEL_BPP; //16;

struct update_marker_data {
	u32 update_marker;
	struct completion update_completion;
	int lut_num;
};

/* This structure represents a list node containing both
 * a memory region allocated as an output buffer for the PxP
 * update processing task, and the update description (mode, region, etc.) */
struct update_data_list {


	struct list_head list;
	struct mxcfb_update_data upd_data;/* Update parameters */		// <--- In \include\linux\mxcfb.h
	dma_addr_t phys_addr;		/* Pointer to phys address of processed Y buf */
	void *virt_addr;
	u32 epdc_offs;			/* Add to buffer pointer to resolve alignment */
	u32 size;
	int lut_num;			/* Assigned before update is processed into working buffer */
	int collision_mask;		/* Set when update results in collision */
					/* Represents other LUTs that we collide with */
	struct update_marker_data *upd_marker_data;
	u32 update_order;		/* Numeric ordering value for update */
	u32 fb_offset;			/* FB offset associated with update */
};

struct mxc_epdc_fb_data {
	struct fb_info info;
	u32 xoffset;
	u32 yoffset;
	u32 pseudo_palette[16];
	char fw_str[24];
	struct list_head list;
	struct mxc_epdc_fb_mode *cur_mode;
	struct mxc_epdc_fb_platform_data *pdata;
	int blank;
	ssize_t map_size;
	dma_addr_t phys_start;
	u32 fb_offset;
	int default_bpp;
	int native_width;
	int native_height;
	int num_screens;
	int epdc_irq;
	struct device *dev;
	int power_state;
	struct clk *epdc_clk_axi;
	struct clk *epdc_clk_pix;
	struct regulator *display_regulator;
	struct regulator *vcom_regulator;
	bool fw_default_load;

	/* FB elements related to EPDC updates */
	bool in_init;
	bool hw_ready;
	bool waiting_for_idle;
	u32 auto_mode;
	u32 upd_scheme;
	struct update_data_list *upd_buf_queue;
	struct update_data_list *upd_buf_free_list;
	struct update_data_list *upd_buf_collision_list;
	struct update_data_list *cur_update;
	spinlock_t queue_lock;
	int trt_entries;
	int temp_index;
	u8 *temp_range_bounds;
	struct mxcfb_waveform_modes wv_modes;
	u32 *waveform_buffer_virt;
	u32 waveform_buffer_phys;
	u32 waveform_buffer_size;
	u32 *working_buffer_virt;
	u32 working_buffer_phys;
	u32 working_buffer_size;
	u32 order_cnt;
	struct update_marker_data update_marker_array[EPDC_MAX_NUM_UPDATES];
	u32 lut_update_order[EPDC_NUM_LUTS];
	struct completion updates_done;
	struct delayed_work epdc_done_work;
	struct workqueue_struct *epdc_submit_workqueue;
	struct work_struct epdc_submit_work;
	bool waiting_for_wb;
	bool waiting_for_lut;
	struct completion update_res_free;
	struct mutex power_mutex;
	bool powering_down;
	int pwrdown_delay;

	/* FB elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct pxp_config_data pxp_conf;
	struct dma_async_tx_descriptor *txd;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
	struct mutex pxp_mutex; /* protects access to PxP */
};

static struct fb_deferred_io mxc_epdc_fb_defio = {
	.delay = HZ,
	.deferred_io = mxc_epdc_fb_deferred_io,
};


static struct device_attribute fb_attrs[] = {
	__ATTR(update, S_IRUGO|S_IWUSR, NULL, store_update),
};


//
// AUO driver functions
//---------------------------------------------------------------------------------------------------
static int auo_epd_fb_open(struct fb_info *info, int user)
{
	jeffrey_head();
//	DBG("<1> auo_epd_fb_open\n");
	return 0;
}


static int auo_epd_fb_release(struct fb_info *info, int user)
{
	jeffrey_head();
//	DBG("<1> auo_epd_fb_release\n");
	return 0;
}

static int auo_epd_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	jeffrey_debug();
//	DBG("<1> auo_epd_fb_check_var: %d\n",var->bits_per_pixel);
	
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8))
		var->bits_per_pixel = default_bpp;

	switch (var->bits_per_pixel) {
	case 8:
		if (var->grayscale != 0) {
			/*
			 * For 8-bit grayscale, R, G, and B offset are equal.
			 *
			 */
			var->red.length = 8;
			var->red.offset = 0;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 0;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		} else {
			var->red.length = 3;
			var->red.offset = 5;
			var->red.msb_right = 0;

			var->green.length = 3;
			var->green.offset = 2;
			var->green.msb_right = 0;

			var->blue.length = 2;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		}
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	switch (var->rotate) {
	case FB_ROTATE_UR:
	case FB_ROTATE_UD:
		var->xres = fb_data->native_width;
		var->yres = fb_data->native_height;
		break;
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		var->xres = fb_data->native_height;
		var->yres = fb_data->native_width;
		break;
	default:
		/* Invalid rotation value */
		var->rotate = 0;
		dev_dbg(fb_data->dev, "Invalid rotation request\n");
		return -EINVAL;
	}

	var->xres_virtual = ALIGN(var->xres, 32);
	var->yres_virtual = ALIGN(var->yres, 128) * fb_data->num_screens;

	var->height = -1;
	var->width = -1;

	return 0;
	
}

static int auo_epd_fb_set_par(struct fb_info *info)
{
	jeffrey_debug();
	

	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->info.var;
	struct mxc_epdc_fb_mode *epdc_modes = fb_data->pdata->epdc_mode;
	int i;
	int ret=0;
	
	

	mutex_lock(&fb_data->pxp_mutex);

	/*
	 * Update PxP config data (used to process FB regions for updates)
	 * based on FB info and processing tasks required
	 */

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = screeninfo->xres;
	proc_data->drect.height = proc_data->srect.height = screeninfo->yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = screeninfo->rotate;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * configure S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	if (screeninfo->grayscale)
		pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_GREY;
	else {
		switch (screeninfo->bits_per_pixel) {
		case 16:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		case 24:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB24;
			break;
		case 32:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB32;
			break;
		default:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		}
	}
	pxp_conf->s0_param.width = screeninfo->xres_virtual;
	pxp_conf->s0_param.height = screeninfo->yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = screeninfo->xres;
	pxp_conf->out_param.height = screeninfo->yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	mutex_unlock(&fb_data->pxp_mutex);

	/*
	 * If HW not yet initialized, check to see if we are being sent
	 * an initialization request.
	 */
	if (!fb_data->hw_ready) {
		struct fb_videomode mode;

		fb_var_to_videomode(&mode, screeninfo);

		/* Match videomode against epdc modes */
		for (i = 0; i < fb_data->pdata->num_modes; i++) {
			if (!fb_mode_is_equal(epdc_modes[i].vmode, &mode))
				continue;
			fb_data->cur_mode = &epdc_modes[i];
			break;
		}

		/* Found a match - Grab timing params */
		screeninfo->left_margin = mode.left_margin;
		screeninfo->right_margin = mode.right_margin;
		screeninfo->upper_margin = mode.upper_margin;
		screeninfo->lower_margin = mode.lower_margin;
		screeninfo->hsync_len = mode.hsync_len;
		screeninfo->vsync_len = mode.vsync_len;

	}

	mxc_epdc_fb_set_fix(info);

	return ret;	
}

static int auo_epd_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp,
		struct fb_info *info)
{	

//	DBG("<1> auo_epd_fb_setcolreg\n");

	if (regno >= 256)	/* no. of hw registers */
		return 1;
		
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}


#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) 
	{
		if (regno >= 16)
			return 1;

		((u32 *) (info->pseudo_palette))[regno] =
		    (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
	}

	return 0;	

}

static int auo_epd_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	jeffrey_debug();
	
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;


	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}
	return 0;
}

static int auo_epd_fb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	jeffrey_debug();
//	DBG("<1> auo_epd_fb_pan_display\n");
	
	
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	u_int y_bottom;
	unsigned long flags;

	dev_dbg(info->device, "%s: var->xoffset %d, info->var.xoffset %d\n",
		 __func__, var->xoffset, info->var.xoffset);
		 
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	    (var->yoffset + var->yres > var->yres_virtual)) 
    {
		dev_dbg(info->device, "x panning not supported\n");
		DBG("x panning not supported\n");
		return -EINVAL;
	}

	if ((fb_data->xoffset == var->xoffset) &&
		(fb_data->yoffset == var->yoffset))
	{
//		DBG("auo_epd_fb_pan_display .. no change..\n");
		return 0;	/* No change, do nothing */
	}

	spin_lock_irqsave(&fb_data->queue_lock, flags);

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
	{
		y_bottom += var->yres;
	}

	if (y_bottom > info->var.yres_virtual) 
	{
		
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return -EINVAL;
	}


	fb_data->fb_offset = (var->yoffset * var->xres_virtual + var->xoffset)
		* (var->bits_per_pixel) / 8;

	fb_data->xoffset = info->var.xoffset = var->xoffset;
	fb_data->yoffset = info->var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
	{
		info->var.vmode |= FB_VMODE_YWRAP;
	}
	else
	{
		info->var.vmode &= ~FB_VMODE_YWRAP;
	}

	spin_unlock_irqrestore(&fb_data->queue_lock, flags);


	return 0;	
	
}


static int auo_epd_fb_blank(int blank_mode, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	dev_dbg(fb_data->dev, "blank_mode = %d\n", blank_mode);

	if (fb_data->blank == blank_mode)
		return 0;

	fb_data->blank = blank_mode;

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		pt_k1901_blank_black();
		break;
	case FB_BLANK_UNBLANK:
		DBG("jeffrey .. 121a\n");		
		break;
	}
	DBG("Jeffrey .. +mxc_epdc_fb_blank :   blank=%d\n",blank_mode);
	return 0;	
}

void auo_epd_fb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
//	jeffrey_head();
	cfb_fillrect(p, rect);
}

void auo_epd_fb_copyarea(struct fb_info *p, const struct fb_copyarea *area) 
{
	cfb_copyarea(p, area);
}

void auo_epd_fb_imageblit(struct fb_info *p, const struct fb_image *image) 
{
//J	DBG("<1> auo_epd_fb_imageblit\n"); 	//&*&*&*J1_debug

	cfb_imageblit(p, image);
}

int auo_epd_fb_sync(struct fb_info *info)
{
	jeffrey_head();
	
	return 0;
}


static int __init auo_epd_fb_probe(struct platform_device *pdev)
{
	int ret = 0;

	struct mxc_epdc_fb_data  *fb_data;			
	struct fb_info *info;						

	struct resource *res;
	char *options, *opt;
	char *panel_str = NULL;
	struct fb_videomode *vmode;
	int xres_virt, yres_virt, buf_size;
	struct fb_var_screeninfo *var_info;
	struct fb_fix_screeninfo *fix_info;
	struct pxp_config_data *pxp_conf;
	struct pxp_proc_data *proc_data;
	struct scatterlist *sg;
	struct update_data_list *upd_list;
	struct update_data_list *plist, *temp_list;
	int i;
	unsigned long x_mem_size = 0;
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	struct mxcfb_update_data update;			
#endif

	jeffrey_debug();


	fb_data = (struct mxc_epdc_fb_data *)framebuffer_alloc(sizeof(struct mxc_epdc_fb_data), &pdev->dev);
	if (fb_data == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	jeffrey_debug();
	/* Get platform data and check validity */
	fb_data->pdata = pdev->dev.platform_data;
	if ((fb_data->pdata == NULL) || (fb_data->pdata->num_modes < 1)
		|| (fb_data->pdata->epdc_mode == NULL)
		|| (fb_data->pdata->epdc_mode->vmode == NULL)) {
		ret = -EINVAL;
		goto out_fbdata;
	}

	jeffrey_debug();
	
	
	

	//Set default bpp & x_mem_size
	fb_data->default_bpp = PANEL_BPP; 
	
	fb_data->dev = &pdev->dev;

	if (!fb_data->default_bpp)			
		fb_data->default_bpp = PANEL_BPP; //16;

	/* Set default (first defined mode) before searching for a match */
	fb_data->cur_mode = &fb_data->pdata->epdc_mode[0];		// default 用mode 0
	vmode = fb_data->cur_mode->vmode;


	platform_set_drvdata(pdev, fb_data);
	info = &fb_data->info;
	gFB_info = info;

	/* Allocate color map for the FB */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto out_fbdata;

	dev_dbg(&pdev->dev, "resolution %dx%d, bpp %d\n", vmode->xres, vmode->yres, fb_data->default_bpp);
	DBG("resolution %dx%d, bpp %d\n", vmode->xres, vmode->yres, fb_data->default_bpp);

	/*
	 * GPU alignment restrictions dictate framebuffer parameters:
	 * - 32-byte alignment for buffer width
	 * - 128-byte alignment for buffer height
	 * => 4K buffer alignment for buffer start
	 */
	xres_virt = ALIGN(vmode->xres, 32);
	yres_virt = ALIGN(vmode->yres, 128);
	buf_size = PAGE_ALIGN(xres_virt * yres_virt * fb_data->default_bpp/8);


	/* Compute the number of screens needed based on X memory requested */
	if (x_mem_size > 0) {
		fb_data->num_screens = DIV_ROUND_UP(x_mem_size, buf_size);
		if (fb_data->num_screens < NUM_SCREENS_MIN)
			fb_data->num_screens = NUM_SCREENS_MIN;
		else if (buf_size * fb_data->num_screens > SZ_16M)
			fb_data->num_screens = SZ_16M / buf_size;
	} else
		fb_data->num_screens = NUM_SCREENS_MIN;

	fb_data->map_size = buf_size * fb_data->num_screens;
	dev_dbg(&pdev->dev, "memory to allocate: %d\n", fb_data->map_size);
	DBG("memory to allocate: %d\n", fb_data->map_size);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto out_cmap;
	}


	epdc_base = ioremap(ELCDIF_BASE, SZ_8K);
	if (epdc_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}

	/* Allocate FB memory */
	info->screen_base = dma_alloc_writecombine(&pdev->dev,fb_data->map_size,&fb_data->phys_start,GFP_DMA);

	if (info->screen_base == NULL) {
		ret = -ENOMEM;
		goto out_mapregs;
	}
	dev_dbg(&pdev->dev, "allocated at %p:0x%x\n", info->screen_base,
		fb_data->phys_start);

	DBG("allocated at %p:0x%x\n", info->screen_base,fb_data->phys_start);

	var_info = &info->var;
	var_info->activate = FB_ACTIVATE_TEST;
	var_info->bits_per_pixel = fb_data->default_bpp;
	var_info->xres = vmode->xres;
	var_info->yres = vmode->yres;
	var_info->xres_virtual = xres_virt;
	/* Additional screens allow for panning  and buffer flipping */
	var_info->yres_virtual = yres_virt * fb_data->num_screens;

	var_info->pixclock = vmode->pixclock;
	var_info->left_margin = vmode->left_margin;
	var_info->right_margin = vmode->right_margin;
	var_info->upper_margin = vmode->upper_margin;
	var_info->lower_margin = vmode->lower_margin;
	var_info->hsync_len = vmode->hsync_len;
	var_info->vsync_len = vmode->vsync_len;
	var_info->vmode = FB_VMODE_NONINTERLACED;

	switch (fb_data->default_bpp) {
	case 32:
	case 24:
		var_info->red.offset = 16;
		var_info->red.length = 8;
		var_info->green.offset = 8;
		var_info->green.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.length = 8;
		break;

	case 16:
		var_info->red.offset = 11;
		var_info->red.length = 5;
		var_info->green.offset = 5;
		var_info->green.length = 6;
		var_info->blue.offset = 0;
		var_info->blue.length = 5;
		break;

	case 8:
		/*
		 * For 8-bit grayscale, R, G, and B offset are equal.
		 *
		 */
		var_info->grayscale = GRAYSCALE_8BIT;

		var_info->red.length = 8;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 8;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;

	default:
		dev_err(&pdev->dev, "unsupported bitwidth %d\n",
			fb_data->default_bpp);
		ret = -EINVAL;
		goto out_dma_fb;
	}

	fix_info = &info->fix;

	strcpy(fix_info->id, "mxcfb_auo");
	fix_info->type = FB_TYPE_PACKED_PIXELS;
	fix_info->visual = FB_VISUAL_TRUECOLOR;
	fix_info->xpanstep = 0;
	fix_info->ypanstep = 0;
	fix_info->ywrapstep = 0;
	fix_info->accel = FB_ACCEL_NONE;
	fix_info->smem_start = fb_data->phys_start;
	fix_info->smem_len = fb_data->map_size;
	fix_info->ypanstep = 0;

	fb_data->native_width = vmode->xres;
	fb_data->native_height = vmode->yres;

	info->fbops = &auo_epd_fb_ops; 
	info->var.activate = FB_ACTIVATE_NOW;
	info->pseudo_palette = fb_data->pseudo_palette;
	info->screen_size = info->fix.smem_len;
	info->flags = FBINFO_FLAG_DEFAULT;

	mxc_epdc_fb_set_fix(info);

	fb_data->auto_mode = AUTO_UPDATE_MODE_AUTOMATIC_MODE; 
	fb_data->upd_scheme = UPDATE_SCHEME_SNAPSHOT;

	fb_data->fb_offset = 0;
	fb_data->xoffset = 0;
	fb_data->yoffset = 0;

	/* Allocate head objects for our lists */
	fb_data->upd_buf_queue =
	    kzalloc(sizeof(struct update_data_list), GFP_KERNEL);
	fb_data->upd_buf_collision_list =
	    kzalloc(sizeof(struct update_data_list), GFP_KERNEL);
	fb_data->upd_buf_free_list =
	    kzalloc(sizeof(struct update_data_list), GFP_KERNEL);
	if ((fb_data->upd_buf_queue == NULL) || (fb_data->upd_buf_free_list == NULL)
	    || (fb_data->upd_buf_collision_list == NULL)) {
		ret = -ENOMEM;
		goto out_dma_fb;
	}

	/*
	 * Initialize lists for update requests, update collisions,
	 * and available update (PxP output) buffers
	 */
	INIT_LIST_HEAD(&fb_data->upd_buf_queue->list);
	INIT_LIST_HEAD(&fb_data->upd_buf_free_list->list);
	INIT_LIST_HEAD(&fb_data->upd_buf_collision_list->list);

	/* Allocate update buffers and add them to the list */
	for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
		upd_list = kzalloc(sizeof(*upd_list), GFP_KERNEL);
		if (upd_list == NULL) {
			ret = -ENOMEM;
			goto out_upd_buffers;
		}

		/* Clear update data structure */
		memset(&upd_list->upd_data, 0,
		       sizeof(struct mxcfb_update_data));

		/*
		 * Each update buffer is 1 byte per pixel, and can
		 * be as big as the full-screen frame buffer
		 */
		upd_list->size = info->var.xres * info->var.yres;

		/* Allocate memory for PxP output buffer */
		upd_list->virt_addr =
		    dma_alloc_coherent(fb_data->info.device, upd_list->size,
				       &upd_list->phys_addr, GFP_DMA);
		if (upd_list->virt_addr == NULL) {
			kfree(upd_list);
			ret = -ENOMEM;
			goto out_upd_buffers;
		}

		/* Add newly allocated buffer to free list */
		list_add(&upd_list->list, &fb_data->upd_buf_free_list->list);

		dev_dbg(fb_data->info.device, "allocated %d bytes @ 0x%08X\n",
			upd_list->size, upd_list->phys_addr);
	}

	fb_data->working_buffer_size = vmode->yres * vmode->xres * 2;
	/* Allocate memory for EPDC working buffer */
	fb_data->working_buffer_virt =
	    dma_alloc_coherent(&pdev->dev, fb_data->working_buffer_size,
			       &fb_data->working_buffer_phys, GFP_DMA);
	if (fb_data->working_buffer_virt == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for working buf!\n");
		ret = -ENOMEM;
		goto out_upd_buffers;
	}
	
	jeffrey_debug();

	// initial variable
	fb_data->in_init = false;
	fb_data->hw_ready = false;

	/*
	 * Set default waveform mode values.
	 * Should be overwritten via ioctl.
	 */
	fb_data->wv_modes.mode_init = 0;			
	fb_data->wv_modes.mode_du = 1;
	fb_data->wv_modes.mode_gc4 = 3;
	fb_data->wv_modes.mode_gc8 = 2;
	fb_data->wv_modes.mode_gc16 = 2;
	fb_data->wv_modes.mode_gc32 = 2;

	/* Initialize all LUTs to inactive */
	for (i = 0; i < EPDC_NUM_LUTS; i++)
		fb_data->lut_update_order[i] = 0;

	if (device_create_file(info->dev, &fb_attrs[0]))
		dev_err(&pdev->dev, "Unable to create file from fb_attrs\n");

	fb_data->cur_update = NULL;

	spin_lock_init(&fb_data->queue_lock);

	mutex_init(&fb_data->pxp_mutex);

	mutex_init(&fb_data->power_mutex);

	// PxP DMA interface 
	dmaengine_get();

	/*
	 * Fill out PxP config data structure based on FB info and
	 * processing tasks required
	 */
	pxp_conf = &fb_data->pxp_conf;
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = fb_data->info.var.xres;
	proc_data->drect.height = proc_data->srect.height = fb_data->info.var.yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * We initially configure PxP for RGB->YUV conversion,
	 * and only write out Y component of the result.
	 */

	/*
	 * Initialize S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->s0_param.width = fb_data->info.var.xres_virtual;
	pxp_conf->s0_param.height = fb_data->info.var.yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize OL0 channel parameters
	 * No overlay will be used for PxP operation
	 */
	for (i = 0; i < 8; i++) {
		pxp_conf->ol_param[i].combine_enable = false;
		pxp_conf->ol_param[i].width = 0;
		pxp_conf->ol_param[i].height = 0;
		pxp_conf->ol_param[i].pixel_fmt = PXP_PIX_FMT_RGB565;
		pxp_conf->ol_param[i].color_key_enable = false;
		pxp_conf->ol_param[i].color_key = -1;
		pxp_conf->ol_param[i].global_alpha_enable = false;
		pxp_conf->ol_param[i].global_alpha = 0;
		pxp_conf->ol_param[i].local_alpha_enable = false;
	}

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->info.var.xres;
	pxp_conf->out_param.height = fb_data->info.var.yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;		//&*&*&*J1_mod

	/*
	 * Ensure this is set to NULL here...we will initialize pxp_chan
	 * later in our thread.
	 */
	fb_data->pxp_chan = NULL;


	jeffrey_debug();
	fb_data->order_cnt = 0;
	fb_data->waiting_for_wb = false;
	fb_data->waiting_for_lut = false;
	fb_data->waiting_for_idle = false;
	fb_data->blank = FB_BLANK_UNBLANK;
	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;
	fb_data->pwrdown_delay = 0;


	/* Register FB */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev,
			"register_framebuffer failed with error %d\n", ret);
		goto out_dmaengine;
	}

	g_fb_data = fb_data;


#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* If FB console included, update display to show logo */
	update.update_region.left = 0;
	update.update_region.width = info->var.xres;
	update.update_region.top = 0;
	update.update_region.height = info->var.yres;
	update.update_mode = UPDATE_MODE_PARTIAL;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_marker = INIT_UPDATE_MARKER;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

#endif


	goto out;

out_dmaengine:
	DBG("error .. out_dmaengine\n");
	dmaengine_put();
out_irq:
	DBG("error .. out_irq\n");
	free_irq(fb_data->epdc_irq, fb_data);
out_dma_work_buf:
	DBG("error .. out_dma_work_buf\n");
	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
		fb_data->working_buffer_virt, fb_data->working_buffer_phys);
	if (fb_data->pdata->put_pins)
		fb_data->pdata->put_pins();
out_upd_buffers:
	DBG("error .. out_upd_buffers\n");
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list->list, list) {
		list_del(&plist->list);
		dma_free_writecombine(&pdev->dev, plist->size, plist->virt_addr,
				      plist->phys_addr);
		kfree(plist);
	}
out_dma_fb:
	DBG("error .. out_dma_fb\n");
	dma_free_writecombine(&pdev->dev, fb_data->map_size, info->screen_base,
			      fb_data->phys_start);

out_mapregs:
	DBG("error .. out_mapregs\n");
	iounmap(epdc_base);
out_cmap:
	DBG("error .. out_cmap\n");
	fb_dealloc_cmap(&info->cmap);
out_fbdata:
	DBG("error .. out_fbdata\n");
	kfree(fb_data);
out:

	auo_sys_var_init();
	auo_epd_i80_ctrl_reg_init();
	

	struct task_struct *thread_work; 
	thread_work = kthread_create(display_update_work_func, NULL, "display_work");
	if (!IS_ERR(thread_work))
	{
		wake_up_process(thread_work);
	}
	else
	{
		DBG("kthread_cretae .. error\n");
	}
	

	return 0;
}

static void __devexit auo_epd_fb_remove(struct platform_device *pdev)
{
	jeffrey_head();

	struct fb_info *fbinfo = platform_get_drvdata(pdev);

	if (fbinfo) {
		unregister_framebuffer(fbinfo);
		fb_dealloc_cmap(&fbinfo->cmap);
		/* ... */
		framebuffer_release(fbinfo);
	}
}

#include <linux/platform_device.h>
/* for platform devices */

#ifdef CONFIG_PM

static int auo_epd_fb_suspend(struct platform_device *dev, pm_message_t msg)
{
	jeffrey_head();
	DBG("AUO display .. into suspend  .. 1\n");
	
	struct fb_info *info = platform_get_drvdata(dev);
	struct auo_epd_fb_par *par = info->par;

	/* suspend here */
	mc13892_swbst_disable(0);		// dislabe AUO Touch
	
	MPU_nSLEEP_Deassert();
	MPU_nRESET_Deassert();


	mc13892_vusb2_disable(0);		// disable usb2

	
	return 0;
}

static int auo_epd_fb_resume(struct platform_device *dev)
{
	jeffrey_head();


	DBG("AUO display .. resume\n");

	struct fb_info *info = platform_get_drvdata(dev);
	struct auo_epd_fb_par *par = info->par;
	/* resume here */
	mc13892_vusb2_enable(0);		// enable usb2
	mc13892_swbst_enable(0);

	return 0;
}

/*
 * Before shutdown system, we sent standby command to TCON.
 * Here we wait until TCON enter standby mode (busyN is low)
 * busyN is low -> wait 140 ms -> pull sleepN low -> wait 10 ms -> pull rstN low -> wait 400 ms -> pull TCON power low
 * This sequence is used to solve poweroff fading issue, but not work.
 */
static int auo_epd_fb_shutdown(struct platform_device *dev)
{
	jeffrey_head();

	return 0;
}
#else
#define auo_epd_fb_suspend NULL
#define auo_epd_fb_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver auo_epd_fb_driver = {
		.probe		= auo_epd_fb_probe,
		.remove		= auo_epd_fb_remove,
		.suspend	= auo_epd_fb_suspend,
		.resume		= auo_epd_fb_resume,
		.shutdown	= auo_epd_fb_shutdown,
		.driver		= {
				.name	= "mxcfb_auo",
				.owner	= THIS_MODULE,
		},
};


#ifndef MODULE
int __init auo_epd_fb_setup(char *options)
{
	jeffrey_head();
	
	/* Parse user speficied options (`video=auo_epd_fb:') */
}
#endif /* MODULE */

static int __init auo_epd_fb_init(void)
{

	int rtn=0;
	jeffrey_head();

	rtn = platform_driver_register( &auo_epd_fb_driver );

	return rtn;

}

static void __exit auo_epd_fb_exit(void)
{
	jeffrey_head();

	platform_driver_unregister(&auo_epd_fb_driver);

}
/*=====================================================*/
/*           Hardware Low-Level Operation              */
/*=====================================================*/

struct pad_desc  auo_panel_enable[] = {
	// MPU control pins
	MX50_PAD_EPDC_D0__ELCDIF_RS,
	MX50_PAD_EPDC_D1__GPIO_3_1,		
	MX50_PAD_EPDC_D2__ELCDIF_WR_RWN,		
	MX50_PAD_EPDC_D3__ELCDIF_RD_E,		

	// MPU data pins
	MX50_PAD_EPDC_SDCE5__ELCDIF_D0,		
	MX50_PAD_EPDC_SDCE4__ELCDIF_D1,		
	MX50_PAD_EPDC_SDCE3__ELCDIF_D2,		
	MX50_PAD_EPDC_SDCE2__ELCDIF_D3,		
	MX50_PAD_EPDC_SDCE1__ELCDIF_D4,		
	MX50_PAD_EPDC_SDCE0__ELCDIF_D5,		
	MX50_PAD_EPDC_BDR1__ELCDIF_D6,		
	MX50_PAD_EPDC_BDR0__ELCDIF_D7,
	MX50_PAD_EPDC_SDLE__ELCDIF_D8,		
	MX50_PAD_EPDC_SDCLKN__ELCDIF_D9,		
	MX50_PAD_EPDC_SDSHR__ELCDIF_D10,		
	MX50_PAD_EPDC_PWRCOM__ELCDIF_D11,	
	MX50_PAD_EPDC_PWRSTAT__ELCDIF_D12,	
	MX50_PAD_EPDC_PWRCTRL0__ELCDIF_D13,	
	MX50_PAD_EPDC_PWRCTRL1__ELCDIF_D14,	
	MX50_PAD_EPDC_PWRCTRL2__ELCDIF_D15,	

	// display control gpio
	MX50_PAD_DISP_RS__GPIO_2_17,			
	MX50_PAD_DISP_RESET__GPIO_2_20,		
	MX50_PAD_DISP_BUSY__GPIO_2_18,		

};

struct pad_desc  auo_panel_disable[] = {
	MX50_PAD_EPDC_D0__GPIO_3_0,
	MX50_PAD_EPDC_D1__GPIO_3_1,
	MX50_PAD_EPDC_D2__GPIO_3_2,
	MX50_PAD_EPDC_D3__GPIO_3_3,
	MX50_PAD_EPDC_D4__GPIO_3_4,
	MX50_PAD_EPDC_D5__GPIO_3_5,
	MX50_PAD_EPDC_D6__GPIO_3_6,
	MX50_PAD_EPDC_D7__GPIO_3_7,
	MX50_PAD_EPDC_GDCLK__GPIO_3_16,
	MX50_PAD_EPDC_GDSP__GPIO_3_17,
	MX50_PAD_EPDC_GDOE__GPIO_3_18,
	MX50_PAD_EPDC_GDRL__GPIO_3_19,
	MX50_PAD_EPDC_SDCLK__GPIO_3_20,
	MX50_PAD_EPDC_SDOE__GPIO_3_23,
	MX50_PAD_EPDC_SDLE__GPIO_3_24,
	MX50_PAD_EPDC_SDSHR__GPIO_3_26,
	MX50_PAD_EPDC_BDR0__GPIO_4_23,
	MX50_PAD_EPDC_SDCE0__GPIO_4_25,
	MX50_PAD_EPDC_SDCE1__GPIO_4_26,
	MX50_PAD_EPDC_SDCE2__GPIO_4_27,
	
};



void auo_sys_var_init(void)
{
	// Display initial settings
	//----------------------------------
	if (PANEL_WIDTH==1024)
	{
		// for dma
		g_iAuoHPL = 0;
		g_iAuoFBG = 0;

		// for ddma
		g_iAuoFlash = 0;
		g_iAuoRefreshMode = 0;
		g_iAuoRotation = 0;		// 橫螢幕
		
		
	}
	else
	{
		// for dma
		g_iAuoHPL = 1;
		g_iAuoFBG = 0;

		// for ddma
		g_iAuoFlash = 0;
		g_iAuoRefreshMode = 0;
		g_iAuoRotation = 1;		// 直螢幕
	}
	
	
}


void auo_epd_i80_ctrl_reg_init(void)
{
	jeffrey_head();

	mxc_iomux_v3_setup_multiple_pads(auo_panel_enable, 	ARRAY_SIZE(auo_panel_enable));


	// remap elcdif_base address
	epdc_base = ioremap(ELCDIF_BASE, SZ_8K);


	g_elcdif_axi_clk = clk_get(g_elcdif_dev, "elcdif_axi");
	if (g_elcdif_axi_clk == NULL) {
		DBG("Jeffrey .. can't get ELCDIF axi clk\n");
	}

	g_elcdif_pix_clk = clk_get(g_elcdif_dev, "elcdif_pix");
	if (g_elcdif_pix_clk == NULL) 
	{
		DBG("Jeffrey .. can't get ELCDIF pix clk\n");
	}




	// setup DISPLAY_AXI_CLK and BUS_CLK
	//-----------------------------------------------
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	/* release prev panel */
	if (!g_elcdif_pix_clk_enable) {
		clk_enable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = true;
	}


	// Set gpio
	//------------------------------
	gpio_request(DISP_RS, "disp_rs");			
	gpio_direction_input(DISP_RS);				

	gpio_request(DISP_BUSY, "disp_busy");		
	gpio_request(DISP_RESET, "disp_reset");		
	gpio_request(EPDC_D1, "epdc_d1");			

	jeffrey_head();


}




void auo_epd_i80_send_cmd(u16 cmd, bool is_final_packet)
{
	jeffrey_head();

#if 0	
	// CMD to I80 bus
	__raw_writel(cmd, S3C_SIFCCON1);

	// HWE goes low (enable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<6), S3C_SIFCCON0);

	// HWE goes high (disable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) &(~(1<<6)), S3C_SIFCCON0);

	// RS goes high as command
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<1), S3C_SIFCCON0);
#endif
}
void auo_epd_i80_send_cmd_delay(u16 cmd, bool is_final_packet)
{	
	jeffrey_head();

#if 0	
	// CMD to I80 bus
	__raw_writel(cmd, S3C_SIFCCON1);
	mdelay(1);
	
	// HWE goes low (enable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<6), S3C_SIFCCON0);
	mdelay(10);
	
	// HWE goes high (disable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) &(~(1<<6)), S3C_SIFCCON0);
	mdelay(1);
	
	// RS goes high as command
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<1), S3C_SIFCCON0);
	mdelay(1);	
#endif
}

void auo_epd_i80_send_data(u16 dat, bool is_final_packet)
{

#if 0
	// Data to I80 bus
	__raw_writel(dat, S3C_SIFCCON1);

	// HWE goes low (enable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<6), S3C_SIFCCON0);

	//modified by Ian (for FPGA)
#ifdef AUO_FPGA
	udelay(8);
#endif

	// HWE goes high (disable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) &(~(1<<6)), S3C_SIFCCON0);
#endif	
}

void auo_epd_i80_read_data(u16 *dat, bool is_final_packet)
{
	jeffrey_head();

#if 0	
	// HRD goes low (enable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<7), S3C_SIFCCON0);

	//modified by Ian (FPGA)
#ifdef AUO_FPGA
	mdelay(5);
#else
	//udelay(1);
#endif

	udelay(100);
	
	// HRD goes high (disable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) &(~(1<<7)), S3C_SIFCCON0);
	
	// Read data from I80 bus
	*dat = __raw_readl(S3C_SIFCCON2)& 0xFFFF;	

#endif
}

void auo_epd_i80_read_data_delay(u16 *dat, bool is_final_packet)
{
	jeffrey_head();

#if 0	
	// HRD goes low (enable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) |(1<<7), S3C_SIFCCON0);

	//modified by Ian (FPGA)
#ifdef AUO_FPGA
	mdelay(5);
#else
	//udelay(1);
#endif

	mdelay(1);
	// HRD goes high (disable)
	__raw_writel(__raw_readl(S3C_SIFCCON0) &(~(1<<7)), S3C_SIFCCON0);
	udelay(1);
	
	// Read data from I80 bus
	*dat = __raw_readl(S3C_SIFCCON2)& 0xFFFF;
#endif
}

//Add by Ian
int check_busy()
{
	return !Get_MPU_nBusy(); 
#if 0	
	if((__raw_readl(S3C_GPGDAT) &(0x1<<5)) == 0x00)
		return 1; //busy
	else
		return 0; //not busy
#endif 
}

/*
 * Write LUT stored in shared memory to TCON
 */
void auo_epd_i80_LUT_write(int szLUTData)
{
	jeffrey_head();

	printk("auo_epd_i80_LUT_write .. 1\n");
	int shmid;
	unsigned int 	lut_size = 0,writenum = 0,idx = 0;
	unsigned long 	start_time, stop_time;
	unsigned char 	*shm_start;
	unsigned short 	*pInputLUT = NULL;
	
	unsigned int writenum_x=0,writenum_y=0,writenum_rem=0;
	u16 data;
	//Step. 1
	//Attach the shared memory in driver
	printk("auo_epd_i80_LUT_write .. 2\n");

	if ((shmid = sys_shmget ( (key_t)1234 , szLUTData , 0) ) == -1 )
	{
		DBG("sys_shmget gets fault!!");
	}

	printk("auo_epd_i80_LUT_write .. 3\n");
	if ((shm_start = (unsigned char *)sys_shmat ( shmid , NULL , 0 ) )==(unsigned char *)(-1))
	{
		DBG("sys_shmat gets fault!!");
	}
	pInputLUT = (unsigned short *)shm_start;

	printk("auo_epd_i80_LUT_write .. 4\n");

	lut_size = szLUTData;// LUT size (bytes)
	writenum = (lut_size/2);

	printk("auo_epd_i80_LUT_write .. 5\n");

	DBG("LUT write size is %d\n",szLUTData);

	//Step. 2
	//select i80 to write LUT
	printk("auo_epd_i80_LUT_write .. 6\n");

	command_buff[0] = I80_CMD_SPI_IF_SEL;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = 0x0000;
	mpu_write_32( MPU_DATA, parameter_buff, 1, 1 );
	MPU_nCS_Deassert();
	mdelay(1);

	printk("auo_epd_i80_LUT_write .. 7\n");

	//Write LUT to i80
	start_time = jiffies;
	pt_k1901_wait_not_busy();
	MPU_nCS_Assert();
	command_buff[0] = I80_CMD_LUT_START ;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );

	printk("auo_epd_i80_LUT_write .. 8\n");

	pt_k1901_wait_not_busy();
	printk("Erase flash completed\n");

	printk("auo_epd_i80_LUT_write .. 9\n");
	writenum_x = writenum /1024;
	writenum_y = 1024;
	writenum_rem = writenum % 1024;
	
	printk("auo_epd_i80_LUT_write .. writenum=%d, x=%d, y=%d, rem=%d\n",writenum,writenum_x,writenum_y,writenum_rem);


	mpu_write_16( MPU_DATA, pInputLUT, writenum_x, writenum_y );		//PS: mpu_write_16的 x,y值不可超過65535
	if (writenum_rem)
	{
		printk("auo_epd_i80_LUT_write .. write.. writenum_rem = %d\n",writenum_rem);
		mpu_write_16( MPU_DATA, pInputLUT, writenum_rem, 1 );		//PS: mpu_write_16的 x,y值不可超過65535
		
	}
	
	MPU_nCS_Deassert();
	jeffrey_tail();	

	printk("auo_epd_i80_LUT_write .. 10\n");
	
	stop_time = jiffies;
	printk("LUT_Write_Data: writing time, elapsed = %ums\n", jiffies_to_msecs(stop_time - start_time));

	printk("auo_epd_i80_LUT_write .. 11\n");

	//Step. 3
	//Release all shared memory
	sys_shmdt(shm_start);
	printk("auo_epd_i80_LUT_write .. 12\n");

}

/*
 * Read LUT from TCON to shared memory
 */
void auo_epd_i80_LUT_read(int szLUTData)
{
	jeffrey_head();
	
}

/*
 * Send pixels from fb to TCON
 * dest : send to DMA/PIP/CSR/ANI buffer in TCON
 * x, y, w, h : The area containing pixels, x and y starts from 1, please reference the TCON spec for constraints of  (x, y, w, h)
 * rotate : HP/L in TCON spec, is 0 when host image is landscape
 * id : There are multiple CSR/ANI buffers, so id is needed when sending pixels to CSR/ANI buffer
 */
void auo_epd_i80_update_screen(int dest, int x, int y, int w, int h, int rotate, int id, struct fb_info *p)
{
	jeffrey_head();
	
	u32 palette;
	u8 *ptr = p->screen_base +((y-1)*PANEL_WIDTH+(x-1))*((p->var.bits_per_pixel>>3));
	struct auo_epd_fb_par *par = p->par;
	u8 *ptr_line_start=ptr;


	if(w==828)		//&*&*&*J1 , ??
		w=825;

	switch(dest)
	{
	case SCREEN_BUFF_TYPE_DMA:
		break;
	case SCREEN_BUFF_TYPE_PIP:
		break;
	case SCREEN_BUFF_TYPE_CSR:
		break;
	case SCREEN_BUFF_TYPE_ANI:
		break;
	}


	pt_k1901_data_stop();


}

/*
 * Send pixels from shared memory to TCON
 * dest : send to DMA/PIP/CSR/ANI buffer in TCON
 * x, y, w, h : The area containing pixels, x and y starts from 1, please reference the TCON spec for constraints of  (x, y, w, h)
 * rotate : HP/L in TCON spec, is 0 when host image is landscape
 * id : There are multiple CSR/ANI buffers, so id is needed when sending pixels to CSR/ANI buffer
 * mem_addr : shared memory address
 * size : pixel count
 */
 
void k1901_send_image_gray256(int dest, int x, int y, int w, int h, int rotate, int id,	u8 *mem_addr,int size)
{
	
	jeffrey_head();
	
	pt_8pp_to_16(w,h,mem_addr);

	switch(dest)
	{
	case SCREEN_BUFF_TYPE_DMA:
		pt_k1901_dma_start( g_iAuoHPL, g_iAuoFBG, x-1, y-1, w, h, lcd_base_temp );	
		DBG("SCREEN_BUFF_TYPE_DMA .. g_iAuoHPL=%d, g_iAuoFBG=%d, x=%d, y=%d, w=%d, h=%d\n",g_iAuoHPL, g_iAuoFBG, x, y, w, h);
		break;
	case SCREEN_BUFF_TYPE_PIP:
		pt_k1901_pip_start( 0x3, g_iAuoHPL, x-1, y-1, w, h, lcd_base_temp  );	
		break;
	case SCREEN_BUFF_TYPE_CSR:
		pt_k1901_cursor_start( g_iAuoHPL, g_iAuoFBG, w, h, lcd_base_temp );		// Cursor Frame 1 (CS[5:0])
		break;
	case SCREEN_BUFF_TYPE_ANI:
		pt_k1901_animation_start( g_iAuoHPL,g_iAuoFBG, x-1, y-1, w, h, lcd_base_temp );	// Animation Frame 1 (AM[5:0])
		break;
	}
	pt_k1901_data_stop();


}

/*
 * Send pixels from shared memory to TCON (previous buffer)
 * Please reference TCON spec for PRE_DISPLAY_START/STOP
 */
void k1901_send_pre_display(int dest, int x, int y, int w, int h, int rotate, int id,
		u8 *mem_addr,int size)
{
	jeffrey_head();

	u16 cmd_data;

	pt_k1901_pre_display_start(  rotate, x, y, w, h, mem_addr );	
	pt_k1901_data_stop();
	


}

/*
 * Show images in TCON's DMA/PIP/CSR/ANI buffer with specified EPD mode
 * dest : send to DMA/PIP/CSR/ANI buffer in TCON
 * x, y, w, h : The area containing pixels, x and y starts from 1, please reference the TCON spec for constraints of  (x, y, w, h)
 * epd_mode : with EPD mode 0~4
 * flip : 1 if want to flip the image
 * id : There are multiple CSR/ANI buffers, so id is needed when for CSR/ANI buffer
 * csr_iprot : cursor rotation inplace
 */
void k1901_show_image(int dest, int x, int y, int w, int h, int epd_mode, int flip, int id, int csr_iprot)
{
	jeffrey_head();
	
	u16 data_to_send;
	pt_k1901_wait_not_busy();

	switch(dest)
	{
	case SCREEN_BUFF_TYPE_DMA:
		pt_k1901_ddma_start( g_iAuoFlash, epd_mode,g_iAuoRotation, x-1, y-1, w, h);
		DBG("IOCTL_CMD_SHOW_IMG_BUFF .. g_iAuoFlash=%d, epd_mode=%d,g_iAuoRotation=%d, x=%d, y=%d, w=%d, h=%d\n",g_iAuoFlash, epd_mode,g_iAuoRotation, x, y, w, h);
		break;
	case SCREEN_BUFF_TYPE_PIP:
		pt_k1901_dpip_start( g_iAuoFlash, epd_mode,g_iAuoRotation, x-1, y-1, w, h);
		break;
	case SCREEN_BUFF_TYPE_CSR:
		pt_k1901_dcursor_start( g_iAuoFlash, epd_mode, g_iAuoRotation, id, csr_iprot, x-1, y-1);		
		break;
	case SCREEN_BUFF_TYPE_ANI:
		pt_k1901_danimation_start( g_iAuoFlash, epd_mode, g_iAuoRotation,id, x-1, y-1, w, h);
		break;
	}
}
//================================================================//
//                                                                //
//           I/O CTRL Function                                    //
//                                                                //
//================================================================//

int auo_epd_fb_ioctl( struct fb_info *info, unsigned int cmd, unsigned long arg )
{
//	jeffrey_head();
	
	int return_val=0;
	
	unsigned int x1=1;	

	
	u16 data_to_send, final_packet;

	u8 * shm_addr;
	u16 *ptr;
	SEND_IMG_BUFF_PARAM * send_img_param_ptr;
	SHOW_IMG_BUFF_PARAM * show_img_param_ptr;

	static bool pending = false;

	u32 tmp;
	static u32 pull_down_time = 100;
	static int previous_mode = 0;

	struct timeval tv_cur, tv_pre;

	void __user *argp = (void __user *)arg;

#if JEF_DEBUG4
	DBG("Jeffrey .. auo_epd_fb_ioctl ..cmd = %d\n",cmd);
#endif	

	switch(cmd)
	{
	// Send pixels from shared memory to TCON
	

	case IOCTL_CMD_SEND_IMG_BUFF:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SEND_IMG_BUFF\n");
#endif
		send_img_param_ptr=(SEND_IMG_BUFF_PARAM *)arg;
		if((g_iRotationAngle==0) || (g_iRotationAngle==180))
		{
			send_img_param_ptr->rotate = 0;
		}
		else
		{
			send_img_param_ptr->rotate = 1;
		}
		shm_addr=(u8*)sys_shmat(send_img_param_ptr->shmid,0,0);
		if(shm_addr==(u8*)-1)
		{
			DBG("<1>SHMAT Failed\n");
			break;
		}
#if JEF_DEBUG4
		DBG("IOCTL_CMD_SEND_IMG_BUFF start\n");
		DBG("shm: addr=%x, size=%d, x=%d, y=%d,w=%d,h=%d, rotate=%d, id=%d, shmid=%d\n",
		send_img_param_ptr->dest,send_img_param_ptr->shm_size,
		send_img_param_ptr->x,send_img_param_ptr->y,send_img_param_ptr->w,send_img_param_ptr->h,
		send_img_param_ptr->rotate,send_img_param_ptr->id,send_img_param_ptr->shmid);
#endif	
		
#if JEF_DEBUG4
			DBG("IOCTL_CMD_SEND_IMG_BUFF .. error args: x=%d,y=%d,w=%d,h=%d\n",
					send_img_param_ptr->x,send_img_param_ptr->y,send_img_param_ptr->w,send_img_param_ptr->h);
#endif	
		x1 = (PANEL_WIDTH-send_img_param_ptr->w+send_img_param_ptr->x);
#if JEF_DEBUG4
			DBG("IOCTL_CMD_SEND_IMG_BUFF .. error args: x=%d,y=%d,w=%d,h=%d\n",
					send_img_param_ptr->x,send_img_param_ptr->y,send_img_param_ptr->w,send_img_param_ptr->h);
#endif	
		
		k1901_send_image_gray256(send_img_param_ptr->dest, x1, send_img_param_ptr->y, send_img_param_ptr->w, send_img_param_ptr->h, send_img_param_ptr->rotate,send_img_param_ptr->id, shm_addr,send_img_param_ptr->shm_size);
#if JEF_DEBUG4
		DBG("IOCTL_CMD_SEND_IMG_BUFF end\n");
#endif	

		if(sys_shmdt(shm_addr)==-1)
			DBG("SHMDT Fail\n");
		break;
		
	// Send pixels from shared memory to TCON (previous buffer)	
	case IOCTL_CMD_PRE_DISP_BUFF:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_PRE_DISP_BUFF\n");
#endif	
	
		break;		
		
	// Send pixels from fb to TCON	
	// This is designed to be sent by Qt's QWSServer when fb is updated.
	// User (application) can make driver ignore this ioctl temporarily by ioctl IOCTL_CMD_PENDNG_FB_DISPLAY
	case IOCTL_CMD_SEND_IMG_FB:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SEND_IMG_FB\n");
#endif	
	
		send_img_param_ptr=(SEND_IMG_BUFF_PARAM *)arg;

		if((g_iRotationAngle==0) || (g_iRotationAngle==180))
		{
			send_img_param_ptr->rotate = 0;
		}
		else
		{
			send_img_param_ptr->rotate = 1;
		}

		if(!pending)
		{
#if JEF_DEBUG4
			DBG("IOCTL_CMD_SEND_IMG_FB %d %d %d %d rotate%d\n",send_img_param_ptr->x, send_img_param_ptr->y,send_img_param_ptr->w, send_img_param_ptr->h, send_img_param_ptr->rotate);
#endif	
			auo_epd_i80_update_screen(send_img_param_ptr->dest, send_img_param_ptr->x, send_img_param_ptr->y, send_img_param_ptr->w, send_img_param_ptr->h, send_img_param_ptr->rotate, send_img_param_ptr->id, gFB_info);
		}
		break;
		
	// Show pixels in TCON buffer
	// epd_mode parameter in this ioctl will be ignored, variable refresh_mode will be used instead.
	// This is because IOCTL_CMD_SHOW_IMG_FB is designed to be sent by Qt's QWSServer when fb is updated.
	// But QWSServer does not know which epd mode is suitable. So we have another ioctl for user(application) to set epd refresh mode.
	// IOCTL_CMD_SHOW_IMG_BUFF is designed to be sent by user(application), but for convenience we make these two ioctls the same. 
	case IOCTL_CMD_SHOW_IMG_BUFF:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SHOW_IMG_BUFF\n");
#endif	
	
		show_img_param_ptr=(SHOW_IMG_BUFF_PARAM *)arg;
		show_img_param_ptr->epd_mode = g_iAuoRefreshMode; //refresh_mode;
		if((g_iRotationAngle==0) || (g_iRotationAngle==90))
		{
			show_img_param_ptr->flip = 0;
		}
		else
		{
			show_img_param_ptr->flip = 1;
		}
		if(show_img_param_ptr->epd_mode!=4)
		{
			// DDMA wait busyN to avoid "current to previous" be interrupted by DDMA command
			// If "current to previous" is interrupted, the previous buffer may be incorrect.
			// And non-flashed mode will looks strange
			while (check_busy()) { }
		}
		
		if ((show_img_param_ptr->w+show_img_param_ptr->x-1)>PANEL_WIDTH)	
		{
#if JEF_DEBUG4
			DBG("IOCTL_CMD_SHOW_IMG_BUFF .. error args: x=%d,y=%d,w=%d,h=%d\n",
					show_img_param_ptr->x,show_img_param_ptr->y,show_img_param_ptr->w,show_img_param_ptr->h);
#endif	
		}
		x1 = (PANEL_WIDTH-show_img_param_ptr->w+show_img_param_ptr->x);
		if (x1>PANEL_WIDTH)
		{
#if JEF_DEBUG4
			DBG("IOCTL_CMD_SHOW_IMG_BUFF .. error args: x=%d,y=%d,w=%d,h=%d\n",
					show_img_param_ptr->x,show_img_param_ptr->y,show_img_param_ptr->w,show_img_param_ptr->h);
#endif	
		}
		k1901_show_image(show_img_param_ptr->dest, x1, show_img_param_ptr->y, show_img_param_ptr->w, 
				show_img_param_ptr->h, show_img_param_ptr->epd_mode, show_img_param_ptr->flip,show_img_param_ptr->pip_mode_or_id, show_img_param_ptr->csr_iprot);
		DBG("IOCTL_CMD_SHOW_IMG_BUFF\n");
		break;
		
	// Show pixels in TCON buffer
	// This is designed to be sent by Qt's QWSServer when fb is updated.
	// User (application) can make driver ignore this ioctl temporarily by ioctl IOCTL_CMD_PENDNG_FB_DISPLAY
	case IOCTL_CMD_SHOW_IMG_FB:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SHOW_IMG_FB\n");
#endif	
	
		show_img_param_ptr=(SHOW_IMG_BUFF_PARAM *)arg;
		show_img_param_ptr->epd_mode = g_iAuoRefreshMode; //refresh_mode;
		if((g_iRotationAngle==0) || (g_iRotationAngle==90))
		{
			show_img_param_ptr->flip = 0;
		}
		else
		{
			show_img_param_ptr->flip = 1;
		}
		//  pending is true -> Ignore this ioctl because user (application) hopes.
		if(pending)
			break;
			
		// autoFlash is true -> when continuous non-flash updates more then non_flash_threshold,
		// a full update with mode 0 will be issued.
		if(autoFlash)
		{
			if((show_img_param_ptr->epd_mode == 1) || (show_img_param_ptr->epd_mode == 2))
			{
				non_flash_count++;
			}
			else
				non_flash_count = 0;
			if(non_flash_count>=non_flash_threshold)
			{					
				show_img_param_ptr->x = 1;
				show_img_param_ptr->y = 1;
				if(g_iRotationAngle==0)
				{
					show_img_param_ptr->w = PANEL_WIDTH;
					show_img_param_ptr->h = PANEL_HEIGHT;
				}
				else
				{
					show_img_param_ptr->w = PANEL_HEIGHT;
					show_img_param_ptr->h = PANEL_WIDTH;
				}
				show_img_param_ptr->epd_mode = 0;
				non_flash_count = 0;
			}
		}
		if(show_img_param_ptr->epd_mode!=4)
		{
			// DDMA wait busyN to avoid "current to previous" be interrupted by DDMA command
			// If "current to previous" is interrupted, the previous buffer may be incorrect.
			// And non-flashed mode will looks strange
			while (check_busy()) { }
		}
		k1901_show_image(show_img_param_ptr->dest, show_img_param_ptr->x, show_img_param_ptr->y, show_img_param_ptr->w,
					show_img_param_ptr->h, show_img_param_ptr->epd_mode, show_img_param_ptr->flip,show_img_param_ptr->pip_mode_or_id, show_img_param_ptr->csr_iprot);
		
		previous_mode = show_img_param_ptr->epd_mode;				
		break;
		
	// Reset display	
	case IOCTL_CMD_DISP_RESET:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_DISP_RESET\n");
#endif	
	
		break;
		
	// Refresh display with current frame	
	case IOCTL_CMD_DISP_REFRESH:
		pt_k1901_full_reshow(g_iAuoRefreshMode);
	
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_DISP_REFRESH\n");
#endif	
	
		break;
	
	// User can use this ioctl to send any command to TCON	
	case IOCTL_CMD_SEND_COMMAND:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SEND_COMMAND .. arg =%x\n",arg);
#endif	

		GET_DISPLAY_OWNER	// get display owner
		
		data_to_send = arg >>16;
		final_packet = arg & 0xFFFF;
		
#if JEF_DEBUG4
		DBG("jeffrey .. data_to_send=%x, final_packet=%x\n",data_to_send,final_packet);
#endif	
		

		// standby command is valid when TCON is not busy
		// Actually there are sevaral other commands need to wait for busyN, please reference TCON spec
		if(data_to_send == 0x0001)
		{
#if JEF_DEBUG4
			DBG("standby wait busyn\n");
#endif	
			while(check_busy())
			{
			}
		}
		// When TCON is in standby mode, or when command is standby/wakeup/reset, a slower I80 command timing 
		if(data_to_send==0x0001 || data_to_send==0x0002 || data_to_send==0x0003 || g_bSleep)
		{
			MPU_nCS_Assert();
			command_buff[0] = data_to_send ;
			mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
			if (final_packet)
			{
				MPU_nCS_Deassert();
#if JEF_DEBUG4
				DBG("jeffrey .. run final_packet\n");
#endif	
			}
			else
			{
#if JEF_DEBUG4
				DBG("jeffrey .. Do not run final_packet\n");
#endif	
			}
		}
		else
		{
			{
				command_buff[0] = data_to_send ;
				MPU_nCS_Assert();
				mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
				if (final_packet)
				{
#if JEF_DEBUG4
					DBG("jeffrey .. run final_packet\n");
#endif	
					MPU_nCS_Deassert();
				}
				else
				{
#if JEF_DEBUG4
					DBG("jeffrey .. Do not run final_packet\n");
#endif	
				}
			}
		}
		
		if(data_to_send==0x0001)
		{
			g_bSleep = 1;
		}
		else if(data_to_send == 0x0002)
		{
			g_bSleep = 0;
		}
		
		FREE_DISPLAY_OWNER	// Release display owner
		break;
	// User can use this ioctl to send data to TCON	
	case IOCTL_CMD_SEND_DATA:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SEND_DATA\n");
#endif	
		GET_DISPLAY_OWNER	// get display owner
		
		data_to_send = arg >>16;
		final_packet = arg & 0xFFFF;

		parameter_buff[0] = data_to_send;
		mpu_write_32( MPU_DATA, parameter_buff, 1 , 1 );		

		if(final_packet)
			MPU_nCS_Deassert();

		FREE_DISPLAY_OWNER	// Release display owner
		break;
		
	// User can use this ioctl to read data from TCON		
	case IOCTL_CMD_READ_DATA:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_READ_DATA\n");
#endif	
	
		copy_to_user((void *)arg,&data_to_send,2);
		break;
	
	// IOCTL_CMD_SHOW_IMG_FB is designed to be sent by Qt's QWSServer when fb is updated.
	// But QWSServer does not know which epd mode is suitable. So we have another ioctl for user(application) to set epd refresh mode.
	case IOCTL_CMD_SET_REFRESH_MODE:
		//refresh_mode = (unsigned int)arg;
		g_iAuoRefreshMode = (unsigned int)arg;
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SET_REFRESH_MODE = %d\n",g_iAuoRefreshMode);
#endif	
		break;
	
	// IOCTL_CMD_SHOW_IMG_FB is designed to be sent by Qt's QWSServer when fb is updated.
	// But QWSServer does not know which rotation angle is suitable. So we have another ioctl for user(application) to set rotation angle.	
	case IOCTL_CMD_SET_ROTATION_ANGLE:
		g_iRotationAngle = (unsigned int)arg;
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SET_ROTATION_ANGLE .. RotationAngle=%d\n",g_iRotationAngle);
#endif	
		break;
	
	// User (application) can make driver ignore this IOCTL_CMD_SEND_IMG_FB/IOCTL_CMD_SHOW_IMG_FB sent by Qt's QWSServer
	case IOCTL_CMD_PENDNG_FB_DISPLAY:		
#if JEF_DEBUG4
		printk("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_PENDNG_FB_DISPLAY\n");
#endif	
				
		pending = true;			
		break;
		
	// User (application) can make driver not to ignore this IOCTL_CMD_SEND_IMG_FB/IOCTL_CMD_SHOW_IMG_FB sent by Qt's QWSServer	
	// When resume to accept IOCTL_CMD_SEND_IMG_FB/IOCTL_CMD_SHOW_IMG_FB, driver will do a full update first.
	case IOCTL_CMD_RESUME_FB_DISPLAY:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_RESUME_FB_DISPLAY\n");
#endif	
		break;
	
	
	//The driver will count continuous non-flash mode (1, 2) updates.
	//if autoFlash true, when non_flash_count exceeds non_flash_threshold, 
    //a full screen mode 0 update will be issued automatically.
	case IOCTL_CMD_DISABLE_AUTO_FLASH:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_DISABLE_AUTO_FLASH\n");
#endif	
	
		autoFlash = false;
		non_flash_count = 0;
		break;
	case IOCTL_CMD_ENABLE_AUTO_FLASH:		
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_ENABLE_AUTO_FLASH\n");
#endif	
	
		autoFlash = true;
		break;
	case IOCTL_CMD_SET_AUTO_FLASH_TIMES:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SET_AUTO_FLASH_TIMES\n");
#endif	
	
		non_flash_threshold = (unsigned int)arg;
		DBG("non_flash_threshold = %d\n",non_flash_threshold);
		break;
	case IOCTL_CMD_AUTO_FLASH_NOW:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_AUTO_FLASH_NOW\n");
#endif	
	
		non_flash_count = non_flash_threshold+1;
		break;
		
	case IOCTL_CMD_WRITE_G2_LUT:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_WRITE_G2_LUT\n");
#endif	
	
		auo_epd_i80_LUT_write((unsigned int)arg);
		break;
	case IOCTL_CMD_READ_G2_LUT:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_READ_G2_LUT\n");
#endif	
	
		auo_epd_i80_LUT_read((unsigned int)arg);
		break;
	case IOCTL_CMD_CHECKBUSY:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_CHECKBUSY\n");
#endif	
		data_to_send = check_busy();
		ptr = (u16*)arg;
		copy_to_user((void *)arg,&data_to_send,2);
		break;
	case IOCTL_CMD_READ_EPD_MODE:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_READ_EPD_MODE\n");
#endif	
	
//		copy_to_user((void *)arg,&refresh_mode,2);
		copy_to_user((void *)arg,&g_iAuoRefreshMode,2);
		
		break;
	case IOCTL_CMD_READ_ROTATION_ANGLE:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_READ_ROTATION_ANGLE\n");
#endif	
	
		copy_to_user((void *)arg,&g_iRotationAngle,2);
		break;
	
	// ioctls below are just for testing
	case IOCTL_CMD_PULL_SLP_RST_DOWN:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_PULL_SLP_RST_DOWN\n");
#endif	
#if 0	
		DBG("IOCTL_CMD_PULL_SLP_RST_DOWN");
		while(((__raw_readl(S3C_GPGDAT) &(0x1<<5)) != 0x00))
		{
			// wait until not busy			
		}
		do_gettimeofday(&tv_pre);
#if JEF_DEBUG4
		DBG("tv_pre %d %d\n",tv_pre.tv_sec, tv_pre.tv_usec);
#endif	
		while(1)
		{
			do_gettimeofday(&tv_cur);
			if((__raw_readl(S3C_GPGDAT) &(0x1<<5)) != 0x00)
			{
#if JEF_DEBUG4
				DBG("return because of busyn is high\n");
				DBG("tv_cur %d %d\n",tv_cur.tv_sec, tv_cur.tv_usec);
#endif	
				return;
			}
			if(((tv_cur.tv_sec * 1000000 + tv_cur.tv_usec) - (tv_pre.tv_sec * 1000000 + tv_pre.tv_usec))>pull_down_time*1000)
			{
				tmp=__raw_readl(S3C_GPHDAT);
				tmp &= (~(1<<12));
				__raw_writel(tmp, S3C_GPHDAT);
				mdelay(10);
				tmp=__raw_readl(S3C_GPFDAT);
				tmp &= (~(1<<5));
				__raw_writel(tmp, S3C_GPFDAT);
				tmp=__raw_readl(S3C_GPBDAT);
				tmp &=  (~(1<<3));
				__raw_writel(tmp, S3C_GPBDAT);
#if JEF_DEBUG4
				DBG("SLP and RST_N and VDD down\n");
#endif	
				return;
			}
			udelay(10);		
		}
#endif		
		break;
	case IOCTL_CMD_PULL_SLP_RST_UP:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_PULL_SLP_RST_UP\n");
#endif	
	
#if 0	
		tmp=__raw_readl(S3C_GPHDAT);
		tmp |= (1<<12);
		__raw_writel(tmp, S3C_GPHDAT);
		tmp=__raw_readl(S3C_GPFDAT);
		tmp |= (1<<5);
		__raw_writel(tmp, S3C_GPFDAT);
#if JEF_DEBUG4
		DBG("SLP and RST_N up\n");
#endif		
#endif
		break;
	case IOCTL_CMD_SET_PULL_DOWN_TIME:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_SET_PULL_DOWN_TIME\n");
#endif	
	
		pull_down_time = (unsigned int)arg;
		DBG("pull_down_time set to = %d (us)\n", pull_down_time);	
		break;

	case IOCTL_CMD_POWER_ON:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_POWER_ON\n");
#endif
#if 0	
		tmp=__raw_readl(S3C_GPBDAT);
		tmp |=  (1<<3);
		__raw_writel(tmp, S3C_GPBDAT);
		tmp=__raw_readl(S3C_GPHDAT);
		tmp |= (1<<12);
		__raw_writel(tmp, S3C_GPHDAT);
		mdelay(1);
		tmp=__raw_readl(S3C_GPFDAT);
		tmp |= (1<<5);
		__raw_writel(tmp, S3C_GPFDAT);
		
		while(((__raw_readl(S3C_GPGDAT) &(0x1<<5)) != 0x00)){}	
		auo_epd_i80_cs_low();
		auo_epd_i80_rs_low();	
		auo_epd_i80_send_cmd(I80_CMD_INIT_SET,false);
		auo_epd_i80_rs_high();
		auo_epd_i80_send_data(0x0000,true);
		auo_epd_i80_cs_high();
#endif		
		break;
	case IOCTL_CMD_POWER_OFF:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_POWER_OFF\n");
#endif	
	
#if JEF_DEBUG4
		DBG("power off\n");
#endif	
#if 0		
		tmp=__raw_readl(S3C_GPBDAT);
		tmp &=  (~(1<<3));
		__raw_writel(tmp, S3C_GPBDAT);

		tmp=__raw_readl(S3C_GPBDAT);
		tmp &=  (~(1<<1));
		__raw_writel(tmp, S3C_GPBDAT);
#endif		
		break;

	//&*&*&*J1_add		
	case IOCTL_CMD_FB_AUTO_REFRESH:
		g_iAutoRefreshMode = (unsigned int)arg;
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = IOCTL_CMD_FB_REFRESH_STOP = %d\n",g_iAutoRefreshMode);
#endif	
		
		break;
	case IOCTL_CMD_SHOW_FB_IMG_BUFF:
		printk("jeffrey .. get IOCTL_CMD_SHOW_FB_IMG_BUFF\n");
		show_img_param_ptr=(SHOW_IMG_BUFF_PARAM *)arg;
		show_img_param_ptr->epd_mode = g_iAuoRefreshMode; //refresh_mode;
		printk("IOCTL_CMD_SHOW_IMG_BUFF .. args: x=%d,y=%d,w=%d,h=%d,mode=%d\n",
				show_img_param_ptr->x,show_img_param_ptr->y,show_img_param_ptr->w,show_img_param_ptr->h,
				show_img_param_ptr->epd_mode);

		if((g_iRotationAngle==0) || (g_iRotationAngle==90))
		{
			show_img_param_ptr->flip = 0;
		}
		else
		{
			show_img_param_ptr->flip = 1;
		}
		if(show_img_param_ptr->epd_mode!=4)
		{
			// DDMA wait busyN to avoid "current to previous" be interrupted by DDMA command
			// If "current to previous" is interrupted, the previous buffer may be incorrect.
			// And non-flashed mode will looks strange
			while (check_busy()) { }
		}

		if ((show_img_param_ptr->w+show_img_param_ptr->x-1)>PANEL_WIDTH)	
		{
		}					
		x1 = (PANEL_WIDTH-show_img_param_ptr->w-show_img_param_ptr->x);
		if (x1>PANEL_WIDTH)
		{
		}

		if ((show_img_param_ptr->x==0)&&(show_img_param_ptr->y==0)&&(show_img_param_ptr->w==768)&&(show_img_param_ptr->h==1024))
		{
			GET_DISPLAY_OWNER
			switch(PANEL_BPP)
			{
				case 24:
					pt_24bpp_to_16();
					break;
				case 16:
				default:
					pt_16bpp_to_16_rev();
					break;
			}
			pt_k1901_full_refresh();
			FREE_DISPLAY_OWNER			
		}
		else
		{
			GET_DISPLAY_OWNER
			pt_16bpp_to_16_rev2(show_img_param_ptr->x, show_img_param_ptr->y, show_img_param_ptr->w,show_img_param_ptr->h);//pt_16bpp_to_16();
			pt_k1901_part_refresh(show_img_param_ptr->epd_mode,x1, show_img_param_ptr->y, show_img_param_ptr->w,show_img_param_ptr->h,lcd_base_temp);
			FREE_DISPLAY_OWNER			
		}
		break;
		
	case MXCFB_SET_WAVEFORM_MODES:
		{
			struct mxcfb_waveform_modes modes;
			if (!copy_from_user(&modes, argp, sizeof(modes))) {
				mxc_epdc_fb_set_waveform_modes(&modes, info);
				return_val = 0;
			}
			break;
		}
	case MXCFB_SET_TEMPERATURE:
		{
			int temperature;
			if (!get_user(temperature, (int32_t __user *) arg))
				return_val = mxc_epdc_fb_set_temperature(temperature,
					info);
			break;
		}
	case MXCFB_SET_AUTO_UPDATE_MODE:
		{
			u32 auto_mode = 0;
			if (!get_user(auto_mode, (__u32 __user *) arg))
				return_val = mxc_epdc_fb_set_auto_update(auto_mode,
					info);
			break;
		}
	case MXCFB_SET_UPDATE_SCHEME:
		{
			u32 upd_scheme = 0;
			if (!get_user(upd_scheme, (__u32 __user *) arg))
				return_val = mxc_epdc_fb_set_upd_scheme(upd_scheme,
					info);
			break;
		}
	case MXCFB_SEND_UPDATE:
		{
			struct mxcfb_update_data upd_data;
			if (!copy_from_user(&upd_data, argp,
				sizeof(upd_data))) 
			{
#if JEF_DEBUG4
			
					DBG("Jeffrey .. c1\n");
#endif	
				return_val = mxc_epdc_fb_send_update(&upd_data, info);
				if (return_val == 0 && copy_to_user(argp, &upd_data,
					sizeof(upd_data)))
					return_val = -EFAULT;
			} else {
				return_val = -EFAULT;
			}

			break;
		}
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		{
			u32 update_marker = 0;
			if (!get_user(update_marker, (__u32 __user *) arg))
				return_val =
				    mxc_epdc_fb_wait_update_complete(update_marker,
					info);
			break;
		}

	case MXCFB_SET_PWRDOWN_DELAY:
		{
			int delay = 0;
			if (!get_user(delay, (__u32 __user *) arg))
				return_val =
				    mxc_epdc_fb_set_pwrdown_delay(delay, info);
			break;
		}

	case MXCFB_GET_PWRDOWN_DELAY:
		{
			int pwrdown_delay = mxc_epdc_get_pwrdown_delay(info);
			if (put_user(pwrdown_delay,
				(int __user *)argp))
				return_val = -EFAULT;
			return_val = 0;
			break;
		}
	default:
#if JEF_DEBUG4
		DBG("Jeffrey .. auo_epd_fb_ioctl = Unknow code:%d\n",cmd);
#endif	
	
		return -ENOTTY;
	}

	return return_val;
}


//================================================================//
//                                                                //
//           Halftone Function                                    //
//                                                                //
//================================================================//
bool halftone_enable=false;

int halftone_matrix[4][4]={
		{ 0,14, 3,13},
		{11, 5, 8, 6},
		{12, 2,15, 1},
		{ 7, 9, 4,10}};

u8 halftone16_render(u8 gray, int x, int y)
{
	jeffrey_head();

	u16 ret_gray;
	ret_gray=gray+ halftone_matrix[x%4][y%4];
	if(ret_gray>255) {
		ret_gray=255;
	}
	return (u8)ret_gray;
}

//=================================================================================================
//===============================================================================//
// Project Name: Startup Logo for EPD                                            //
// Engineer: Stanley Fan                                                         //
// Create Date: 2010/07/12                                                       //
// Revision:                                                                     //
// Version: 0.1 Release Date: 2009/07/12                                         //
//===============================================================================//
#define u8 unsigned char



//&*&*&*J1_add: for AUO panel K1901
//=======================================================================================================================
#define SEND_DELAY	20 //3 //3 //20 //5 //20 //5 //20	// Delay ok : 20,10,5

static void	mpu_write_32( unsigned int is_data, unsigned char* buffer, unsigned int count_width, unsigned int count_height )
{
	unsigned int val;

	msleep(SEND_DELAY); 
	
	__raw_writel((0x03 << 10) | (0x03 << 8),epdc_base + HW_ELCDIF_CTRL_CLR);

	val =( __raw_readl(epdc_base + HW_ELCDIF_CTRL1) & ~(0x0f << 16 ) ) | (0x03 << 16 );	

	__raw_writel(val,epdc_base + HW_ELCDIF_CTRL1);

	__raw_writel((0x02<<24) | (0x02<<16) | (0x02<<8) | (0x02<<0),epdc_base + HW_ELCDIF_TIMING);

	if ( is_data )
	{
		__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_SET);
	}
	else
	{
		__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_CLR);
	}

	__raw_writel(CTRL_READ_WRITEB,epdc_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(CTRL_LCDIF_MASTER,epdc_base + HW_ELCDIF_CTRL_SET);
	__raw_writel((count_height << 16) | count_width ,epdc_base + HW_ELCDIF_TRANSFER_COUNT);
	__raw_writel(__pa(buffer),epdc_base + HW_ELCDIF_CUR_BUF);

	__raw_writel(CTRL_RUN,epdc_base + HW_ELCDIF_CTRL_SET);

	while(1)
	{
		val =( __raw_readl(epdc_base + HW_ELCDIF_CTRL) & CTRL_RUN  );	
//		DBG("jeffrey .. val=%d\n",val);
		if (!val)
			break;
	}
		
	__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_SET);

	
	msleep(SEND_DELAY); //&*&*&*j1_add: delay for stable
	
	jeffrey_tail();
}



static void	mpu_write_16( unsigned int is_data, unsigned char* buffer, unsigned int count_width, unsigned int count_height )
{
	unsigned int val;
	

	jeffrey_head();
	msleep(SEND_DELAY); 
	__raw_writel((0x03 << 10) | (0x03 << 8),epdc_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel((0x0f << 16),epdc_base + HW_ELCDIF_CTRL1_SET);
	__raw_writel( (0x02<<24) | (0x02<<16) | (0x02<<8) | (0x02<<0),epdc_base + HW_ELCDIF_TIMING);
	__raw_writel(CTRL_READ_WRITEB,epdc_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel( CTRL_LCDIF_MASTER,epdc_base + HW_ELCDIF_CTRL_SET);
	__raw_writel( ((count_height << 16) | count_width),epdc_base + HW_ELCDIF_TRANSFTER_COUNT);
	__raw_writel( __pa(buffer),epdc_base + HW_ELCDIF_CUR_BUF);
	if ( is_data )
	{
		__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_SET);
	}
	else
	{
		__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_CLR);
	}
	__raw_writel(CTRL_RUN,epdc_base + HW_ELCDIF_CTRL_SET);
	while(1)
	{
		val =( __raw_readl(epdc_base + HW_ELCDIF_CTRL) & CTRL_RUN  );	
//		DBG("jeffrey .. val=%d\n",val);
		if (!val)
			break;
	}
	__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_SET);
	printk("jeffrey .. aaa\n");
	msleep(SEND_DELAY); //&*&*&*j1_add: delay for stable

	jeffrey_tail();
}



static void	mpu_read_32( unsigned int is_data, unsigned char* buffer, unsigned int count_width, unsigned int count_height )
{
	unsigned int val;


	jeffrey_tag();

	__raw_writel( (0x03 << 10) | (0x03 << 8),epdc_base + HW_ELCDIF_CTRL_SET);
	val =( (__raw_readl(epdc_base + HW_ELCDIF_CTRL1) & ~(0x0f << 16 ) ) | (0x07 << 16) );	
	__raw_writel(val,epdc_base + HW_ELCDIF_CTRL1);
	__raw_writel((8<<24) | (8<<16) | (8<<8) | (8<<0),epdc_base + HW_ELCDIF_TIMING);
	if ( is_data )
	{
		__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_SET);
	}
	else
	{
		__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_CLR);
	}
		
		__raw_writel(CTRL_READ_WRITEB,epdc_base + HW_ELCDIF_CTRL_SET);
		__raw_writel(CTRL_LCDIF_MASTER,epdc_base + HW_ELCDIF_CTRL_CLR);
		__raw_writel( (count_height << 16) | count_width,epdc_base + HW_ELCDIF_TRANSFTER_COUNT);
		__raw_writel(__pa(buffer),epdc_base + HW_ELCDIF_CUR_BUF);
		__raw_writel(CTRL_RUN,epdc_base + HW_ELCDIF_CTRL_SET);
	while(1)
	{
		val =( __raw_readl(epdc_base + HW_ELCDIF_CTRL) & CTRL_RUN  );	
//		DBG("jeffrey .. val=%d\n",val);
		if (!val)
			break;
	}
	__raw_writel(CTRL_DATA_SELECT,epdc_base + HW_ELCDIF_CTRL_SET);

	jeffrey_tag();
}


//------------
static void	pt_k1901_dma_start( unsigned int HPL, unsigned int FBG, 
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff )
{

	jeffrey_head();	
	MPU_nCS_Assert();
	command_buff[0] = 0x1001 ;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = ( HPL << 13 ) | ( FBG << 12 ) | ( x + 1) ;
	parameter_buff[1] = ( y + 1 );
	parameter_buff[2] = w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4 , 1 );
	mpu_write_16( MPU_DATA, buff, w/4, h );
	MPU_nCS_Deassert();
	jeffrey_tail();
}

static void pt_k1901_ddma_start( unsigned int flash, unsigned int mode, unsigned int rota_n,
								unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
	jeffrey_head();
	command_buff[0] = 0x1009;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = ( flash << 15 ) | ( mode << 12 ) | (rota_n << 11 );
	parameter_buff[1] = ( x + 1 ) ;
	parameter_buff[2] = ( y + 1 );
	parameter_buff[3] = w;
	parameter_buff[4] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 5, 1 );
	MPU_nCS_Deassert();
	jeffrey_tail();
}



//------------
static void	pt_k1901_pip_start( unsigned int HPL, unsigned int FBG, 
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff )
{

	jeffrey_head();	
	MPU_nCS_Assert();
	command_buff[0] = 0x1008 ;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = ( HPL << 13 ) | ( FBG << 12 ) | ( x + 1) ;
	parameter_buff[1] = ( y + 1 );
	parameter_buff[2] = w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4 , 1 );
	mpu_write_16( MPU_DATA, buff, w/4, h );
	MPU_nCS_Deassert();
	jeffrey_tail();
}

static void pt_k1901_dpip_start( unsigned int flash, unsigned int mode, unsigned int rota_n,
								unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
	jeffrey_head();
	command_buff[0] = 0x100B;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = (rota_n << 15 ) | ( mode << 12 ) | ( x + 1 ) ; ;
	parameter_buff[1] = ( flash << 15 ) | ( y + 1 );
	parameter_buff[2] = w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4, 1 );
	MPU_nCS_Deassert();	
	jeffrey_tail();
}


//------------
static void	pt_k1901_cursor_start( unsigned int CPL, unsigned int CS, 
		unsigned int w, unsigned int h,
		unsigned char* buff )
{

	jeffrey_head();	
	MPU_nCS_Assert();
	command_buff[0] = 0x1007 ;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = ( CPL << 12)  ;
	parameter_buff[1] = ((CS>>4)&0x3) << 12;
	parameter_buff[2] = ((CS&0xf) << 12) | w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4 , 1 );
	mpu_write_16( MPU_DATA, buff, w/4, h );
	MPU_nCS_Deassert();
	jeffrey_tail();
}


static void pt_k1901_dcursor_start( unsigned int flash, unsigned int mode, unsigned int rota_n, 
								unsigned int CS, unsigned int CSR_IPROT,
								unsigned int x, unsigned int y )
{
	jeffrey_head();
	command_buff[0] = 0x100A;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = (rota_n << 15 ) | ( mode << 12 ) | ( x + 1 ) ;
	parameter_buff[1] = ( flash << 15 ) | (((CS>>4)&0x3)<<12) | ( y + 1 );
	parameter_buff[2] = ((CS&0xf)<<12)  | (CSR_IPROT << 10);
	mpu_write_32( MPU_DATA, parameter_buff, 3, 1 );
	MPU_nCS_Deassert();	
	jeffrey_tail();
}

//------------
static void	pt_k1901_animation_start( unsigned int APL, unsigned int AMFrameNo,
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff )
{

	jeffrey_head();	
	MPU_nCS_Assert();
	command_buff[0] = 0x100C ;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] =  ( APL << 12 ) | ( x + 1) ;
	parameter_buff[1] = (((AMFrameNo>>4)&0x3) << 12) | ( y + 1 );
	parameter_buff[2] = (( AMFrameNo & 0xf) << 12) | w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4 , 1 );
	mpu_write_16( MPU_DATA, buff, w/4, h );
	MPU_nCS_Deassert();
	jeffrey_tail();
}

static void pt_k1901_danimation_start( unsigned int flash, unsigned int mode, unsigned int rota_n,unsigned int AMFrameNo,
								unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
	jeffrey_head();
	command_buff[0] = 0x100D;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = (rota_n << 15 ) | ( mode << 12 ) | ( x + 1 ) ;
	parameter_buff[1] = ( flash << 15 ) | (((AMFrameNo>>4)&0x3)<<12) |( y + 1 );
	parameter_buff[2] = ((AMFrameNo&0xf) << 12) | w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4, 1 );
	MPU_nCS_Deassert();	
	jeffrey_tail();
}


//------------
static void	pt_k1901_pre_display_start( unsigned int EPL, 
		unsigned int x, unsigned int y, 
		unsigned int w, unsigned int h,
		unsigned char* buff )
{
	jeffrey_head();	
	
	MPU_nCS_Assert();
	command_buff[0] = 0x100E ;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = ( EPL << 13 ) | ( x + 1 ) ;
	parameter_buff[1] = ( y + 1 );
	parameter_buff[2] = w;
	parameter_buff[3] = h;
	mpu_write_32( MPU_DATA, parameter_buff, 4 , 1 );
	mpu_write_16( MPU_DATA, buff, w/4, h );
	MPU_nCS_Deassert();

	jeffrey_tail();
}





static void	pt_k1901_data_stop(void)
{
	command_buff[0] = 0x1002;
	MPU_nCS_Assert();
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	MPU_nCS_Deassert();
	jeffrey_tail();
}



static void	pt_k1901_reset(void)
{
//	jeffrey_debug();	
	command_buff[0] = 0x0003;
	MPU_nCS_Assert();
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	msleep(5);
	MPU_nCS_Deassert();
	jeffrey_tail();
}

static void	pt_k1901_wait_not_busy(void)
{
	jeffrey_head();
	while( ! (Get_MPU_nBusy()) )
		udelay( 10 );
	jeffrey_tail();
}



static void pt_k1901_hw_reset(void)
{
	// Reset auo chip
	//----------------------------------------
	MPU_nRESET_Deassert();
	MPU_nRESET_Assert();
	msleep(5);
	MPU_nRESET_Deassert();
	MPU_nSLEEP_Deassert();
	
}

static void pt_k1901_init_set(void)
{
	command_buff[0] = 0x0000;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = 0x04;
	mpu_write_32( MPU_DATA, parameter_buff, 1, 1 );
	MPU_nCS_Deassert();
}

	
	

static void	pt_k1901_aging(void)
{
	jeffrey_debug();	
	command_buff[0] = 0x6000;
	
	MPU_nCS_Assert();
	jeffrey_head();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	jeffrey_head();	
	MPU_nCS_Deassert();
	jeffrey_tail();
}

static void	pt_k1901_aging_exit(void)
{
	jeffrey_head();	
	command_buff[0] = 0x6001;
	MPU_nCS_Assert();
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	MPU_nCS_Deassert();
	jeffrey_tail();
}



static void	pt_k1901_read_version(void)		
{
	jeffrey_head();
	command_buff[0] = 0x4000;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	mpu_read_32( MPU_DATA, parameter_buff, 1, 4 );
	MPU_nCS_Deassert();	

	DBG( "===LULU=== [1] = %08X\r\n", parameter_buff[0] );
	DBG( "===LULU=== [2] = %08X\r\n", parameter_buff[1] );
	DBG( "===LULU=== [3] = %08X\r\n", parameter_buff[2] );
	DBG( "===LULU=== [4] = %08X\r\n", parameter_buff[3] );

	jeffrey_tail();	
}


static void	pt_k1901_read_status(void)	//&*&*&*J1_add: not test
{
	jeffrey_head();
	command_buff[0] = 0x4001;
	MPU_nCS_Assert();	
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	mpu_read_32( MPU_DATA, parameter_buff, 1, 1 );
	MPU_nCS_Deassert();	

	DBG( "===LULU=== Status = %08X\r\n", parameter_buff[0] );
	jeffrey_tail();	

}



static void pt_k1901_full_refresh(void)
{
	int iCount=0;
	/* Draw black border around framebuffer*/
loop1:	
	pt_k1901_dma_start( g_iAuoHPL, g_iAuoFBG, 0, 0, PANEL_WIDTH, PANEL_HEIGHT, lcd_base );	//1024*768
	pt_k1901_data_stop();
	pt_k1901_wait_not_busy();
	pt_k1901_ddma_start( g_iAuoFlash, g_iAuoRefreshMode, g_iAuoRotation, 0, 0, PANEL_WIDTH, PANEL_HEIGHT );	//1024*768
	if (Get_MPU_nBusy())	// Not run
	{
		iCount++;
		printk("T.\n");
		if (iCount<16)
			goto loop1;
	}
}

static void pt_k1901_full_refresh_mode(unsigned int mode)
{

	int iCount=0;

	/* Draw black border around framebuffer*/
loop1:	
	pt_k1901_dma_start( g_iAuoHPL, g_iAuoFBG, 0, 0, PANEL_WIDTH, PANEL_HEIGHT, lcd_base );	//1024*768
	pt_k1901_data_stop();
	pt_k1901_wait_not_busy();
	pt_k1901_ddma_start( g_iAuoFlash, mode, g_iAuoRotation, 0, 0, PANEL_WIDTH, PANEL_HEIGHT );	//1024*768


	if (Get_MPU_nBusy())	// Not run
	{
		iCount++;
		
		printk("T.\n");
		if (iCount<16)
			goto loop1;
		
	}
}



static void pt_k1901_full_reshow(unsigned int mode)
{
	/* Draw black border around framebuffer*/
	pt_k1901_ddma_start( g_iAuoFlash, mode, g_iAuoRotation, 0, 0, PANEL_WIDTH, PANEL_HEIGHT );	//1024*768
}

static void pt_k1901_part_refresh(unsigned int mode,unsigned int x, unsigned int y, unsigned int w, unsigned int h,unsigned char* buff)
{
//	DBG("g_iAuoHPL=%d,g_iAuoFBG=%d,g_iAuoFlash=%d,g_iAuoRefreshMode=%d,g_iAuoRotation=%d\n"
//		,g_iAuoHPL,g_iAuoFBG,g_iAuoFlash,g_iAuoRefreshMode,g_iAuoRotation);
	printk("x=%d,y=%d,w=%d,h=%d\n",x,y,w,h);

	/* Draw black border around framebuffer*/
	
	int iCount=0;
	
loop1:		
	pt_k1901_dma_start( g_iAuoHPL, g_iAuoFBG, x, y, w, h, buff );	
	pt_k1901_data_stop();		
	pt_k1901_wait_not_busy();
	pt_k1901_ddma_start( g_iAuoFlash, mode, g_iAuoRotation, x, y, w, h );	
	if (Get_MPU_nBusy())	
	{
		iCount++;
		
		printk("T.\n");
		if (iCount<16)
			goto loop1;
		
	}
}

static void pt_k1901_part_reshow(unsigned int mode,unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
//	DBG("g_iAuoHPL=%d,g_iAuoFBG=%d,g_iAuoFlash=%d,g_iAuoRefreshMode=%d,g_iAuoRotation=%d\n"
//		,g_iAuoHPL,g_iAuoFBG,g_iAuoFlash,g_iAuoRefreshMode,g_iAuoRotation);
	/* Draw black border around framebuffer*/
	pt_k1901_ddma_start( g_iAuoFlash, mode, g_iAuoRotation, x, y, w, h );	//1024*768
}


static void	pt_k1901_blank_black(void)
{

	jeffrey_head();

	memset( lcd_base, 0x00, PANEL_WIDTH * PANEL_HEIGHT / 2 );
	pt_k1901_full_refresh();
	jeffrey_tail();

}

static void	pt_k1901_blank_white(void)
{

	jeffrey_head();
	memset( lcd_base, 0xFF, PANEL_WIDTH * PANEL_HEIGHT / 2 );
	pt_k1901_full_refresh();
	jeffrey_tail();

}



#define FB_DEBUG1 	0
#define FB_DEBUG2	0
#define FB_DEBUG3	0
static void pt_24bpp_to_16(void)
{
	int i;
	int j;
	int k1,k2;
	unsigned int data_width=0;
	
	unsigned int idx1,idx2;
	struct fb_info *info;						// fbinfo

	
	
	k1=0;	
	k2=0;
	
	info = &g_fb_data->info;
	

	data_width = PANEL_WIDTH/2;
	for (i=1;i<=PANEL_HEIGHT;i++)
	{
		k1 = data_width*i-1;
		for(j=0;j<data_width;j++)
		{
			idx1 = GrayScaleLuTable[((gFB_info->screen_base[k2]*77) + (gFB_info->screen_base[k2+1]*151) + (gFB_info->screen_base[k2+2]*28))>>8]; 
			idx2 = GrayScaleLuTable[((gFB_info->screen_base[k2+3]*77) + (gFB_info->screen_base[k2+4]*151) + (gFB_info->screen_base[k2+5]*28))>>8]; 
			lcd_base[k1-j] = (idx1&0xf) << 4 | (idx2&0xf);
			k2=k2+6;
		}
	}	
}

#define RGB565_MASK_RED 0xF800 
#define RGB565_MASK_GREEN 0x07E0 
#define RGB565_MASK_BLUE 0x001F 

static void pt_16bpp_to_16(void)
{

	int i;
	int j;
	int k1,k2;
	unsigned int data_width=0;
	
	unsigned int idx1,idx2;
	unsigned int r,g,b;
	unsigned int rgb;
	
	k1=0;	
	k2=0;
	
	
	
	


	k1=0;	
	k2=0;

	data_width = PANEL_WIDTH/2;
	for (i=1;i<=PANEL_HEIGHT;i++)
	{
		k1 = data_width*i-1;
		for(j=0;j<data_width;j++)
		{
			r = gFB_info->screen_base[k2]& 0xf8;
			g = ((gFB_info->screen_base[k2]&0x7)<<5) | ((gFB_info->screen_base[k2+1]>>3)&0x1c);
			b = (gFB_info->screen_base[k2+1]&0x1f)<<3;
			
			idx1 = GrayScaleLuTable[(r*77 + g*151 + b*28) >> 8];


			r = gFB_info->screen_base[k2+2]&0xf8;
			g = ((gFB_info->screen_base[k2+2]&0x7)<<5) | ((gFB_info->screen_base[k2+3]>>3)&0x1c);
			b = (gFB_info->screen_base[k2+3]&0x1f)<<3;

			idx2 = GrayScaleLuTable[(r*77 + g*151 + b*28)>>8];
			
			lcd_base[k1-j] = (idx1&0xf) << 4 | (idx2&0xf);

			k2=k2+4;
		}
	}	
}



static void pt_16bpp_to_16_rev(void)
{

	int i;
	int j;
	int k1,k2;
	unsigned int data_width=0;
	
	unsigned int idx1,idx2;
	unsigned int r,g,b;
	unsigned int rgb;
	
	k1=0;	
	k2=0;
	
	
#if FB_DEBUG1	
	DBG("--------------------> framebuffer source <------------------------\n");
	for(i=0;i<40;i++) 
	{
		DBG("S%03d:",i);
		for(j=0;j<PANEL_WIDTH;j++)
		{
			lcd_base[k1]=gFB_info->screen_base[k1];
			if ((j<60) && (i<40))
				DBG("%02x",gFB_info->screen_base[k1]);
			k1++;
		}
		if (i<41)
			DBG("\n");
	}
#endif	
	
	

#if FB_DEBUG3	

	k1=0;	
	k2=0;

	DBG("--------------------> RGB source <------------------------\n");
	for(i=0;i<40;i++) 
	{
		DBG("S%03d:",i);
		for(j=0;j<PANEL_WIDTH/2;j++)
		{
			lcd_base[k1]=gFB_info->screen_base[k1];
			if ((j<40)&&(j>=20) && (i<40))
			{
//				r = gFB_info->screen_base[k1]& 0xf8;
//				g = ((gFB_info->screen_base[k1]&0x7)<<5) | ((gFB_info->screen_base[k1+1]>>3)&0x1c);
//				b = (gFB_info->screen_base[k1+1]&0x1f)<<3;

				r = gFB_info->screen_base[k2]>>3;
				g = ((gFB_info->screen_base[k2]&0x7)<<3) | (gFB_info->screen_base[k2+1]>>5);
				b = gFB_info->screen_base[k2+1]&0x1f;
				
				idx1 = GrayScaleLuTable[(r*77 + g*151 + b*28) >> 8];
				
				
				DBG("%02x%02x%02x,%02x:%02x,",r,g,b,(r*77 + g*151 + b*28) >> 8,idx1);
			}
			k1=k1+2;
		}
		if (i<41)
			DBG("\n");
	}
#endif
	
#if  FB_DEBUG2
	DBG("--------------------> framebuffer send to auo <------------------------\n");
#endif

	k1=0;	
	k2=0;

	data_width = PANEL_WIDTH/2;
	for (i=1;i<=PANEL_HEIGHT;i++)
	{
		k1 = data_width*i-1;
//		DBG("Line i=%d .. ",i);
#if  FB_DEBUG2
		if (i<40)
			DBG("L%03d:",i);
#endif			
		for(j=0;j<data_width;j++)
		{
			r = gFB_info->screen_base[k2+1]& 0xf8;
			g = ((gFB_info->screen_base[k2+1]&0x7)<<5) | ((gFB_info->screen_base[k2]>>3)&0x1c);
			b = (gFB_info->screen_base[k2]&0x1f)<<3;
			
			idx1 = GrayScaleLuTable[(r*77 + g*151 + b*28) >> 8];


			r = gFB_info->screen_base[k2+3]&0xf8;
			g = ((gFB_info->screen_base[k2+3]&0x7)<<5) | ((gFB_info->screen_base[k2+2]>>3)&0x1c);
			b = (gFB_info->screen_base[k2+2]&0x1f)<<3;

			idx2 = GrayScaleLuTable[(r*77 + g*151 + b*28)>>8];
			
			lcd_base[k1-j] = (idx1&0xf) << 4 | (idx2&0xf);
#if  FB_DEBUG2
			if ((j<20) && (i<40))
				DBG("%04x",lcd_base[k1-j]);
#endif				

			k2=k2+4;
		}
#if  FB_DEBUG2
		if (i<41)
			DBG("\n");
#endif			
	}	
}



static void pt_16bpp_to_16_rev1(unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{

	int i;
	int j;
	int k0;
	int k1,k2;
	unsigned int data_width=0;
	
	unsigned int idx1,idx2;
	unsigned int r,g,b;
	unsigned int rgb;
	unsigned int x1,w1,y1,h1;
	
	
	k0=0;
	k1=0;	
	k2=0;

	data_width = PANEL_WIDTH/2;
	x1 = x/2;
	w1 = (x+w)/2;
	y1 = y;
	h1 = y+h;
	printk("x1=%d,w1=%d, y1=%d, h1=%d\n",x1,w1,y1,h1);

	for (i=0;i<PANEL_HEIGHT;i++)
	{
		if ((i>=y)&&(i<h1))
		{
			k0++;
			k1 = w*k0/2-1;
		}
		for(j=0;j<data_width;j++)
		{
					
			if ((j>=x1)&&(j<w1)&&(i>=y)&&(i<h1))
			{
				
				r = gFB_info->screen_base[k2+1]& 0xf8;
				g = ((gFB_info->screen_base[k2+1]&0x7)<<5) | ((gFB_info->screen_base[k2]>>3)&0x1c);
				b = (gFB_info->screen_base[k2]&0x1f)<<3;
				
				idx1 = GrayScaleLuTable[(r*77 + g*151 + b*28) >> 8];
	
	
				r = gFB_info->screen_base[k2+3]&0xf8;
				g = ((gFB_info->screen_base[k2+3]&0x7)<<5) | ((gFB_info->screen_base[k2+2]>>3)&0x1c);
				b = (gFB_info->screen_base[k2+2]&0x1f)<<3;
	
				idx2 = GrayScaleLuTable[(r*77 + g*151 + b*28)>>8];
				
				lcd_base_temp[k1] = (idx1&0xf) << 4 | (idx2&0xf);
				k1--;
			}		

			k2=k2+4;
		}
	}	

}



static void pt_16bpp_to_16_rev2(unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{

	int i;
	int j;
	unsigned int k0;
	unsigned int k1,k2;
	unsigned int data_width=0;
	
	unsigned int idx1,idx2;
	unsigned int r,g,b;
	unsigned int rgb;
	unsigned int x1,w1,y1,h1;
	
	
	k0=0;
	k1=0;	
	k2=0;

	data_width = PANEL_WIDTH/2;
	x1 = x/2;
	w1 = (x+w)/2;
	y1 = y;
	h1 = y+h;
	printk("x1=%d,w1=%d, y1=%d, h1=%d\n",x1,w1,y1,h1);

	for (i=y1;i<h1;i++)
	{
		k0++;
		k1 = w*k0/2-1;

		for(j=x1;j<w1;j++)
		{

			k2 = ((i* data_width) + j) * 4;

			{
				r = gFB_info->screen_base[k2+1]& 0xf8;
				g = ((gFB_info->screen_base[k2+1]&0x7)<<5) | ((gFB_info->screen_base[k2]>>3)&0x1c);
				b = (gFB_info->screen_base[k2]&0x1f)<<3;
				
				idx1 = GrayScaleLuTable[(r*77 + g*151 + b*28) >> 8];
	
	
				r = gFB_info->screen_base[k2+3]&0xf8;
				g = ((gFB_info->screen_base[k2+3]&0x7)<<5) | ((gFB_info->screen_base[k2+2]>>3)&0x1c);
				b = (gFB_info->screen_base[k2+2]&0x1f)<<3;
	
				idx2 = GrayScaleLuTable[(r*77 + g*151 + b*28)>>8];
				
				lcd_base_temp[k1] = (idx1&0xf) << 4 | (idx2&0xf);
				k1--;
			}		
		}
	}	

}


#define FB_DEBUG4		0
#define FB_DEBUG2		0
static void pt_8pp_to_16(unsigned int width, unsigned int height,unsigned char* buff)
{
	int i;
	int j;
	int k1,k2;
	unsigned int data_width=0;
	
	unsigned int idx1,idx2;
	struct fb_info *info;						// fbinfo

	
	
	k1=0;	
	k2=0;
	
	
#if FB_DEBUG4	
	DBG("--------------------> framebuffer source <------------------------\n");
	for(i=0;i<40;i++) 
	{
		DBG("S%03d:",i);
		for(j=0;j<width;j++)
		{
			if ((j<80) &&(j>=40)&& (i<40))
				DBG("%02x",buff[k1]);
			k1++;
		}
		if (i<41)
			DBG("\n");
	}
#endif	
	
	data_width = width/2;
	for (i=1;i<=height;i++)
	{
#if  FB_DEBUG2
		if (i<40)
			DBG("L%03d:",i);
#endif			
		
		k1 = data_width*i-1;
		for(j=0;j<data_width;j++)
		{
			idx1 = GrayScaleLuTable[buff[k2]]; 
			idx2 = GrayScaleLuTable[buff[k2+1]]; 
			lcd_base_temp[k1-j] = (idx1&0xf) << 4 | (idx2&0xf);
#if  FB_DEBUG2
			if ((j<20) && (i<40))
				DBG("%04x",lcd_base_temp[k1-j]);
#endif				
			k2=k2+2;
		}
#if  FB_DEBUG2
		if (i<41)
			DBG("\n");
#endif			
		
	}	
}


static void show_test_pattern()
{
	int i;
	int j;
	int k1,k2,k3;

	k1 = 0;
	k2 = 0;
	k3 = 0;
	
	for(i=0;i<PANEL_WIDTH;i++)
	{
		for(j=0;j<PANEL_HEIGHT/2;j++)
		{
			lcd_base[k1] = k2;
			k1++;
			k3++;
			if (k3>(PANEL_WIDTH*PANEL_HEIGHT/2/16))
			{
				k3 = 0;
				k2=k2+0x11;
			}
		}
	}
}

static void show_blank_pattern()
{
	int i;
	int j;
	int k1,k2,k3;

	k1 = 0;
	k2 = 0;
	k3 = 0;
	
	for(i=0;i<PANEL_WIDTH;i++)
	{
		for(j=0;j<PANEL_HEIGHT/2;j++)
		{
			lcd_base[k1] = 0x00;
			k1++;
			k3++;
			if (k3>(PANEL_WIDTH*PANEL_HEIGHT/2/16))
			{
//				DBG("----------------------> k1=%d, k2=%d ,k3=%d\n",k1,k2,k3);
				k3 = 0;
				k2=k2+0x11;
			}
		}
	}
}

static void show_white_pattern()
{
	int i;
	int j;
	int k1,k2,k3;

	k1 = 0;
	k2 = 0;
	k3 = 0;
	
	for(i=0;i<PANEL_WIDTH;i++)
	{
		for(j=0;j<PANEL_HEIGHT/2;j++)
		{
			lcd_base[k1] = 0xff;
			k1++;
			k3++;
			if (k3>(PANEL_WIDTH*PANEL_HEIGHT/2/16))
			{
//				DBG("----------------------> k1=%d, k2=%d ,k3=%d\n",k1,k2,k3);
				k3 = 0;
				k2=k2+0x11;
			}
		}
	}
}


static void pt_test_display()
{
	show_test_pattern();
	pt_k1901_full_refresh();
	show_blank_pattern();
	pt_k1901_full_refresh();
	show_test_pattern();
	pt_k1901_full_refresh();
	show_white_pattern();
	pt_k1901_full_refresh();

}



// Display update work function
//======================================================================================================================

struct mutex auo_display_mutex;

unsigned long porta_get_time(void)
{
	struct timeval tv;
	
	do_gettimeofday(&tv	);
	return (tv.tv_sec*1000+tv.tv_usec/1000);
}


//unsigned char startup_logo_1_1[PANEL_WIDTH*PANEL_HEIGHT/2];

void startup_logo_selection(int bmpnum)
{
	
   switch(bmpnum)
   {
     case 1:
             k1901_send_image_gray256( 0, 1, 1, 768, 1024, 0, 0, startup_logo_1_1, 786432 );
             while(0 != check_busy()) {/*printk("k1901_send_command: EPD is busy now \n");*/}
				printk("first image\n");
             k1901_show_image( 0, 1, 1, PANEL_WIDTH, PANEL_HEIGHT, 1, 0, 0, 0 );
             break;
     case 2:          
             msleep(2500);
             k1901_send_image_gray256( 0, 1, 1, 768, 1024, 0, 0, startup_logo_1_2, 786432 );
             while(0 != check_busy()) {/*printk("k1901_send_command: EPD is busy now \n");*/;}
             k1901_show_image( 0, 1, 1, PANEL_WIDTH, PANEL_HEIGHT, 1, 0, 0, 0 );
             break;
     case 3:
             msleep(2500);
             k1901_send_image_gray256( 0, 1, 1, 768, 1024, 0, 0, startup_logo_1_3, 786432 );
             while(0 != check_busy()) {/*printk("k1901_send_command: EPD is busy now \n");*/;}
             k1901_show_image( 0, 1, 1, PANEL_WIDTH, PANEL_HEIGHT, 1, 0, 0, 0 );
             break;
     case 4:
             msleep(2500);
             k1901_send_image_gray256( 0, 1, 1, 768, 1024, 0, 0, startup_logo_1_4, 786432 );
             while(0 != check_busy()) {/*printk("k1901_send_command: EPD is busy now \n");*/;}
             k1901_show_image( 0, 1, 1, PANEL_WIDTH, PANEL_HEIGHT, 1, 0, 0, 0 );
             break;
     case 5:
             msleep(2500);
             k1901_send_image_gray256( 0, 1, 1, 768, 1024, 0, 0, startup_logo_1_5, 786432 );
             while(0 != check_busy()) {/*printk("k1901_send_command: EPD is busy now \n");*/;}
             k1901_show_image( 0, 1, 1, PANEL_WIDTH, PANEL_HEIGHT, 1, 0, 0, 0 );
             break; 
     default:
             printk("Startup logo display error!!");
             break;
   } 
}


void booting_startup_logo(void)
{
	printk("Show booting logo .. start \n");
	startup_logo_selection(1);
	startup_logo_selection(2);
	startup_logo_selection(3);
	startup_logo_selection(4);
	startup_logo_selection(5);
	printk("Show booting logo .. end\n");

}

void auo_mode4_initial(void)
{

	printk("auo_mode4_initial ..\n"); 

    unsigned int LUT_parameter[30]={   0x0609, 0x0704, 0x0812, 0x0904, 0x0A2F, 0x0B04, 0x0CA2, 0x0D03, 0x0E12, 0x0F04,\
			0x102F, 0x1104, 0x12A2, 0x1303, 0x1406, 0x1520, 0x16D4, 0x1703, 0x1890, 0x1900,\
			0x1A20, 0x1BD4, 0x1C03, 0x1D90, 0x1E00};

 
 	int index;
 	
	MPU_nCS_Assert();	
	for( index=0;index<26;index++)       // Tuning driver IC output timing
	{
		MPU_nCS_Assert();	
		command_buff[0] = 0x0004;
		mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
		parameter_buff[0] = LUT_parameter[index];
		mpu_write_32( MPU_DATA, parameter_buff, 1, 1 );
		MPU_nCS_Deassert();
	}
	MPU_nCS_Assert();	
	command_buff[0] = 0x0004;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = 0x3996;
	mpu_write_32( MPU_DATA, parameter_buff, 1, 1 );
	MPU_nCS_Deassert();

	MPU_nCS_Assert();	
	command_buff[0] = 0x0004;
	mpu_write_32( MPU_COMMAND, command_buff, 1, 1 );
	parameter_buff[0] = 0x3A96; //3A69;
	mpu_write_32( MPU_DATA, parameter_buff, 1, 1 );
	MPU_nCS_Deassert();
}


int display_update_work_func(void)
{
	DBG("jeffrey  .. +display_update_work_func\n");
	jeffrey_debug();
	
	
#if AUO_BOOTING_LOGO	
	// show booting logo of image
	booting_startup_logo();
#endif	
	
#if AUO_SUPPORT_MODE4
//	auo_mode4_initial();
#endif	
	

    // signal the semaphore by incrementing the semaphore count
	int iii=0;

	while(1)
	{
		for (iii=0;iii<15;iii++)
		{
			msleep(100);
			schedule();			
		}
			
		if (g_iAutoRefreshMode)
		{
//			DBG("+");
			GET_DISPLAY_OWNER
			
			switch(PANEL_BPP)
			{
				case 24:
					pt_24bpp_to_16();
					break;
				case 16:
				default:
					pt_16bpp_to_16_rev();//pt_16bpp_to_16();
					break;
			}
			pt_k1901_full_refresh();

			FREE_DISPLAY_OWNER
		}
	}




   DBG("killing thread gets successfull!!!\n");
}

// Freescale epdc functions
//--------------------------------------------------------------------------------------------------------------
/* this is called back from the deferred io workqueue */
static void mxc_epdc_fb_deferred_io(struct fb_info *info, 
				    struct list_head *pagelist)
{
	

}

static void epdc_done_work_func(struct work_struct *work)
{
	
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data,
			epdc_done_work.work);
	DBG("---------------> Jeffrey .. b2\n");
	epdc_powerdown(fb_data);
}


static void epdc_submit_work_func(struct work_struct *work)
{
		

}

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxc_epdc_fb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	jeffrey_debug();
	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	if (var->grayscale)
		fix->visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}


static ssize_t store_update(struct device *device,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (strncmp(buf, "direct", 6) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (strncmp(buf, "gc16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (strncmp(buf, "gc4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc4;

	/* Now, request full screen update */
	update.update_region.left = 0;
	update.update_region.width = info->var.xres;
	update.update_region.top = 0;
	update.update_region.height = info->var.yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.temp = TEMP_USE_AMBIENT;
	update.update_marker = 0;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	return count;
}


static void epdc_powerdown(struct mxc_epdc_fb_data *fb_data)
{

}

int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,struct fb_info *info)
{
#if 0 //&*&*&*J1_待補	
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_data_list *upd_data_list = NULL;
	unsigned long flags;
	int i;
	struct mxcfb_rect *screen_upd_region; /* Region on screen to update */
	int temp_index;
	int ret;

//	DBG("Jeffrey .. epdc_fb .. 17\n");
//	msleep(3000);
//	DBG("Jeffrey .. epdc_fb .. 17e\n");

	/* Has EPDC HW been initialized? */
	if (!fb_data->hw_ready) {
		dev_err(fb_data->dev, "Display HW not properly initialized."
			"  Aborting update.\n");
		return -EPERM;
	}

	/* Check validity of update params */
	if ((upd_data->update_mode != UPDATE_MODE_PARTIAL) &&
		(upd_data->update_mode != UPDATE_MODE_FULL)) {
		dev_err(fb_data->dev,
			"Update mode 0x%x is invalid.  Aborting update.\n",
			upd_data->update_mode);
		return -EINVAL;
	}
	if ((upd_data->waveform_mode > 255) &&
		(upd_data->waveform_mode != WAVEFORM_MODE_AUTO)) {
		dev_err(fb_data->dev,
			"Update waveform mode 0x%x is invalid."
			"  Aborting update.\n",
			upd_data->waveform_mode);
		return -EINVAL;
	}
	if ((upd_data->update_region.left + upd_data->update_region.width > fb_data->info.var.xres) ||
		(upd_data->update_region.top + upd_data->update_region.height > fb_data->info.var.yres)) {
		dev_err(fb_data->dev,
			"Update region is outside bounds of framebuffer."
			"Aborting update.\n");
		return -EINVAL;
	}
	if ((upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) &&
		((upd_data->update_region.width != upd_data->alt_buffer_data.alt_update_region.width) ||
		(upd_data->update_region.height != upd_data->alt_buffer_data.alt_update_region.height))) {
		dev_err(fb_data->dev,
			"Alternate update region dimensions must match screen update region dimensions.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&fb_data->queue_lock, flags);

	/*
	 * If we are waiting to go into suspend, or the FB is blanked,
	 * we do not accept new updates
	 */
	if ((fb_data->waiting_for_idle) ||
		(fb_data->blank != FB_BLANK_UNBLANK)) {
		dev_dbg(fb_data->dev, "EPDC not active."
			"Update request abort.\n");
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return -EPERM;
	}

	/*
	 * Get available intermediate (PxP output) buffer to hold
	 * processed update region
	 */
	if (list_empty(&fb_data->upd_buf_free_list->list)) {
		dev_err(fb_data->dev,
			"No free intermediate buffers available.\n");
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return -ENOMEM;
	}

	/* Grab first available buffer and delete it from the free list */
	upd_data_list =
	    list_entry(fb_data->upd_buf_free_list->list.next,
		       struct update_data_list, list);

	list_del_init(&upd_data_list->list);

	/* copy update parameters to the current update data object */
	memcpy(&upd_data_list->upd_data, upd_data,
	       sizeof(struct mxcfb_update_data));
	memcpy(&upd_data_list->upd_data.update_region, &upd_data->update_region,
	       sizeof(struct mxcfb_rect));

	upd_data_list->fb_offset = fb_data->fb_offset;
	/* If marker specified, associate it with a completion */
	if (upd_data->update_marker != 0) {
		/* Find available update marker and set it up */
		for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
			/* Marker value set to 0 signifies it is not currently in use */
			if (fb_data->update_marker_array[i].update_marker == 0) {
				fb_data->update_marker_array[i].update_marker = upd_data->update_marker;
				init_completion(&fb_data->update_marker_array[i].update_completion);
				upd_data_list->upd_marker_data = &fb_data->update_marker_array[i];
				break;
			}
		}
	} else {
		if (upd_data_list->upd_marker_data)
			upd_data_list->upd_marker_data->update_marker = 0;
	}

	upd_data_list->update_order = fb_data->order_cnt++;

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list,
				  &fb_data->upd_buf_queue->list);

		spin_unlock_irqrestore(&fb_data->queue_lock, flags);

		/* Signal workqueue to handle new update */
		queue_work(fb_data->epdc_submit_workqueue,
			&fb_data->epdc_submit_work);

		return 0;
	}
#endif
	/* Snapshot update scheme processing */

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_send_update);

void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,	struct fb_info *info)
{

}
EXPORT_SYMBOL(mxc_epdc_fb_set_waveform_modes);

int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info)
{
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_temperature);

int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info)
{
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_auto_update);


int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info)
{
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_upd_scheme);


int mxc_epdc_fb_wait_update_complete(u32 update_marker, struct fb_info *info)
{

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_wait_update_complete);
 
int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ? (struct mxc_epdc_fb_data *)info:g_fb_data;

	fb_data->pwrdown_delay = pwrdown_delay;

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_pwrdown_delay);

int mxc_epdc_get_pwrdown_delay(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ? (struct mxc_epdc_fb_data *)info:g_fb_data;

	return fb_data->pwrdown_delay;
}
EXPORT_SYMBOL(mxc_epdc_get_pwrdown_delay);


/*=====================================================*/
/*           Modularization                            */
/*=====================================================*/

late_initcall(auo_epd_fb_init); 
module_exit(auo_epd_fb_remove);

MODULE_AUTHOR("Portalinks, Inc.");
MODULE_DESCRIPTION("MXC AUO framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");


