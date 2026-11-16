/**
 * @file dev_gt2818s.h
 * @brief GT2818S AFE 设备对象头文件 (BSP 层)
 * @author 
 * @date 2026-07-16
 */

#ifndef DEV_GT2818S_H
#define DEV_GT2818S_H

#include <stdint.h>
#include "periph_io.h"
#include "periph_spi.h"

#define NUM_VOL_REGS  6
#define NUM_AUX_REGS  4
/* ==================== 寄存器结构体定义 ==================== */

typedef struct _cfg_a_group_ //ConfigA data structure(配置A组寄存器数据结构).
{
	struct //CFGAR0 
    {                               
		uint8_t adcopt		:1;		//0-MD[1:0]选择27KHz、7KHz、422Hz或26Hz模式	  1-MD[1:0]选择14KHz、3KHz、1KHz或2KHz模式
		uint8_t none		:1;		//无效空位
        uint8_t refon		:1;     //VREF1(基准电压)配置 1 :VREF1保持上电直到WDT溢出  0 :ADC转换完之后VREF1关闭 (默认)
        uint8_t gpio_pd_en	:5;     // GPIO1~5引脚控制  	 写：0-GPIOx引脚下拉电路导通；1-GPIOx引脚下拉电路关断 
    };
    uint8_t vuv_L8;          //CFGAR1, 欠压阈值低8位
    struct //CFGAR2
    {                    
        uint8_t vuv_H4  	:4;     //欠压阈值高4位
        uint8_t vov_L4  	:4;     //过压阈值低4位
    };
    uint8_t vov_H8;          //CFGAR3,过压阈值高8位
    uint8_t dcc1_8;          //CFGAR4,放电通道低1~8   	1-闭合放电开关；0-断开放电开关
    struct //CFGAR5
    {                    
        uint8_t dcc9_12  	:4;     //放电通道低9~12
        uint8_t dct  		:4;     //软件定时器设置
    };
    uint8_t    pec_H;           //PEC校验H
    uint8_t    pec_L;           //PEC校验L
	
}Cfg_A_Group_Def;//配置寄存器A

typedef struct _cfg_b_group_//ConfigB data structure(配置B组寄存器数据结构).
{
	struct //CFGBR0
    { 
		uint8_t gpio_pd_en		:4; // GPIO6~9引脚控制  	 写：0-GPIOx引脚下拉电路导通；1-GPIOx引脚下拉电路关断 
		uint8_t dcc13_16 		:4; //放电通道低13~16
	};	
	struct //CFGBR1	
    { 				
		uint8_t dcc17_18		:2;  //放电通道低 17 18
		uint8_t dcc0		    :1;  //表示GPIO9的下拉，1使能，0失能，
		uint8_t dtmen		    :1;  //使能放电计时器
		uint8_t ps			    :2;  //数字冗余诊断通道选择 
		uint8_t fdrf		    :1;  //1：强制数字冗余诊断失效 	0：不强制数字冗余诊断失效 
		uint8_t mute		    :1;  //1：静默使能 	0：静默不使能
	};
	uint8_t cfgbr2; 		//CFGBR2, 预留
	uint8_t cfgbr3; 		//CFGBR3, 预留
	uint8_t cfgbr4; 		//CFGBR4, 预留
	uint8_t cfgbr5; 		//CFGBR5, 预留
	uint8_t pec_H;			//PEC校验H
	uint8_t pec_L;			//PEC校验L

}Cfg_B_Group_Def;//配置寄存器B

typedef struct _stat_a_group_ //State A data structure(状态A组寄存器数据结构).
{
    // GT2818
    uint8_t    socL;          // soc L
    uint8_t    socH;          // soc H    
    uint8_t    itmpL;         // chip inner temp L
    uint8_t    itmpH;         // chip inner temp H
    uint8_t    vaL;           // 模拟电源电压 L
    uint8_t    vaH;           // 模拟电源电压 H  
    uint8_t    pec_H;           //PEC校验H
    uint8_t    pec_L;           //PEC校验L
	
}Stat_A_Group_Def;//状态寄存器A

typedef struct _stat_b_group_ //State B data structure(状态B组寄存器数据结构).
{
    // GT2818
    uint8_t    vdL;          // 数字电源电压 L
    uint8_t    vdH;          // 数字电源电压 H  
    struct //CFGBR2  
    {                 
        uint8_t c1uv		:1; // 欠压
        uint8_t c1ov		:1;  // 过压
        uint8_t c2uv		:1; // 欠压
        uint8_t c2ov		:1;  // 过压
        uint8_t c3uv		:1; // 欠压
        uint8_t c3ov		:1;  // 过压
        uint8_t c4uv		:1; // 欠压
        uint8_t c4ov		:1;  // 过压
	};
    struct //CFGBR3  
    {                 
        uint8_t c5uv		:1; // 欠压
        uint8_t c5ov		:1;  // 过压
        uint8_t c6uv		:1; // 欠压
        uint8_t c6ov		:1;  // 过压
        uint8_t c7uv		:1; // 欠压
        uint8_t c7ov		:1;  // 过压
        uint8_t c8uv		:1; // 欠压
        uint8_t c8ov		:1;  // 过压
	};
    struct //CFGBR4  
    {                 
        uint8_t c9uv		:1; // 欠压
        uint8_t c9ov		:1;  // 过压
        uint8_t c10uv		:1; // 欠压
        uint8_t c10ov		:1;  // 过压
        uint8_t c11uv		:1; // 欠压
        uint8_t c11ov		:1;  // 过压
        uint8_t c12uv		:1; // 欠压
        uint8_t c12ov		:1;  // 过压
	};
    struct //CFGBR5   
    {                
        uint8_t thsd		:1; 
        uint8_t muxfail		:1; 
        uint8_t rsvd        :2;
        uint8_t rev		    :4; 
	};
    uint8_t    pec_H;           //PEC校验H
    uint8_t    pec_L;           //PEC校验L
	
}Stat_B_Group_Def;//状态寄存器B

typedef struct _cell_vol_group_{
    uint16_t cell_vol[3];
    uint8_t  pec_H; //PEC校验H
    uint8_t  pec_L; //PEC校验L
}Cell_Vol_Group_Def;//电压寄存器

typedef struct _aux_group_{
    uint16_t aux[3];
    uint8_t  pec_H; //PEC校验H
    uint8_t  pec_L; //PEC校验L
}Aux_Group_Def;//辅助寄存器

typedef struct _afe_register_list_ {
    Cfg_A_Group_Def    cfg_a_set;
    Cfg_A_Group_Def    cfg_a_read;
    Cfg_B_Group_Def    cfg_b_set;
    Cfg_B_Group_Def    cfg_b_read;
    Stat_A_Group_Def   stat_a;
    Stat_B_Group_Def   stat_b;
    Cell_Vol_Group_Def cell_vol_reg[NUM_VOL_REGS];
    Aux_Group_Def      cell_aux_reg[NUM_AUX_REGS];
} Afe_Register_List_t;

typedef struct _ref_periph_list_ {
    Periph_Spi_t *spi;
    Periph_Gpio_t  *io_cs;
} Ref_Periph_List_t;

typedef struct _afe_func_list
{
    uint8_t  isInitialized;
    void     (*init)(struct Dev_Gt2818s *self);
    void     (*deinit)(struct Dev_Gt2818s *self);
    void     (*wakeup)(struct Dev_Gt2818s *self);
    void     (*cs_set)(struct Dev_Gt2818s *self,uint8_t level);
    uint8_t  (*write_cfg)(struct Dev_Gt2818s *self);
    uint8_t  (*read_cfg)(struct Dev_Gt2818s *self);
    uint8_t  (*adcv)(struct Dev_Gt2818s *self,uint8_t adc_mode,uint8_t ch);
    uint8_t  (*rdcv)(struct Dev_Gt2818s *self);
    uint8_t  (*adgp)(struct Dev_Gt2818s *self,uint8_t adc_mode,uint8_t ch);
    uint8_t  (*rdgp)(struct Dev_Gt2818s *self);
    uint8_t  (*clrcell)(struct Dev_Gt2818s *self);
    uint8_t  (*clraux)(struct Dev_Gt2818s *self);
} Afe_Func_List_t;

typedef struct Dev_Gt2818s
{
    Ref_Periph_List_t periph;
    Afe_Register_List_t reg;
    Afe_Func_List_t func;
}Dev_Gt2818s_t;



/* 全局 GT2818S 设备对象声明 */
extern Dev_Gt2818s_t dev_gt2818s_01;
extern Dev_Gt2818s_t dev_gt2818s_02;
extern Dev_Gt2818s_t dev_gt2818s_03;
extern Dev_Gt2818s_t dev_gt2818s_04;

#endif /* DEV_GT2818S_H */