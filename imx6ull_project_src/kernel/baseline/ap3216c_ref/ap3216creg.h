#ifndef AP3216C_REG_H
#define AP3216C_REG_H

#define AP3216C_ADDR         0x1E    /* AP3216C器件地址 */

/* AP3216C寄存器 */
#define AP3216C_SYSTEMCONG   0x00    /* 配置寄存器 */
#define AP3216C_INTSTATUS    0x01    /* 中断状态寄存器 */
#define AP3216C_INTCLEAR     0x02    /* 中断清除寄存器 */
#define AP3216C_IRDATALOW    0x0A    /* IR数据低字节 */
#define AP3216C_IRDATAHIGH   0x0B    /* IR数据高字节 */
#define AP3216C_ALSDATALOW   0x0C    /* ALS数据低字节 */
#define AP3216C_ALSDATAHIGH  0x0D    /* ALS数据高字节 */
#define AP3216C_PSDATALOW    0x0E    /* PS数据低字节 */
#define AP3216C_PSDATAHIGH   0x0F    /* PS数据高字节 */
#define AP3216C_ALSCONFIG    0x10    /* ALS配置寄存器 */
#define AP3216C_PSCONFIG     0x20    /* PS配置寄存器 */
#define AP3216C_PSLEDCONFIG  0x21    /* PS LED配置寄存器 */
#define AP3216C_PSLEDWT      0x24    /* PS测量间隔寄存器 */
#define AP3216C_PSLTHDL      0x2A    /* PS低阈值低位 */
#define AP3216C_PSLTHDH      0x2B    /* PS低阈值高位 */
#define AP3216C_PSHTHDL      0x2C    /* PS高阈值低位 */
#define AP3216C_PSHTHDH      0x2D    /* PS高阈值高位 */

/* AP3216C 工作模式 */
#define AP3216C_MODE_PWDOWN         0x00    /* 关机模式 */
#define AP3216C_MODE_ALS_ONLY        0x01    /* 仅ALS模式 */
#define AP3216C_MODE_PS_ONLY         0x02    /* 仅PS模式 */
#define AP3216C_MODE_ALS_PS_IR       0x03    /* ALS+PS+IR模式 */

/* PS中断相关配置 - 原厂默认值 */
#define PS_THRESHOLD_NEAR    80      /* 接近阈值，越小越近 */
#define PS_THRESHOLD_FAR     100     /* 远离阈值 */
#define PS_CONFIG_DEFAULT    0x05    /* PS中断使能 */
#define PS_LED_CURRENT_DEFAULT 0x13  /* LED驱动电流 */

/* 中断状态位 */
#define AP3216C_ALSINT_STATE  0x01   /* ALS中断状态位 */
#define AP3216C_PSINT_STATE   0x02   /* PS中断状态位 */

#endif
