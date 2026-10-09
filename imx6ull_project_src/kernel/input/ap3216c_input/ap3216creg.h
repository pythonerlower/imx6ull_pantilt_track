#ifndef AP3216C_REG_H
#define AP3216C_REG_H

/* AP3216C I2C 地址 */
#define AP3216C_ADDR         0x1E

/* 寄存器定义 */
#define AP3216C_SYSTEMCONG   0x00    /* 系统配置寄存器 */
#define AP3216C_INTSTATUS    0x01    /* 中断状态寄存器 */
#define AP3216C_INTCLEAR     0x02    /* 中断清除模式寄存器 */
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
/* PS 中断算法模式 (datasheet / AP3216_WE): 阈值窗口外触发 vs 滞回 */
#define AP3216C_PS_INT_FORM  0x22
#define AP3216C_PS_INT_ZONE          0x00    /* 测量值在阈值带之外时触发 */
#define AP3216C_PS_INT_HYSTERESIS    0x01    /* 上阈/下阈滞回（类似手机贴近） */
#define AP3216C_PSLTHDL      0x2A    /* PS低阈值低位 */
#define AP3216C_PSLTHDH      0x2B    /* PS低阈值高位 */
#define AP3216C_PSHTHDL      0x2C    /* PS高阈值低位 */
#define AP3216C_PSHTHDH      0x2D    /* PS高阈值高位 */

/* 工作模式 */
#define AP3216C_MODE_PWDOWN         0x00
#define AP3216C_MODE_ALS_ONLY       0x01
#define AP3216C_MODE_PS_ONLY        0x02
#define AP3216C_MODE_ALS_PS_IR      0x03

/* 中断状态位 */
#define AP3216C_ALSINT_STATE  0x01
#define AP3216C_PSINT_STATE   0x02

/* PS配置 - 使能PS和中断 */
/* PS配置寄存器(0x20):
 * bit 0: PS Enable
 * bit 1: Reserved
 * bit 2: PS Interrupt Enable
 * bit 3-4: PS Interrupt Persistence (防抖)
 * bit 4-5: LED Current
 * 0x05 = PS Enable + Interrupt Enable
 */
#define AP3216C_PSCONFIG_DEFAULT  0x05

/* LED配置 - 红外LED驱动电流 */
/* LED配置寄存器(0x21):
 * bit 0-1: LED driver ratio (0-3)
 * 0x11 = LED Ratio 1 (与 rEDTii 驱动一致)
 */
#define AP3216C_PSLED_DEFAULT    0x11

#endif
