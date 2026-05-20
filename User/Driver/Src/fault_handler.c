/**
 * @file fault_handler.c
 * @brief HardFault / BusFault / MemManage / UsageFault 捕获
 *
 *        Ozone 调试时在 Watch 窗口添加 fault_frame / fault_CFSR 等变量
 *        即可直接看到出错时的 PC、LR 和故障原因。
 */

#include "fault_handler.h"
#include "stm32h7xx.h"

/* -------- 全局快照变量 -------- */
volatile FaultFrame_t  fault_frame;
volatile uint32_t      fault_CFSR;
volatile uint32_t      fault_HFSR;
volatile uint32_t      fault_MMFAR;
volatile uint32_t      fault_BFAR;
volatile uint32_t      fault_LR;

/* -------- Fault_Enable -------- */
void Fault_Enable(void)
{
    // 使能 MemManage / BusFault / UsageFault（默认只有 HardFault 是开的）
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
                 | SCB_SHCSR_BUSFAULTENA_Msk
                 | SCB_SHCSR_USGFAULTENA_Msk;

    // 保留除零捕获；UNALIGN_TRP 暂不打开，避免库/协议解析中的非对齐访问被放大。
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    /* SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk; */
}

/* -------- 通用 Fault 解析（供 stm32h7xx_it.c 中 4 个 handler 共用） -------- */

/*
 * ARM Cortex-M7 进入 Exception 时自动压栈：
 *   [SP+0]  R0   [SP+4]  R1   [SP+8]  R2   [SP+12] R3
 *   [SP+16] R12  [SP+20] LR   [SP+24] PC   [SP+28] xPSR
 *
 * EXC_RETURN (LR) bit2: 0 = MSP, 1 = PSP
 *
 * 由 stm32h7xx_it.c 中的 HardFault_Handler 等通过汇编 "B Fault_Capture" 跳转，
 * 入口 R0 = 栈指针（MSP 或 PSP）。
 */
void Fault_Capture(uint32_t *stack)
{
    fault_frame.r0  = stack[0];
    fault_frame.r1  = stack[1];
    fault_frame.r2  = stack[2];
    fault_frame.r3  = stack[3];
    fault_frame.r12 = stack[4];
    fault_frame.lr  = stack[5];
    fault_frame.pc  = stack[6];
    fault_frame.psr = stack[7];

    fault_CFSR  = SCB->CFSR;
    fault_HFSR  = SCB->HFSR;
    fault_MMFAR = SCB->MMFAR;
    fault_BFAR  = SCB->BFAR;
    fault_LR    = stack[5];

    //__BKPT(0);
    //while (1) { __NOP(); }
}
