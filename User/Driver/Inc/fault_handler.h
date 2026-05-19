#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fault 现场快照（在 Ozone Watch 窗口可直接查看）
 */
typedef struct {
    uint32_t r0, r1, r2, r3;
    uint32_t r12, lr, pc, psr;
} FaultFrame_t;

extern volatile FaultFrame_t  fault_frame;
extern volatile uint32_t      fault_CFSR;
extern volatile uint32_t      fault_HFSR;
extern volatile uint32_t      fault_MMFAR;
extern volatile uint32_t      fault_BFAR;
extern volatile uint32_t      fault_LR;

/**
 * @brief Fault 现场捕获（由 stm32h7xx_it.c 中汇编桩跳转，R0 = 栈指针）
 */
void Fault_Capture(uint32_t *stack);

/**
 * @brief 使能全部 Fault（MemManage / BusFault / UsageFault）
 *        在 main() 中 HAL_Init() 之后、外设初始化之前调用
 */
void Fault_Enable(void);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_HANDLER_H */
