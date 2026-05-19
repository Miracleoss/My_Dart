#ifndef DRV_CACHE_H
#define DRV_CACHE_H

#include <stdint.h>
#include "stm32h7xx.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline uint8_t DCache_IsEnabled(void)
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

static inline void DCache_Clean_IfEnabled(void *addr, uint32_t len)
{
    if (!DCache_IsEnabled() || addr == 0 || len == 0U) {
        return;
    }

    uintptr_t start = (uintptr_t)addr & ~((uintptr_t)31U);
    uintptr_t end = ((uintptr_t)addr + len + 31U) & ~((uintptr_t)31U);

    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static inline void DCache_Invalidate_IfEnabled(void *addr, uint32_t len)
{
    if (!DCache_IsEnabled() || addr == 0 || len == 0U) {
        return;
    }

    uintptr_t start = (uintptr_t)addr & ~((uintptr_t)31U);
    uintptr_t end = ((uintptr_t)addr + len + 31U) & ~((uintptr_t)31U);

    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

#ifdef __cplusplus
}
#endif

#endif
