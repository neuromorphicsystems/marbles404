/*************************************************************************
 * SCAMP Vision Chip Development System Library
 *------------------------------------------------------------------------
 * Copyright (c) 2020 The University of Manchester. All Rights Reserved.
 *
 *************************************************************************/

#ifndef VS_COMMON_H
#define VS_COMMON_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdlib.h>
#include <stdint.h>

#include <_vs_version.h>

#ifdef __GNUC__
#define _SNPRINTF_FAMILY   __attribute__ ((format(printf,3,4)))
#define _SSCANF_FAMILY   __attribute__ ((format(scanf,2,3)))
#define _PRINTF_FAMILY   __attribute__ ((format(printf,1,2)))
#define _SCANF_FAMILY   __attribute__ ((format(scanf,1,2)))
#define PRINTF_ARG __attribute__ ((format(printf,1,2)))
#define SNPRINTF_ARG __attribute__ ((format(printf,3,4)))
#else
#define PRINTF_ARG  
#define SNPRINTF_ARG  
#endif

#ifdef __LPC43XX__
#define __ASM            __asm
#define __INLINE         inline
#define __STATIC_INLINE  static inline
#include <core_cmInstr.h>
#define VS_INLINE 		INLINE
#define VS_IPC_IRQ		__SEV
#define VS_GPIO_ADDR(idx,bit)  (const uint8_t[]){idx,bit}
#define VS_GPIO_SET(addr) LPC_GPIO_PORT->SET[addr[0]] = 0x01UL<<addr[1]
#define VS_GPIO_CLR(addr) LPC_GPIO_PORT->CLR[addr[0]] = 0x01UL<<addr[1]
#define VS_GPIO_NOT(addr) LPC_GPIO_PORT->NOT[addr[0]] = 0x01UL<<addr[1]
#endif

typedef uint8_t vs_bool;
typedef uint32_t vs_handle;

/** general purpose time type, unit = 1/1000 second */
typedef uint32_t vs_time;


#define VS_TIME_Q   		1000
#define VS_MASTERCLOCK_Q   144000UL
/** master clock time type, unit = 1/144000 second */
typedef uint64_t vs_mct64;
typedef uint32_t vs_mct32;

#define VS_TIME_TO_MCT32(t) ((vs_mct32)(VS_MASTERCLOCK_Q/VS_TIME_Q)*(t))
#define VS_TIME_TO_MCT64(t) ((vs_mct64)(VS_MASTERCLOCK_Q/VS_TIME_Q)*(t))


#ifdef __cplusplus
}
#endif

#endif
