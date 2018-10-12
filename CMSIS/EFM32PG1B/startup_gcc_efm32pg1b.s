/* @file startup_efm32pg1b.S
 * @brief startup file for Silicon Labs EFM32PG1B devices.
 *        For use with GCC for ARM Embedded Processors
 * @version 5.5.0
 * Date:    12 June 2014
 *
 */
/* Copyright (c) 2011 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

    .syntax     unified
    .arch       armv7-m
    .section    .stack
    .align      3
#ifdef __STACK_SIZE
    .equ        Stack_Size, __STACK_SIZE
#else
    .equ        Stack_Size, 0x00000400
#endif
    .globl      __StackTop
    .globl      __StackLimit
__StackLimit:
    .space      Stack_Size
    .size       __StackLimit, . - __StackLimit
__StackTop:
    .size       __StackTop, . - __StackTop

    .section    .heap
    .align      3
#ifdef __HEAP_SIZE
    .equ        Heap_Size, __HEAP_SIZE
#else
    .equ        Heap_Size, 0x00000C00
#endif
    .globl      __HeapBase
    .globl      __HeapLimit
__HeapBase:
    .if Heap_Size
    .space      Heap_Size
    .endif
    .size       __HeapBase, . - __HeapBase
__HeapLimit:
    .size       __HeapLimit, . - __HeapLimit

    .section    .vectors
    .align      2
    .globl      __Vectors
__Vectors:
    .long       __StackTop                 /* Top of Stack */
    .long       Reset_Handler              /* Reset Handler */
    .long       NMI_Handler                /* NMI Handler */
    .long       HardFault_Handler          /* Hard Fault Handler */
    .long       MemManage_Handler          /* MPU Fault Handler */
    .long       BusFault_Handler           /* Bus Fault Handler */
    .long       UsageFault_Handler         /* Usage Fault Handler */
    .long       Default_Handler            /* Reserved */
    .long       Default_Handler            /* Reserved */
    .long       Default_Handler            /* Reserved */
    .long       Default_Handler            /* Reserved */
    .long       SVC_Handler                /* SVCall Handler */
    .long       DebugMon_Handler           /* Debug Monitor Handler */
    .long       Default_Handler            /* Reserved */
    .long       PendSV_Handler             /* PendSV Handler */
    .long       SysTick_Handler            /* SysTick Handler */

    /* External interrupts */
    .long       EMU_IRQHandler             /* 0 - EMU */
    .long       Default_Handler            /* 1 - Reserved */
    .long       WDOG0_IRQHandler           /* 2 - WDOG0 */
    .long       Default_Handler            /* 3 - Reserved */
    .long       Default_Handler            /* 4 - Reserved */
    .long       Default_Handler            /* 5 - Reserved */
    .long       Default_Handler            /* 6 - Reserved */
    .long       Default_Handler            /* 7 - Reserved */
    .long       LDMA_IRQHandler            /* 8 - LDMA */
    .long       GPIO_EVEN_IRQHandler       /* 9 - GPIO_EVEN */
    .long       TIMER0_IRQHandler          /* 10 - TIMER0 */
    .long       USART0_RX_IRQHandler       /* 11 - USART0_RX */
    .long       USART0_TX_IRQHandler       /* 12 - USART0_TX */
    .long       ACMP0_IRQHandler           /* 13 - ACMP0 */
    .long       ADC0_IRQHandler            /* 14 - ADC0 */
    .long       IDAC0_IRQHandler           /* 15 - IDAC0 */
    .long       I2C0_IRQHandler            /* 16 - I2C0 */
    .long       GPIO_ODD_IRQHandler        /* 17 - GPIO_ODD */
    .long       TIMER1_IRQHandler          /* 18 - TIMER1 */
    .long       USART1_RX_IRQHandler       /* 19 - USART1_RX */
    .long       USART1_TX_IRQHandler       /* 20 - USART1_TX */
    .long       LEUART0_IRQHandler         /* 21 - LEUART0 */
    .long       PCNT0_IRQHandler           /* 22 - PCNT0 */
    .long       CMU_IRQHandler             /* 23 - CMU */
    .long       MSC_IRQHandler             /* 24 - MSC */
    .long       CRYPTO_IRQHandler          /* 25 - CRYPTO */
    .long       LETIMER0_IRQHandler        /* 26 - LETIMER0 */
    .long       Default_Handler            /* 27 - Reserved */
    .long       Default_Handler            /* 28 - Reserved */
    .long       RTCC_IRQHandler            /* 29 - RTCC */
    .long       Default_Handler            /* 30 - Reserved */
    .long       CRYOTIMER_IRQHandler       /* 31 - CRYOTIMER */
    .long       Default_Handler            /* 32 - Reserved */
    .long       FPUEH_IRQHandler           /* 33 - FPUEH */


    .size       __Vectors, . - __Vectors

    .text
    .thumb
    .thumb_func
    .align      2
    .globl      Reset_Handler
    .type       Reset_Handler, %function
Reset_Handler:
#ifndef __NO_SYSTEM_INIT
    ldr     r0, =SystemInit
    blx     r0
#endif

/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
    ldr     r4, =__copy_table_start__
    ldr     r5, =__copy_table_end__

.L_loop0:
    cmp     r4, r5
    bge     .L_loop0_done
    ldr     r1, [r4]
    ldr     r2, [r4, #4]
    ldr     r3, [r4, #8]

.L_loop0_0:
    subs    r3, #4
    ittt    ge
    ldrge   r0, [r1, r3]
    strge   r0, [r2, r3]
    bge     .L_loop0_0

    adds    r4, #12
    b       .L_loop0

.L_loop0_done:
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
    ldr     r1, =__etext
    ldr     r2, =__data_start__
    ldr     r3, =__data_end__

.L_loop1:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt     .L_loop1
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __zero_table_start__ and __zero_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
    ldr     r3, =__zero_table_start__
    ldr     r4, =__zero_table_end__

.L_loop2:
    cmp     r3, r4
    bge     .L_loop2_done
    ldr     r1, [r3]
    ldr     r2, [r3, #4]
    movs    r0, 0

.L_loop2_0:
    subs    r2, #4
    itt     ge
    strge   r0, [r1, r2]
    bge     .L_loop2_0
    adds    r3, #8
    b       .L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
    ldr     r1, =__bss_start__
    ldr     r2, =__bss_end__

    movs    r0, 0
.L_loop3:
    cmp     r1, r2
    itt     lt
    strlt   r0, [r1], #4
    blt     .L_loop3
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __START
#define __START _start
#endif
    bl      __START

    .pool
    .size   Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak   Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       .
    .size   Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro  def_irq_handler	handler_name
    .weak   \handler_name
    .set    \handler_name, Default_Handler
    .endm

    def_irq_handler     NMI_Handler
    def_irq_handler     HardFault_Handler
    def_irq_handler     MemManage_Handler
    def_irq_handler     BusFault_Handler
    def_irq_handler     UsageFault_Handler
    def_irq_handler     SVC_Handler
    def_irq_handler     DebugMon_Handler
    def_irq_handler     PendSV_Handler
    def_irq_handler     SysTick_Handler


    def_irq_handler     EMU_IRQHandler
    def_irq_handler     WDOG0_IRQHandler
    def_irq_handler     LDMA_IRQHandler
    def_irq_handler     GPIO_EVEN_IRQHandler
    def_irq_handler     TIMER0_IRQHandler
    def_irq_handler     USART0_RX_IRQHandler
    def_irq_handler     USART0_TX_IRQHandler
    def_irq_handler     ACMP0_IRQHandler
    def_irq_handler     ADC0_IRQHandler
    def_irq_handler     IDAC0_IRQHandler
    def_irq_handler     I2C0_IRQHandler
    def_irq_handler     GPIO_ODD_IRQHandler
    def_irq_handler     TIMER1_IRQHandler
    def_irq_handler     USART1_RX_IRQHandler
    def_irq_handler     USART1_TX_IRQHandler
    def_irq_handler     LEUART0_IRQHandler
    def_irq_handler     PCNT0_IRQHandler
    def_irq_handler     CMU_IRQHandler
    def_irq_handler     MSC_IRQHandler
    def_irq_handler     CRYPTO_IRQHandler
    def_irq_handler     LETIMER0_IRQHandler
    def_irq_handler     RTCC_IRQHandler
    def_irq_handler     CRYOTIMER_IRQHandler
    def_irq_handler     FPUEH_IRQHandler

    .end
