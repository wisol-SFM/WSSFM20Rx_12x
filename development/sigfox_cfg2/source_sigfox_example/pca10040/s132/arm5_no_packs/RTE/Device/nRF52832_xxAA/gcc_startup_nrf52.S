    .syntax unified
    .arch armv7e-m





    .section .stack
    .align 3





    .equ Stack_Size, 8192

    .globl __StackTop
    .globl __StackLimit
__StackLimit:
    .space Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3





    .equ Heap_Size, 8192

    .globl __HeapBase
    .globl __HeapLimit
__HeapBase:
    .if Heap_Size
    .space Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long __StackTop
    .long Reset_Handler
    .long NMI_Handler
    .long HardFault_Handler
    .long MemoryManagement_Handler
    .long BusFault_Handler
    .long UsageFault_Handler
    .long 0
    .long 0
    .long 0
    .long 0
    .long SVC_Handler
    .long DebugMon_Handler
    .long 0
    .long PendSV_Handler
    .long SysTick_Handler


    .long POWER_CLOCK_IRQHandler
    .long RADIO_IRQHandler
    .long UARTE0_UART0_IRQHandler
    .long SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler
    .long SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
    .long NFCT_IRQHandler
    .long GPIOTE_IRQHandler
    .long SAADC_IRQHandler
    .long TIMER0_IRQHandler
    .long TIMER1_IRQHandler
    .long TIMER2_IRQHandler
    .long RTC0_IRQHandler
    .long TEMP_IRQHandler
    .long RNG_IRQHandler
    .long ECB_IRQHandler
    .long CCM_AAR_IRQHandler
    .long WDT_IRQHandler
    .long RTC1_IRQHandler
    .long QDEC_IRQHandler
    .long COMP_LPCOMP_IRQHandler
    .long SWI0_EGU0_IRQHandler
    .long SWI1_EGU1_IRQHandler
    .long SWI2_EGU2_IRQHandler
    .long SWI3_EGU3_IRQHandler
    .long SWI4_EGU4_IRQHandler
    .long SWI5_EGU5_IRQHandler
    .long TIMER3_IRQHandler
    .long TIMER4_IRQHandler
    .long PWM0_IRQHandler
    .long PDM_IRQHandler
    .long 0
    .long 0
    .long MWU_IRQHandler
    .long PWM1_IRQHandler
    .long PWM2_IRQHandler
    .long SPIM2_SPIS2_SPI2_IRQHandler
    .long RTC2_IRQHandler
    .long I2S_IRQHandler
    .long FPU_IRQHandler
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0

    .size __isr_vector, . - __isr_vector




    .text
    .thumb
    .thumb_func
    .align 1
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
# 357 "gcc_startup_nrf52.S"
    ldr r1, =__etext
    ldr r2, =__data_start__
    ldr r3, =__bss_start__

    subs r3, r2
    ble .L_loop1_done

.L_loop1:
    subs r3, #4
    ldr r0, [r1,r3]
    str r0, [r2,r3]
    bgt .L_loop1

.L_loop1_done:
# 400 "gcc_startup_nrf52.S"
    bl SystemInit







    bl _start

    .pool
    .size Reset_Handler,.-Reset_Handler

    .section ".text"




    .weak NMI_Handler
    .type NMI_Handler, %function
NMI_Handler:
    b .
    .size NMI_Handler, . - NMI_Handler


    .weak HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    b .
    .size HardFault_Handler, . - HardFault_Handler


    .weak MemoryManagement_Handler
    .type MemoryManagement_Handler, %function
MemoryManagement_Handler:
    b .
    .size MemoryManagement_Handler, . - MemoryManagement_Handler


    .weak BusFault_Handler
    .type BusFault_Handler, %function
BusFault_Handler:
    b .
    .size BusFault_Handler, . - BusFault_Handler


    .weak UsageFault_Handler
    .type UsageFault_Handler, %function
UsageFault_Handler:
    b .
    .size UsageFault_Handler, . - UsageFault_Handler


    .weak SVC_Handler
    .type SVC_Handler, %function
SVC_Handler:
    b .
    .size SVC_Handler, . - SVC_Handler


    .weak DebugMon_Handler
    .type DebugMon_Handler, %function
DebugMon_Handler:
    b .
    .size DebugMon_Handler, . - DebugMon_Handler


    .weak PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    b .
    .size PendSV_Handler, . - PendSV_Handler


    .weak SysTick_Handler
    .type SysTick_Handler, %function
SysTick_Handler:
    b .
    .size SysTick_Handler, . - SysTick_Handler




    .globl Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b .
    .size Default_Handler, . - Default_Handler

    .macro IRQ handler
    .weak \handler
    .set \handler, Default_Handler
    .endm

    IRQ POWER_CLOCK_IRQHandler
    IRQ RADIO_IRQHandler
    IRQ UARTE0_UART0_IRQHandler
    IRQ SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler
    IRQ SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
    IRQ NFCT_IRQHandler
    IRQ GPIOTE_IRQHandler
    IRQ SAADC_IRQHandler
    IRQ TIMER0_IRQHandler
    IRQ TIMER1_IRQHandler
    IRQ TIMER2_IRQHandler
    IRQ RTC0_IRQHandler
    IRQ TEMP_IRQHandler
    IRQ RNG_IRQHandler
    IRQ ECB_IRQHandler
    IRQ CCM_AAR_IRQHandler
    IRQ WDT_IRQHandler
    IRQ RTC1_IRQHandler
    IRQ QDEC_IRQHandler
    IRQ COMP_LPCOMP_IRQHandler
    IRQ SWI0_EGU0_IRQHandler
    IRQ SWI1_EGU1_IRQHandler
    IRQ SWI2_EGU2_IRQHandler
    IRQ SWI3_EGU3_IRQHandler
    IRQ SWI4_EGU4_IRQHandler
    IRQ SWI5_EGU5_IRQHandler
    IRQ TIMER3_IRQHandler
    IRQ TIMER4_IRQHandler
    IRQ PWM0_IRQHandler
    IRQ PDM_IRQHandler
    IRQ MWU_IRQHandler
    IRQ PWM1_IRQHandler
    IRQ PWM2_IRQHandler
    IRQ SPIM2_SPIS2_SPI2_IRQHandler
    IRQ RTC2_IRQHandler
    IRQ I2S_IRQHandler
    IRQ FPU_IRQHandler

  .end
