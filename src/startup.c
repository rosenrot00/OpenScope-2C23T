#include <stdint.h>

#include "app_config.h"

extern uint32_t __data_load;
extern uint32_t __data_start;
extern uint32_t __data_end;
extern uint32_t __bss_start;
extern uint32_t __bss_end;
extern uint32_t __stack_top;

void Reset_Handler(void);
void Reset_Handler_C(void);
void Default_Handler(void);
void EXTI3_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
void TMR1_UP_IRQHandler(void);
void USART3_IRQHandler(void);
void power_key_irq_handler(void);
int main(void);

__attribute__((section(".isr_vector"), used))
void (*const vector_table[128])(void) = {
    [0] = (void (*)(void))&__stack_top,
    [1] = Reset_Handler,
    [2] = Default_Handler,
    [3] = Default_Handler,
    [4] = Default_Handler,
    [5] = Default_Handler,
    [6] = Default_Handler,
    [11] = Default_Handler,
    [12] = Default_Handler,
    [14] = Default_Handler,
    [15] = Default_Handler,
    [16 ... 24] = Default_Handler,
    [25] = EXTI3_IRQHandler,
    [26 ... 35] = Default_Handler,
    [36] = USB_LP_CAN1_RX0_IRQHandler,
    [37 ... 38] = Default_Handler,
    [39] = EXTI9_5_IRQHandler,
    [40] = Default_Handler,
    [41] = TMR1_UP_IRQHandler,
    [42 ... 54] = Default_Handler,
    [55] = USART3_IRQHandler,
    [56 ... 127] = Default_Handler,
};

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define SCB_VTOR   0xE000ED08u
#define SYST_CSR   0xE000E010u
#define NVIC_ICER0 0xE000E180u
#define NVIC_ICPR0 0xE000E280u
#define RCC_APB2ENR 0x40021018u
#define GPIOA_CRL   0x40010800u
#define GPIOA_BRR   0x40010814u
#define GPIOB_CRL   0x40010C00u
#define GPIOB_BSRR  0x40010C10u

__attribute__((naked))
void Reset_Handler(void) {
    __asm__ volatile(
        "ldr r0, =__stack_top\n"
        "msr msp, r0\n"
        "cpsid i\n"
        "ldr r0, =0x40021018\n"
        "ldr r1, [r0]\n"
        "movs r2, #0x2c\n"
        "orrs r1, r2\n"
        "str r1, [r0]\n"
        "ldr r1, [r0]\n"
        "ldr r0, =0x40010c00\n"
        "ldr r1, [r0]\n"
        "bic r1, r1, #0x0f00\n"
        "orr r1, r1, #0x0100\n"
        "str r1, [r0]\n"
        "ldr r0, =0x40010c10\n"
        "movs r1, #0x04\n"
        "str r1, [r0]\n"
        "ldr r0, =0x40010800\n"
        "ldr r1, [r0]\n"
        "bic r1, r1, #0x0f\n"
        "orr r1, r1, #0x01\n"
        "str r1, [r0]\n"
        "ldr r0, =0x40010814\n"
        "movs r1, #0x01\n"
        "str r1, [r0]\n"
        "dsb\n"
        "b Reset_Handler_C\n"
    );
}

static void disable_pending_interrupts(void) {
    REG32(SCB_VTOR) = APP_BASE_ADDR;
    REG32(SYST_CSR) = 0;
    for (uint32_t i = 0; i < 8; ++i) {
        REG32(NVIC_ICER0 + i * 4u) = 0xFFFFFFFFu;
        REG32(NVIC_ICPR0 + i * 4u) = 0xFFFFFFFFu;
    }
    __asm__ volatile("dsb\nisb");
}

static void early_panel_blank(void) {
    REG32(RCC_APB2ENR) |= (1u << 2) | (1u << 3); // GPIOA/GPIOB clock
    REG32(GPIOB_CRL) = (REG32(GPIOB_CRL) & ~(0xFu << 8)) | (0x1u << 8); // PB2 power hold
    REG32(GPIOB_BSRR) = 1u << 2;
    REG32(GPIOA_CRL) = (REG32(GPIOA_CRL) & ~0xFu) | 0x1u; // PA0 output
    REG32(GPIOA_BRR) = 1u; // LCD backlight off before clearing large .bss
}

void Reset_Handler_C(void) {
    early_panel_blank();
    disable_pending_interrupts();

    uint32_t *src = &__data_load;
    for (uint32_t *dst = &__data_start; dst < &__data_end;) {
        *dst++ = *src++;
    }

    for (uint32_t *dst = &__bss_start; dst < &__bss_end;) {
        *dst++ = 0;
    }

    (void)main();
    while (1) {
    }
}

void Default_Handler(void) {
    while (1) {
    }
}

void EXTI3_IRQHandler(void) {
    power_key_irq_handler();
}
