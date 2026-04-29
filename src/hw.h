#pragma once

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define REG16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))

#define PERIPH_BASE 0x40000000u
#define RCC_BASE    0x40021000u
#define AFIO_BASE   0x40010000u
#define EXTI_BASE   0x40010400u
#define GPIOA_BASE  0x40010800u
#define GPIOB_BASE  0x40010C00u
#define GPIOC_BASE  0x40011000u
#define GPIOD_BASE  0x40011400u
#define GPIOE_BASE  0x40011800u
#define ADC1_BASE   0x40012400u
#define TMR1_BASE   0x40012C00u
#define DMA1_BASE   0x40020000u
#define FLASH_R_BASE 0x40022000u
#define TMR5_BASE   0x40000C00u
#define SPI2_BASE   0x40003800u
#define SPI3_BASE   0x40003C00u
#define USART3_BASE 0x40004800u
#define USB_BASE    0x40005C00u
#define USB_PMA_BASE 0x40006000u
#define FSMC_BASE   0xA0000000u
#define SCB_VTOR    0xE000ED08u
#define SYST_CSR    0xE000E010u
#define SYST_RVR    0xE000E014u
#define SYST_CVR    0xE000E018u
#define NVIC_ISER0  0xE000E100u
#define NVIC_ISER1  0xE000E104u

#define RCC_AHBENR   REG32(RCC_BASE + 0x14u)
#define RCC_APB2ENR  REG32(RCC_BASE + 0x18u)
#define RCC_APB1ENR  REG32(RCC_BASE + 0x1Cu)
#define RCC_CFGR     REG32(RCC_BASE + 0x04u)
#define RCC_CFGR2    REG32(RCC_BASE + 0x30u)

#define SYSTICK_CTRL REG32(SYST_CSR)
#define SYSTICK_LOAD REG32(SYST_RVR)
#define SYSTICK_VAL  REG32(SYST_CVR)

#define AFIO_EXTICR1 REG32(AFIO_BASE + 0x08u)
#define AFIO_EXTICR2 REG32(AFIO_BASE + 0x0Cu)
#define AFIO_MAPR    REG32(AFIO_BASE + 0x04u)

#define EXTI_IMR     REG32(EXTI_BASE + 0x00u)
#define EXTI_RTSR    REG32(EXTI_BASE + 0x08u)
#define EXTI_FTSR    REG32(EXTI_BASE + 0x0Cu)
#define EXTI_PR      REG32(EXTI_BASE + 0x14u)

#define GPIO_CRL(base)  REG32((base) + 0x00u)
#define GPIO_CRH(base)  REG32((base) + 0x04u)
#define GPIO_IDR(base)  REG32((base) + 0x08u)
#define GPIO_ODR(base)  REG32((base) + 0x0Cu)
#define GPIO_BSRR(base) REG32((base) + 0x10u)
#define GPIO_BRR(base)  REG32((base) + 0x14u)

#define FSMC_BCR1   REG32(FSMC_BASE + 0x000u)
#define FSMC_BTR1   REG32(FSMC_BASE + 0x004u)
#define FSMC_BWTR1  REG32(FSMC_BASE + 0x104u)
#define FSMC_WPCR1  REG32(FSMC_BASE + 0x220u)

#define DMA_ISR     REG32(DMA1_BASE + 0x00u)
#define DMA_IFCR    REG32(DMA1_BASE + 0x04u)
#define DMA_CCR1    REG32(DMA1_BASE + 0x08u)
#define DMA_CNDTR1  REG32(DMA1_BASE + 0x0Cu)
#define DMA_CPAR1   REG32(DMA1_BASE + 0x10u)
#define DMA_CMAR1   REG32(DMA1_BASE + 0x14u)

#define FLASH_STS   REG32(FLASH_R_BASE + 0x0Cu)
#define FLASH_CTRL  REG32(FLASH_R_BASE + 0x10u)
#define FLASH_ADDR  REG32(FLASH_R_BASE + 0x14u)
#define FLASH_KEYR  REG32(FLASH_R_BASE + 0x04u)

#define ADC_SR(base)    REG32((base) + 0x00u)
#define ADC_CR1(base)   REG32((base) + 0x04u)
#define ADC_CR2(base)   REG32((base) + 0x08u)
#define ADC_SMPR2(base) REG32((base) + 0x10u)
#define ADC_SQR1(base)  REG32((base) + 0x2Cu)
#define ADC_SQR3(base)  REG32((base) + 0x34u)
#define ADC_DR(base)    REG32((base) + 0x4Cu)

#define USART_STS(base)   REG32((base) + 0x00u)
#define USART_DT(base)    REG32((base) + 0x04u)
#define USART_BAUDR(base) REG32((base) + 0x08u)
#define USART_CTRL1(base) REG32((base) + 0x0Cu)
#define USART_CTRL2(base) REG32((base) + 0x10u)
#define USART_CTRL3(base) REG32((base) + 0x14u)

#define TMR_CTRL1(base) REG32((base) + 0x00u)
#define TMR_IDEN(base)  REG32((base) + 0x0Cu)
#define TMR_STS(base)   REG32((base) + 0x10u)
#define TMR_EG(base)    REG32((base) + 0x14u)
#define TMR_CCM1(base)  REG32((base) + 0x18u)
#define TMR_CCEN(base)  REG32((base) + 0x20u)
#define TMR_PSC(base)   REG32((base) + 0x28u)
#define TMR_PR(base)    REG32((base) + 0x2Cu)
#define TMR_C1DT(base)  REG32((base) + 0x34u)
#define TMR_C2DT(base)  REG32((base) + 0x38u)

#define SPI_CTRL1(base) REG32((base) + 0x00u)
#define SPI_CTRL2(base) REG32((base) + 0x04u)
#define SPI_STS(base)   REG32((base) + 0x08u)
#define SPI_DT(base)    REG32((base) + 0x0Cu)

#define USB_EPR(ep)  REG16(USB_BASE + ((uint32_t)(ep) * 4u))
#define USB_CNTR     REG16(USB_BASE + 0x40u)
#define USB_ISTR     REG16(USB_BASE + 0x44u)
#define USB_FNR      REG16(USB_BASE + 0x48u)
#define USB_DADDR    REG16(USB_BASE + 0x4Cu)
#define USB_BTABLE   REG16(USB_BASE + 0x50u)
#define USB_PMA16(addr) REG16(USB_PMA_BASE + ((uint32_t)(addr) * 2u))

#define LCD_CMD_ADDR  0x6001FFFEu
#define LCD_DATA_ADDR 0x60020000u
#define LCD_CMD  REG16(LCD_CMD_ADDR)
#define LCD_DATA REG16(LCD_DATA_ADDR)

static inline void gpio_set(uint32_t base, uint32_t mask) {
    GPIO_BSRR(base) = mask;
}

static inline void gpio_clear(uint32_t base, uint32_t mask) {
    GPIO_BRR(base) = mask;
}

static inline uint32_t gpio_read(uint32_t base, uint32_t mask) {
    return GPIO_IDR(base) & mask;
}
