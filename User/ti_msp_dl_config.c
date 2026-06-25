/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initDigitalOutputFeatures(GPIOA_LED_BLUE_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutput(GPIOA_LED_GREEN_IOMUX);

    DL_GPIO_initDigitalOutput(GPIOA_LED_RED_IOMUX);

    DL_GPIO_clearPins(GPIOA_PORT, GPIOA_LED_BLUE_PIN |
		GPIOA_LED_GREEN_PIN |
		GPIOA_LED_RED_PIN);
    DL_GPIO_enableOutput(GPIOA_PORT, GPIOA_LED_BLUE_PIN |
		GPIOA_LED_GREEN_PIN |
		GPIOA_LED_RED_PIN);

}


/* 时钟: 内部SYSOSC(~32.8MHz) → SYSPLL → ~82MHz
 * (板载40MHz晶振实测不起振, 已确认是板子硬件, 改用内部RC; 见进展记录)
 * SYSOSC × (qDiv4+1)=VCO≈164MHz, rDivClk0=0 → CLK0=VCO/2≈82MHz */
static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq   = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x   = 1, .rDivClk1 = 0, .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1  = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0  = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK  = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef   = DL_SYSCTL_SYSPLL_REF_SYSOSC,
    .qDiv        = 4,
    .pDiv        = DL_SYSCTL_SYSPLL_PDIV_1
};
/* 外部40MHz晶振 HFXT → SYSPLL: 40×(qDiv3+1)=VCO160 → CLK0=160/2=80.000MHz */
static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig_HFXT = {
    .inputFreq   = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x   = 1, .rDivClk1 = 0, .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1  = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0  = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK  = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef   = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv        = 3,
    .pDiv        = DL_SYSCTL_SYSPLL_PDIV_1
};
volatile uint8_t g_clk_hfxt_ok = 0;   /* 1=外部40M晶振起振并锁定(80.000MHz); 0=回退内部RC(~82MHz) */

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);   /* 32MHz下2WS富余, 安全 */
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);         /* SYSOSC=32MHz */
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);          /* ULPCLK=~16MHz */
    g_clk_hfxt_ok = 0;
    /* 不配SYSPLL: MCLK默认就是SYSOSC=32MHz(实测~32.8). 不超频、不用PLL, 最稳 */
}


