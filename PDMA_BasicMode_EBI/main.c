/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use PDMA channel 2 to transfer data from memory to memory.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       192000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 256;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8SrcArray[256];
uint8_t au8DestArray[256];
#else
__attribute__((aligned(4))) uint8_t au8SrcArray[256]= {0};
__attribute__((aligned(4))) uint8_t au8DestArray[256] = {0};
#endif

uint32_t volatile g_u32IsTestOver = 0;

uint32_t volatile g_u32SysTick_last = 0;
uint32_t volatile g_u32SysTick_current = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M480.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
    g_u32SysTick_current = SysTick->VAL;

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 2 has abort error */
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA,PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        SysTick->CTRL = 0;
        /* Check transmission of channel 2 has been transfer done */
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA,PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(EBI_MODULE);

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
}

void UART0_Init()
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI AD0~5 pins on PC.0~5 */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_EBI_AD0 | SYS_GPC_MFPL_PC1MFP_EBI_AD1 |
                     SYS_GPC_MFPL_PC2MFP_EBI_AD2 | SYS_GPC_MFPL_PC3MFP_EBI_AD3 |
                     SYS_GPC_MFPL_PC4MFP_EBI_AD4 | SYS_GPC_MFPL_PC5MFP_EBI_AD5;

    /* EBI AD6, AD7 pins on PD.8, PD.9 */
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD8MFP_EBI_AD6 | SYS_GPD_MFPH_PD9MFP_EBI_AD7;

    /* EBI AD8, AD9 pins on PE.14, PE.15 */
    SYS->GPE_MFPH |= SYS_GPE_MFPH_PE14MFP_EBI_AD8 | SYS_GPE_MFPH_PE15MFP_EBI_AD9;

    /* EBI AD10, AD11 pins on PE.1, PE.0 */
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE1MFP_EBI_AD10 | SYS_GPE_MFPL_PE0MFP_EBI_AD11;

    /* EBI AD12~15 pins on PH.8~11 */
    SYS->GPH_MFPH |= SYS_GPH_MFPH_PH8MFP_EBI_AD12 | SYS_GPH_MFPH_PH9MFP_EBI_AD13 |
                     SYS_GPH_MFPH_PH10MFP_EBI_AD14 | SYS_GPH_MFPH_PH11MFP_EBI_AD15;

    /* EBI ADR16, ADR17 pins on PF.9, PF.8 */
    SYS->GPF_MFPH |= SYS_GPF_MFPH_PF9MFP_EBI_ADR16 | SYS_GPF_MFPH_PF8MFP_EBI_ADR17;

    /* EBI ADR18, ADR19 pins on PF.7, PF.6 */
    SYS->GPF_MFPL |= SYS_GPF_MFPL_PF7MFP_EBI_ADR18 | SYS_GPF_MFPL_PF6MFP_EBI_ADR19;


    /* EBI RD and WR pins on PE.4 and PE.5 */
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE4MFP_EBI_nWR | SYS_GPE_MFPL_PE5MFP_EBI_nRD;

    /* EBI WRL and WRH pins on PG.7 and PG.8 */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG7MFP_EBI_nWRL;
    SYS->GPG_MFPH |= SYS_GPG_MFPH_PG8MFP_EBI_nWRH;

    /* EBI CS0 pin on PD.12 */
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_EBI_nCS0;

    /* EBI ALE pin on PE.2 */
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE2MFP_EBI_ALE;

    /* EBI MCLK pin on PE.3 */
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE3MFP_EBI_MCLK;
}



int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_8BIT, EBI_TIMING_FAST, 0, EBI_CS_ACTIVE_LOW);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");

    /* Prepare trasfer data */
    for (int i=0; i<256; i++)
        au8SrcArray[i] = i;

    /* Open Channel 2 */
    PDMA_Open(PDMA,1 << 2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8-bit */
    PDMA_SetTransferCnt(PDMA,2, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA,2, (uint32_t)au8SrcArray, PDMA_SAR_INC, (uint32_t)EBI_BANK0_BASE_ADDR, PDMA_DAR_FIX);
    /* Request source is memory to memory */
    //PDMA_SetTransferMode(PDMA,2, PDMA_MEM, FALSE, 0);
    PDMA_SetTransferMode(PDMA,2, PDMA_TMR0, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    //PDMA_SetBurstType(PDMA,2, PDMA_REQ_BURST, PDMA_BURST_4);
    PDMA_SetBurstType(PDMA,2, PDMA_REQ_SINGLE, 0);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA,2, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;

    SysTick->LOAD = 16777216 - 1;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    asm("nop");

    /* Generate request to trigger transfer with PDMA channel 2  */
    //PDMA_Trigger(PDMA,2);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 10000000);
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_PDMA);
    
    g_u32SysTick_last = SysTick->VAL;
    TIMER_Start(TIMER0);

    /* Waiting for transfer done */
    while(g_u32IsTestOver == 0);

    /* Check transfer result */
    if(g_u32IsTestOver == 1)
        printf("test done...\n");
    else if(g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 2 */
    PDMA_Close(PDMA);

    /* Print the transfer result */
    for(int i=0; i<256; i++)
    {
        if(i%8 == 0)
            printf("\r\n");
        printf("%d ", EBI0_READ_DATA8(i));
    }

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    printf("\r\n");
    printf("Execute clock cycle: %d\r\n", (g_u32SysTick_last - g_u32SysTick_current));
    //printf("%d\r\n", g_u32SysTick_last);
    //printf("%d\r\n", g_u32SysTick_current);

    while(1);
}
