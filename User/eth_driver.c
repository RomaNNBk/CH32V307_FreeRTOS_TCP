#include "eth_driver.h"
#include "FreeRTOS.h"
#include "debug.h"
#include "string.h"
#include "task.h"


#define USE10BASE_T

__attribute__((aligned(4))) ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB]; /* жЋҐж”¶жЏЏиї°з¬¦иЎЁ */
__attribute__((aligned(4))) ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB]; /* еЏ‘йЂЃжЏЏиї°з¬¦иЎЁ */

#define ETH_DMARxDesc_FrameLengthShift 16

/*********************************************************************
 * @fn      ETH_SetClock
 *
 * @brief   Set ETH Clock(60MHZ).
 *
 * @param   none.
 *
 * @return  none.
 */
void ETH_SetClock(void)
{
    RCC_PLL3Cmd(DISABLE);
    RCC_PREDIV2Config(RCC_PREDIV2_Div2);                             /* HSE = 8M */
    RCC_PLL3Config(RCC_PLL3Mul_15);                                  /* 4M*15 = 60MHz */
    RCC_PLL3Cmd(ENABLE);
    while(RESET == RCC_GetFlagStatus(RCC_FLAG_PLL3RDY));
}

/*********************************************************************
 * @fn      ETH_LedLinkSet
 *
 * @brief   set eth link led,setbit 0 or 1,the link led turn on or turn off
 *
 * @return  none
 */
void ETH_LedLinkSet( uint8_t mode )
{
    if( mode == LED_OFF )
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    }
}

/*********************************************************************
 * @fn      ETH_LedDataSet
 *
 * @brief   set eth data led,setbit 0 or 1,the data led turn on or turn off
 *
 * @return  none
 */
void ETH_LedDataSet( uint8_t mode )
{
    if( mode == LED_OFF )
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    }
}

/*********************************************************************
 * @fn      ETH_LedConfiguration
 *
 * @brief   set eth data and link led pin
 *
 * @param   none.
 *
 * @return  none.
 */
void ETH_LedConfiguration(void)
{
    GPIO_InitTypeDef  GPIO={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO);
    ETH_LedDataSet(LED_OFF);
    ETH_LedLinkSet(LED_OFF);
}

void init_phy(void) {
    GPIO_InitTypeDef GPIO = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO.GPIO_Pin = GPIO_Pin_8;
    GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO);

    ETH_SetClock();

    //vTaskDelay(100 / portTICK_PERIOD_MS);

    while (RESET == RCC_GetFlagStatus(RCC_FLAG_PLL3RDY)) {
        printf("Wait for PLL3 ready. \r\n");
        Delay_Ms(100);
        //vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    printf("PLL3 is ready. \r\n");

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    ETH_LedConfiguration();
    /*
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_RNG, ENABLE);
        RNG_Cmd(ENABLE);
        printf("enable rng ok \r\n");
    */

    /* Enable Ethernet MAC clock */
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

#ifdef USE10BASE_T
    /* Enable internal 10BASE-T PHY*/
    EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN; /* дЅїиѓЅ10Mд»Ґе¤ЄзЅ‘з‰©зђ†е±‚   */
#endif

#ifdef USE_GIGA_MAC
    /* Enable 1G MAC*/
    EXTEN->EXTEN_CTR |= EXTEN_ETH_RGMII_SEL;       /* дЅїиѓЅ1Gд»Ґе¤ЄзЅ‘MAC */
    RCC_ETH1GCLKConfig(RCC_ETH1GCLKSource_PB1_IN); /* йЂ‰ж‹©е¤–йѓЁ125MHzиѕ“е…Ґ */
    RCC_ETH1G_125Mcmd(ENABLE);                     /* дЅїиѓЅ125MHzж—¶й’џ */

    /*  Enable RGMII GPIO */
    GETH_pin_init();
#endif

#ifdef USE_FAST_MAC
    /*  Enable MII or RMII GPIO */
    FETH_pin_init();
#endif

    /* Reset ETHERNET on AHB Bus */
    ETH_DeInit();

    /* Software reset */
    ETH_SoftwareReset();

    /* Ethernet_Configuration */
    static ETH_InitTypeDef ETH_InitStructure = {0};
    static uint32_t timeout;

    /* Wait for software reset */
    timeout = 10;
    // OS_SUBNT_SET_STATE();
    if (ETH->DMABMR & ETH_DMABMR_SR) {
        timeout--;
        if (timeout == 0) {
            printf("Error:Eth soft-reset timeout!\nPlease check RGMII TX & RX clock line. \r\n");
        }
        Delay_Ms(100);
        //vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    /* ETHERNET Configuration ------------------------------------------------------*/
    /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
    ETH_StructInit(&ETH_InitStructure);
    /* Fill ETH_InitStructure parametrs */
    /*------------------------   MAC   -----------------------------------*/
    ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
    ETH_InitStructure.ETH_Speed = ETH_Speed_1000M;
    ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
    ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
    ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
    ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
    ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Enable;
    ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
    ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif
    /*------------------------   DMA   -----------------------------------*/
    /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
    the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can
    insert/verify the checksum, if the checksum is OK the DMA can handle the frame otherwise the
    frame is dropped */
    ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
    ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
    ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Enable;
    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;
    ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;
    ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
    ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
    ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

    /* Configure Ethernet */
    uint32_t tmpreg = 0;
    static uint16_t RegValue = 0;

    /*---------------------- Physical layer configuration -------------------*/
    /* Set the SMI interface clock, set as the main frequency divided by 42  */
    tmpreg = ETH->MACMIIAR;
    tmpreg &= MACMIIAR_CR_MASK;
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;
    ETH->MACMIIAR = (uint32_t)tmpreg;

    /* е¤ЌдЅЌз‰©зђ†е±‚ */
    ETH_WritePHYRegister(PHY_ADDRESS, PHY_BCR, PHY_Reset); /* е¤ЌдЅЌз‰©зђ†е±‚  */

    //vTaskDelay(100 / portTICK_PERIOD_MS);

    timeout = 10000; /* жњЂе¤§и¶…ж—¶еЌЃз§’   */
    RegValue = 0;

    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BCR);
    if ((RegValue & (PHY_Reset))) {
        timeout--;
        if (timeout <= 0) {
            printf(
                "Error:Wait phy software timeout!\nPlease cheak PHY/MID.\nProgram has been "
                "blocked!\n");
            while (1)
                ;
        }
        Delay_Ms(1000);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* з­‰еѕ…з‰©зђ†е±‚дёЋеЇ№з«Їе»єз«‹LINK */
    timeout = 10000; /* жњЂе¤§и¶…ж—¶еЌЃз§’   */
    RegValue = 0;

    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BSR);
    if ((RegValue & (PHY_Linked_Status)) == 0) {
        timeout--;
        if (timeout <= 0) {
            printf(
                "Error:Wait phy linking timeout!\nPlease cheak MID.\nProgram has been blocked!\n");
            while (1)
                ;
        }
		Delay_Ms(1000);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* з­‰еѕ…з‰©зђ†е±‚е®Њж€ђи‡ЄеЉЁеЌЏе•† */
    timeout = 10000; /* жњЂе¤§и¶…ж—¶еЌЃз§’   */
    RegValue = 0;

    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BSR);
    if ((RegValue & PHY_AutoNego_Complete) == 0) {
        timeout--;
        if (timeout <= 0) {
            printf(
                "Error:Wait phy auto-negotiation complete timeout!\nPlease cheak MID.\nProgram has "
                "been blocked!\n");
            while (1)
                ;
        }
        Delay_Ms(1000);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x10);
    printf("PHY_SR value:%04x. \r\n", RegValue);

    if (RegValue & (1 << 2)) {
        ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
        printf("Full Duplex. \r\n");
    } else {
        ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
        printf("Half Duplex. \r\n");
    }
    ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
    if (RegValue & (1 << 3)) {
        printf("Loopback_10M \r\n");
    } else {
    }
    Delay_Ms(10);
    //vTaskDelay(10 / portTICK_PERIOD_MS);

    /* з‚№дє®linkзЉ¶жЂЃledзЃЇ */
    ETH_LedLinkSet(0);

    /*------------------------ MACеЇ„е­�е™Ёй…ЌзЅ®  ----------------------- --------------------*/
    /* MACCCRењЁRGMIIжЋҐеЏЈжЁЎејЏдё‹е…·жњ‰и°ѓж•ґRGMIIжЋҐеЏЈж—¶еєЏзљ„еџџпјЊиЇ·жіЁж„Џ */
    tmpreg = ETH->MACCR;
    tmpreg &= MACCR_CLEAR_MASK;
    tmpreg |=
        (uint32_t)(ETH_InitStructure.ETH_Watchdog | ETH_InitStructure.ETH_Jabber |
                   ETH_InitStructure.ETH_InterFrameGap | ETH_InitStructure.ETH_CarrierSense |
                   ETH_InitStructure.ETH_Speed | ETH_InitStructure.ETH_ReceiveOwn |
                   ETH_InitStructure.ETH_LoopbackMode | ETH_InitStructure.ETH_Mode |
                   ETH_InitStructure.ETH_ChecksumOffload | ETH_InitStructure.ETH_RetryTransmission |
                   ETH_InitStructure.ETH_AutomaticPadCRCStrip | ETH_InitStructure.ETH_BackOffLimit |
                   ETH_InitStructure.ETH_DeferralCheck);
    /* е†™MACжЋ§е€¶еЇ„е­�е™Ё */
    ETH->MACCR = (uint32_t)tmpreg;
#ifdef USE10BASE_T
    ETH->MACCR |= ETH_Internal_Pull_Up_Res_Enable; /* еђЇз”Ёе†…йѓЁдёЉж‹‰  */
#endif
    ETH->MACFFR =
        (uint32_t)(ETH_InitStructure.ETH_ReceiveAll | ETH_InitStructure.ETH_SourceAddrFilter |
                   ETH_InitStructure.ETH_PassControlFrames |
                   ETH_InitStructure.ETH_BroadcastFramesReception |
                   ETH_InitStructure.ETH_DestinationAddrFilter |
                   ETH_InitStructure.ETH_PromiscuousMode |
                   ETH_InitStructure.ETH_MulticastFramesFilter |
                   ETH_InitStructure.ETH_UnicastFramesFilter);
    /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
    /* Write to ETHERNET MACHTHR */
    ETH->MACHTHR = (uint32_t)ETH_InitStructure.ETH_HashTableHigh;
    /* Write to ETHERNET MACHTLR */
    ETH->MACHTLR = (uint32_t)ETH_InitStructure.ETH_HashTableLow;
    /*----------------------- ETHERNET MACFCR Configuration --------------------*/
    /* Get the ETHERNET MACFCR value */
    tmpreg = ETH->MACFCR;
    /* Clear xx bits */
    tmpreg &= MACFCR_CLEAR_MASK;

    tmpreg |=
        (uint32_t)((ETH_InitStructure.ETH_PauseTime << 16) | ETH_InitStructure.ETH_ZeroQuantaPause |
                   ETH_InitStructure.ETH_PauseLowThreshold |
                   ETH_InitStructure.ETH_UnicastPauseFrameDetect |
                   ETH_InitStructure.ETH_ReceiveFlowControl |
                   ETH_InitStructure.ETH_TransmitFlowControl);
    ETH->MACFCR = (uint32_t)tmpreg;

    ETH->MACVLANTR = (uint32_t)(ETH_InitStructure.ETH_VLANTagComparison |
                                ETH_InitStructure.ETH_VLANTagIdentifier);

    tmpreg = ETH->DMAOMR;
    /* Clear xx bits */
    tmpreg &= DMAOMR_CLEAR_MASK;

    tmpreg |= (uint32_t)(ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame |
                         ETH_InitStructure.ETH_ReceiveStoreForward |
                         ETH_InitStructure.ETH_FlushReceivedFrame |
                         ETH_InitStructure.ETH_TransmitStoreForward |
                         ETH_InitStructure.ETH_TransmitThresholdControl |
                         ETH_InitStructure.ETH_ForwardErrorFrames |
                         ETH_InitStructure.ETH_ForwardUndersizedGoodFrames |
                         ETH_InitStructure.ETH_ReceiveThresholdControl |
                         ETH_InitStructure.ETH_SecondFrameOperate);
    ETH->DMAOMR = (uint32_t)tmpreg;

    ETH->DMABMR =
        (uint32_t)(ETH_InitStructure.ETH_AddressAlignedBeats | ETH_InitStructure.ETH_FixedBurst |
                   ETH_InitStructure.ETH_RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx
                                                               it is applied for the other */
                   ETH_InitStructure.ETH_TxDMABurstLength |
                   (ETH_InitStructure.ETH_DescriptorSkipLength << 2) |
                   ETH_InitStructure.ETH_DMAArbitration | ETH_DMABMR_USP);

    /* Enable the Ethernet Rx Interrupt */
    ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T | ETH_DMA_IT_PHYLINK, ENABLE);

    NVIC_EnableIRQ(ETH_IRQn);
}



/*********************************************************************
 * @fn      ETH_TxPkt_ChainMode
 *
 * @brief   MAC send a ethernet frame in chain mode.
 *
 * @param   Send length
 *
 * @return  Send status.
 */
uint32_t ETH_TxPkt_ChainMode(u16 FrameLength) {
    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if ((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET) {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }

    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);
#ifdef CHECKSUM_BY_HARDWARE
    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one
     * descriptor) */
    DMATxDescToSet->Status |=
        ETH_DMATxDesc_LS | ETH_DMATxDesc_FS | ETH_DMATxDesc_CIC_TCPUDPICMP_Full;
#else
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;
#endif
    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET) {
        /* Clear TBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_TBUS;
        /* Resume DMA transmission*/

        ETH->DMATPDR = 0;
    }

    /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*)(DMATxDescToSet->Buffer2NextDescAddr);

    /* Return SUCCESS */
    return ETH_SUCCESS;
}

