/*
 * FreeRTOS+TCP V2.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DNS.h"
#include "FreeRTOS_Routing.h"
#include "FreeRTOS_ND.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
#include "FreeRTOSIPConfig.h"

#include "ch32v30x_rng.h"
#include "eth_driver.h"


/* Interrupt events to process.  Currently only the Rx event is processed
 * although code for other events is included to allow for possible future
 * expansion. */
#define EMAC_IF_RX_EVENT        1UL
#define EMAC_IF_TX_EVENT        2UL
#define EMAC_IF_ERR_EVENT       4UL
#define EMAC_IF_LINK_EVENT      8UL
#define EMAC_IF_ALL_EVENT       ( EMAC_IF_RX_EVENT | EMAC_IF_TX_EVENT | EMAC_IF_ERR_EVENT | EMAC_IF_LINK_EVENT )

/* Default the size of the stack used by the EMAC deferred handler task to twice
 * the size of the stack used by the idle task - but allow this to be overridden in
 * FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
    #define configEMAC_TASK_STACK_SIZE    ( 2 * configMINIMAL_STACK_SIZE )
#endif
#ifndef niEMAC_HANDLER_TASK_PRIORITY
    #define niEMAC_HANDLER_TASK_PRIORITY    configMAX_PRIORITIES - 1
#endif
#ifndef niDESCRIPTOR_WAIT_TIME_MS
    #define niDESCRIPTOR_WAIT_TIME_MS    250uL
#endif











extern ETH_DMADESCTypeDef *DMATxDescToSet;
static __IO ETH_DMADESCTypeDef * DMATxDescToClear;


static NetworkInterface_t * pxMyInterface;
static TaskHandle_t xEMACTaskHandle = NULL;
static SemaphoreHandle_t xTXDescriptorSemaphore = NULL;

#if ( ipconfigZERO_COPY_RX_DRIVER == 0 )
__attribute__((aligned(4))) uint8_t Rx_Buff[ETH_RXBUFNB][ETH_MAX_PACKET_SIZE];
__attribute__((aligned(4))) uint8_t Tx_Buff[ETH_TXBUFNB][ETH_MAX_PACKET_SIZE];
#endif /* ipconfigZERO_COPY_RX_DRIVER */

void ETH_DMATxDescChainInitLocal(ETH_DMADESCTypeDef *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount)
{
    uint32_t            i = 0;
    ETH_DMADESCTypeDef *DMATxDesc;

    DMATxDescToSet = DMATxDescTab;

    for(i = 0; i < TxBuffCount; i++)
    {
        DMATxDesc = DMATxDescTab + i;
        DMATxDesc->Status = ETH_DMATxDesc_TCH | ETH_DMATxDesc_IC;

		#if ( ipconfigZERO_COPY_TX_DRIVER == 0 )
        {
            /* Set Buffer1 address pointer */
        	DMATxDesc->Buffer1Addr = (uint32_t)(&TxBuff[i * ETH_MAX_PACKET_SIZE]);
        }
        #endif

        if(i < (TxBuffCount - 1))
        {
            DMATxDesc->Buffer2NextDescAddr = (uint32_t)(DMATxDescTab + i + 1);
        }
        else
        {
            DMATxDesc->Buffer2NextDescAddr = (uint32_t)DMATxDescTab;
        }
    }

    ETH->DMATDLAR = (uint32_t)DMATxDescTab;
}

void ETH_DMARxDescChainInitLocal(ETH_DMADESCTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
    uint32_t            i = 0;
    ETH_DMADESCTypeDef *DMARxDesc;

    DMARxDescToGet = DMARxDescTab;

    for(i = 0; i < RxBuffCount; i++)
    {
        DMARxDesc = DMARxDescTab + i;

        DMARxDesc->ControlBufferSize = ETH_DMARxDesc_RCH | (uint32_t)ETH_MAX_PACKET_SIZE;

		#if ( ipconfigZERO_COPY_RX_DRIVER != 0 )
        {
            /* Set Buffer1 address pointer */
            NetworkBufferDescriptor_t * pxBuffer;

            pxBuffer = pxGetNetworkBufferWithDescriptor( ETH_MAX_PACKET_SIZE, 100ul );

            /* If the assert below fails, make sure that there are at least 'ETH_RXBUFNB'
             * Network Buffers available during start-up ( ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ) */
            configASSERT( pxBuffer != NULL );

            if( pxBuffer != NULL )
            {
            	DMARxDesc->Buffer1Addr = ( uint32_t ) pxBuffer->pucEthernetBuffer;
            	DMARxDesc->Status = ETH_DMARxDesc_OWN;
            }
        }
        #else /* if ( ipconfigZERO_COPY_RX_DRIVER != 0 ) */
        {
            /* Set Buffer1 address pointer */
        	DMARxDesc->Buffer1Addr = (uint32_t)(&RxBuff[i * ETH_MAX_PACKET_SIZE]);
            /* Set Own bit of the Rx descriptor Status */
        	DMARxDesc->Status = ETH_DMARxDesc_OWN;
        }
        #endif /* if ( ipconfigZERO_COPY_RX_DRIVER != 0 ) */

        if(i < (RxBuffCount - 1))
        {
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab + i + 1);
        }
        else
        {
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab);
        }
    }

    ETH->DMARDLAR = (uint32_t)DMARxDescTab;
}

/*-----------------------------------------------------------*/

NetworkInterface_t * xLoopbackInterface;

/*-----------------------------------------------------------*/
static void prvEMACHandlerTask(void * pvParameters);
static BaseType_t CH32F307_NetworkInterfaceInitialise( NetworkInterface_t * pxInterface )
{
    xLoopbackInterface = pxInterface;

    init_phy();

	#if ( ipconfigZERO_COPY_RX_DRIVER == 0 )
	ETH_DMARxDescChainInitLocal(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
	#else
	ETH_DMARxDescChainInitLocal(DMARxDscrTab, NULL, ETH_RXBUFNB);
	#endif /* ipconfigZERO_COPY_RX_DRIVER */

	#if ( ipconfigZERO_COPY_TX_DRIVER == 0 )
	ETH_DMATxDescChainInitLocal(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
	#else
	ETH_DMATxDescChainInitLocal(DMATxDscrTab, NULL, ETH_TXBUFNB);
	#endif /* ipconfigZERO_COPY_RX_DRIVER */

    DMATxDescToClear = DMATxDescToSet;

    ETH_Start();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_RNG, ENABLE);
	RNG_Cmd(ENABLE);

	xTXDescriptorSemaphore = xSemaphoreCreateCounting( ( UBaseType_t ) ETH_TXBUFNB, ( UBaseType_t ) ETH_TXBUFNB );

	if( xTaskCreate( prvEMACHandlerTask, "EMAC", configEMAC_TASK_STACK_SIZE, NULL, niEMAC_HANDLER_TASK_PRIORITY, &xEMACTaskHandle ) == pdPASS )
	{
		/* The xTXDescriptorSemaphore and the task are created successfully. */
		//xMacInitStatus = eMACPass;
	}
	else
	{
		//xMacInitStatus = eMACFailed;
	}

    return pdTRUE;
}
/*-----------------------------------------------------------*/

static BaseType_t CH32F307_NetworkInterfaceOutput( NetworkInterface_t * pxInterface,
                                                  NetworkBufferDescriptor_t * const pxDescriptor,
                                                  BaseType_t bReleaseAfterSend );

static BaseType_t CH32F307_NetworkInterface_GetPhyLinkStatus( NetworkInterface_t * pxInterface )
{
    /* This function returns true if the Link Status in the PHY is high. */
    ( void ) pxInterface;

    uint16_t phy_bsr = ETH_ReadPHYRegister( PHY_ADDRESS, PHY_BSR);
    return (phy_bsr & PHY_Linked_Status) ? pdTRUE : pdFALSE;
}
/*-----------------------------------------------------------*/

NetworkInterface_t * pxFillInterfaceDescriptor( BaseType_t xEMACIndex,
                                                         NetworkInterface_t * pxInterface )
{
/* This function pxLoopback_FillInterfaceDescriptor() adds a network-interface.
 * Make sure that the object pointed to by 'pxInterface'
 * is declared static or global, and that it will remain to exist. */
	memset( pxInterface, '\0', sizeof( *pxInterface ) );
    pxInterface->pcName = "Loopback";                /* Just for logging, debugging. */
    pxInterface->pvArgument = ( void * ) xEMACIndex; /* Has only meaning for the driver functions. */
    pxInterface->pfInitialise = CH32F307_NetworkInterfaceInitialise;
    pxInterface->pfOutput = CH32F307_NetworkInterfaceOutput;
    pxInterface->pfGetPhyLinkStatus = CH32F307_NetworkInterface_GetPhyLinkStatus;

    pxMyInterface = pxInterface;

    FreeRTOS_AddNetworkInterface( pxInterface );

    return pxInterface;
}
/*-----------------------------------------------------------*/

static BaseType_t CH32F307_NetworkInterfaceOutput( NetworkInterface_t * pxInterface,
                                                  NetworkBufferDescriptor_t * const pxDescriptor,
                                                  BaseType_t bReleaseAfterSend )
{
    BaseType_t xReturn = pdFAIL;
    uint32_t ulTransmitSize = 0;
    __IO ETH_DMADESCTypeDef * pxDmaTxDesc;
/* Do not wait too long for a free TX DMA buffer. */
    const TickType_t xBlockTimeTicks = pdMS_TO_TICKS( 50u );

    /* As there is only a single instance of the EMAC, there is only one pxInterface object. */
    ( void ) pxInterface;

    /* Open a do {} while ( 0 ) loop to be able to call break. */
    do
    {
        #if ( ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM != 0 )
        {
            const IPPacket_t * pxIPPacket;

            pxIPPacket = ( const IPPacket_t * ) pxDescriptor->pucEthernetBuffer;
            #if ( ipconfigUSE_IPv6 != 0 )
                if( pxIPPacket->xEthernetHeader.usFrameType == ipIPv6_FRAME_TYPE )
                {
                    const IPHeader_IPv6_t * pxIPPacket_IPv6;

                    pxIPPacket_IPv6 = ( const IPHeader_IPv6_t * ) &( pxDescriptor->pucEthernetBuffer[ ipSIZE_OF_ETH_HEADER ] );

                    if( pxIPPacket_IPv6->ucNextHeader == ( uint8_t ) ipPROTOCOL_ICMP_IPv6 )
                    {
                        ICMPHeader_IPv6_t * pxICMPHeader_IPv6;

                        pxICMPHeader_IPv6 = ( ICMPHeader_IPv6_t * ) &( pxDescriptor->pucEthernetBuffer[ ipSIZE_OF_ETH_HEADER + ipSIZE_OF_IPv6_HEADER ] );
                        pxICMPHeader_IPv6->usChecksum = 0U;
                    }
                }
                else
            #endif /* if ( ipconfigUSE_IPv6 != 0 ) */

            if( pxIPPacket->xEthernetHeader.usFrameType == ipIPv4_FRAME_TYPE )
            {
                if( pxIPPacket->xIPHeader.ucProtocol == ( uint8_t ) ipPROTOCOL_ICMP )
                {
                    ICMPHeader_t * pxICMPHeader;

                    pxICMPHeader = ( ICMPHeader_t * ) &( pxDescriptor->pucEthernetBuffer[ ipSIZE_OF_ETH_HEADER + ipSIZE_OF_IPv4_HEADER ] );
                    pxICMPHeader->usChecksum = ( uint16_t ) 0U;
                }
            }
        }
        #endif /* ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM */

        if( 1 != 0 ) //TODO!!!! xPhyObject.ulLinkStatusMask
        {
            if( xSemaphoreTake( xTXDescriptorSemaphore, xBlockTimeTicks ) != pdPASS )
            {
                /* Time-out waiting for a free TX descriptor. */
                break;
            }

            /* This function does the actual transmission of the packet. The packet is
             * contained in 'pxDescriptor' that is passed to the function. */
            pxDmaTxDesc = DMATxDescToSet;

            /* Is this buffer available? */
            configASSERT( ( pxDmaTxDesc->Status & ETH_DMATxDesc_OWN ) == 0 );

            {
                /* Is this buffer available? */
                /* Get bytes in current buffer. */
                ulTransmitSize = pxDescriptor->xDataLength;

                if( ulTransmitSize > ETH_MAX_PACKET_SIZE )
                {
                    ulTransmitSize = ETH_MAX_PACKET_SIZE;
                }

                #if ( ipconfigZERO_COPY_TX_DRIVER == 0 )
                {
                    /* Copy the bytes. */
                    memcpy( ( void * ) pxDmaTxDesc->Buffer1Addr, pxDescriptor->pucEthernetBuffer, ulTransmitSize );
                }
                #else
                {
                    configASSERT( bReleaseAfterSend != 0 );

                    /* Move the buffer. */
                    pxDmaTxDesc->Buffer1Addr = ( uint32_t ) pxDescriptor->pucEthernetBuffer;
                    /* The Network Buffer has been passed to DMA, no need to release it. */
                    bReleaseAfterSend = pdFALSE_UNSIGNED;
                }
                #endif /* ipconfigZERO_COPY_TX_DRIVER */

                ETH_TxPkt_ChainMode(ulTransmitSize);

                /* Ensure completion of memory access */
                //__DSB();
                /* Resume DMA transmission*/
                //xETH.Instance->DMATPDR = 0;
                iptraceNETWORK_INTERFACE_TRANSMIT();
                xReturn = pdPASS;
            }
        }
        else
        {
            /* The PHY has no Link Status, packet shall be dropped. */
        }
    } while( 0 );

    /* The buffer has been sent so can be released. */
    if( bReleaseAfterSend != pdFALSE )
    {
        vReleaseNetworkBufferAndDescriptor( pxDescriptor );
    }

    return xReturn;
}

/*  CALLED BY FREERTOS
 *  Function that returns a random number for TCP.  This is taken to be a random number. */
uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
                                             uint16_t usSourcePort,
                                             uint32_t ulDestinationAddress,
                                             uint16_t usDestinationPort )
{
    uint32_t pulNumber = 0;

    xApplicationGetRandomNumber( &pulNumber );
    return pulNumber;
}

/*  CALLED BY FREERTOS
 *  Function that sets pulNumber to a random number, and then returns pdTRUE.
 *  If the random number could not be obtained, then the function will return pdFALSE. */
BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber )
{
    *pulNumber = 0;

    while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET)
	{
	}
    uint32_t num = RNG_GetRandomNumber();

    if( num == 0 )
    {
        return pdFALSE;
    }

    *pulNumber = num;
    return pdTRUE;
}

/* Defined by the application code, but called by FreeRTOS-Plus-TCP when the network
   connects/disconnects (if ipconfigUSE_NETWORK_EVENT_HOOK is set to 1 in
FreeRTOSIPConfig.h). */
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
static BaseType_t xTasksAlreadyCreated = pdFALSE;
int8_t cBuffer[ 16 ];

    /* Check this was a network up event, as opposed to a network down event. */
    if( eNetworkEvent == eNetworkUp )
    {
        /* Create the tasks that use the TCP/IP stack if they have not already been
           created. */
        if( xTasksAlreadyCreated == pdFALSE )
        {
            /*
             * Create the tasks here.
             */

            xTasksAlreadyCreated = pdTRUE;
        }

        /* The network is up and configured. Print out the configuration,
           which may have been obtained from a DHCP server. */
        FreeRTOS_GetAddressConfiguration( &ulIPAddress,
                                          &ulNetMask,
                                          &ulGatewayAddress,
                                          &ulDNSServerAddress );


        /* Convert the IP address to a string then print it out. */
        FreeRTOS_inet_ntoa( ulIPAddress, cBuffer );
        configPRINTF(( "IP Address: %s\r\n", cBuffer ));

        /* Convert the net mask to a string then print it out. */
        FreeRTOS_inet_ntoa( ulNetMask, cBuffer );
        configPRINTF(( "Subnet Mask: %s\r\n", cBuffer ));

        /* Convert the IP address of the gateway to a string then print it out. */
        FreeRTOS_inet_ntoa( ulGatewayAddress, cBuffer );
        configPRINTF(( "Gateway IP Address: %s\r\n", cBuffer ));

        /* Convert the IP address of the DNS server to a string then print it out. */
        FreeRTOS_inet_ntoa( ulDNSServerAddress, cBuffer );
        configPRINTF(( "DNS server IP Address: %s\r\n", cBuffer ));
    }
}
/*-----------------------------------------------------------*/
#define ETH_DMARxDesc_FrameLengthShift 16


static void vClearTXBuffers()
{
    __IO ETH_DMADESCTypeDef * txLastDescriptor = DMATxDescToSet;
    size_t uxCount = ( ( UBaseType_t ) ETH_TXBUFNB ) - uxSemaphoreGetCount( xTXDescriptorSemaphore );

    #if ( ipconfigZERO_COPY_TX_DRIVER != 0 )
        NetworkBufferDescriptor_t * pxNetworkBuffer;
        uint8_t * ucPayLoad;
    #endif

    /* This function is called after a TX-completion interrupt.
     * It will release each Network Buffer used in xNetworkInterfaceOutput().
     * 'uxCount' represents the number of descriptors given to DMA for transmission.
     * After sending a packet, the DMA will clear the 'ETH_DMATXDESC_OWN' bit. */
    while( ( uxCount > 0 ) && ( ( DMATxDescToClear->Status & ETH_DMATxDesc_OWN ) == 0 ) )
    {
        if( ( DMATxDescToClear == txLastDescriptor ) && ( uxCount != ETH_TXBUFNB ) )
        {
            break;
        }

        #if ( ipconfigZERO_COPY_TX_DRIVER != 0 )
        {
            ucPayLoad = ( uint8_t * ) DMATxDescToClear->Buffer1Addr;

            if( ucPayLoad != NULL )
            {
                pxNetworkBuffer = pxPacketBuffer_to_NetworkBuffer( ucPayLoad );

                if( pxNetworkBuffer != NULL )
                {
                    vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
                }

                DMATxDescToClear->Buffer1Addr = ( uint32_t ) 0u;
            }
        }
        #endif /* ipconfigZERO_COPY_TX_DRIVER */

        DMATxDescToClear = ( ETH_DMADESCTypeDef * ) ( DMATxDescToClear->Buffer2NextDescAddr );

        uxCount--;
        /* Tell the counting semaphore that one more TX descriptor is available. */
        xSemaphoreGive( xTXDescriptorSemaphore );
    }
}

static BaseType_t xMayAcceptPacket( uint8_t * pucEthernetBuffer )
{
    const ProtocolPacket_t * pxProtPacket = ( const ProtocolPacket_t * ) pucEthernetBuffer;

    return pdTRUE;
    switch( pxProtPacket->xTCPPacket.xEthernetHeader.usFrameType )
    {
        case ipARP_FRAME_TYPE:
            /* Check it later. */
            return pdTRUE;

            #if ( ipconfigUSE_IPv6 != 0 )
                case ipIPv6_FRAME_TYPE:
                    /* Check it later. */
                    return pdTRUE;
            #endif
        case ipIPv4_FRAME_TYPE:
            /* Check it here. */
            break;

        default:
            /* Refuse the packet. */
            return pdFALSE;
    }

    #if ( ipconfigETHERNET_DRIVER_FILTERS_PACKETS == 1 )
    {
        const IPHeader_t * pxIPHeader = &( pxProtPacket->xTCPPacket.xIPHeader );
        uint32_t ulDestinationIPAddress;

        /* Ensure that the incoming packet is not fragmented (only outgoing packets
         * can be fragmented) as these are the only handled IP frames currently. */
        if( ( pxIPHeader->usFragmentOffset & ipFRAGMENT_OFFSET_BIT_MASK ) != 0U )
        {
            return pdFALSE;
        }

        /* HT: Might want to make the following configurable because
         * most IP messages have a standard length of 20 bytes */

        /* 0x45 means: IPv4 with an IP header of 5 x 4 = 20 bytes
         * 0x47 means: IPv4 with an IP header of 7 x 4 = 28 bytes */
        if( ( pxIPHeader->ucVersionHeaderLength < 0x45 ) || ( pxIPHeader->ucVersionHeaderLength > 0x4F ) )
        {
            return pdFALSE;
        }

        ulDestinationIPAddress = pxIPHeader->ulDestinationIPAddress;

        /* Is the packet for this node? */
        if( ( ulDestinationIPAddress != *ipLOCAL_IP_ADDRESS_POINTER ) &&
            /* Is it a broadcast address x.x.x.255 ? */
            ( ( FreeRTOS_ntohl( ulDestinationIPAddress ) & 0xff ) != 0xff ) &&
            #if ( ipconfigUSE_LLMNR == 1 )
                ( ulDestinationIPAddress != ipLLMNR_IP_ADDR ) &&
            #endif
            ( *ipLOCAL_IP_ADDRESS_POINTER != 0 ) )
        {
            FreeRTOS_debug_printf( ( "Drop IP %lxip\n", FreeRTOS_ntohl( ulDestinationIPAddress ) ) );
            return pdFALSE;
        }

        if( pxIPHeader->ucProtocol == ipPROTOCOL_UDP )
        {
            #if ( ipconfigUSE_LLMNR == 1 ) || ( ipconfigUSE_MDNS == 1 ) || ( ipconfigUSE_NBNS == 1 ) || ( ipconfigUSE_DNS == 1 )
                uint16_t usSourcePort = FreeRTOS_ntohs( pxProtPacket->xUDPPacket.xUDPHeader.usSourcePort );
                uint16_t usDestinationPort = FreeRTOS_ntohs( pxProtPacket->xUDPPacket.xUDPHeader.usDestinationPort );
            #endif

            if( ( xPortHasUDPSocket( pxProtPacket->xUDPPacket.xUDPHeader.usDestinationPort ) == pdFALSE )
                #if ipconfigUSE_LLMNR == 1
                    && ( usDestinationPort != ipLLMNR_PORT ) &&
                    ( usSourcePort != ipLLMNR_PORT )
                #endif
                #if ipconfigUSE_MDNS == 1
                    && ( usDestinationPort != ipMDNS_PORT ) &&
                    ( usSourcePort != ipMDNS_PORT )
                #endif
                #if ipconfigUSE_NBNS == 1
                    && ( usDestinationPort != ipNBNS_PORT ) &&
                    ( usSourcePort != ipNBNS_PORT )
                #endif
                #if ipconfigUSE_DNS == 1
                    && ( usSourcePort != ipDNS_PORT )
                #endif
                )
            {
                /* Drop this packet, not for this device. */
                /* FreeRTOS_printf( ( "Drop: UDP port %d -> %d\n", usSourcePort, usDestinationPort ) ); */
                return pdFALSE;
            }
        }
    }
    #endif /* ipconfigETHERNET_DRIVER_FILTERS_PACKETS */
    return pdTRUE;
}

static void prvPassEthMessages( NetworkBufferDescriptor_t * pxDescriptor )
{
    IPStackEvent_t xRxEvent;

    xRxEvent.eEventType = eNetworkRxEvent;
    xRxEvent.pvData = ( void * ) pxDescriptor;

    if( xSendEventStructToIPTask( &xRxEvent, ( TickType_t ) 1000 ) != pdPASS )
    {
        /* The buffer could not be sent to the stack so must be released again.
         * This is a deferred handler task, not a real interrupt, so it is ok to
         * use the task level function here. */
        #if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 )
        {
            do
            {
                NetworkBufferDescriptor_t * pxNext = pxDescriptor->pxNextBuffer;
                vReleaseNetworkBufferAndDescriptor( pxDescriptor );
                pxDescriptor = pxNext;
            } while( pxDescriptor != NULL );
        }
        #else
        {
            vReleaseNetworkBufferAndDescriptor( pxDescriptor );
        }
        #endif /* ipconfigUSE_LINKED_RX_MESSAGES */
        iptraceETHERNET_RX_EVENT_LOST();
        FreeRTOS_printf( ( "prvPassEthMessages: Can not queue return packet!\n" ) );
    }
    else
    {
        iptraceNETWORK_INTERFACE_RECEIVE();
    }
}

static BaseType_t prvNetworkInterfaceInput( void )
{
    #if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 )
        NetworkBufferDescriptor_t * pxFirstDescriptor = NULL;
        NetworkBufferDescriptor_t * pxLastDescriptor = NULL;
    #endif
    BaseType_t xReceivedLength = 0;
    __IO ETH_DMADESCTypeDef * pxDMARxDescriptor;
    const TickType_t xDescriptorWaitTime = pdMS_TO_TICKS( niDESCRIPTOR_WAIT_TIME_MS );
    uint8_t * pucBuffer;

    pxDMARxDescriptor = DMARxDescToGet;

    while( ( pxDMARxDescriptor->Status & ETH_DMARxDesc_OWN ) == 0u )
    {
        NetworkBufferDescriptor_t * pxCurDescriptor;
        NetworkBufferDescriptor_t * pxNewDescriptor = NULL;
        BaseType_t xAccepted = pdTRUE;

        /* Get the Frame Length of the received packet: subtract 4 bytes of the CRC */
        xReceivedLength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;

        pucBuffer = ( uint8_t * ) pxDMARxDescriptor->Buffer1Addr;

        /* Update the ETHERNET DMA global Rx descriptor with next Rx descriptor */
        /* Chained Mode */
        /* Selects the next DMA Rx descriptor list for next buffer to read */
        DMARxDescToGet = ( ETH_DMADESCTypeDef * ) pxDMARxDescriptor->Buffer2NextDescAddr;

        /* In order to make the code easier and faster, only packets in a single buffer
         * will be accepted.  This can be done by making the buffers large enough to
         * hold a complete Ethernet packet, minus ipBUFFER_PADDING.
         * Therefore, two sanity checks: */
        configASSERT( xReceivedLength <= ETH_MAX_PACKET_SIZE );

        if( xAccepted != pdFALSE )
        {
            /* The packet will be accepted, but check first if a new Network Buffer can
             * be obtained. If not, the packet will still be dropped. */
            pxNewDescriptor = pxGetNetworkBufferWithDescriptor( ETH_MAX_PACKET_SIZE, xDescriptorWaitTime );

            if( pxNewDescriptor == NULL )
            {
                /* A new descriptor can not be allocated now. This packet will be dropped. */
                xAccepted = pdFALSE;
            }
        }

        #if ( ipconfigZERO_COPY_RX_DRIVER != 0 )
		/* Find out which Network Buffer was originally passed to the descriptor. */
		pxCurDescriptor = pxPacketBuffer_to_NetworkBuffer( pucBuffer );
		configASSERT( pxCurDescriptor != NULL );
		configASSERT( pxCurDescriptor->pucEthernetBuffer != NULL );
        #else
		/* In this mode, the two descriptors are the same. */
		pxCurDescriptor = pxNewDescriptor;
		if( pxNewDescriptor != NULL )
		{
			/* The packet is accepted and a new Network Buffer was created,
			 * copy data to the Network Buffer. */
			memcpy( pxNewDescriptor->pucEthernetBuffer, pucBuffer, xReceivedLength );
		}
        #endif /* if ( ipconfigZERO_COPY_RX_DRIVER != 0 ) */

        if( xAccepted != pdFALSE )
        {
            pxCurDescriptor->xDataLength = xReceivedLength;
            pxCurDescriptor->pxInterface = pxMyInterface;
            pxCurDescriptor->pxEndPoint = FreeRTOS_MatchingEndpoint( pxMyInterface, pxCurDescriptor->pucEthernetBuffer );

            #if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 )
            {
                pxCurDescriptor->pxNextBuffer = NULL;

                if( pxFirstDescriptor == NULL )
                {
                    /* Becomes the first message */
                    pxFirstDescriptor = pxCurDescriptor;
                }
                else if( pxLastDescriptor != NULL )
                {
                    /* Add to the tail */
                    pxLastDescriptor->pxNextBuffer = pxCurDescriptor;
                }

                pxLastDescriptor = pxCurDescriptor;
            }
            #else /* if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 ) */
            {
                prvPassEthMessages( pxCurDescriptor );
            }
            #endif /* if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 ) */
        }

        /* Release descriptors to DMA */
        #if ( ipconfigZERO_COPY_RX_DRIVER != 0 )
        {
            /* Set Buffer1 address pointer */
            if( pxNewDescriptor != NULL )
            {
                pxDMARxDescriptor->Buffer1Addr = ( uint32_t ) pxNewDescriptor->pucEthernetBuffer;
            }
            else
            {
                /* The packet was dropped and the same Network
                 * Buffer will be used to receive a new packet. */
            }
        }
        #endif /* ipconfigZERO_COPY_RX_DRIVER */

        /* Set Buffer1 size and Second Address Chained bit */
        pxDMARxDescriptor->ControlBufferSize = ETH_DMARxDesc_RCH | ETH_MAX_PACKET_SIZE;
        pxDMARxDescriptor->Status = ETH_DMARxDesc_OWN;

        pxDMARxDescriptor = DMARxDescToGet;
    }

    #if ( ipconfigUSE_LINKED_RX_MESSAGES != 0 )
    {
        if( pxFirstDescriptor != NULL )
        {
            prvPassEthMessages( pxFirstDescriptor );
        }
    }
    #endif /* ipconfigUSE_LINKED_RX_MESSAGES */

    return( xReceivedLength > 0 );
}


static void prvEMACHandlerTask( void * pvParameters )
{
    UBaseType_t uxCurrentCount;
    BaseType_t xResult;
    const TickType_t ulMaxBlockTime = pdMS_TO_TICKS( 100UL );
    uint32_t ulISREvents = 0U;

    /* Remove compiler warnings about unused parameters. */
    ( void ) pvParameters;

    ETH_LedLinkSet(CH32F307_NetworkInterface_GetPhyLinkStatus(NULL));

    for( ; ; )
	{
		xResult = 0;

		#if ( ipconfigHAS_PRINTF != 0 )
		{
			/* Call a function that monitors resources: the amount of free network
			 * buffers and the amount of free space on the heap.  See FreeRTOS_IP.c
			 * for more detailed comments. */
			vPrintResourceStats();
		}
		#endif /* ( ipconfigHAS_PRINTF != 0 ) */

		if( xTXDescriptorSemaphore != NULL )
		{
			static UBaseType_t uxLowestSemCount = ( UBaseType_t ) ETH_TXBUFNB - 1;

			uxCurrentCount = uxSemaphoreGetCount( xTXDescriptorSemaphore );

			if( uxLowestSemCount > uxCurrentCount )
			{
				uxLowestSemCount = uxCurrentCount;
				FreeRTOS_printf( ( "TX DMA buffers: lowest %lu\n", uxLowestSemCount ) );
			}
		}

		/* Wait for a new event or a time-out. */
		xTaskNotifyWait( 0U,                /* ulBitsToClearOnEntry */
						 EMAC_IF_ALL_EVENT, /* ulBitsToClearOnExit */
						 &( ulISREvents ),  /* pulNotificationValue */
						 ulMaxBlockTime );

		if( ( ulISREvents & EMAC_IF_RX_EVENT ) != 0 )
		{
			xResult = prvNetworkInterfaceInput();
		}

		if( ( ulISREvents & EMAC_IF_TX_EVENT ) != 0 )
		{
			/* Code to release TX buffers in case zero-copy is used. */
			/* Check if DMA packets have been delivered. */
			vClearTXBuffers();

		}

		if( ( ulISREvents & EMAC_IF_LINK_EVENT ) != 0 )
		{
			ETH_LedLinkSet(CH32F307_NetworkInterface_GetPhyLinkStatus(NULL));
		}

		if( ( ulISREvents & EMAC_IF_ERR_EVENT ) != 0 )
		{
			/* Future extension: logging about errors that occurred. */
		}

		//if( xPhyCheckLinkStatus( &xPhyObject, xResult ) != 0 )
		//{
			/* Something has changed to a Link Status, need re-check. */
		//    prvEthernetUpdateConfig( pdFALSE );

		//    #if ( ipconfigSUPPORT_NETWORK_DOWN_EVENT != 0 )
		//    {
		//        if( xGetPhyLinkStatus( pxMyInterface ) == pdFALSE )
		//        {
		//            FreeRTOS_NetworkDown( pxMyInterface );
		//        }
		//    }
		//    #endif /* ( ipconfigSUPPORT_NETWORK_DOWN_EVENT != 0 ) */
		//}

	}
}

void HAL_ETH_RxCpltCallback( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Pass a TX-event and wakeup the prvEMACHandlerTask. */
	if( xEMACTaskHandle != NULL )
	{
		xTaskNotifyFromISR( xEMACTaskHandle, EMAC_IF_RX_EVENT, eSetBits, &( xHigherPriorityTaskWoken ) );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void HAL_ETH_TxCpltCallback( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Pass a TX-event and wakeup the prvEMACHandlerTask. */
	if( xEMACTaskHandle != NULL )
	{
		xTaskNotifyFromISR( xEMACTaskHandle, EMAC_IF_TX_EVENT, eSetBits, &( xHigherPriorityTaskWoken ) );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void HAL_ETH_PhyLinkCallback(){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		/* Pass a TX-event and wakeup the prvEMACHandlerTask. */
		if( xEMACTaskHandle != NULL )
		{
			xTaskNotifyFromISR( xEMACTaskHandle, EMAC_IF_LINK_EVENT, eSetBits, &( xHigherPriorityTaskWoken ) );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
}

void ETH_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/*********************************************************************
 * @fn      ETH_IRQHandler
 *
 * @brief   This function handles ETH exception.
 *
 * @return  none
 */
void ETH_IRQHandler(void) {
    if (ETH->DMASR & ETH_DMA_IT_R) {
        ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
        //HandleEthRx();
        HAL_ETH_RxCpltCallback();
    }
    if (ETH->DMASR & ETH_DMA_IT_T) {
    	HAL_ETH_TxCpltCallback();
        ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
    }
    if (ETH->DMASR & ETH_DMA_IT_PHYLINK) {
		HAL_ETH_PhyLinkCallback();
		ETH_DMAClearITPendingBit(ETH_DMA_IT_PHYLINK);
	}
    ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}
