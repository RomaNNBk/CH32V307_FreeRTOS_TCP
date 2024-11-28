#ifndef NET__H
#define NET__H

#include "ch32v30x_eth.h"
#include "ch32v30x_exti.h"
#include "ch32v30x_rng.h"

#define PHY_ADDRESS 0x01

#define ETH_RXBUFNB 4
#define ETH_TXBUFNB 4

#define LED_OFF 0
#define LED_ON 1

extern ETH_DMADESCTypeDef *DMATxDescToSet;
extern ETH_DMADESCTypeDef *DMARxDescToGet;

extern ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB]; /* 接收描述符表 */
extern ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB]; /* 发送描述符表 */

void ETH_LedLinkSet( uint8_t mode );
extern uint32_t ETH_TxPkt_ChainMode(u16 FrameLength);
void ETH_Init( uint8_t *macAddr );
void init_phy(void);
#endif
