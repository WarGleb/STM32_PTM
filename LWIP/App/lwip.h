/**
  ******************************************************************************
  * File Name          : LWIP.h
  * Description        : This file provides code for the configuration
  *                      of the LWIP.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *************************************************************************

  */
#ifndef __mx_lwip_H
#define __mx_lwip_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"

 /* Network interface name */
 #define IFNAME0 						's'
 #define IFNAME1 						't'
 /* ETH Setting  */
 #define ETH_DMA_TRANSMIT_TIMEOUT		( 20U )
 #define ETH_RX_BUFFER_CNT				( 12U )
 #define ETH_TX_BUFFER_MAX				((ETH_TX_DESC_CNT) * 2U)
 /* The time to block waiting for input. */
 #define TIME_WAITING_FOR_INPUT			( portMAX_DELAY )
 /* Stack size of the interface thread */
 #define INTERFACE_THREAD_STACK_SIZE	( 350 )
 /* Memory Pool Declaration */

/* Global Variables ----------------------------------------------------------*/
extern ETH_HandleTypeDef heth;


/* Public function prototypes. */
void MX_LWIP_Init(void);

void Error_Handler(void);
u32_t sys_jiffies(void);
u32_t sys_now(void);

//err_t ethernetif_init(struct netif *netif);
//void ethernet_link_thread(void const * argument);

#ifdef __cplusplus
}
#endif
#endif /*__ mx_lwip_H */

