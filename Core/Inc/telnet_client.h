/**
  ******************************************************************************
  * @file           : telnet_client.h
  * @brief          : Header for telnet_client.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Stuart Kenny.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_TELNETCLIENT_H_
#define INC_TELNETCLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32h7xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

//void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
//void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
//Telnet commands taken from RFC854 (May 1983)
#define TEL_CMD_ECHO 1
#define TEL_CMD_WINDOW_SIZE 31
#define TEL_SE 0xf0 //240    End of subnegotiation parameters.
#define TEL_NOP 0xf1 //241    No operation.
#define TEL_DATA_MARK 0xf2 //242    The data stream portion of a Synch.
#define TEL_BREAK 0xf3 //243    NVT character BRK.
#define TEL_INTERRUPT_PROCESS 0xf4 //244    The function IP.
#define TEL_ABORT_OUTPUT 0xf5 //245    The function AO.
#define TEL_ARE_YOU_THERE 0xf6 //246    The function AYT.
#define TEL_ERASE_CHARACTER 0xf7 //247    The function EC.
#define TEL_ERASE_LINE 0xf8 //248    The function EL.
#define TEL_GO_AHEAD 0xf9 //249    The GA signal.
#define TEL_SB 0xfa //250    Beginning of subnegotiation.
#define TEL_WILL 0xfb //251
#define TEL_WONT 0xfc //252
#define TEL_DO 0xfd //253
#define TEL_DONT 0xfe //254
#define TEL_CMD 0xff //255
#define LASERID "Stanford_Research_Systems,LDC501,s/n148374,ver2.46"

//LDC5xx laser controller has a 64-byte input buffer and 256-byte output queue
#define BUFLEN 20

#ifdef __cplusplus
}
#endif

#endif /* INC_TELNETCLIENT_H_ */
