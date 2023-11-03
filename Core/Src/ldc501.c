/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  	   lad501.c
  Modified By:     ControllersTech.com & Stuart Kenny
  Updated:    	   Oct-2023

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable  application.
 *
 **/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <ldc501.h>
#include "lwip/tcp.h"
#include <stdbool.h>
#include <string.h>

/* Typedefs -----------------------------------------------------------*/
/* structure for maintaining connection infos to be passed as argument
   to LwIP callbacks*/
struct telnet_client_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

/*  protocol states */
enum telnet_client_states
{
  TC_NONE = 0,
  TC_CONNECTED,
  TC_RECEIVING,
  TC_CLOSING
};

///* structure for managing comms to/from the laser driver*/
//struct ldc_comms
//{
//  u8_t state; //current connection state
//  u8_t retries;
//  char message[257]; //maximum sized message is 256 B
//};

/*  protocol states */
enum ldc_comms_states
{
  LDC_DISCONNECTED = 0,
  LDC_CONNECTED_IDLE,
  LDC_AWAITING_RESPONSE,
  LDC_RESPONSE_RECEIVED
};

extern struct netif gnetif;
extern TIM_HandleTypeDef htim1;

/* Defines ------------------------------------------------------------*/
//#define TELNET_DEBUG
#define LDC_DEBUG
#define LASER_ENABLE //disable for testing
#define TELNET_RETRIES 1 //don't raise above 1 until code can flush extra responses
#define TELNET_TIMEOUT 6000 //3s
#define TEC_STABILISE_TIME 10000 //5s
#define LDC_ADDR1 192
#define LDC_ADDR2 168
#define LDC_ADDR3 1
#define LDC_ADDR4 11 //11 for LDC, 12 for Oscar, 14 for Micawber
#define LDC_PORT 8886 //choose 8888 for command port and 8886 for debug (provides status feedback)
/* LDC status and error messages */
#define DEBUG_CONNECTED_MESSAGE "220 Welcome DBG server!"
#define CONTROLLER_ID "Stanford_Research_Systems,LDC501,s/n148374,ver2.46" //the expected response from the laser controller
#define LDC_ERROR "err? @ parser"
//#define SIMULATE_LDC
//#define LDC_SYNTH_MESSAGE "Hercules on Micawber"
#define TEC_CURRENT_AT_LIMIT "IMAX=1 at TECR"
#define TEC_CURRENT_BACK_INSIDE_LIMIT "IMAX=0 at TECR"
/* LDC commands */
#define LASER_DIODE_ONOFF "LDON"
#define SET_LASER_DIODE_CURRENT "SILD"
#define READ_LASER_DIODE_CURRENT "RILD" //can be different from the setpoint, e.g. when modulation used
#define READ_LASER_POWER "RWPD" //returns reading in mW
#define LASER_CONTROL_MODE "SMOD" //should be 0 for constant current
#define READ_INTERLOCK_STATE "ILOC"
#define TEC_CURRENT_ONOFF "TEON"
#define TEMP "TEMP"
#define READ_LD_TEMPERATURE "TTRD" //returns reading in degrees C
#define TEC_CONTROL_MODE "TMOD" //should be 1 for constant temperature

/* Variables ---------------------------------------------------------*/

/* create a struct to store data */
struct telnet_client_struct *tcTx = 0;
struct tcp_pcb *pcbTx = 0;

/* store LDC comms related info as globals*/
u8_t ldc_comms_state = LDC_DISCONNECTED; //current connection state
u8_t ldc_comms_retries = 0;
char ldc_comms_message[257] = {"Initial message"}; //maximum sized message is 256 B
char ldc_comms_command[20] = {"YOLO"};

/* Function prototypes -----------------------------------------------*/
/* Establishes connection with telnet server */
__attribute__((section(".itcm"))) bool telnet_client_init(void);
/* Initialises Ethernet comms with LDC501 */
__attribute__((section(".itcm"))) bool init_ldc_comms(void);
/* Initialises LDC501 TEC */
__attribute__((section(".itcm"))) bool init_ldc_tec(void);
/* Sends a string to the LDC501 over Ethernet */
__attribute__((section(".itcm"))) void ldc_tx(const char str[]);
/* Sets laser current */
__attribute__((section(".itcm"))) void set_laser_current(const double i);
/* Power up/down laser */
__attribute__((section(".itcm"))) bool set_laser_state(bool laserstate);
/* Sends a command to the LDC501 and waits for a response */
__attribute__((section(".itcm"))) bool ldc_query(const char str[]);
/* This callback will be called, when the client is connected to the server */
__attribute__((section(".itcm"))) static err_t telnet_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
/* This callback will be called, when the client receives data from the server */
__attribute__((section(".itcm"))) static err_t telnet_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
/* This callback will be called, when the server Polls for the Client */
__attribute__((section(".itcm"))) static err_t telnet_client_poll(void *arg, struct tcp_pcb *tpcb);
/* This callback will be called, when the server acknowledges the data sent by the client */
__attribute__((section(".itcm"))) static err_t telnet_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
/* A Function to send the data to the server */
__attribute__((section(".itcm"))) static void telnet_client_send(struct tcp_pcb *tpcb, struct telnet_client_struct *tc);
/* Function to close the connection */
__attribute__((section(".itcm"))) static void telnet_client_connection_close(struct tcp_pcb *tpcb, struct telnet_client_struct *tc);
/* This is the part where we are going to handle the incoming data from the server */
__attribute__((section(".itcm"))) static void example_client_handle (struct tcp_pcb *tpcb, struct telnet_client_struct *tc);
__attribute__((section(".itcm"))) static void telnet_client_handle (struct tcp_pcb *tpcb, struct telnet_client_struct *tc, struct pbuf *p);
extern uint32_t start_timer(TIM_TypeDef * timer);
extern uint32_t stop_timer(TIM_TypeDef * timer);
extern uint32_t check_timer(TIM_TypeDef *timer);
//extern void Error_Handler(void);
extern void ethernetif_input(struct netif *netif);
extern void sys_check_timeouts(void);

/**
  * @brief  Function x.
  * @retval int
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	char buf[100];
////	printf("1s interrupt kicked.\r\n");
//
//	/* Prepare the first message to send to the server */
//	int len = sprintf (buf, "Sending TCPclient Message %d\n", counter);
//
//	if (counter !=0)
//	{
//		/* allocate pbuf */
//		tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL);
//		/* copy data to pbuf */
//		pbuf_take(tcTx->p, (char*)buf, len);
//		telnet_client_send(pcbTx, tcTx);
//		pbuf_free(tcTx->p);
//
//      unsigned char tmp1[10] = {83, 116, 117, 97, 114, 116, 10}; //Stuart NEWLINE
//    	len = sprintf (buf, tmp1);
//    	tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
//    	pbuf_take(tcTx->p, (char*)buf, len); // copy data to pbuf
//    	telnet_client_send(pcbTx, tcTx); //send it
//    	pbuf_free(tcTx->p); //free up the pbuf
//
//    	float laservalue = 160.56789;
//    	len = sprintf (buf, "SILD%.2f\n", laservalue);
//    	tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
//    	pbuf_take(tcTx->p, (char*)buf, len); // copy data to pbuf
//    	telnet_client_send(pcbTx, tcTx); //send it
//    	pbuf_free(tcTx->p); //free up the pbuf
//	}

}

/**
 * Initialises the connection to the LDC by initiating connection
 * and setting up the necessary callbacks
 * @return ERR_VAL if invalid arguments are given
 *         ERR_OK if connect request has been sent
 *         other err_t values if connect request couldn't be sent
 */
bool telnet_client_init(void)
{
	/* create new tcp pcb */
	struct tcp_pcb *tpcb;
	tpcb = tcp_new();

	/* Connect to the server */
	ip_addr_t destIPADDR;
	IP_ADDR4(&destIPADDR, LDC_ADDR1, LDC_ADDR2, LDC_ADDR3, LDC_ADDR4);
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Beginning TCP connection.\n\r");
		printf("[Telnet Client] Connecting to %s on port %d.\n\r", ipaddr_ntoa(&destIPADDR), LDC_PORT);
	#endif //TELNET_DEBUG
	err_t ret = tcp_connect(tpcb, &destIPADDR, LDC_PORT, telnet_client_connected);
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Called tcp_connect, awaiting callback.\n\r");
		printf("[Telnet Client] tcp_connect returned %i\n\r", ret);
	#endif //TELNET_DEBUG
	if(ret == ERR_OK) {//if the TCP connection request worked
		ldc_comms_state = LDC_CONNECTED_IDLE;
		start_timer(ETHERNET_TIMER);
		while ((ldc_comms_state == LDC_CONNECTED_IDLE) && (check_timer(ETHERNET_TIMER) < TELNET_TIMEOUT)) {//loop here until timeout or response is received
		}
		stop_timer(ETHERNET_TIMER);
		if(strncmp(ldc_comms_message, DEBUG_CONNECTED_MESSAGE, strlen(DEBUG_CONNECTED_MESSAGE)) == 0) {
			#ifdef LDC_DEBUG
				printf("Confirmation received - Connected to debug port");
			#endif //LDC_DEBUG
			return (true);
		}
#ifdef SIMULATE_LDC
		printf("SIMULATING successful connection to debug port\n\r");
		return (true);
#endif //SIMULATE_LDC
	} else {
		printf("[Telnet Client] ERROR: tcp_connect returned %i\n\r", ret);
	}
	return (false);
}

/**
 * Initialise Ethernet comms with LDC501 and check ID
 * @return TRUE if expected ID confirmed
 */
bool init_ldc_comms(void)
{
	ldc_tx("\r\n"); //return character
	ldc_tx("uloc1\r\n"); //unlock comms
//	ldc_tx("TEON1\r\n"); //Turn TEC on
//	ldc_tx("SILD159.90\r\n"); //Set laser current to 159.9mA
	if(ldc_query("*idn?\r\n")){; //request ID
//		printf("Successful command response\r\n");
		if(strncmp(ldc_comms_message, CONTROLLER_ID, strlen(CONTROLLER_ID)) == 0) {
			#ifdef LDC_DEBUG
		    	printf("[LDC] Expected controller ID received\r\n");
			#endif //LDC_DEBUG
		    return(true);
		}
	}
	return(false);
}

/**
 * Initialise Ethernet comms with LDC501 and check ID
 * @return TRUE if expected ID confirmed
 */
bool init_ldc_tec(void)
{
	ldc_tx("TMOD1\r\n"); //Constant temperature mode
	ldc_tx("TEMP21.15\r\n"); //Set operating point of 21.15C
	ldc_tx("TEON1\r\n"); //Turn TEC on
	printf ("TEC powered up - allowing 5s for TEC to stabilise\r\n");
	start_timer(ETHERNET_TIMER);
	while (check_timer(ETHERNET_TIMER) < TEC_STABILISE_TIME) {//loop here for a few seconds
//			printf ("Spinning round loop waiting for a response");
	    /* Ethernet handling */
		/* This allows receipt of IMAX warning messages */
		ethernetif_input(&gnetif);
		sys_check_timeouts();
	}
	stop_timer(ETHERNET_TIMER);
	if(ldc_query("TTRD?\r\n")){; //read laser temperature
		printf ("Measured diode temperature: %s\r\n", ldc_comms_message);
		    return(true);
	}
	return(false);
}

/* Send a string to the LDC501 over telnet */
void ldc_tx(const char str[])
{
	strcpy(ldc_comms_command, str); //make a copy of str in case a retry is required
	uint16_t len = strlen(str);
	tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
	pbuf_take(tcTx->p, (char*)str, len); // copy data to pbuf
	telnet_client_send(pcbTx, tcTx); //send it
	#ifdef LDC_DEBUG
//		printf("[LDC] Sent over Ethernet: %s\r\n", str);
		printf("[LDC] Sent over Ethernet: %s", str); //no newline as one is included with the telnet message
	#endif //LDC_DEBUG
	/*disabled as the pbuf is already null.
	 * This causes assertion errors but worried as this could overflow
	 */
	//pbuf_free(tcTx->p); //free up the pbuf
}

/* Set laser current */
void set_laser_current(const double i)
{
	char string[12];
//	uint8_t len = sprintf (string, "SILD%.2f\n", i);
	sprintf (string, "SILD%.2f\n", i);
	ldc_tx(string); //send command
}

/* Power up/down laser */
bool set_laser_state(bool laserstate)
{
	if (laserstate) {//power on requested
		if (ldc_query("ILOC?")) {//check interlock status
			printf("Response to interlock query: %s\r\n", ldc_comms_message);
			if (ldc_comms_message[0] == "0") {
//				(strncmp(ldc_comms_message, 0, 1) == 0)
				printf("Bingely bon, laser on\r\n");
//				ldc_tx("LDON1"); //turn on laser diode
			} else {
			printf("INTERLOCK OPEN: Can't power up laser diode\r\n");
			}
		}
	} else {
		ldc_tx("LDON0"); //turn off laser diode
		printf("Laser diode powered down\r\n");
	}
}

/* Send a command to the LDC501 over telnet and await response */
bool ldc_query(const char str[])
{
	ldc_comms_retries = 0;
	while (ldc_comms_retries < TELNET_RETRIES) {
#ifdef LDC_DEBUG
		printf ("[LDC] Entered command while loop\r\n");
		printf ("[LDC] Started Ethernet timer\r\n");
#endif //LDC_DEBUG
		start_timer(ETHERNET_TIMER);
		ldc_tx(str); //Send string
		ldc_comms_state = LDC_AWAITING_RESPONSE; //set state to flag that a response is needed before sending anything further
		while ((ldc_comms_state == LDC_AWAITING_RESPONSE) && (check_timer(ETHERNET_TIMER) < TELNET_TIMEOUT)) {//loop here until timeout or response is received
//			printf ("Spinning round loop waiting for a response");
			/* Ethernet handling */
			ethernetif_input(&gnetif);
			sys_check_timeouts();
		}
		stop_timer(ETHERNET_TIMER);
		if (ldc_comms_state == LDC_RESPONSE_RECEIVED) {
			#ifdef LDC_DEBUG
				printf ("[LDC] Response to query received\r\n");
			#endif //LDC_DEBUG
			return (true); //success
		}
		ldc_comms_retries++; //increase retry count
		#ifdef LDC_DEBUG
			printf ("[LDC] Issued command %s but no response. Retry %u\r\n", str, ldc_comms_retries);
		#endif //LDC_DEBUG
	}
	#ifdef LDC_DEBUG
		printf ("[LDC] Command %s failed after %u attempts\r\n", str, ldc_comms_retries);
	#endif //LDC_DEBUG
	return (false); //failure after timeout
}

/** This callback is called, when the client is connected to the server
 * Here we will initialise few other callbacks
 * and in the end, call the client handle function
  */
static err_t telnet_client_connected(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct telnet_client_struct *tc;

#ifdef TELNET_DEBUG
	printf("[Telnet Client] telnet_client_connected function called.\n\r");
#endif #endif //TELNET_DEBUG

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* allocate structure tc to maintain tcp connection information */
  tc = (struct telnet_client_struct *)mem_malloc(sizeof(struct telnet_client_struct));
  if (tc != NULL)
  {
    tc->state = TC_CONNECTED;
    tc->pcb = newpcb;
    tc->retries = 0;
    tc->p = NULL;

    /* pass newly allocated tc structure as argument to newpcb */
    tcp_arg(newpcb, tc);

    /* initialize lwip tcp_recv callback function for newpcb  */
    tcp_recv(newpcb, telnet_client_recv);

    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, telnet_client_poll, 0);

    /* initialize LwIP tcp_sent callback function */
    tcp_sent(newpcb, telnet_client_sent);

    /* handle the TCP data */
//    example_client_handle(newpcb, tc);
//    telnet_client_handle(newpcb, tc, p);

    #ifdef TELNET_DEBUG
		printf("[Telnet Client] Successful connection.\n\r");
	#endif //TELNET_DEBUG

//	telnet_initialised = 1;
    ret_err = ERR_OK;
  }
  else
  {
    /*  close tcp connection */
    telnet_client_connection_close(newpcb, tc);
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Connection closed due to memory error.\n\r");
	#endif //TELNET_DEBUG
    /* return memory error */
    ret_err = ERR_MEM;
  }
  return ret_err;
}

/** This callback is called, when the client receives some data from the server
 * if the data received is valid, we will handle the data in the client handle function
  */
static err_t telnet_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  #ifdef TELNET_DEBUG
    printf("[Telnet Client] Received data from server.\n\r");
  #endif //TELNET_DEBUG
  struct telnet_client_struct *tc;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);

  tc = (struct telnet_client_struct *)arg;

  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    tc->state = TC_CLOSING;
    if(tc->p == NULL)
    {
       /* we're done sending, close connection */
       telnet_client_connection_close(tpcb, tc);
    }
    else
    {
      /* we're not done yet */
//      /* acknowledge received packet */
//      tcp_sent(tpcb, telnet_client_sent);

      /* send remaining data*/
//      telnet_client_send(tpcb, tc);
    }
    ret_err = ERR_OK;
  }
  /* else : a non empty frame was received from server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      tc->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(tc->state == TC_CONNECTED)
  {
   /* store reference to incoming pbuf (chain) */
    tc->p = p;

    // tcp_sent has already been initialized in the beginning.
//    /* initialize LwIP tcp_sent callback function */
//    tcp_sent(tpcb, telnet_client_sent);

    /* Acknowledge the received data */
    tcp_recved(tpcb, p->tot_len);
    #ifdef TELNET_DEBUG
      printf("[Telnet Client] Acknowledging received data.\n\r");
    #endif //TELNET_DEBUG

    /* handle the received data */
//    example_client_handle(tpcb, tc);
    telnet_client_handle(tpcb, tc, p);

    pbuf_free(p);

    ret_err = ERR_OK;
  }
  else if(tc->state == TC_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    tc->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unknown tc->state, trash data  */
    tcp_recved(tpcb, p->tot_len);
    tc->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}


static err_t telnet_client_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct telnet_client_struct *tc;

  tc = (struct telnet_client_struct *)arg;
  if (tc != NULL)
  {
    if (tc->p != NULL)
    {
        // tcp_sent has already been initialized in the beginning.
//      tcp_sent(tpcb, telnet_client_sent);
      /* there is a remaining pbuf (chain) , try to send data */
//      telnet_client_send(tpcb, tc);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(tc->state == TC_CLOSING)
      {
        /*  close tcp connection */
        telnet_client_connection_close(tpcb, tc);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/** This callback is called, when the server acknowledges the data sent by the client
 * If there is no more data left to sent, we will simply close the connection
  */
static err_t telnet_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Server has acknowledged data sent by client.\n\r");
	#endif //TELNET_DEBUG
  struct telnet_client_struct *tc;

  LWIP_UNUSED_ARG(len);

  tc = (struct telnet_client_struct *)arg;
  tc->retries = 0;

  if(tc->p != NULL)
  {
	// tcp_sent has already been initialized in the beginning.
    /* still got pbufs to send */
//    tcp_sent(tpcb, telnet_client_sent);


//    telnet_client_send(tpcb, tc);
  }
  else
  {
    /* if no more data to send and client closed connection*/
    if(tc->state == TC_CLOSING)
      telnet_client_connection_close(tpcb, tc);
  }
  return ERR_OK;
}


/** A function to send the data to the server
  */
static void telnet_client_send(struct tcp_pcb *tpcb, struct telnet_client_struct *tc)
{
  #ifdef TELNET_DEBUG
    printf("[Telnet Client] Sending data to server.\n\r");
  #endif //TELNET_DEBUG
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

  while ((wr_err == ERR_OK) &&
         (tc->p != NULL) &&
         (tc->p->len <= tcp_sndbuf(tpcb)))
  {

    /* get pointer on pbuf from tc structure */
    ptr = tc->p;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);

    if (wr_err == ERR_OK)
    {
      u16_t plen;
      u8_t freed;

      plen = ptr->len;

      /* continue with next pbuf in chain (if any) */
      tc->p = ptr->next;

      if(tc->p != NULL)
      {
        /* increment reference count for tc->p */
        pbuf_ref(tc->p);
      }

     /* chop first pbuf from chain */
      do
      {
        /* try hard to free pbuf */
        freed = pbuf_free(ptr);
      }
      while(freed == 0);
     /* we can read more data now */
//     tcp_recved(tpcb, plen);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     tc->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}


static void telnet_client_connection_close(struct tcp_pcb *tpcb, struct telnet_client_struct *tc)
{
  #ifdef TELNET_DEBUG
    printf("[Telnet Client] Closing connection.\n\r");
  #endif //TELNET_DEBUG
  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);

  /* delete tc structure */
  if (tc != NULL)
  {
    mem_free(tc);
  }

  /* close tcp connection */
  tcp_close(tpcb);
}

/* Function to handle the incoming TCP Data */

static void telnet_client_handle (struct tcp_pcb *tpcb, struct telnet_client_struct *tc, struct pbuf *p)
{
	//function has been called as telnet_client_handle(tpcb, tc, p);
  #ifdef TELNET_DEBUG
    printf("[Telnet Client] Handling incoming data.\n\r");
  #endif //TELNET_DEBUG
    /* get the Remote IP */
	ip4_addr_t inIP = tpcb->remote_ip;
//	uint16_t inPort = tpcb->remote_port;

	/* Extract the IP */
//	char *remIP = ipaddr_ntoa(&inIP);
	tcTx = tc;
	pcbTx = tpcb;

	if (p -> len != p -> tot_len) {//spans more that one buffer and I haven't allowed for this yet
		printf("[Telnet Client] ERROR - Received data spans more than one pbuf.\n\r");
	}

	/* Copy payload into a string */
	uint16_t len = p -> len; //length of the payload
	memcpy(ldc_comms_message, p -> payload, p -> len); //copy the payload across
	ldc_comms_message[len] = '\0'; //assigns null character to terminate string
	if(strncmp(ldc_comms_message, LDC_ERROR, strlen(LDC_ERROR)) == 0) {
		printf("LDC did not understand the last command\n\r");
		printf("RETRY: %s", ldc_comms_command);
		ldc_tx(ldc_comms_command); //retry the last command
	} else {
		ldc_comms_state = LDC_RESPONSE_RECEIVED;
	}
	#ifdef LDC_DEBUG
		printf("[LDC] Message received: %s",ldc_comms_message); //no newline as one is included with the telnet message received
	#endif //LDC_DEBUG
}


