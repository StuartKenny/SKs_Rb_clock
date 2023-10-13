/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  	   telnet_client.c
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
#include "tcp_client.h"
#include "lwip/tcp.h"
#include <stdbool.h>

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

extern TIM_HandleTypeDef htim1;

/* Defines ------------------------------------------------------------*/
#define TELNET_DEBUG

/* Variables ---------------------------------------------------------*/
bool telnet_initialised = 0;
int counter = 0;
uint8_t data[100];

/* create a struct to store data */
struct telnet_client_struct *tcTx = 0;
struct tcp_pcb *pcbTx = 0;

/* Function prototypes -----------------------------------------------*/
/* Establishes connection with telnet server */
void telnet_client_init(void);
/* Returns the state of the telnet link */
__attribute__((section(".itcm"))) bool is_telnet_initialised(void);
/* Me dicking about sending a few packets to see if it works */
__attribute__((section(".itcm"))) void one_off (void);
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
__attribute__((section(".itcm"))) static void telnet_client_handle (struct tcp_pcb *tpcb, struct telnet_client_struct *tc);

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

/* IMPLEMENTATION FOR TELNET CLIENT

1. Create TCP block.
2. connect to the server
3. start communicating
*/

void telnet_client_init(void)
{
	/* 1. create new tcp pcb */
	struct tcp_pcb *tpcb;

	tpcb = tcp_new();

	/* 2. Connect to the server */
	ip_addr_t destIPADDR;
//	IP_ADDR4(&destIPADDR, 192, 168, 1, 11); //IP address of LDC501
	IP_ADDR4(&destIPADDR, 192, 168, 1, 14); //IP address of Micawber
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Beginning TCP connection.\n\r");
		printf("[Telnet Client] Connecting to 192.168.1.12 on port %d.\n\r", 8886);
	#endif
	tcp_connect(tpcb, &destIPADDR, 8886, telnet_client_connected);
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Called tcp_connect, awaiting callback.\n\r");
	#endif
}

/** Returns 1 if the telnet link has been initialised and has not been subsequently closed.
  */
bool is_telnet_initialised(void) {
	return telnet_initialised;
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
#endif

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
    telnet_client_handle(newpcb, tc);

    #ifdef TELNET_DEBUG
		printf("[Telnet Client] Successful connection.\n\r");
	#endif

	telnet_initialised = 1;
    ret_err = ERR_OK;
  }
  else
  {
    /*  close tcp connection */
    telnet_client_connection_close(newpcb, tc);
	#ifdef TELNET_DEBUG
		printf("[Telnet Client] Connection closed due to memory error.\n\r");
	#endif
    /* return memory error */
    ret_err = ERR_MEM;
  }
  return ret_err;
}

void one_off (void) {
	char buf[100];
	uint8_t counter = 0;

	/* Prepare the first message to send to the server */
	int len = sprintf (buf, "Sending telnet_client Message %d\n", counter);

	/* allocate pbuf */
	tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL);
	/* copy data to pbuf */
	pbuf_take(tcTx->p, (char*)buf, len);
	telnet_client_send(pcbTx, tcTx);
	pbuf_free(tcTx->p);

	unsigned char tmp1[10] = {83, 116, 117, 97, 114, 116, 10}; //Stuart NEWLINE
	len = sprintf (buf, "%s", tmp1);
	tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
	pbuf_take(tcTx->p, (char*)buf, len); // copy data to pbuf
	telnet_client_send(pcbTx, tcTx); //send it
	pbuf_free(tcTx->p); //free up the pbuf

	float laservalue = 160.56789;
	len = sprintf (buf, "SILD%.2f\n", laservalue);
	tcTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
	pbuf_take(tcTx->p, (char*)buf, len); // copy data to pbuf
	telnet_client_send(pcbTx, tcTx); //send it
	pbuf_free(tcTx->p); //free up the pbuf
}

/** This callback is called, when the client receives some data from the server
 * if the data received is valid, we will handle the data in the client handle function
  */
static err_t telnet_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  #ifdef TELNET_DEBUG
    printf("[Telnet Client] Received data from server.\n\r");
  #endif
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
    #endif

    /* handle the received data */
    telnet_client_handle(tpcb, tc);

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
#endif
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
  #endif
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
  #endif
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

  telnet_initialised = 0;
}

/* Handle the incoming TCP Data */

static void telnet_client_handle (struct tcp_pcb *tpcb, struct telnet_client_struct *tc)
{
  #ifdef TELNET_DEBUG
    printf("[Telnet Client] Handling incoming data.\n\r");
  #endif
    /* get the Remote IP */
	ip4_addr_t inIP = tpcb->remote_ip;
	uint16_t inPort = tpcb->remote_port;

	/* Extract the IP */
	char *remIP = ipaddr_ntoa(&inIP);

	tcTx->state = tc->state;
	tcTx->pcb = tc->pcb;
	tcTx->p = tc->p;

	tcTx = tc;
	pcbTx = tpcb;

	counter++;

}
