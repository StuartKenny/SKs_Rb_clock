/**
 ******************************************************************************
 * @file           : telnet_client.c
 * @brief          : Simplet telnet client for communication with LDC501 over Ethernet
 * @author		   : Stuart Kenny
 ******************************************************************************
   * @attention
  *
  * Copyright (c) 2023 Stuart Kenny.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "telnet_client.h"
#include "lwip/tcp.h"
//#include "main.h" //needed for port and timer definitions
//#include <stdio.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/


/* Function prototypes -----------------------------------------------*/
//__attribute__((section(".itcm"))) static uint32_t template_function(const uint32_t data, const bool verify);
//extern void Error_Handler(void);

/**
  * @brief  Function x.
  * @retval int
  */

///* telnet connection state */
//enum telnet_client_states
//{
//  TELNET_NONE = 0,
//  TELNET_CONNECTING,
//  TELNET_CONNECTED,
//  TELNET_CLOSED
//};
//u8_t telnet_client_state = TELNET_NONE;

//	int len = sprintf (buf, "Sending TCPclient Message %d\n", counter);
//	esTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
//	pbuf_take(esTx->p, (char*)buf, len); // copy data to pbuf
//	tcp_client_send(pcbTx, esTx);
//	pbuf_free(esTx->p);

/**
  * @brief  Telnet negotiation function.
  * @retval None
  * Screen size is advertised as 24x80, otherwise will respond to a DO request with WON'T, and a WILL with DO
  */
void negotiate(int sock, unsigned char *buf, int len) {
    int i;
    int len;

    if (buf[1] == TEL_DO && buf[2] == TEL_CMD_WINDOW_SIZE) {
        unsigned char tmp1[10] = {255, 251, 31}; //CMD WILL CMD_WINDOW_SIZE
//        if (send(sock, tmp1, 3 , 0) < 0)
//            exit(1);
    	len = sprintf (buf, tmp1);
    	esTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
    	pbuf_take(esTx->p, (char*)buf, len); // copy data to pbuf
    	tcp_client_send(pcbTx, esTx); //send it
    	pbuf_free(esTx->p); //free up the pbuf

        unsigned char tmp2[10] = {255, 250, 31, 0, 80, 0, 24, 255, 240}; //CMD Subnegotiate CMD_WINDOW_SIZE NUL 80 NUL 24 CMD End-subnegotiation
//        if (send(sock, tmp2, 9, 0) < 0)
//            exit(1);
    	len = sprintf (buf, tmp2);
    	esTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
    	pbuf_take(esTx->p, (char*)buf, len); // copy data to pbuf
    	tcp_client_send(pcbTx, esTx); //send it
    	pbuf_free(esTx->p); //free up the pbuf
        return;
    }

    for (i = 0; i < len; i++) {
        if (buf[i] == TEL_DO)
            buf[i] = TEL_WONT; //change DO to WONT
        else if (buf[i] == TEL_WILL)
            buf[i] = TEL_DO; //change WILL to DO
    }

//    if (send(sock, buf, len , 0) < 0)
//        exit(1);
	esTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL); //allocate pbuf
	pbuf_take(esTx->p, (char*)buf, len); // copy data to pbuf
	tcp_client_send(pcbTx, esTx); //send it
	pbuf_free(esTx->p); //free up the pbuf
}

static struct termios tin;

static void terminal_set(void) {
    // save terminal configuration
    tcgetattr(STDIN_FILENO, &tin);

    static struct termios tlocal;
    memcpy(&tlocal, &tin, sizeof(tin));
    cfmakeraw(&tlocal);
    tcsetattr(STDIN_FILENO,TCSANOW,&tlocal);
}

static void terminal_reset(void) {
    // restore terminal upon exit
    tcsetattr(STDIN_FILENO,TCSANOW,&tin);
}

int main(int argc , char *argv[]) {
    int sock;
    struct sockaddr_in server;
    unsigned char buf[BUFLEN + 1];
    int len;
    int i;

//    if (argc < 2 || argc > 3) {
//        printf("Usage: %s address [port]\n", argv[0]);
//        return 1;
//    }
//    int port = 23;
//    if (argc == 3)
//        port = atoi(argv[2]);
//
//    //Create socket
//    sock = socket(AF_INET , SOCK_STREAM , 0);
//    if (sock == -1) {
//        perror("Could not create socket. Error");
//        return 1;
//    }
//
//    server.sin_addr.s_addr = inet_addr(argv[1]);
//    server.sin_family = AF_INET;
//    server.sin_port = htons(port);
//
//    //Connect to remote server
//    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0) {
//        perror("connect failed. Error");
//        return 1;
//    }
//    puts("Connected...\n");
//
//    // set terminal
//    terminal_set();
//    atexit(terminal_reset);
//
//    struct timeval ts;
//    ts.tv_sec = 1; // 1 second
//    ts.tv_usec = 0;

    while (1) {
        // select setup
        fd_set fds;
        FD_ZERO(&fds);
        if (sock != 0)
            FD_SET(sock, &fds);
        FD_SET(0, &fds);

        // wait for data
        int nready = select(sock + 1, &fds, (fd_set *) 0, (fd_set *) 0, &ts);
        if (nready < 0) {
            perror("select. Error");
            return 1;
        }
        else if (nready == 0) {
            ts.tv_sec = 1; // 1 second
            ts.tv_usec = 0;
        }
        else if (sock != 0 && FD_ISSET(sock, &fds)) {
            // start by reading a single byte
            int rv;
            if ((rv = recv(sock , buf , 1 , 0)) < 0)
                return 1;
            else if (rv == 0) {
                printf("Connection closed by the remote end\n\r");
                return 0;
            }

            if (buf[0] == CMD) {
                // read 2 more bytes
                len = recv(sock , buf + 1 , 2 , 0);
                if (len  < 0)
                    return 1;
                else if (len == 0) {
                    printf("Connection closed by the remote end\n\r");
                    return 0;
                }
                negotiate(sock, buf, 3);
            }
            else {
                len = 1;
                buf[len] = '\0';
                printf("%s", buf);
                fflush(0);
            }
        }

        else if (FD_ISSET(0, &fds)) {
            buf[0] = getc(stdin); //fgets(buf, 1, stdin);
            if (send(sock, buf, 1, 0) < 0)
                return 1;
            if (buf[0] == '\n') // with the terminal in raw mode we need to force a LF
                putchar('\r');
        }
    }
    close(sock);
    return 0;
}
