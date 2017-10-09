#include <stdio.h>
#include <rtt_stdio.h>
#include "shell.h"
#include "thread.h"
#include "xtimer.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"

#include <at30ts74.h>
#include <mma7660.h>
#include <periph/gpio.h>

// 1 second, defined in us
#define INTERVAL (1000000U)
#define NETWORK_RTT_US 1000000
#define SAMPLEN 15
#define OPENTHOLD 6
#define CLOSETHOLD 3

extern int _netif_config(int argc, char **argv);
extern void send_udp(char *addr_str, uint16_t port_str, char *data, uint16_t datalen);
extern void start_server(char *port_str);
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
extern void handle_input_line(const shell_command_t *command_list, char *line);

// listen for UDP packets
void server(void)
{
  printf("Starting the server\n");
  msg_t msg;
  msg_t msg_queue[16];
  msg_init_queue(msg_queue, 16);
  xtimer_ticks32_t led_sleep;

/*
  gnrc_netreg_entry_t entry = { NULL, 4747, thread_getpid() };
  if (gnrc_netreg_register(GNRC_NETTYPE_UDP, &entry)) {
    printf("ERROR listening\n");
  }
*/
  printf("start listening for packets\n");
  while(1) // server loop
  {
    //xtimer_msg_receive_timeout(&msg, NETWORK_RTT_US);
    msg_receive(&msg);

    switch(msg.type)
    {
        case GNRC_NETAPI_MSG_TYPE_RCV:
        {
          printf("Received a packet \n");
          gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)(msg.content.ptr);
          gnrc_pktsnip_t *tmp;
          LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_UDP);
          //udp_hdr_t *udp = (udp_hdr_t *)tmp->data;
          LL_SEARCH_SCALAR(pkt, tmp, type, GNRC_NETTYPE_IPV6);
          //ipv6_hdr_t *ip = (ipv6_hdr_t *)tmp->data;
          //
          char *bytes = (char *)pkt->data;
          //z = (int8_t)bytes[4];
	  
	  if (bytes[2] == 0x11) {
	    printf("door open\n");
	  }
	  else if (bytes[2] == 0x00) {
 	    printf("door closed\n");
	  }
          
          gnrc_pktbuf_release((gnrc_pktsnip_t *) msg.content.ptr);

          xtimer_periodic_wakeup(&led_sleep, 250000);
          break;
        }
        default:
        {
          printf("Expected %d but got %d\n", GNRC_NETAPI_MSG_TYPE_RCV, msg.type);
        }
    }

  }
}

int* sample(mma7660_t *acc, xtimer_ticks32_t *last_wakeup, int *ret) {
	    int *xVals = malloc(sizeof(int)*SAMPLEN);
	    int *yVals = malloc(sizeof(int)*SAMPLEN);
	    int *zVals = malloc(sizeof(int)*SAMPLEN);
	    int8_t x,y,z;
	    for (int i = 0; i < SAMPLEN; i++) {
	        mma7660_read(acc, &x, &y, &z);
		xVals[i] = x + y + z;
		yVals[i] = y;
		zVals[i] = z;
		xtimer_periodic_wakeup(last_wakeup, 10000);
	    }
	    int xSum = 0;
	    int xSum1 = 0;
	    int ySum = 0;
	    int zSum = 0;
	    int ySum1 = 0;
	    int zSum1 = 0;
	    // calculate sums
	    for (int i = 0; i < SAMPLEN - 1; i++) {
	        xSum += xVals[i] * xVals[i+1];
		xSum1 += xVals[i];
		ySum += yVals[i] * yVals[i+1];
		ySum1 += yVals[i];
		zSum += zVals[i] * zVals[i+1];
		zSum1 += zVals[i];
	    }
	    // calculate auto correlations
	    int xCor = xSum/SAMPLEN - 
			pow(xSum1, 2)/pow(SAMPLEN, 2);
	    int yCor = ySum/SAMPLEN - 
			pow(ySum1, 2)/pow(SAMPLEN, 2);
	    int zCor = zSum/SAMPLEN - 
			pow(zSum1, 2)/pow(SAMPLEN, 2);
            free(xVals);
	    free(yVals);
	    free(zVals);
	    ret[0] = xCor;
	    ret[1] = yCor;
	    ret[2] = zCor;
	    return ret;
}

void set(int *x, int *y, int *z, int xVal, int yVal, int zVal)
{
    *x = xVal;
    *y = yVal;
    *z = zVal;
} 

void client(void)
{
    // initialize accelerometer
    mma7660_t acc;
    if (mma7660_init(&acc, I2C_0, MMA7660_ADDR) != 0) {
      printf("Failed to init ACC\n");
    } else {
      printf("Init acc ok\n");
    }
    if (mma7660_set_mode(&acc, 1, 0, 0, 0) != 0) {
      printf("Failed to set mode\n");
    } else {
      printf("Set mode ok\n");
    }
    if (mma7660_config_samplerate(&acc, MMA7660_SR_AM64, MMA7660_SR_AW32, 1) != 0) {
      printf("Failed to config SR\n");
    }
   
    xtimer_ticks32_t last_wakeup;
    int xcutOff = -1;
    int ycutOff = -1;
    int zcutOff = -1;
    int8_t x,y,z;
    last_wakeup = xtimer_now();
    union value2 {int32_t number; uint8_t bytes[2];} message_num;
    memset(&message_num, 0, sizeof(message_num));
    message_num.number = 0;
   /*
    xtimer_periodic_wakeup(&last_wakeup, INTERVAL*30);
   
    LED_ON; 
    // calculate cutoff
    int i;
    for (i = 0; i < 20; i++) {
	int *corr = malloc(sizeof(int)*3);
	sample(&acc, &last_wakeup, corr);
	xcutOff += corr[0];
	ycutOff += corr[1];
	zcutOff += corr[2];
	free(corr);
    }
    
    xcutOff = xcutOff/20;
    ycutOff = ycutOff/20;
    zcutOff = zcutOff/20;
    printf("%d", xcutOff);
    printf("%d", ycutOff);
    printf("%d", zcutOff);
    LED_OFF;
    */
    int open_count = 0;
    int close_count = 0;
    char buf[7];
    uint32_t time = xtimer_usec_from_ticks(xtimer_now());
    while(1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL/4);
	uint32_t new_time = xtimer_usec_from_ticks(xtimer_now());
	if ((new_time - time) > INTERVAL*10)
	{
	    buf[2] = 0x22;
	    send_udp("ff02::1", 4747, buf, 7);
	    time = xtimer_usec_from_ticks(xtimer_now()); 
	}

        // increment message number
        message_num.number++;
        // pack it
        buf[0] = 0x10;
        buf[1] = 0x00;

        // read accelerometer
        if (mma7660_read(&acc, &x, &y, &z) != 0)
        {
            printf("Could not read accel\n");
        }
	else {
	    int * corr = malloc(sizeof(int)*3);
	    sample(&acc, &last_wakeup, corr);
	    int xCor = corr[0];
	    int yCor = corr[1];
	    int zCor = corr[2];
	    free(corr);
	    printf("xCor: %d\n", corr[0]);
	    printf("yCor: %d\n", corr[1]);
	    printf("zCor: %d\n", corr[2]);
	    
	    // initialize accelerometer cutoff values 
	    if (xcutOff == -1) {
	        set(&xcutOff, &ycutOff, &zcutOff, xCor, yCor, zCor);
	    }
            // correlation is above threshold
	    else if (abs(xCor-xcutOff) > OPENTHOLD || abs(yCor-ycutOff) > OPENTHOLD || abs(zCor-zcutOff) > OPENTHOLD) {
		printf("door opening\n");
		open_count++;
		buf[2] = 0x11;
		buf[3] = open_count & 0xff;
		buf[4] = open_count >> 8;
		buf[5] = close_count & 0xff;
		buf[6] = close_count >> 8;
                LED_ON;
                xtimer_periodic_wakeup(&last_wakeup, 20000);
                send_udp("ff02::1", 4747, buf, 7);
	  	printf("sending msg! num %" PRId32 "\n", message_num.number);
		LED_OFF;

		// sleep while door is open for 3 seconds
		xtimer_periodic_wakeup(&last_wakeup, 3*INTERVAL);
		
		// detect door as closed
		xcutOff = -1;
		int counter = 0;

		while (true) {
		    LED_ON;		
		    int * samp = malloc(sizeof(int)*3);
		    sample(&acc, &last_wakeup, samp);
		    set(&xCor, &yCor, &zCor, samp[0], samp[1], samp[2]);
		    printf("closedx: %d\n", xCor);
	            printf("closedy: %d\n", yCor);
	            printf("closedz: %d\n", zCor);
		    free(samp);

		    if (xcutOff == -1) {
			set(&xcutOff, &ycutOff, &zcutOff, xCor, yCor, zCor);
		    } else if (abs(xCor-xcutOff) < CLOSETHOLD && abs(yCor-ycutOff) < CLOSETHOLD && abs(zCor-zcutOff) < CLOSETHOLD) {
			if (counter == 1) {
			    LED_OFF;
			    break;
			}
			set(&xcutOff, &ycutOff, &zcutOff, xCor, yCor, zCor);
			counter++;
		    } else {
			set(&xcutOff, &ycutOff, &zcutOff, xCor, yCor, zCor);
			counter = 0;
		    }
		    LED_OFF;
		    xtimer_periodic_wakeup(&last_wakeup, 500000);
		}
	
		close_count++;	
		buf[2] = 0x00;
		printf("door closed\n");
		buf[5] = close_count & 0xff;
		buf[6] = close_count >> 8;

		send_udp("ff02::1", 4747, buf, 7);
 		// reset cutoff values		
		set(&xcutOff, &ycutOff, &zcutOff, -1, -1, -1);
	    }
	    // reset cutoff values
	    else {
		set(&xcutOff, &ycutOff, &zcutOff, xCor, yCor, zCor);
            }          
	}
    }
}

		

int main(void)
{
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    server();
    //client();
    return 0;
}
