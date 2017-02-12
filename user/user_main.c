#include <string.h>
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "user_interface.h"
#include "espconn.h"
#include "driver/uart.h"
#include <mem.h>

#include "APA102.h"

#define TASK_PRIORITY	(0)
#define TASK_MSG_REFRESH	(0)
#define TASK_QUEUE_SIZE	(10)

#define AP_CHANNEL	6
#define AP_MAX_CONNECTIONS	4
#define AP_SSID	"LightNode"
#define AP_PSK	"l1ghtn0de"

#define STATION_SSID	""
#define STATION_PSK		""

#define AP_GATEWAY	"192.168.1.1"
#define AP_NETMASK	"255.255.255.0"

#define DHCP_IP_START	"192.168.1.10"
#define DHCP_IP_END		"192.168.1.15"

#define LISTEN_PORT		(54923)
#define SEND_PORT			(54924)

#define ALIVE_TIMER_RATE	(1000)
#define LED_COUNT	(71)


static void ap_init();
static void station_init();
static void wifi_handler(System_Event_t *event);
static void task_handler(os_event_t *event);

static void __recvHandler(void *arg, char *data, unsigned short len);
static void __sendHandler(void *arg);
static void aliveTimerHandler(void *arg);

static os_event_t _taskQueue[TASK_QUEUE_SIZE];
static struct espconn _udpConn;
static APA102_Strip *_strip;

//Protocol stuff
enum {
	DISCONNECTED = 0,
	CONNECTED
} _state;
os_timer_t _aliveTimer;
struct espconn _sendConn;

#define TYPE_PING		0
#define TYPE_INIT		1
#define TYPE_INFO		2
#define TYPE_UPDATE	3
#define TYPE_ALIVE	4
#define TYPE_ACK		5
#define TYPE_NACK		6

//Type of node (digital strip)
#define LIGHT_NODE_TYPE		1


void task_handler(os_event_t *event) {
	switch(event->sig) {
		case TASK_MSG_REFRESH:
			APA102_display(_strip);
		break;

		default:
			//?
		break;
	}
}

void aliveTimerHandler(void *arg) {
	uint8 dgram[3] = {0xAA, 0x55, TYPE_ALIVE};

	sint8 err = espconn_sent(&_sendConn, dgram, sizeof(dgram));

	if(err != 0) {
		char msg[128];
		os_sprintf(msg, "[aliveTimerHandler] Error sending alive message: %d\r\n", (int)err);
		uart_debugSend(msg);
	}
	else {
		uart_debugSend("[aliveTimerHandler] Sent alive message\r\n");
	}
}

void __recvHandler(void *arg, char *data, unsigned short len) {
	if(len < 3 || data[0] != 0xAA || data[1] != 0x55) {
		uart_debugSend("[__recvHandler] Received invalid datagram\r\n");

		return;
	}

	char msg[128];
	os_sprintf(msg, "[__recvHandler] Received message of type %d\r\n", data[2]);
	uart_debugSend(msg);
	
	struct espconn *pConn = (struct espconn*)arg;

	static uint8 replyData[6] = {0xAA, 0x55};
	uint8 replyLen = 2;
	
	switch(data[2]) {
		case TYPE_PING:
			//Reply with ACK
			replyData[2] = TYPE_ACK;
			replyLen = 3;
		break;

		case TYPE_INIT:
			//If connected, reply NACK
			//Else, connect to sender, reply with INFO
			if(_state == CONNECTED) {
				replyData[2] = TYPE_NACK;
				replyLen = 3;

				uart_debugSend("[__recvHandler] Info received, already connected\r\n");
			}
			else {
				_state = CONNECTED;
				
				os_memcpy(_sendConn.proto.udp->remote_ip, pConn->proto.udp->remote_ip, 4);
				_sendConn.proto.udp->remote_port = SEND_PORT;

				espconn_create(&_sendConn);

				//Start alive timer
				os_timer_arm(&_aliveTimer, ALIVE_TIMER_RATE, 1);

				replyData[2] = TYPE_INFO;
				replyData[3] = LIGHT_NODE_TYPE;
				replyData[4] = LED_COUNT >> 8;
				replyData[5] = LED_COUNT & 0xFF;
				replyLen = 6;

				uart_debugSend("[__recvHandler] Info received, now connected\r\n");
			}
		break;

		case TYPE_UPDATE:
			//Contains LED update data
			if(len != (3*LED_COUNT + 3)) {
				replyData[2] = TYPE_NACK;
				replyLen = 3;
			}
			else {
				uint32_t i;
				for(i = 0; i < LED_COUNT; ++i) {
					uint32_t index = 3*i + 3;
					APA102_setColor(_strip, i, data[index], data[index+1], data[index+2]);
				}

				system_os_post(TASK_PRIORITY, TASK_MSG_REFRESH, 0);
			}
		break;

		case TYPE_ALIVE:
			//Feed "watchdog"
		break;

		default:
			//Do nothing (eventually, send NACK)
		break;
	}

	if(replyLen > 2) {
		//Send datagram reponse
		sint8 err = espconn_sent(pConn, replyData, replyLen);

		if(err != 0) {
			char msg[128];
			os_sprintf(msg, "[__recvHandler] Error sending: %d\r\n", (int)err);
			uart_debugSend(msg);
		}
		else {
			uart_debugSend("[__recvHandler] Sent response\r\n");
		}
	}
}

void __sendHandler(void *arg) {
	uart_debugSend("[__sendHandler]\r\n");
}

void ap_init() {
	//Set to SoftAP mode
	wifi_set_opmode(SOFTAP_MODE);

	//SoftAP configuration
	struct softap_config apConfig = {
		.channel = AP_CHANNEL,
		.authmode = AUTH_WPA2_PSK,
		.ssid_hidden = 0,
		.max_connection = AP_MAX_CONNECTIONS,
		.beacon_interval = 100
	};
	strcpy(apConfig.ssid, AP_SSID);
	strcpy(apConfig.password, AP_PSK);
	apConfig.ssid_len = strlen(AP_SSID);

	//Set configuration
	wifi_softap_set_config(&apConfig);

	//Disable power saving mode
	//This seems to help reduce jitter in latency
	//wifi_set_sleep_type(NONE_SLEEP_T);

	//DHCP configuration
	struct dhcps_lease dhcpLease = {
		.start_ip = ipaddr_addr(DHCP_IP_START),
		.end_ip = ipaddr_addr(DHCP_IP_END)
	};

	//Disable DHCP server while making changes
	wifi_softap_dhcps_stop();
	wifi_softap_set_dhcps_lease(&dhcpLease);

	//Set IP info
	struct ip_info ipInfo = {
		.ip = ipaddr_addr(AP_GATEWAY),
		.gw = ipaddr_addr(AP_GATEWAY),
		.netmask = ipaddr_addr(AP_NETMASK)
	};
	wifi_set_ip_info(SOFTAP_IF, &ipInfo);
	
	//Restart DHCP server
	wifi_softap_dhcps_start();

	//Set WiFi event handler
	wifi_set_event_handler_cb(&wifi_handler);
}

void station_init() {
	//Set to station mode
	wifi_set_opmode(STATION_MODE);

	//Station configuration
	struct station_config sConfig;
	sConfig.bssid_set = 0;
	os_memcpy(&sConfig.ssid, STATION_SSID, strlen(STATION_SSID) + 1);
	os_memcpy(&sConfig.password, STATION_PSK, strlen(STATION_PSK) + 1);
	wifi_station_set_config(&sConfig);
}



void wifi_handler(System_Event_t *event) {
	switch(event->event) {
		case EVENT_SOFTAPMODE_STACONNECTED:
			//TODO: Perhaps do something with this information
		break;

		case EVENT_SOFTAPMODE_STADISCONNECTED:
			//TODO: Maybe do something here?
		break;

		default:
			break;
	}
}

//Init function 
void ICACHE_FLASH_ATTR
user_init()
{
		uart_init(115200, 115200);
		
		APA102_init();

		_strip = APA102_alloc(LED_COUNT);
		APA102_display(_strip);

    //Remove debug statements from UART0
    system_set_os_print(0);
		
		// Initialize the GPIO subsystem.
    //gpio_init();

    //Set GPIO2 to output mode
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);

    //Set GPIO2 low
    //gpio_output_set(0, BIT0, BIT0, 0);

		//Initialize WiFi
		//station_init();
		ap_init();

		//Initialize UDP socket
		_udpConn.type = ESPCONN_UDP;
		_udpConn.state = ESPCONN_NONE;
		_udpConn.proto.udp = (esp_udp*)os_malloc(sizeof(esp_udp));
		_udpConn.proto.udp->local_port = LISTEN_PORT;
		espconn_create(&_udpConn);
		espconn_regist_recvcb(&_udpConn, __recvHandler);
		espconn_regist_sentcb(&_udpConn, __sendHandler);

		_sendConn.type = ESPCONN_UDP;
		_udpConn.state = ESPCONN_NONE;
		_udpConn.proto.udp = (esp_udp*)os_malloc(sizeof(esp_udp));

		//Start periodic light update timer
		os_timer_disarm(&_aliveTimer);
		os_timer_setfn(&_aliveTimer, (os_timer_func_t*)aliveTimerHandler, NULL);

    //Start os task
    system_os_task(task_handler, TASK_PRIORITY, _taskQueue, TASK_QUEUE_SIZE);
}
