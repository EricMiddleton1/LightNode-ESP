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

#define AP_GATEWAY	"192.168.1.1"
#define AP_NETMASK	"255.255.255.0"

#define DHCP_IP_START	"192.168.1.10"
#define DHCP_IP_END		"192.168.1.15"

#define LISTEN_PORT		(54923)
#define SEND_PORT			(54924)

#define ALIVE_TIMER_RATE	(1000)
#define WATCHDOG_TIMER_TIMEOUT	(2000)
#define CONNECT_TIMER_TIMEOUT	(20000)
#define WIDTH	7
#define HEIGHT	7
#define LED_COUNT	9
#define NAME		"LightNode-Nameplate"


static void ap_init(char *ssid, char *psk);
static void station_connect(char *ssid, char *psk);
static void wifi_handler(System_Event_t *event);
static void task_handler(os_event_t *event);

static int __addrIsEqual(struct espconn*, struct espconn*);

static void __recvHandler(void *arg, char *data, unsigned short len);
static void __sendHandler(void *arg);
static void aliveTimerHandler(void *arg);
static void watchdogTimerHandler(void *arg);
static void connectTimerHandler(void *arg);
static void __feedWatchdog();

static os_event_t _taskQueue[TASK_QUEUE_SIZE];
static struct espconn _udpConn;
static APA102_Strip *_strip;

//Protocol stuff
enum {
	DISCONNECTED = 0,
	CONNECTED
} _state;
os_timer_t _aliveTimer, _watchdogTimer, _connectTimer;
struct espconn _clientConn;
int connecting;

#define TYPE_PING		0x00
#define TYPE_INIT		0x01
#define TYPE_INFO		0x02
#define TYPE_UPDATE	0x03
#define TYPE_ALIVE	0x04

#define TYPE_WIFI_CONNECT	0x14
#define TYPE_WIFI_START_AP	0x15

#define TYPE_ACK		0xFE
#define TYPE_NACK		0xFF

//Type of node (Digital Strip)
//#define LIGHT_NODE_TYPE		1
#define LIGHT_NODE_TYPE	2

int __addrIsEqual(struct espconn *conn1, struct espconn *conn2) {
	return (conn1->proto.udp->remote_ip[3] == conn2->proto.udp->remote_ip[3]) &&
		(conn1->proto.udp->remote_ip[2] == conn2->proto.udp->remote_ip[2]) &&
		(conn1->proto.udp->remote_ip[1] == conn2->proto.udp->remote_ip[1]) &&
		(conn1->proto.udp->remote_ip[0] == conn2->proto.udp->remote_ip[0]);
}

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

void __feedWatchdog() {
	os_timer_disarm(&_watchdogTimer);
	os_timer_arm(&_watchdogTimer, WATCHDOG_TIMER_TIMEOUT, 0);
}

void aliveTimerHandler(void *arg) {
	uint8 dgram[3] = {0xAA, 0x55, TYPE_ALIVE};

	sint8 err = espconn_sent(&_clientConn, dgram, sizeof(dgram));

	if(err != 0) {
		char msg[128];
		os_sprintf(msg, "[aliveTimerHandler] Error sending alive message: %d\r\n", (int)err);
		uart_debugSend(msg);
	}
	else {
		uart_debugSend("[aliveTimerHandler] Sent alive message\r\n");
	}
}

void watchdogTimerHandler(void *arg) {
	//Watchdog not fed in time, disconnect
	_state = DISCONNECTED;
	espconn_delete(&_clientConn);
	os_timer_disarm(&_aliveTimer);

	uart_debugSend("[watchdogTimerHandler] Client timed out\r\n");
}

void connectTimerHandler(void *arg) {
	int opmode = wifi_get_opmode();
	
	if(opmode != SOFTAP_MODE) {
		uart_debugSend("[connectTimerHandler] ");

		switch(wifi_station_get_connect_status()) {
			case STATION_IDLE:
				uart_debugSend("Station idle\r\n");
			break;

			case STATION_CONNECTING:
				uart_debugSend("Station connecting\r\n");
			break;

			case STATION_WRONG_PASSWORD:
				uart_debugSend("Station wrong password\r\n");
			break;

			case STATION_NO_AP_FOUND:
				uart_debugSend("Station no AP found\r\n");
			break;

			case STATION_CONNECT_FAIL:
				uart_debugSend("Station connect fail\r\n");
			break;

			case STATION_GOT_IP:
				uart_debugSend("Station got IP\r\n");
			break;

			default:
				uart_debugSend("Unknown station state\r\n");
		}

		ap_init(NULL, NULL);
	}

	connecting = 0;
}

void __recvHandler(void *arg, char *data, unsigned short len) {
	if(len < 3 || data[0] != 0xAA || data[1] != 0x55) {
		uart_debugSend("[__recvHandler] Received invalid datagram\r\n");

		return;
	}

	int inBand = 0;

	struct espconn *pConn = (struct espconn*)arg;
	remot_info *pRemote = NULL;
	if(espconn_get_connection_info(pConn, &pRemote, 0) == ESPCONN_OK) {
		pConn->proto.udp->remote_ip[0] = pRemote->remote_ip[0];
		pConn->proto.udp->remote_ip[1] = pRemote->remote_ip[1];
		pConn->proto.udp->remote_ip[2] = pRemote->remote_ip[2];
		pConn->proto.udp->remote_ip[3] = pRemote->remote_ip[3];
		pConn->proto.udp->remote_port = SEND_PORT;

		if(_state == CONNECTED && __addrIsEqual(pConn, &_clientConn))
			inBand = 1;
	}

	if(inBand && data[2] != TYPE_PING) {
		__feedWatchdog();
	}

	uint8 replyData[32] = {0xAA, 0x55};
	uint8 replyLen = 2;
	
	switch(data[2]) {
		case TYPE_PING:
			if(_state != CONNECTED) {
				//Reply with INFO
				replyData[2] = TYPE_INFO;
				replyData[3] = 0;	//# Analog strips
				replyData[4] = 1; //# Digital strips
				replyData[5] = 0; //# Matricies
				replyData[6] = LED_COUNT >> 8; //Led count
				replyData[7] = LED_COUNT & 0xFF;
				memcpy(replyData+8, NAME, strlen(NAME)); //Name
				replyLen = 8 + strlen(NAME);
			}
		break;

		case TYPE_INIT:
			//If connected, reply NACK
			//Else, connect to sender, reply with INFO
			if(_state == CONNECTED) {
				replyData[2] = TYPE_NACK;
				replyData[3] = data[2];
				replyLen = 4;

				uart_debugSend("[__recvHandler] Init received, already connected\r\n");
			}
			else {
				_state = CONNECTED;
				
				os_memcpy(_clientConn.proto.udp->remote_ip, pConn->proto.udp->remote_ip, 4);
				_clientConn.proto.udp->remote_port = SEND_PORT;

				espconn_create(&_clientConn);

				//Start alive timer
				os_timer_arm(&_aliveTimer, ALIVE_TIMER_RATE, 1);

				//Start watchdog timer
				os_timer_arm(&_watchdogTimer, WATCHDOG_TIMER_TIMEOUT, 0);

				replyData[2] = TYPE_ACK;
				replyData[3] = data[2];
				replyLen = 4;

				uart_debugSend("[__recvHandler] Init received, now connected\r\n");
			}
		break;

		case TYPE_UPDATE:
			if(!inBand || (len != (3*(LED_COUNT) + 3))) {
				replyData[2] = TYPE_NACK;
				replyData[3] = data[2];
				replyLen = 4;
				
				if(!inBand) {
					uart_debugSend("[__recvHandler] Update received from out of band packet\r\n");
				}
				else {
					uart_debugSend("[__recvHandler] Update received with wrong payload size\r\n");
				}
			}
			else {
				uint32_t i;
				for(i = 0; i < (LED_COUNT); ++i) {
					uint32_t index = 3*i + 3;
					APA102_setColor(_strip, i, data[index], data[index+1], data[index+2]);
				}

				system_os_post(TASK_PRIORITY, TASK_MSG_REFRESH, 0);
			}
		break;

		case TYPE_ALIVE:
			//Feed "watchdog"
		break;

		case TYPE_WIFI_CONNECT: {
			char *ssid = NULL, *psk = NULL;
			int ssidLen = 0;
			
			ssid = data + 3;
			ssidLen = os_strlen(ssid);
			psk = ssid + ssidLen + 1;

			uart_debugSend("[__recvHandler] WiFi Connect to station: ");
			uart_debugSend(ssid);
			uart_debugSend(" with psk ");
			uart_debugSend(psk);
			uart_debugSend("\r\n");

			station_connect(ssid, psk);
		}
		break;

		case TYPE_WIFI_START_AP: {
			char *ssid = NULL, *psk = NULL;
			int ssidLen = 0;
			
			ssid = data + 3;
			ssidLen = os_strlen(ssid);
			psk = ssid + ssidLen + 1;

			uart_debugSend("[__recvHandler] WiFi start AP: ");
			uart_debugSend(ssid);
			uart_debugSend(" with psk ");
			uart_debugSend(psk);
			uart_debugSend("\r\n");

			ap_init(ssid, psk);
		}
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
	}
}

void __sendHandler(void *arg) {
	//uart_debugSend("[__sendHandler]\r\n");
}

void ap_init(char *ssid, char *psk) {
	//Set to SoftAP mode
	wifi_set_opmode(SOFTAP_MODE);

	if(ssid != NULL && psk != NULL) {
		//SoftAP configuration
		struct softap_config apConfig = {
			.channel = AP_CHANNEL,
			.authmode = AUTH_WPA2_PSK,
			.ssid_hidden = 0,
			.max_connection = AP_MAX_CONNECTIONS,
			.beacon_interval = 100
		};
		strcpy(apConfig.ssid, ssid);
		strcpy(apConfig.password, psk);
		apConfig.ssid_len = strlen(ssid);

		//Set configuration
		wifi_softap_set_config(&apConfig);
	}

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

}

void station_connect(char *ssid, char *psk) {

	if(wifi_get_opmode() != STATION_MODE) {
		//Set to station mode
		wifi_set_opmode(STATION_MODE);
	}

	if(wifi_station_get_connect_status() == STATION_GOT_IP) {
		wifi_station_disconnect();
	}

	//Station configuration
	struct station_config sConfig;
	sConfig.bssid_set = 0;
	os_memcpy(&sConfig.ssid, ssid, strlen(ssid) + 1);
	os_memcpy(&sConfig.password, psk, strlen(psk) + 1);
	wifi_station_set_config(&sConfig);

	wifi_station_connect();

	os_timer_arm(&_connectTimer, CONNECT_TIMER_TIMEOUT, 0);

	connecting = 1;
}



void wifi_handler(System_Event_t *event) {
	switch(event->event) {
		case EVENT_SOFTAPMODE_STACONNECTED:
			//TODO: Perhaps do something with this information
		break;

		case EVENT_SOFTAPMODE_STADISCONNECTED:
			//TODO: Maybe do something here?
		break;

		case EVENT_STAMODE_GOT_IP:
			uart_debugSend("[wifi_handler] Station mode received IP\r\n");
			//Cancel connect timer
			os_timer_disarm(&_connectTimer);
			connecting = 0;
		break;

		case EVENT_STAMODE_DISCONNECTED:
			uart_debugSend("[wifi_handler] Station mode disconnected\r\n");
			//Start connect timer
			if(!connecting) {
				station_connect(NULL, NULL);
			}
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
		
		//Remove debug statements from UART0
    system_set_os_print(0);
		
		APA102_init();
		_strip = APA102_alloc(LED_COUNT);
		APA102_display(_strip);
		
		//Initialize UDP socket
		_udpConn.type = ESPCONN_UDP;
		_udpConn.state = ESPCONN_NONE;
		_udpConn.proto.udp = (esp_udp*)os_malloc(sizeof(esp_udp));
		_udpConn.proto.udp->local_port = LISTEN_PORT;
		espconn_create(&_udpConn);
		espconn_regist_recvcb(&_udpConn, __recvHandler);
		espconn_regist_sentcb(&_udpConn, __sendHandler);

		_clientConn.type = ESPCONN_UDP;
		_clientConn.state = ESPCONN_NONE;
		_clientConn.proto.udp = (esp_udp*)os_malloc(sizeof(esp_udp));

		//Configure alive timer
		os_timer_disarm(&_aliveTimer);
		os_timer_setfn(&_aliveTimer, (os_timer_func_t*)aliveTimerHandler, NULL);

		//Configure watchdog timer
		os_timer_disarm(&_watchdogTimer);
		os_timer_setfn(&_watchdogTimer, (os_timer_func_t*)watchdogTimerHandler, NULL);

		//Configure connect timer
		os_timer_disarm(&_connectTimer);
		os_timer_setfn(&_connectTimer, (os_timer_func_t*)connectTimerHandler, NULL);

		int opmode = wifi_get_opmode();
		if(opmode == STATION_MODE) {
			//Start connect timer
			os_timer_arm(&_connectTimer, CONNECT_TIMER_TIMEOUT, 0);
			connecting = 1;
		}
		else if(opmode != SOFTAP_MODE) {
			//Start AP mode
			ap_init(NULL, NULL);
			connecting = 0;
		}

		//Set WiFi event handler
		wifi_set_event_handler_cb(&wifi_handler);

    //Start os task
    system_os_task(task_handler, TASK_PRIORITY, _taskQueue, TASK_QUEUE_SIZE);
}
