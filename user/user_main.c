#include <string.h>
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "driver/uart.h"
#include "user_interface.h"
#include "espconn.h"
#include "user_tcp.h"
#include <mem.h>

#define TASK_QUEUE_SIZE	(10)

#define AP_CHANNEL	6
#define AP_MAX_CONNECTIONS	4
#define AP_SSID	"MicroCART"
#define AP_PSK	"m1cr0cart"

#define AP_GATEWAY	"192.168.1.1"
#define AP_NETMASK	"255.255.255.0"

#define DHCP_IP_START	"192.168.1.10"
#define DHCP_IP_END		"192.168.1.15"

static void wifi_handler(System_Event_t *event);
static void task_handler(os_event_t *event);

static os_event_t _taskQueue[TASK_QUEUE_SIZE];

void task_handler(os_event_t *event) {

}

void wifi_init() {
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
    //Remove debug statements from UART0
    system_set_os_print(0);
		
		// Initialize the GPIO subsystem.
    gpio_init();

    //Set GPIO2 to output mode
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);

    //Set GPIO2 low
    gpio_output_set(0, BIT0, BIT0, 0);

		//Initialize WiFi
		wifi_init();

    //Start os task
    system_os_task(uart_task, UART_TASK_PRIORITY,user_procTaskQueue,
			user_procTaskQueueLen);
}
