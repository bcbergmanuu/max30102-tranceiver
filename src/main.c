

#include <zephyr.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
#include "max30102.h"

#define LOG_MODULE_NAME bt_hr_sensor
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)


#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static K_SEM_DEFINE(ble_init_ok, 0, 1);

K_FIFO_DEFINE(ppg_items);
K_SEM_DEFINE(sem_initialized_max30102, 0, 1);
K_SEM_DEFINE(sem_interruptmax30102, 0, 1);

const struct device *gpio0Port;	
struct gpio_callback interrupt_max30102;

static struct bt_conn *current_conn;
//static struct bt_conn *auth_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct gpio_dt_spec signal = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, signal_gpios);
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);
	sys_reboot();
}



BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,

};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	(void)conn;
	LOG_INF("Received data ");
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

typedef struct {
    void *fifo_reserved;   /* 1st word reserved for use by FIFO */
    ppg_item_t ppgitem;
} fifo_item_t;

void max30102isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {	
	k_sem_give(&sem_interruptmax30102);		
}

void attachinterrupt(void) {
	
	__ASSERT(device_is_ready(signal.port), "custom device not ready");
	gpio_pin_configure_dt(&signal, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&signal, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&interrupt_max30102, max30102isr, BIT(signal.pin));
	gpio_add_callback(signal.port, &interrupt_max30102);	
}


void main(void)
{	
	//const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	//uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return;
	}

	/* Poll if the DTR flag was set */
	// while (!dtr) {
	// 	uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
	// 	/* Give CPU resources to low priority threads. */
	// 	k_sleep(K_MSEC(100));
	// }


	int err = 0;	

	err = bt_enable(NULL);	

	LOG_INF("Bluetooth initialized");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}


	LOG_INF("interrupt-attached");

	if(max30102_init()) {
		LOG_ERR("could not initialize");
		k_sleep(K_FOREVER);		
	}	
	attachinterrupt();
	uint8_t max_read_state = max30102_interrupt_state(); //clear interrupt;

	LOG_INF("max initialized, read state:%i", max_read_state);
	
	
	k_sem_give(&sem_initialized_max30102);

	k_sem_give(&ble_init_ok);
	LOG_INF("bt-enabled");

	for (;;) {		
		k_sleep(K_MSEC(2000));
		LOG_INF("alive");
	}
}

void nus_fifo_service(void)
{	
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	char buffer[sizeof(ppg_item_t)];
	
	for (;;) {
		
		fifo_item_t *package;
		package = k_fifo_get(&ppg_items, K_FOREVER);
		ppg_item_t ppgitem = package->ppgitem;	
		memcpy(buffer, (uint8_t*)&ppgitem, sizeof(ppg_item_t));
		if (bt_nus_send(NULL, buffer, sizeof(ppg_item_t))) {
			LOG_WRN("Failed to send data over BLE connection");
		} else {			
			LOG_INF("send ppg ir=%d, red=%d", ppgitem.ir, ppgitem.red);
		}
		k_free(package);
	}
}

void max30102read(void) {	
	k_sem_take(&sem_initialized_max30102, K_FOREVER);
	
	uint8_t amount_samples;
	uint8_t isrstate;
	ppg_item_t buffer[32]; //maximum amount for ic
	
	while (1) {		
		k_sem_take(&sem_interruptmax30102, K_MSEC(5000));
		isrstate = max30102_interrupt_state();
		if (isrstate != 0b10000000) {
			LOG_WRN("unexpected max30102int: %d", isrstate);	
		} 
		amount_samples = GetSamplesAvailable();		
		
		max30102data(buffer, amount_samples);
		
		for (int i = 0; i < amount_samples; i++)
		{			
			fifo_item_t *toqueue = k_malloc(sizeof(fifo_item_t));
			toqueue->ppgitem = buffer[i];			
			k_fifo_put(&ppg_items, toqueue);
		} 			
	}
}

K_THREAD_DEFINE(max30102_read_thread_id, 1024, max30102read, NULL, NULL,
		NULL, 7, 0, 0);

K_THREAD_DEFINE(nus_fifo_service_id, STACKSIZE, nus_fifo_service, NULL, NULL,
		NULL, 8, 0, 0);
