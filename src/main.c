#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>

#define LOG_MODULE_NAME app
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

static K_SEM_DEFINE(bt_init_ok, 1, 1);
static struct bt_conn *current_conn;


// Определяем значение UUID
#define BT_UUID_REMOTE_SERV_VAL \
  BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)
#define BT_UUID_REMOTE_MESSAGE_CHRC_VAL \
  BT_UUID_128_ENCODE(0xe9ea0003, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)
#define BT_UUID_SEND_STATUS_CHRC_VAL \
  BT_UUID_128_ENCODE(0xe9ea0005, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)


#define BT_UUID_REMOTE_SERVICE          BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_UUID_REMOTE_MESSAGE_CHRC     BT_UUID_DECLARE_128(BT_UUID_REMOTE_MESSAGE_CHRC_VAL)
#define BT_UUID_SEND_STATUS_CHRC       BT_UUID_DECLARE_128(BT_UUID_SEND_STATUS_CHRC_VAL)


static bool notify_mysensor_enabled;


struct bt_remote_service_cb {
  void (*data_received)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};

static const struct bt_data ad[] = {
  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
  BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

static const struct bt_data sd[] = {
  BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL),
};

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash_controller));

typedef struct inf
{
  uint8_t state;
} inf;
static inf my_info;

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static void mylbsbc_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value)
{
  notify_mysensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}

//Макрос
BT_GATT_SERVICE_DEFINE(remote_srv,
BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),
  BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_MESSAGE_CHRC,
    BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
    NULL, on_write, NULL),
  BT_GATT_CHARACTERISTIC(BT_UUID_SEND_STATUS_CHRC,
    BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_READ,
  NULL, NULL, NULL),
);

static struct bt_conn_cb bluetooth_callbacks;
static struct bt_remote_service_cb remote_callbacks;

static const uint32_t offset = 0xf8000;

// Чтение из flash памяти состояние светодиода 
void download_info()
{
  flash_read(flash_dev, offset, &my_info, sizeof(inf));
  gpio_pin_set_dt(&led0, my_info.state);
}

// Запись в flash память состояние светодиода 
void save_led_state(inf my_info)
{
  printk("LED state: %d\n", my_info.state);
  flash_erase(flash_dev, offset, 4096);
  flash_write(flash_dev, offset, &my_info, sizeof(inf));
  flash_read(flash_dev, offset, &my_info, sizeof(inf));
}

// Вызов функции запииси в flash память состояние светодиода
void write_info(int a)
{
  my_info.state = a;
  save_led_state(my_info);
}

// Получает и обрабатывает данные, которые были переданы 
static ssize_t on_write(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        const void *buf,
                        uint16_t len,
                        uint16_t offset,
                        uint8_t flags)
{
  LOG_INF("Received data as text: %.*s", len, (char *)buf);
  if (strncmp((char *)buf, "start", len) == 0) {
    // Включаем светодиод
    gpio_pin_set_dt(&led0, 1);
    write_info(1);

    // Уведомление об успешном выполнении


  } else if (strncmp((char *)buf, "stop", len) == 0) {

    // Выключаем светодиод
    gpio_pin_set_dt(&led0, 0);
    write_info(0);

  }    
  uint8_t status = BT_GATT_CCC_NOTIFY;
bt_gatt_notify(conn, &remote_srv.attrs[2], &status, sizeof(status));
  if (remote_callbacks.data_received) {
    remote_callbacks.data_received(conn, buf, len);
  }
  return len;
}
// Функция вызывается при подключении устройства 
void on_connected(struct bt_conn *conn, uint8_t err)
{
  if(err) {
    LOG_ERR("connection err: %d", err);
    return;
  }
  LOG_INF("Connected.");
  current_conn = bt_conn_ref(conn);
}

// Функция вызывается при отключении устройства 
void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
  LOG_INF("Disconnected (reason: %d)", reason);
  if(current_conn) {
    bt_conn_unref(current_conn);
    current_conn = NULL;
  }
}


//Проверка инициализации BLE
void bt_ready(int err)
{
  if (err) {
    LOG_ERR("bt_ready returned %d", err);
  }
  k_sem_give(&bt_init_ok);
}


// Инициализация BLE
int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_service_cb *remote_cb)
{
  int err;
  LOG_INF("Initializing bluetooth...");

  if (bt_cb == NULL || remote_cb == NULL) {
    return -NRFX_ERROR_NULL;
  }

  bt_conn_cb_register(bt_cb);
  remote_callbacks.data_received = remote_cb->data_received;

  err = bt_enable(bt_ready);
  if (err) {
    LOG_ERR("bt_enable returned %d", err);
    return err;
  }

  k_sem_take(&bt_init_ok, K_FOREVER);

  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Couldn't start advertising (err = %d)", err);
    return err;
  }
  return err;
}


// Проверка инициализации светодиода
void check_device_led()
{
  if (!device_is_ready(led0.port))
  {
    printk("GPIO device is not ready\r\n");
  }
}

// Конфиг светодиода
void config_led()
{
  gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
}

void main(void)
{
  int err;
  LOG_INF("Hello World! %s\n", CONFIG_BOARD);
  check_device_led();
  config_led();
  download_info();
  bluetooth_callbacks.connected = on_connected;
  bluetooth_callbacks.disconnected = on_disconnected;

  err = bluetooth_init(&bluetooth_callbacks, &remote_callbacks);
  if (err) {
    LOG_ERR("bt_enable returned %d", err);
  }

  LOG_INF("Running...");
  for (;;) {
    k_sleep(K_MSEC(1000));
  }
}
