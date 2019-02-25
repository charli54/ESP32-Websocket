#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/i2c.h"



#include "lwip/api.h"

#include "WebSocket_Task.h"
//#include "pageHTML.h"
//#include "ESP32_WS.h"
#include "Traxxas.h"

//------------WIFI-AP Declarations-------------------------------
#define SSID "esp32_seb"

//-------------INTERRUPT Declaration----------------------------------
#define GPIO_HALL_SENSOR 4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_HALL_SENSOR)
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;
volatile uint64_t previousTime = 0;
volatile uint64_t deltaTime = 0;
#define DISTANCE (10.3673 / 1000)
//#define DISTANCE (20.5774 / 1000)
#define ANGLE (3.14 / 5)
double speed;
double angularSpeed;
double distance = 0;

//------------------DEBUGGING------------------------------------
static const char *TAG = "Acces Point";

//--------------------WEB SERVER---------------------------------
const static char http_html_hdr[] =
    "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";

static EventGroupHandle_t wifi_event_group;
static tcpip_adapter_ip_info_t ipInfo;

//----------------------WEBSOCKET-------------------------------
//WebSocket frame receive queue
QueueHandle_t WebSocket_rx_queue;
QueueHandle_t WebSocket_connection_status;
QueueHandle_t StartTest;

//****************TIMER CONFIG CONSTANT******************
#define TIMER_DIVIDER 8
#define TIMER_SCALE   (TIMER_BASE_CLK / TIMER_DIVIDER) //80 000 000 / 16
#define TIMER_FINE_ADJ (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/10000000)
#define TIMER_INTERVAL_SEC (0.0250)
#define TEST_WITHOUT_RELOAD 0
#define TEST_WITH_RELOAD    1

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

//****************ADC CONFIG CONSTANT******************
#define V_REF   1133
#define ADC1_GPIO_34 (ADC1_GPIO34_CHANNEL)      //GPIO 34
#define ADC1_GPIO_35 (ADC1_GPIO35_CHANNEL)
//#define V_REF_TO_GPIO  //Remove comment on define to route v_ref to GPIO
#define ADC_RESOLUTION      4096
#define CONVERTION ((double) V_REF / ADC_RESOLUTION) * 7.736

//****************I2C CONFIG CONSTANT*****************************
#define I2C_SCL_IO          				19               //SCL
#define I2C_SDA_IO          				18 				//SDA
#define I2C_FREQ_HZ							100000			//100KHz
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 	0
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE	0

#define ADS1015_ADD                 		0x48             /*!< slave address for BH1750 sensor */
#define ADS1015_CHANNEL_0				   	0b11000010
#define ADS1015_CH_0_SINGLE					0b11000011
#define ADS1015_CHANNEL_1				   	0b01010000
#define ADS1015_CHANNEL_2				   	0b01100000
#define ADS1015_CHANNEL_3				   	0b01110000
#define ADS1015_DIFF_23						0b00110000
#define ADS1015_CHANNEL_3_FSR_4_096		    0b11110010
#define ADS1015_CH_3_SINGLE					0b11110011
#define ADS1015_CHANNEL_2_FSR_4_096			0b01100010
#define ADS1015_CHANNEL_01_FSR_4_096		0b00000010
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1 				/*!< I2C nack value */

int current;
double voltage;

int c;
int v;

SemaphoreHandle_t print_mux = NULL;

/*!< I2C nack value */
/*--------------------------------------------------------------
//---------------FUNCTIONS IMPELMENTATIONS----------------------
 *
 **************************************************************/
static esp_err_t event_handler(void *ctx, system_event_t *event){
	switch(event->event_id){
		case SYSTEM_EVENT_AP_START :
			ESP_LOGI(TAG, "SYSTEM_EVENT_AP_START");
			break;
		case SYSTEM_EVENT_AP_STACONNECTED:
			ESP_LOGI(TAG, "Client connected to ESP32");
			break;
		case SYSTEM_EVENT_ETH_GOT_IP:
			ESP_LOGI(TAG, "ESP32 got IP");
			break;
		case SYSTEM_EVENT_GOT_IP6 :
			ESP_LOGI(TAG, "ESP32 got IP IP6");
			break;
		default:
			break;
	}
	return ESP_OK;
}

//WIFI initialisation as Acces Point-------------------------------------
//-----------------------------------------------------------------------
static void wifiInitAP(void){

	tcpip_adapter_init();

	/*tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);

	tcpip_adapter_ip_info_t ipInfo;
	IP4_ADDR(&ipInfo.ip, 192,168,0,99);
	IP4_ADDR(&ipInfo.gw, 192,168,1,1);
	IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
	tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo);*/
	tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo);

	ESP_LOGI(TAG, "ETH_IP: "IPSTR, IP2STR(&ipInfo.ip));

	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifiInitConfig));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	wifi_config_t wifiConfig = {
		.ap = {
			.ssid = SSID,
			.channel = 0,
			.authmode = WIFI_AUTH_OPEN,
			.ssid_hidden = 0,
			.max_connection = 1,
			.beacon_interval = 100
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifiConfig));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void task_process_WebSocket( void *pvParameters ){
    (void)pvParameters;

    char *data = "Received !";
    //frame buffer
    WebSocket_frame_t __RX_frame;

    //create WebSocket RX Queue
    WebSocket_rx_queue = xQueueCreate(10,sizeof(WebSocket_frame_t));

    while (1){
        //receive next WebSocket frame from queue
        if(xQueueReceive(WebSocket_rx_queue,&__RX_frame, 3*portTICK_PERIOD_MS)==pdTRUE){

        	//write frame inforamtion to UART
        	//printf("New Websocket frame. Length %d, payload %.*s \r\n", __RX_frame.payload_length, __RX_frame.payload_length, __RX_frame.payload);

        	//loop back frame
        	//WS_write_data(__RX_frame.payload, __RX_frame.payload_length);

        	if(strstr(__RX_frame.payload, "start") != NULL){
        		//WS_write_data(data, strlen(data));
        		int start = 1;
        		xQueueSend(StartTest,&start, portMAX_DELAY);
        	}
        	else if(strstr(__RX_frame.payload, "stop") != NULL){
        		int start = 0;
        		xQueueSend(StartTest,&start, portMAX_DELAY);
        	}
        	//free memory
			if (__RX_frame.payload != NULL)
				free(__RX_frame.payload);

        }
    }
}

static void processRequest(struct netconn *conn){
	ESP_LOGI(TAG, "Incomming Request !");
	struct netbuf *inbuf;
	u16_t buflen;
	char *buf;
	err_t err;
	char request[32];

	err = netconn_recv(conn, &inbuf);
	if(err == ERR_OK){
		netbuf_data(inbuf,(void**)&buf, &buflen);

		netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
		//printf("%s\r\n", buf);
		if(sscanf(buf, "GET /%s %*s", request) == 1){
			if(strcmp(request, "index") == 0){
				netconn_write(conn, http_index_hml, sizeof(http_index_hml)-1, NETCONN_NOCOPY);
			}
			else{

			}
		}
	}

	netconn_close(conn);
	netbuf_delete(inbuf);
}

static void httpServer(void *pvParameters){
	struct netconn *conn, *newConn;
	err_t err;
	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	do{
		err = netconn_accept(conn, &newConn);
		if(err == ERR_OK){
			processRequest(newConn);
			netconn_delete(newConn);
		}
	}while(err == ERR_OK);
	netconn_close(conn);
	netconn_delete(conn);
}
/********************************************************************************************
 *                                    GPIO INTERRUPT
 *
 *******************************************************************************************/
void writeGpio(void* pvParameters){
	uint64_t timeDiff;
	char *data = "Received !";
	while(1){
		if(xQueueReceive(gpio_evt_queue,&timeDiff,portMAX_DELAY)){
			speed = DISTANCE / ((double) timeDiff / TIMER_SCALE);
			angularSpeed = (ANGLE / ((double) timeDiff / TIMER_SCALE)) * (60 / 6.28);
			distance = distance + DISTANCE;
			//printf("Speed : %.4f s\n", speed);
			//printf("Delta Time : %.4f s\n", (double) timeDiff / TIMER_SCALE);
			//WS_write_data(data, strlen(data));
		}
	}
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint64_t time;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &time);
    deltaTime = time - previousTime;
    previousTime = time;
    xQueueSendFromISR(gpio_evt_queue, &deltaTime, NULL);
}

void init_interrupt(void){
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(GPIO_HALL_SENSOR, gpio_isr_handler, (void*) GPIO_HALL_SENSOR);
}

/*********************************************************************************************
 *                          TIMER IMPLEMENTATION
 *
 ********************************************************************************************/
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value =
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

void configTimerAlarm(int timer_idx, double timer_interval_sec){



	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_0, timer_idx, (timer_interval_sec * TIMER_SCALE) - TIMER_FINE_ADJ);
	timer_enable_intr(TIMER_GROUP_0, timer_idx);
	timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

	timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
}

static void tg0_timer_init(int timer_idx, double timer_interval_sec)
{

	/* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TIMER_AUTORELOAD_DIS;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    //timer_disable_intr(TIMER_GROUP_0, TIMER_0);
    //timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
   // timer_set_alarm_value(TIMER_GROUP_0, timer_idx, (timer_interval_sec * TIMER_SCALE) - TIMER_FINE_ADJ);
    //timer_enable_intr(TIMER_GROUP_0, timer_idx);
    //timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //timer_start(TIMER_GROUP_0, timer_idx);
    configTimerAlarm(timer_idx, timer_interval_sec);
}

void startTimer(void* pvParameters){
	while(1){
		int status;
		int startTime;
		//if(xQueueReceive(WebSocket_connection_status, &status, portMAX_DELAY)){
		if(xQueueReceive(StartTest, &startTime, portMAX_DELAY)){
			if(startTime){
				//timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
				timer_start(TIMER_GROUP_0, TIMER_0);
				distance = 0;
			}
			else{
				timer_pause(TIMER_GROUP_0, TIMER_0);
				timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (TIMER_INTERVAL_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
				TIMERG0.hw_timer[TIMER_0].reload = 1;
			}
		}

	}
}

void sendTimedData(void* pvParameters){
	char data[32];
	static int cnt = 0;
	while(1){

		//Variable declaration
		timer_event_t evt;
	    int voltageOut;       //Voltage value from ADC convertion
	    int raw;              //Raw ADC value
	    double currentOut;


		if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
			cnt++;
			//raw = adc1_get_raw(ADC1_GPIO35_CHANNEL);
			//voltageOut = (double) raw * CONVERTION;
			//raw = adc1_get_raw(ADC1_GPIO_34);
			//currentOut = (((((double) raw) * 1100) / 4096) - 550) / (40/3);
			//printf("GPIO35 : %d mV RAW : %d Conv : %.6f\n",voltageOut, raw, CONVERTION);
		    sprintf(data, "%.2f,%.6f,%.2f, %d, %.2f\n",(double) evt.timer_counter_value / TIMER_SCALE,  distance ,((double) v) * 4.36, (((c * 2) - 491) * 5), angularSpeed);
		    WS_write_data(data, strlen(data));
		    printf("Data Sent\n");
		}
	}
}

/*********************************************************************************************
 *                          ANALOG TO DIGITAL CONVERTION
 *                                 IMPLEMENTATION
 *
 ********************************************************************************************/
void init_adc1(){
	esp_adc_cal_characteristics_t characteristics;

	adc1_config_width(ADC_WIDTH_BIT_12);
	//adc1_config_width(ADC_WIDTH_BIT_10);
	    //adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_0);
	adc1_config_channel_atten(ADC1_GPIO_35, ADC_ATTEN_DB_0);
	adc1_config_channel_atten(ADC1_GPIO_34, ADC_ATTEN_DB_0);

	esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, &characteristics);
}

/*********************************************************************************************
 *                         			 I2C IMPLEMENTATION
 *
 ********************************************************************************************/
static void init_I2C_master()
{
    int i2c_master_port = I2C_NUM_1;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}
/*
static esp_err_t read_i2c_register(i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l, uint8_t channel)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1015_ADD << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00000001, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, channel, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b11100011, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(2 / portTICK_RATE_MS);
    //ets_delay_us(2000);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1015_ADD << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00000000, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    ets_delay_us(40);

    //vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1015_ADD << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    ets_delay_us(20);

    return ret;
}
*/

static esp_err_t read_i2c_register(i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l, uint8_t* data2_h, uint8_t* data2_l, uint8_t channel1, uint8_t channel2)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1015_ADD << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00000001, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, channel1, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b11100011, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(1 / portTICK_RATE_MS);
    //ets_delay_us(1000);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1015_ADD << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00000000, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    ets_delay_us(100);

    //vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1015_ADD << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    ets_delay_us(100);

    //vTaskDelay(2000 / portTICK_RATE_MS);

    	cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ADS1015_ADD << 1 | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0b00000001, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, channel2, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0b11100011, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(1 / portTICK_RATE_MS);
        //ets_delay_us(1000);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ADS1015_ADD << 1 | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0b00000000, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            return ret;
        }

        ets_delay_us(100);

        //vTaskDelay(30 / portTICK_RATE_MS);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ADS1015_ADD << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data2_h, ACK_VAL);
        i2c_master_read_byte(cmd, data2_l, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        ets_delay_us(100);
        //vTaskDelay(30 / portTICK_RATE_MS);

    return ret;
}

static void get_i2c_data(void* arg)
{
    /*int i = 0;
    uint32_t task_idx = (uint32_t) arg;
    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_wr = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_rd = (uint8_t*) malloc(DATA_LENGTH);*/

	int ret;
    uint8_t current_h, current_l;
    uint8_t voltage_h, voltage_l;


    int cnt = 0;
    while (1) {

    	ret = read_i2c_register( I2C_NUM_1, &voltage_h, &voltage_l, &current_h, &current_l, ADS1015_CHANNEL_0, ADS1015_CHANNEL_3_FSR_4_096);
        //printf("test cnt: %d\n", cnt++);
        //ret = read_i2c_register( I2C_NUM_1, &current_h, &current_l, ADS1015_CHANNEL_3_FSR_4_096);

        //ret = read_i2c_register( I2C_NUM_1, &current_h, &current_l,ADS1015_CH_3_SINGLE);

        //ret = read_i2c_register(I2C_NUM_1, &voltage_h, &voltage_l, ADS1015_CHANNEL_0);

        //ret = read_i2c_register(I2C_NUM_1, &voltage_h, &voltage_l, ADS1015_CH_0_SINGLE);
        //vTaskDelay(1 / portTICK_RATE_MS);
        current = (((((uint16_t) current_h << 4 | current_l >> 4) * 2) - 491) * 5);
        voltage = (double) (voltage_h << 4 | voltage_l >> 4) * 4.36;

        c = (current_h << 4 | current_l >> 4);
        v = (voltage_h << 4 | voltage_l >> 4);

        printf("%u, %.2f\n", current, voltage);

        /*xSemaphoreTake(print_mux, portMAX_DELAY);
        if(ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
        } else if(ret == ESP_OK) {
            printf("CHANNEL 2\n");
            printf("sensor val: %d\n", (sensor_data_h << 4 | sensor_data_l >> 4));
            printf("CHANNEL 3\n");
            printf("sensor val: %d\n", (sensor_2_h << 4 | sensor_2_l >> 4));
        } else {
            printf("No ack, sensor not connected...skip...\n");
        }
        xSemaphoreGive(print_mux);*/
        //vTaskDelay(50 / portTICK_RATE_MS);

    }
}

/********************************************************************************************************
 *                                      MAIN APPLICATION
 *******************************************************************************************************/
void app_main(){
	esp_err_t ret = nvs_flash_init();
	if(ret == ESP_ERR_NVS_NO_FREE_PAGES){
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	timer_queue = xQueueCreate(10, sizeof(timer_event_t));
	WebSocket_connection_status = xQueueCreate(10,sizeof(int));
	StartTest = xQueueCreate(10, sizeof(int));

	print_mux = xSemaphoreCreateMutex();

	//Wifi initialisation as Acces Point
	wifiInitAP();
	init_interrupt();
	init_adc1();
	init_I2C_master();

	tg0_timer_init(TIMER_0, TIMER_INTERVAL_SEC);

	xTaskCreate(sendTimedData, "printTime", 2048, NULL, 5, NULL);

	//Create a http Server task
	xTaskCreate(&httpServer, "httpServer", 2048, NULL, 5, NULL);

    //create WebSocker receive task
    xTaskCreate(&task_process_WebSocket, "ws_process_rx", 2048, NULL, 5, NULL);

    //Create Websocket Server Task
    xTaskCreate(&ws_server, "ws_server", 2048, NULL, 5, NULL);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint64_t));

    xTaskCreate(&writeGpio, "writeGpio", 2048, NULL, 1, NULL);
    xTaskCreate(startTimer, "startTimer", 2048, NULL, 2, NULL);
    xTaskCreate(get_i2c_data, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
}
