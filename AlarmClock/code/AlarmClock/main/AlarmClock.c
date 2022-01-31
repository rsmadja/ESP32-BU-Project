/*
  Adapted I2C example code to work with the Adafruit 14-segment Alphanumeric Display. Key notes: MSB!!
  http://timmurphy.org/2010/05/04/pthreads-in-c-a-minimal-working-example/
  Emily Lam, Sept 2018, Updated Aug 2019
*/

#include <rom/uart.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "driver/i2c.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "esp_intr_alloc.h"
#include "driver/timer.h"
#include "freertos/semphr.h"
#include "esp_types.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/apps/sntp.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

#define ALARM1_VAL            1000  // Time in ms
#define TIMER_DIVIDER         80    //  Hardware timer clock divider
#define DIVIDER_TO_MILLIS     1000
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

//WIFI SNPT CONNECTION
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

//SERVO CONTROL
#define SERVO_MIN_PULSEWIDTH 650 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
#define SERVO_MAX_TIMERANGE 60 //Maximum time range (minute/second) the servo will represent
#define SERVO_MINUTE 18//GPIO of the servo representing minutes
#define SERVO_SECONDE 33//GPIO of the servo representing secondes

//Setting up global variable


/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "example";

time_t rawtime;
time_t t;
struct tm *info;
char mySecs[80];
int mytime[3];
int alarmflag = 0;  //flag de/activating the alarm
char alarmtime[] = "00:00";
int alarmint[2] = {0,0};  //Alarm time
char timezone[] = "EST5EDT,M3.2.0/2,M11.1.0"; //default time zone EST



// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;


//display bitmap
static const uint16_t myBin[] =  {
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000000000, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000000000, // space
    0b0100000000000000  // .
};

//Functions Prototypes
/*SNTP obtain actual time via WIFI*/
static void obtain_time(void);

/*init SNTP*/
static void initialize_sntp(void);

/*init wifi*/
static void initialise_wifi(void);

/*SNTP event handler*/
static esp_err_t event_handler(void *ctx, system_event_t *event);

/*update servo position by inputing time*/
void time_to_servo(int time, int typetime);

/*Signal alarm via display*/
void alarmDisplay(void);


// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// I2C Functions /////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}


// Alphanumeric Functions //////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// convert ascii value to bitmap for display
int * string2bin(char *mystring) {
    int theInt;
    static int binLoc[4];
    for (int i=0;i<4;i++) {
        theInt = (int)(mystring[i]);
        if ((theInt>=97)&&(theInt<=122)) {
            theInt = theInt-32;
        }
        if ((theInt>=65)&&(theInt<=90)) {
            theInt = theInt-54;
        }
        else if ((theInt>=48)&&(theInt<=57)) {
            theInt = theInt-47;
        }
        else if (theInt==47) { // slash
            theInt = 0;
        }
        else if (theInt==32) {
            theInt = 37;
        }
        else if (theInt==46) {
            theInt = 38;
        }
        else {
            theInt = 37;
        }
        binLoc[i] = theInt;
    }
    return binLoc;
}

/*Update time to display and activate alarm if flag = 1*/

static void update_display(int theHour, int theMinute) {

    if(alarmflag && theHour == alarmint[0] && theMinute == alarmint[1]){
      //printf("ALARM\n");
      alarmDisplay();
    }
    // Debug
    int ret;
    char theString[4];
    // Write to characters to buffer
    uint16_t displaybuffer[8];

    int *myLoc;  // Create array to store location of matching character in the binary array myBin
    if(mytime[0] < 10){     // Accounts for cushioning zero
      sprintf(theString,"0%d%d",theHour,theMinute);  // Converts theHour and theMinute into a single string
    }
    else{
      sprintf(theString,"%d%d",theHour,theMinute);  //  Converts theHour and theMinute into a single string without a 
    }



    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    // Set display blink off
    ret = no_blink();

    ret = set_brightness_max(0xF);


    myLoc = string2bin(theString);

    if (mytime[1]<10) { // Adds a cushioning zero to the minutes if the minute is a single digit number
      myLoc[3] = myLoc[2];
      myLoc[2] = 1;
    }
    if (mytime[0]==0) { // Writes myBin zeros to both hour elements in the myLoc array as to not leave a blank space for the minutes to accidentally fill
      myLoc[0] = 1;
      myLoc[1] = 1;
    }
    if (mytime[1]==0) { // Writes myBin zeros to both minute elements in the myLoc array
      myLoc[2] = 1;
      myLoc[3] = 1;
    }
    displaybuffer[0] = myBin[*myLoc];      // Prints first digit/character on the display
    displaybuffer[1] = myBin[*(myLoc+1)];  // Prints second digit/character on the display
    displaybuffer[2] = myBin[*(myLoc+2)];  // Prints third digit/character on the display
    displaybuffer[3] = myBin[*(myLoc+3)];  // Prints fourth digit/character on the display

    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
      i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);



}

/*display "ALRM" when alarm is call in update display*/
void alarmDisplay(){

  int ret;

  // Set up routines
  // Turn on alpha oscillator
  ret = alpha_oscillator();
  // Set display blink off
  ret = no_blink();

  ret = set_brightness_max(0xF);

  // Write to characters to buffer
  uint16_t displaybuffer[8];
  displaybuffer[0] = 0b0000000011110111;  // A
  displaybuffer[1] = 0b0000000000111000;  // L
  displaybuffer[2] = 0b0010000011110011;  // R
  displaybuffer[3] = 0b0000010100110110;  // M

  // Send commands characters to display over I2C
  i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
  i2c_master_start(cmd4);
  i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
  for (uint8_t i=0; i<8; i++) {
    i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
  }
  i2c_master_stop(cmd4);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd4);

  vTaskDelay(200 / portTICK_RATE_MS);/*delay so the message blink between time and message*/

}

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void alarm_init() {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (DIVIDER_TO_MILLIS*ALARM1_VAL));
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// The main task of this example program
/*activate at every event*/
static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1) {
            time(&rawtime);
            setenv("TZ", timezone, 1);
            tzset();
            info = localtime(&rawtime);
            strftime(mySecs,80,"%H:%M:%S",info);
            //printf("Time:%s\n", mySecs);
            mytime[0] = atoi(strtok(mySecs,":"));
            mytime[1] = atoi(strtok(NULL,":"));
            mytime[2] = atoi(strtok(NULL,":"));
            update_display(mytime[0], mytime[1]);
            time_to_servo(mytime[1], 1);
            time_to_servo(mytime[2], 0);


        }
    }
}

/*SNTP fnct, retrive time via WIFI*/
static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    ESP_ERROR_CHECK( esp_wifi_stop() );
}

/*init SNTP protocol*/
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}
/*init wifi connection*/
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

/*SNTP/WIFI event handler*/
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

// SERVO fonctions ///////////////////////////////////
/*init PWM for servos in corresponding GPIO*/
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_MINUTE);    //Set GPIO 18 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_SECONDE);    //Set GPIO 33 as PWM0B, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_TIMERANGE)));
    return cal_pulsewidth;
}


/*init servo*/
void init_pwm_servo(void){
    mcpwm_example_gpio_initialize();

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

}

/*represent minutes with servo*/
void time_to_servo(int time, int typetime){

  uint32_t pulse_width;

  pulse_width = servo_per_degree_init(time);
  if(typetime == 0){
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width);
  }
  else{
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, pulse_width);
  }


}


//MAIN /////////////////////////////////////////////////////////////////////////
void app_main(void) {


  //variables
    char menu[1];
    char NewTime[10];

//INIT calls
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    i2c_example_master_init();
    i2c_scanner();
    // Create a FIFO queue for timer-based
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Create task to handle timer-based events
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initiate alarm using timer API
    alarm_init();

    //Obtain EST time via wifi
    obtain_time();

    //init servo
    init_pwm_servo();


    printf("Time Set via SNTP on EST\n");
// MENU user input
    while(1){
//print menu
      printf("Please chose a mode :\n");
      printf("1 - Setting time zone\n");
      printf("2 - De/Activating alarm\n");
      printf("3 - Setting an alarm\n");
//wai user input
      gets(menu);
//select chosed mode
      switch(menu[0]){
        //mode 1 - select time zone
        case '1':
          printf("Please choose a time zone :\n");
          printf("a - EST(Boston, New york, Montreal)\n");
          printf("b - PST(SF, LA)\n");
          printf("c - UTC+2(Paris, Berlin)\n");
          gets(NewTime);
          switch (NewTime[0]) {
            case 'a':
              strcpy(timezone,"EST5EDT,M3.2.0/2,M11.1.0");
              break;
            case 'b':
              strcpy(timezone,"PST8PDT");
              break;
            case 'c':
              strcpy(timezone,"CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00");
              break;
            default:
              strcpy(timezone,"EST5EDT,M3.2.0/2,M11.1.0");
              break;
          }
          //printf("Time zone set to : %s \n", timezone);
          break;
        //mode 2 - de/activate alarm
        case '2':
          alarmflag = !alarmflag; //changing flag
          if (alarmflag == 1){
            printf("Alarm set up for %s\n", alarmtime);
          }
          else{
            printf("alarm set off\n");
          }
          break;

        //moe 3 - setting alarm time
        case '3':
          printf("Please enter new alarm time in format HH:MM\n");
          gets(NewTime);
          strcpy(alarmtime, NewTime);
          alarmint[0] = atoi(strtok(NewTime,":"));
          alarmint[1] = atoi(strtok(NULL,":"));
          printf("New alarm time is: %s\n", alarmtime);
          break;
          //default
        default:
          break;
      }
    }
}
