#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "time.h"
#include <stdbool.h>
#include <math.h>

#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"

#include "lmic.h"
#include "nmea.h"
#include "util.h"
#include "minifloat.h"
#include "minifix.h"


const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rst = 14,
  .dio = { 34, 26, 35 },
  .spi = { 19, 27, 5 },
  .rxtx = LMIC_UNUSED_PIN,
};

u1_t NWKSKEY[16] = { 0x6F, 0x6E, 0xBF, 0xFA, 0xCE, 0xA2, 0x5A, 0xE4, 0xF7, 0xBE, 0xE7, 0x3F, 0xD3, 0xB5, 0xBD, 0xEE };
u1_t APPSKEY[16] = { 0xB9, 0xFC, 0x5D, 0x30, 0xBE, 0xA4, 0xBA, 0x9E, 0x1E, 0xD3, 0x5F, 0x49, 0xDF, 0x8B, 0x36, 0x41 };
u4_t DEVADDR = 0x26011766;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

#define UART_BUFFER_SIZE 4096

#define GPS_FIX_MAX_AGE 30

static struct nmea nmea;

struct SSD1306_Device display;

uint16_t battery_mv = 0;

// System services

void sys_get_time(struct timeval* t) {
  t->tv_sec = osticks2us(os_getTime()) / 1000000UL;
  t->tv_usec = osticks2us(os_getTime()) - t->tv_sec * 1000000UL;
}

// LMIC

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
      printf("OP_TXRXPEND, not sending");
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
  } else if(nmea_fix_valid(&nmea)) {
      uint8_t hdop_num;
      uint8_t hdop_frac;
      struct {
        fix24 lat;
        fix24 lng;
        uint8_t hdop;
        uint8_t alt;
      } txdata;
      // Prepare upstream data transmission at the next possible time.
      txdata.lat = double_to_fix24(nmea.fix.lat.dir != 'N' ? -nmea.fix.lat.deg : nmea.fix.lat.deg);
      txdata.lng = double_to_fix24(nmea.fix.lng.dir != 'E' ? -nmea.fix.lng.deg : nmea.fix.lng.deg);
      hdop_num = max(0, min(15, (int)nmea.fix.hdop));
      hdop_frac = (uint8_t)ceil((nmea.fix.hdop - (double)hdop_num) * 10);
      txdata.hdop = (hdop_frac & 0xf) | ((hdop_num & 0xf) << 4);
      txdata.alt = float_to_ufloat8(nmea.fix.alt_msl);
      LMIC_setTxData2(1, &txdata, sizeof(txdata), 0);
      printf("Packet queued");
  } else {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
  }
}

void onEvent (ev_t ev) {
    printf("%d", os_getTime());
    printf(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            printf("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            printf("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            printf("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            printf("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            printf("EV_JOINING");
            break;
        case EV_JOINED:
            printf("EV_JOINED");
            break;
        case EV_RFU1:
            printf("EV_RFU1");
            break;
        case EV_JOIN_FAILED:
            printf("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            printf("EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            printf("EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
              printf("Received ack");
            if (LMIC.dataLen) {
              printf("Received %d bytes of payload", LMIC.dataLen);
            }

            if (LMIC.opmode & OP_TXRXPEND) {
                printf("OP_TXRXPEND, not sending");
            } else {
                // Prepare upstream data transmission at the next possible time.
          printf("Scheduling message for: %d\n", (s4_t)(os_getTime() + sec2osticks(TX_INTERVAL)));
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            }
            break;
        case EV_LOST_TSYNC:
            printf("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            printf("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            printf("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            printf("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            printf("EV_LINK_ALIVE");
            break;
         default:
            printf("Unknown event: %d", ev);
            break;
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

// UART

#define UBLOX_UART_NUM       UART_NUM_2
#define UBLOX_UART_BUFF_SIZE 4096
#define TAG                  "ttn-tracker"

char uart_buffer[UBLOX_UART_BUFF_SIZE];
int uart_last_pos = 0;
QueueHandle_t ublox_uart_queue;

static void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  size_t buffered_size;
  while(1) {
    //Waiting for UART event.
    if(xQueueReceive(ublox_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
      switch(event.type) {
        // We don't really care about data events
/*
        case UART_DATA:
          ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
          uart_read_bytes(UBLOX_UART_NUM, (unsigned char*)uart_buffer, event.size, portMAX_DELAY);
          ESP_LOGI(TAG, "[DATA EVT]: %s", uart_buffer);
          break;
*/
        //Event of HW FIFO overflow detected
        case UART_FIFO_OVF:
          ESP_LOGI(TAG, "hw fifo overflow");
          // If fifo overflow happened, you should consider adding flow control for your application.
          // The ISR has already reset the rx FIFO,
          // As an example, we directly flush the rx buffer here in order to read more data.
          uart_flush_input(UBLOX_UART_NUM);
          xQueueReset(ublox_uart_queue);
          break;
        //Event of UART ring buffer full
        case UART_BUFFER_FULL:
          ESP_LOGI(TAG, "ring buffer full");
          // If buffer full happened, you should consider encreasing your buffer size
          // As an example, we directly flush the rx buffer here in order to read more data.
          uart_flush_input(UBLOX_UART_NUM);
          xQueueReset(ublox_uart_queue);
          break;
/*
        //Event of UART RX break detected
        case UART_BREAK:
          ESP_LOGI(TAG, "uart rx break");
          break;
        //Event of UART parity check error
        case UART_PARITY_ERR:
          ESP_LOGI(TAG, "uart parity error");
          break;
        //Event of UART frame error
        case UART_FRAME_ERR:
          ESP_LOGI(TAG, "uart frame error");
          break;
*/
        // Newline detected, NMEA message received
        case UART_PATTERN_DET:
          uart_get_buffered_data_len(UBLOX_UART_NUM, &buffered_size);
          int pos = uart_pattern_pop_pos(UBLOX_UART_NUM);
//          ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
          if (pos == -1) {
            // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
            // record the position. We should set a larger queue size.
            // As an example, we directly flush the rx buffer here.
            uart_flush_input(UBLOX_UART_NUM);
          }
          else if (pos < sizeof(uart_buffer) - 1) {
            uart_read_bytes(UBLOX_UART_NUM, (unsigned char*)uart_buffer, pos, 0);
            uint8_t pat;
            uart_read_bytes(UBLOX_UART_NUM, &pat, 1, 0);
//            ESP_LOGI(TAG, "read data: %s", uart_buffer);
            uart_buffer[pos] = 0;
            nmea_parse_msg(&nmea, uart_buffer);
          } else {
            ESP_LOGE(TAG, "pattern data too long, flushing data");
            uart_flush_input(UBLOX_UART_NUM);
          }
          break;
        //Others
        default:
          ESP_LOGI(TAG, "uart event type: %d", event.type);
          break;
      }
    }
  }
  vTaskDelete(NULL);
}

// Display handling

enum {
  DISPLAY_LINE_BATTERY = 0,
  DISPLAY_LINE_GPS,
  DISPLAY_LINE_SATS,
  DISPLAY_LINE_POS,
  DISPLAY_LINE_AGE,
};

static void display_table_show_line(unsigned int line, char* left, char* right) {
  unsigned int width = display.Width;
  unsigned int height = display.Height;
  unsigned int offset_y = line * (SSD1306_FontGetHeight(&display) + 0);
  unsigned int lower_y = offset_y + SSD1306_FontGetHeight(&display);
  int offset_x_l, offset_x_r, _;
  SSD1306_FontGetAnchoredStringCoords(&display, &offset_x_l, &_, TextAnchor_NorthWest, left);
  SSD1306_FontGetAnchoredStringCoords(&display, &offset_x_r, &_, TextAnchor_NorthEast, right);
  SSD1306_DrawBox(&display, 0, offset_y, width - 1, lower_y, SSD_COLOR_BLACK, true);
  SSD1306_FontDrawString(&display, offset_x_l, offset_y, left, SSD_COLOR_WHITE);
  SSD1306_FontDrawString(&display, offset_x_r - 1, offset_y, right, SSD_COLOR_WHITE);
}

static void display_task(void* args) {
  char fmt_buff[32];

  assert(SSD1306_I2CMasterInitDefault() == true);
  SSD1306_I2CMasterAttachDisplayDefault(&display, 128, 64, 0x3C, 16);
  SSD1306_Clear(&display, SSD_COLOR_BLACK);
  SSD1306_SetFont(&display, &Font_droid_sans_fallback_11x13);
  SSD1306_FontDrawAnchoredString(&display, TextAnchor_NorthWest, "GPS: ", SSD_COLOR_WHITE);
  SSD1306_Update(&display);

  memset(fmt_buff, 0, sizeof(fmt_buff));
  while(1) {
    bool fix_valid = nmea_fix_valid(&nmea);
    SSD1306_Clear(&display, SSD_COLOR_BLACK);
    ESP_LOGI(TAG, "GPS fix @ %.6f %c %.6f %c height: %.2f m hdop: %.2f quality: %u fix: %s SATs: %u\n", nmea.fix.lat.deg,
      nmea.fix.lat.dir, nmea.fix.lng.deg, nmea.fix.lng.dir, nmea.fix.alt_msl, nmea.fix.hdop, nmea.fix.quality,
      nmea_fix_3d(&nmea) ? "3D" : nmea_fix_2d(&nmea) ? "2D" : "None", nmea.num_sats);

// Handle battery voltage
    snprintf(fmt_buff, sizeof(fmt_buff), "%u mV", battery_mv);
    display_table_show_line(DISPLAY_LINE_BATTERY, "Battery:", fmt_buff);

// Handle fix
    if(fix_valid) {
      snprintf(fmt_buff, sizeof(fmt_buff), "Lock (%s, %d SATs)", nmea_fix_3d(&nmea) ? "3D" : "2D", nmea.fix.num_sats);
    } else {
      snprintf(fmt_buff, sizeof(fmt_buff), "Offline");
    }
    display_table_show_line(DISPLAY_LINE_GPS, "GPS:", fmt_buff);

// Handle number of satellites
    snprintf(fmt_buff, sizeof(fmt_buff), "%d", nmea.num_sats);
    display_table_show_line(DISPLAY_LINE_SATS, "Visible satellites:", fmt_buff);

// Handle position
    if(fix_valid) {
      snprintf(fmt_buff, sizeof(fmt_buff), "%.5f %c %.5f %c", nmea.fix.lat.deg, nmea.fix.lat.dir, nmea.fix.lng.deg, nmea.fix.lng.dir);
    } else {
      snprintf(fmt_buff, sizeof(fmt_buff), "Fix invalid");
    }
    display_table_show_line(DISPLAY_LINE_POS, fmt_buff, "");

// Show age of fix
    if(fix_valid) {
      struct timeval age;
      nmea_fix_age(&nmea, &age);
      snprintf(fmt_buff, sizeof(fmt_buff), "Age of fix: %lu", age.tv_sec);
    } else {
      snprintf(fmt_buff, sizeof(fmt_buff), "Fix timeout");
    }
    display_table_show_line(DISPLAY_LINE_AGE, fmt_buff, "");

    SSD1306_Update(&display);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// I2C
static esp_err_t i2c_read_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data) {
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if(!cmd) {
    err = ESP_ERR_NO_MEM;
    goto fail;
  }
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  // first, send device address (indicating write) & register to be read
  if((err = i2c_master_write_byte(cmd, ( i2c_addr << 1 ), 1))) {
    goto fail_link;
  }
  // send register we want
  if((err = i2c_master_write_byte(cmd, i2c_reg, 1))) {
    goto fail_link;
  }
  // Send repeated start
  if((err = i2c_master_start(cmd))) {
    goto fail_link;
  }
  // now send device address (indicating read) & read data
  if((err = i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | I2C_MASTER_READ, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_read_byte(cmd, data, 1))) {
    goto fail_link;
  }
  if((err = i2c_master_stop(cmd))) {
    goto fail_link;
  }
  err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
fail_link:
  i2c_cmd_link_delete(cmd);
fail:
  return err;
}

static void battery_task(void* args) {
  while(true) {
    union {
      struct {
        uint8_t low;
        uint8_t high;
      } bytes;
      uint16_t mv;
    } voltage;
    // Half-soft i2c implementation on ATtiny 84 is still a little shitty
    // just keep retrying until read eventually succeeds
    if(i2c_read_reg(I2C_NUM_1, 0x42, 2, &voltage.bytes.high)) {
      continue;
    }
    if(i2c_read_reg(I2C_NUM_1, 0x42, 3, &voltage.bytes.low)) {
      continue;
    }
    battery_mv = voltage.mv;

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  os_init();

  LMIC_reset();
  printf("LMIC RESET");

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 21);

//  xTaskCreate(runloop, "os_runloop", 1024 * 2, (void* )0, 10, NULL);

// GPS setup

  nmea_init(&nmea);

  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(uart_param_config(UBLOX_UART_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UBLOX_UART_NUM, UART_PIN_NO_CHANGE, 13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(UBLOX_UART_NUM, UART_BUFFER_SIZE, 0, 32, &ublox_uart_queue, 0));
  ESP_ERROR_CHECK(uart_enable_pattern_det_intr(UBLOX_UART_NUM, '\n', 1, 100000, 0, 0));
  ESP_ERROR_CHECK(uart_pattern_queue_reset(UBLOX_UART_NUM, 32));
  xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);

// Battery controller setup

  i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 23,
    .scl_io_num = 22,
    .master.clk_speed = 100,
  };

  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

/*
  uint8_t i2c_data;
  ESP_ERROR_CHECK(i2c_read_reg(I2C_NUM_1, 0x42, 2, &i2c_data));
  ESP_LOGI("i2c", "Read high byte of battery voltage: %u\n", i2c_data);
  ESP_ERROR_CHECK(i2c_read_reg(I2C_NUM_1, 0x42, 3, &i2c_data));
  ESP_LOGI("i2c", "Read low byte of battery voltage: %u\n", i2c_data);
*/

  xTaskCreate(battery_task, "battery_task", 2048, NULL, 12, NULL);

// Display setup

  xTaskCreate(display_task, "display_task", 4096, NULL, 12, NULL);

  do_send(&sendjob);

  while(1) {
    os_runloop_once();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
