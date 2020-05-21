#include <Arduino.h>

#include "driver/spi_master.h"
#include "driver/spi_slave.h"

#define TX_HOST    VSPI_HOST
#define DMA_CHAN_TX    2

#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define RX_HOST HSPI_HOST
#define DMA_CHAN_RX    1
#define PIN_NUM_RX_MISO -1
#define PIN_NUM_RX_MOSI 13
#define PIN_NUM_RX_CLK  14
#define PIN_NUM_RX_CS   15

spi_device_handle_t spi_tx;
spi_device_handle_t spi_rx;


void setup() {

    Serial.begin(921600);
    Serial.println("Hello world!");

    esp_err_t ret;


    //Initialize the TX SPI bus and "device"
    spi_bus_config_t tx_bus_config = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1000,
    };
    spi_device_interface_config_t tx_device_config = {
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .mode=0,
        .duty_cycle_pos=0,
        .cs_ena_pretrans=0,
        .cs_ena_posttrans=0,
        .clock_speed_hz=8*1000*1000,
        .input_delay_ns=0,
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .flags=0,
        .queue_size=1,
        .pre_cb=NULL,
        .post_cb=NULL,
    };
    ret=spi_bus_initialize(TX_HOST, &tx_bus_config, DMA_CHAN_TX);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(TX_HOST, &tx_device_config, &spi_tx);
    ESP_ERROR_CHECK(ret);



    //Configuration for the RX SPI bus
    spi_bus_config_t rx_bus_config = {
        .mosi_io_num = PIN_NUM_RX_MOSI,
        .miso_io_num = PIN_NUM_RX_MISO,
        .sclk_io_num = PIN_NUM_RX_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1000,
    };
    spi_slave_interface_config_t rx_slave_config = {
        .spics_io_num = PIN_NUM_RX_CS,
        .flags = 0,
        .queue_size = 3,
        .mode = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };
    // //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    // gpio_set_pull_mode(PIN_NUM_RX_MOSI, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode(PIN_NUM_RX_CLK, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode(PIN_NUM_RX_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RX_HOST, &rx_bus_config, &rx_slave_config, DMA_CHAN_RX);
    assert(ret==ESP_OK);

}

WORD_ALIGNED_ATTR uint8_t buf[100];
WORD_ALIGNED_ATTR uint8_t recvbuf[100];

char b[100];

void loop() {
    esp_err_t ret;
    uint32_t time = micros();
    Serial.print("Hello world!");
    Serial.print(time);
    Serial.println();

    buf[0] = time & 0xFF;
    buf[1] = (time & 0xFF00) >> 8;
    buf[2] = (time & 0xFF0000) >> 16;
    buf[3] = (time & 0xFF000000) >> 24;


    // Configure receiver
    memset(recvbuf, 0, 100);
    spi_slave_transaction_t rx_transaction;
    memset(&rx_transaction, 0, sizeof(rx_transaction));
    rx_transaction.length = 32;
    rx_transaction.rx_buffer = recvbuf;
    ret=spi_slave_queue_trans(RX_HOST, &rx_transaction, portMAX_DELAY);
    assert(ret==ESP_OK);

    // Send data
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=32;
    t.tx_buffer=&buf;
    ret=spi_device_polling_transmit(spi_tx, &t);
    assert(ret==ESP_OK);

    // Wait for received data
    spi_slave_transaction_t* rx_completed_transaction;
    ret = spi_slave_get_trans_result(RX_HOST, &rx_completed_transaction, portMAX_DELAY);
    assert(ret == ESP_OK);

    uint32_t v =
      recvbuf[0] |
      recvbuf[1] << 8 |
      recvbuf[2] << 16 |
      recvbuf[3] << 24;
    sprintf(b, "Received %u\n", v);
    Serial.println(b);

    delay(100);
}