#include <Arduino.h>

#include "driver/spi_master.h"
#include "driver/spi_slave.h"

#define TX_HOST    VSPI_HOST
#define DMA_CHAN_TX    2

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define OUT_LATCH_PIN (12)//Any pin will work
#define IN_LATCH_PIN (27) //Any pin will work

spi_device_handle_t spi_tx;
spi_device_handle_t spi_rx;

spi_transaction_t tx_transaction;
spi_transaction_t rx_transaction;

WORD_ALIGNED_ATTR uint8_t buf[60];
WORD_ALIGNED_ATTR uint8_t recvbuf[30];
WORD_ALIGNED_ATTR uint8_t dummybuf[30];

void posttx(spi_transaction_t *trans) {
    digitalWrite(OUT_LATCH_PIN, HIGH);
    digitalWrite(OUT_LATCH_PIN, LOW);
}

void prerx(spi_transaction_t *trans) {
    digitalWrite(IN_LATCH_PIN, LOW);
    digitalWrite(IN_LATCH_PIN, HIGH);
}

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
    ret=spi_bus_initialize(TX_HOST, &tx_bus_config, DMA_CHAN_TX);
    ESP_ERROR_CHECK(ret);

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
        .spics_io_num=PIN_NUM_CS,
        .flags=0,
        .queue_size=1,
        .pre_cb=NULL,
        .post_cb=&posttx,
    };
    ret=spi_bus_add_device(TX_HOST, &tx_device_config, &spi_tx);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t rx_device_config = {
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .mode=2,
        .duty_cycle_pos=0,
        .cs_ena_pretrans=0,
        .cs_ena_posttrans=0,
        .clock_speed_hz=8*1000*1000,
        .input_delay_ns=0,
        .spics_io_num=-1, //PIN_NUM_CS,
        .flags=0,
        .queue_size=1,
        .pre_cb=&prerx,
        .post_cb=NULL,
    };
    ret=spi_bus_add_device(TX_HOST, &rx_device_config, &spi_rx);
    ESP_ERROR_CHECK(ret);

    memset(&tx_transaction, 0, sizeof(tx_transaction));
    tx_transaction.length = 120*4;
    tx_transaction.tx_buffer = &buf;

    memset(&rx_transaction, 0, sizeof(rx_transaction));
    rx_transaction.length = 120/4*8;
    rx_transaction.tx_buffer = &dummybuf;
    rx_transaction.rx_buffer = &recvbuf;

    memset(buf, 0, sizeof(buf));
    memset(recvbuf, 0, sizeof(recvbuf));
    memset(dummybuf, 0, sizeof(dummybuf));

  pinMode(OUT_LATCH_PIN, OUTPUT);
  digitalWrite(IN_LATCH_PIN, LOW);
  pinMode(IN_LATCH_PIN, OUTPUT);
  digitalWrite(IN_LATCH_PIN, HIGH);
}


char b[100];

uint32_t x = 0;

void loop() {
    esp_err_t ret;
    Serial.print("Send ");
    Serial.print(x);
    Serial.println();

    buf[59] = x & 0xFF;
    x++;

    // Send data
    ret=spi_device_polling_transmit(spi_tx, &tx_transaction);
    assert(ret==ESP_OK);

    // Receive data
    ret=spi_device_polling_transmit(spi_rx, &rx_transaction);
    assert(ret==ESP_OK);

    uint32_t v =
      recvbuf[0] |
      recvbuf[1] << 8 |
      recvbuf[2] << 16 |
      recvbuf[3] << 24;
    sprintf(b, "Recv %u\n", v);
    Serial.println(b);

    delay(100);
}