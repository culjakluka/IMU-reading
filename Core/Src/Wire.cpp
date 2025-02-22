#include "stm32f4xx_hal.h"  // Include the appropriate HAL header for your STM32 series
#include "Wire.h"

extern I2C_HandleTypeDef hi2c1;  // Assume I2C1 is being used, adjust accordingly

#define I2C_TIMEOUT 1000  // Timeout for I2C operations in milliseconds

TwoWire::TwoWire() {
    rx_buf_idx = 0;
    rx_buf_len = 0;
    tx_addr = 0;
    tx_buf_idx = 0;
    tx_buf_overflow = false;
}

/*
 * Joins I2C bus as master on given SDA and SCL pins.
 */
void TwoWire::beginTransmission(uint8_t slave_address) {
    tx_addr = slave_address;
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    rx_buf_idx = 0;
    rx_buf_len = 0;
}

void TwoWire::beginTransmission(int slave_address) {
    beginTransmission((uint8_t)slave_address);
}

uint8_t TwoWire::endTransmission(void) {
    if (tx_buf_overflow) return EDATA;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, tx_addr << 1, tx_buf, tx_buf_idx, I2C_TIMEOUT);
    if (status != HAL_OK) {
        return ENACKADDR;  // Return error if transmission failed
    }

    tx_buf_idx = 0;
    tx_buf_overflow = false;
    return SUCCESS;
}

uint8_t TwoWire::requestFrom(uint8_t address, int num_bytes) {
    if (num_bytes > WIRE_BUFSIZ) num_bytes = WIRE_BUFSIZ;

    rx_buf_idx = 0;
    rx_buf_len = 0;

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, address << 1, rx_buf, num_bytes, I2C_TIMEOUT);
    if (status == HAL_OK) {
        rx_buf_len = num_bytes;
    } else {
        rx_buf_len = 0;
    }

    return rx_buf_len;
}

uint8_t TwoWire::requestFrom(int address, int numBytes) {
    return TwoWire::requestFrom((uint8_t)address, (uint8_t) numBytes);
}

void TwoWire::write(uint8_t value) {
    if (tx_buf_idx == WIRE_BUFSIZ) {
        tx_buf_overflow = true;
        return;
    }

    tx_buf[tx_buf_idx++] = value;
}

void TwoWire::write(uint8_t* buf, int len) {
    for (uint8_t i = 0; i < len; i++) write(buf[i]);
}

void TwoWire::write(int value) {
    write((uint8_t)value);
}

void TwoWire::write(int* buf, int len) {
    write((uint8_t*)buf, (uint8_t)len);
}

void TwoWire::write(char* buf) {
    uint8_t *ptr = (uint8_t*)buf;
    while(*ptr) {
        write(*ptr);
        ptr++;
    }
}

uint8_t TwoWire::available() {
    return rx_buf_len - rx_buf_idx;
}

uint8_t TwoWire::read() {
    if (rx_buf_idx == rx_buf_len) return 0;
    return rx_buf[rx_buf_idx++];
}

// Declare the instance that the users of the library can use
TwoWire Wire;
