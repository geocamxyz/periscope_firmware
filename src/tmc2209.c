#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "tmc2209.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmc2209, LOG_LEVEL_INF);

// TMC2209 UART constants
#define TMC2209_SYNC_NIBBLE     0x05    // 0101 sync pattern
#define TMC2209_MASTER_ADDR     0xFF    // Master address for read replies
#define TMC2209_WRITE_FLAG      0x80    // Bit 7 set for write access
#define TMC2209_READ_FLAG       0x00    // Bit 7 clear for read access

// Datagram sizes
#define TMC2209_WRITE_DATAGRAM_SIZE  8   // bytes
#define TMC2209_READ_REQUEST_SIZE    4   // bytes
#define TMC2209_READ_REPLY_SIZE      8   // bytes

// Timing constants
#define TMC2209_REPLY_TIMEOUT_MS    50   // Timeout for read replies
#define TMC2209_BYTE_TIMEOUT_MS     10   // Timeout between bytes

// RX buffer management
#define RX_BUFFER_SIZE 32
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_double_buffer[RX_BUFFER_SIZE];
static volatile size_t rx_index = 0;
static volatile bool rx_complete = false;
static volatile size_t expected_rx_bytes = 0;
static bool async_api_available = false;

// Synchronization
static K_SEM_DEFINE(rx_sem, 0, 1);
static K_MUTEX_DEFINE(uart_mutex);

// Function prototypes
void tmc_uart_crc(uint8_t *data, uint8_t data_len);
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data);
static void uart_isr(const struct device *dev, void *user_data);

void tmc_uart_crc(uint8_t *data, uint8_t data_len) {
    int i,j;
    uint8_t *crc = data + (data_len-1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;
    for (i=0; i<(data_len-1); i++) { // Execute for all bytes of a message
        currentByte = data[i]; // Retrieve a byte to be sent from Array
        for (j=0; j<8; j++) {
            if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            else
                *crc = (*crc << 1);
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

/**
 * UART async callback function for handling events
 */
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        LOG_DBG("RX ready: offset=%d, len=%d", evt->data.rx.offset, evt->data.rx.len);

        // Copy received data to our buffer
        if (rx_index + evt->data.rx.len <= RX_BUFFER_SIZE) {
            memcpy(&rx_buffer[rx_index],
                   evt->data.rx.buf + evt->data.rx.offset,
                   evt->data.rx.len);
            rx_index += evt->data.rx.len;

            // Check if we've received the expected number of bytes
            if (expected_rx_bytes > 0 && rx_index >= expected_rx_bytes) {
                rx_complete = true;
                k_sem_give(&rx_sem);
            }
        } else {
            LOG_ERR("RX buffer overflow");
            rx_complete = true;
            k_sem_give(&rx_sem);
        }
        break;

    case UART_RX_BUF_REQUEST:
        LOG_DBG("RX buffer request");
        uart_rx_buf_rsp(dev, rx_double_buffer, RX_BUFFER_SIZE);
        break;

    case UART_RX_BUF_RELEASED:
        LOG_DBG("RX buffer released");
        break;

    case UART_RX_DISABLED:
        LOG_DBG("UART RX disabled");
        break;

    case UART_RX_STOPPED:
        LOG_DBG("UART RX stopped");
        rx_complete = true;
        k_sem_give(&rx_sem);
        break;

    case UART_TX_DONE:
        LOG_DBG("TX done");
        break;

    case UART_TX_ABORTED:
        LOG_WRN("TX aborted");
        break;

    default:
        LOG_DBG("Unhandled UART event: %d", evt->type);
        break;
    }
}

/**
 * UART interrupt service routine (fallback for non-async mode)
 */
static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;
    static bool waiting_for_reply = false;
    static uint8_t sync_pattern = 0x50; // TMC2209 reply sync pattern

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            while (uart_fifo_read(dev, &c, 1) == 1) {
                // Only collect data if we're expecting it
                if (expected_rx_bytes > 0) {
                    // Look for TMC2209 reply sync pattern (0x50) to start collecting
                    if (rx_index == 0) {
                        if (c == sync_pattern) {
                            rx_buffer[rx_index++] = c;
                            waiting_for_reply = true;
                        } else {
                            // Discard bytes until we see sync pattern
                            LOG_DBG("Discarding non-sync byte: 0x%02X", c);
                        }
                    } else if (waiting_for_reply && rx_index < RX_BUFFER_SIZE) {
                        rx_buffer[rx_index++] = c;

                        // Check if we've received expected bytes
                        if (rx_index >= expected_rx_bytes) {
                            rx_complete = true;
                            waiting_for_reply = false;
                            // Disable RX to stop further interrupts
                            uart_irq_rx_disable(dev);
                            k_sem_give(&rx_sem);
                        }
                    } else if (rx_index >= RX_BUFFER_SIZE) {
                        LOG_ERR("RX buffer overflow, resetting");
                        rx_index = 0;
                        waiting_for_reply = false;
                        rx_complete = true;
                        uart_irq_rx_disable(dev);
                        k_sem_give(&rx_sem);
                    }
                } else {
                    // Unexpected data - just discard it
                    LOG_DBG("Discarding unexpected RX byte: 0x%02X", c);
                }
            }
        }
    }
}

/**
 * Initialize UART for TMC2209 communication
 *
 * @param uart_dev UART device pointer
 * @return 0 on success, negative error code on failure
 */
int tmc2209_init_uart(const struct device *uart_dev)
{
    int ret;

    if (!uart_dev) {
        return -EINVAL;
    }

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    // Try to use async API first
    ret = uart_callback_set(uart_dev, uart_callback, NULL);
    if (ret == 0) {
        async_api_available = true;
        LOG_INF("Using UART async API");
    } else {
        LOG_WRN("Async API not available (%d), falling back to interrupt mode", ret);
        async_api_available = false;

        // Set up interrupt-driven UART
        uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
        // Don't enable RX interrupts here - only when we need them
    }

    LOG_INF("TMC2209 UART initialized (async=%s)", async_api_available ? "yes" : "no");
    return 0;
}

/**
 * Clear/flush RX buffer of any stale data
 */
static void flush_rx_buffer(const struct device *uart_dev)
{
    uint8_t dummy;
    int count = 0;

    // Read out any stale bytes
    while (uart_fifo_read(uart_dev, &dummy, 1) > 0 && count < 32) {
        LOG_DBG("Flushing stale byte: 0x%02X", dummy);
        count++;
    }

    // Reset our buffer state
    rx_index = 0;
    rx_complete = false;
}

/**
 * Wait for TX transmission to complete (equivalent to serialFlush)
 */
static void wait_tx_complete(const struct device *uart_dev)
{
    // Small delay to ensure transmission completes
    k_usleep(100);
}

/**
 * Write a 32-bit value to a TMC2209 register
 *
 * @param uart_dev  UART device pointer
 * @param node_addr Node address (0-3, set by MS1/MS2 pins)
 * @param reg_addr  Register address (0-127)
 * @param data      32-bit data to write
 * @return 0 on success, negative error code on failure
 */
int tmc2209_write_register(const struct device *uart_dev, uint8_t node_addr,
                          uint8_t reg_addr, uint32_t data)
{
    uint8_t datagram[TMC2209_WRITE_DATAGRAM_SIZE];
    int ret;

    if (!uart_dev) {
        return -EINVAL;
    }

    if (node_addr > 3) {
        LOG_ERR("Invalid node address: %d (must be 0-3)", node_addr);
        return -EINVAL;
    }

    if (reg_addr > 0x7F) {
        LOG_ERR("Invalid register address: 0x%02X (must be 0-127)", reg_addr);
        return -EINVAL;
    }

    k_mutex_lock(&uart_mutex, K_FOREVER);

    // Clear any stale RX data first
    flush_rx_buffer(uart_dev);

    // Build write datagram according to specification
    // Byte 0: Sync nibble (0101) + reserved nibble (included in CRC)
    datagram[0] = TMC2209_SYNC_NIBBLE;

    // Byte 1: Node address (bits 0-1), reserved bits 2-7
    datagram[1] = node_addr & 0x03;

    // Byte 2: Write flag (bit 7) + register address (bits 0-6)
    datagram[2] = TMC2209_WRITE_FLAG | (reg_addr & 0x7F);

    // Bytes 3-6: 32-bit data, highest byte first
    datagram[3] = (data >> 24) & 0xFF;  // Data byte 3 (MSB)
    datagram[4] = (data >> 16) & 0xFF;  // Data byte 2
    datagram[5] = (data >> 8) & 0xFF;   // Data byte 1
    datagram[6] = data & 0xFF;          // Data byte 0 (LSB)

    // Byte 7: CRC (calculated by tmc_uart_crc)
    datagram[7] = 0x00;  // Will be filled by CRC function

    // Calculate CRC
    tmc_uart_crc(datagram, TMC2209_WRITE_DATAGRAM_SIZE);

    LOG_DBG("Write reg 0x%02X to node %d: 0x%08X", reg_addr, node_addr, data);
    LOG_HEXDUMP_DBG(datagram, TMC2209_WRITE_DATAGRAM_SIZE, "TX datagram:");

    // Transmit datagram (unidirectional - no echo handling needed)
    for (int i = 0; i < TMC2209_WRITE_DATAGRAM_SIZE; i++) {
        uart_poll_out(uart_dev, datagram[i]);
    }

    // Wait for transmission to complete
    wait_tx_complete(uart_dev);

    k_mutex_unlock(&uart_mutex);
    return 0;
}

/**
 * Read a 32-bit value from a TMC2209 register
 *
 * @param uart_dev  UART device pointer
 * @param node_addr Node address (0-3, set by MS1/MS2 pins)
 * @param reg_addr  Register address (0-127)
 * @param data      Pointer to store the read 32-bit data
 * @return 0 on success, negative error code on failure
 */
int tmc2209_read_register(const struct device *uart_dev, uint8_t node_addr,
                         uint8_t reg_addr, uint32_t *data)
{
    uint8_t request[TMC2209_READ_REQUEST_SIZE];
    int ret;
    uint32_t echo_delay_us = 0;
    const uint32_t ECHO_DELAY_MAX_US = 5000;
    const uint32_t ECHO_DELAY_INC_US = 100;

    if (!uart_dev || !data) {
        return -EINVAL;
    }

    if (node_addr > 3) {
        LOG_ERR("Invalid node address: %d (must be 0-3)", node_addr);
        return -EINVAL;
    }

    if (reg_addr > 0x7F) {
        LOG_ERR("Invalid register address: 0x%02X (must be 0-127)", reg_addr);
        return -EINVAL;
    }

    k_mutex_lock(&uart_mutex, K_FOREVER);

    // Build read request datagram
    // Byte 0: Sync nibble (0101) + reserved nibble
    request[0] = TMC2209_SYNC_NIBBLE;

    // Byte 1: Node address (bits 0-1), reserved bits 2-7
    request[1] = node_addr & 0x03;

    // Byte 2: Read flag (bit 7 clear) + register address (bits 0-6)
    request[2] = TMC2209_READ_FLAG | (reg_addr & 0x7F);

    // Byte 3: CRC
    request[3] = 0x00;  // Will be filled by CRC function

    // Calculate CRC for request
    tmc_uart_crc(request, TMC2209_READ_REQUEST_SIZE);

    LOG_DBG("Read reg 0x%02X from node %d", reg_addr, node_addr);
    LOG_HEXDUMP_DBG(request, TMC2209_READ_REQUEST_SIZE, "TX request:");

    // Clear RX buffer before starting
    flush_rx_buffer(uart_dev);

    // Transmit read request
    for (int i = 0; i < TMC2209_READ_REQUEST_SIZE; i++) {
        uart_poll_out(uart_dev, request[i]);
    }

    // Wait for transmission to complete
    wait_tx_complete(uart_dev);

    // Wait for echo and discard it (like Arduino implementation)
    echo_delay_us = 0;
    rx_index = 0;
    while (rx_index < TMC2209_READ_REQUEST_SIZE && echo_delay_us < ECHO_DELAY_MAX_US) {
        uint8_t byte;
        if (uart_fifo_read(uart_dev, &byte, 1) > 0) {
            LOG_DBG("Discarding echo byte: 0x%02X", byte);
            rx_index++;
        } else {
            k_usleep(ECHO_DELAY_INC_US);
            echo_delay_us += ECHO_DELAY_INC_US;
        }
    }

    // Now wait for the actual reply
    rx_index = 0;
    rx_complete = false;
    expected_rx_bytes = TMC2209_READ_REPLY_SIZE;

    uint32_t reply_delay_us = 0;
    const uint32_t REPLY_DELAY_MAX_US = 50000; // 50ms timeout
    const uint32_t REPLY_DELAY_INC_US = 1000;  // 1ms increments

    while (rx_index < TMC2209_READ_REPLY_SIZE && reply_delay_us < REPLY_DELAY_MAX_US) {
        uint8_t byte;
        if (uart_fifo_read(uart_dev, &byte, 1) > 0) {
            rx_buffer[rx_index++] = byte;
        } else {
            k_usleep(REPLY_DELAY_INC_US);
            reply_delay_us += REPLY_DELAY_INC_US;
        }
    }

    if (rx_index < TMC2209_READ_REPLY_SIZE) {
        LOG_ERR("Incomplete reply received: %zu/%d bytes (timeout)",
                rx_index, TMC2209_READ_REPLY_SIZE);
        k_mutex_unlock(&uart_mutex);
        return -ETIMEDOUT;
    }

    LOG_HEXDUMP_DBG(rx_buffer, TMC2209_READ_REPLY_SIZE, "RX reply:");

    // Verify reply structure
    if (rx_buffer[1] != TMC2209_MASTER_ADDR) {
        LOG_ERR("Invalid master address in reply: 0x%02X (expected 0xFF)", rx_buffer[1]);
        LOG_HEXDUMP_ERR(rx_buffer, TMC2209_READ_REPLY_SIZE, "Bad reply:");
        k_mutex_unlock(&uart_mutex);
        return -EBADMSG;
    }

    if ((rx_buffer[2] & 0x7F) != reg_addr) {
        LOG_ERR("Register address mismatch: got 0x%02X, expected 0x%02X",
                rx_buffer[2] & 0x7F, reg_addr);
        k_mutex_unlock(&uart_mutex);
        return -EBADMSG;
    }

    // Verify CRC
    uint8_t expected_crc = rx_buffer[7];
    uint8_t reply_copy[TMC2209_READ_REPLY_SIZE];
    memcpy(reply_copy, rx_buffer, TMC2209_READ_REPLY_SIZE);
    tmc_uart_crc(reply_copy, TMC2209_READ_REPLY_SIZE);

    if (reply_copy[7] != expected_crc) {
        LOG_ERR("CRC mismatch: got 0x%02X, calculated 0x%02X",
                expected_crc, reply_copy[7]);
        k_mutex_unlock(&uart_mutex);
        return -EBADMSG;
    }

    // Extract 32-bit data (highest byte first)
    *data = ((uint32_t)rx_buffer[3] << 24) |
            ((uint32_t)rx_buffer[4] << 16) |
            ((uint32_t)rx_buffer[5] << 8) |
            ((uint32_t)rx_buffer[6]);

    LOG_DBG("Read successful: 0x%08X", *data);

    k_mutex_unlock(&uart_mutex);
    return 0;
}

/**
 * Example usage function
 */
void tmc2209_example_usage(const struct device *uart_dev)
{
    uint32_t read_data;
    int ret;

    // Initialize UART for TMC2209
    ret = tmc2209_init_uart(uart_dev);
    if (ret < 0) {
        LOG_ERR("UART init failed: %d", ret);
        return;
    }

    // Example: Write to GCONF register (0x00)
    ret = tmc2209_write_register(uart_dev, 0, 0x00, 0x000001E0);
    if (ret < 0) {
        LOG_ERR("Write failed: %d", ret);
        return;
    }

    // Wait a bit for the write to take effect
    k_msleep(10);

    // Example: Read back GCONF register
    ret = tmc2209_read_register(uart_dev, 0, 0x00, &read_data);
    if (ret < 0) {
        LOG_ERR("Read failed: %d", ret);
        return;
    }

    LOG_INF("GCONF register: 0x%08X", read_data);
}