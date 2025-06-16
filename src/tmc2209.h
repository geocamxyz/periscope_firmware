//
// Created by linus on 6/12/25.
//

#ifndef TMC2209_H
#define TMC2209_H

#include <zephyr/kernel.h>

int tmc2209_init_uart(const struct device *uart_dev);
int tmc2209_write_register(const struct device *uart_dev, uint8_t node_addr,
                          uint8_t reg_addr, uint32_t data);
int tmc2209_read_register(const struct device *uart_dev, uint8_t node_addr,
                         uint8_t reg_addr, uint32_t *data);

#endif //TMC2209_H
