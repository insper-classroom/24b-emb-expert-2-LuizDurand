/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "ssd1306.h"
#include "gfx.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/dma.h"

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

const uint BTN_1_OLED = 28;
const uint BTN_2_OLED = 26;
const uint BTN_3_OLED = 27;

const uint LED_1_OLED = 20;
const uint LED_2_OLED = 21;
const uint LED_3_OLED = 22;

#define ADC_PIN 26  // GPIO26 corresponds to ADC0

QueueHandle_t adcQueue;

void adc_task(void *params) {
    // Initialize the ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0); // ADC0

    uint16_t adc_value;

    while (1) {
        // Read ADC value
        adc_value = adc_read();

        // Send ADC value to the queue
        xQueueSend(adcQueue, &adc_value, portMAX_DELAY);

        // Delay for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void ssd1306_put_page_dma(uint8_t *data, uint8_t page, uint8_t column, uint8_t width) {
    // Set page and column addresses
    ssd1306_set_page_address(page);
    ssd1306_set_column_address(column);

    // Prepare for data transfer
    gpio_put(SSD1306_DATA_CMD_SEL, 1); // Set DC to data mode
    spi_cs_select();                   // CS low to select the device

    // Configure DMA
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&dma_cfg, spi_get_index(SPI_PORT) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
    channel_config_set_read_increment(&dma_cfg, true);
    channel_config_set_write_increment(&dma_cfg, false);

    dma_channel_configure(
        dma_chan,
        &dma_cfg,
        &spi_get_hw(SPI_PORT)->dr, // Destination: SPI data register
        data,                      // Source: data buffer
        width,                     // Number of bytes
        true                       // Start immediately
    );

    // Wait for the DMA transfer to complete
    dma_channel_wait_for_finish_blocking(dma_chan);

    // Clean up
    dma_channel_unclaim(dma_chan);
    spi_cs_deselect(); // CS high to deselect the device
}

void gfx_show_dma(ssd1306_t *p) {
    for (uint8_t page = 0; page < p->pages; page++) {
        ssd1306_put_page_dma(p->buffer + (page * p->width), page, 0, p->width);
    }
}

void oled_task(void *params) {
    // Initialize the OLED display
    ssd1306_init();
    ssd1306_t display;
    gfx_init(&display, 128, 32);

    uint16_t adc_value;
    char buffer[20];

    while (1) {
        // Receive ADC value from the queue
        if (xQueueReceive(adcQueue, &adc_value, portMAX_DELAY) == pdTRUE) {
            // Clear the display buffer
            gfx_clear_buffer(&display);

            // Calculate voltage from ADC value
            float voltage = (adc_value * 3.3f) / 4095.0f;
            snprintf(buffer, sizeof(buffer), "Voltage: %.2fV", voltage);

            // Draw the voltage string
            gfx_draw_string(&display, 0, 0, 1, buffer);

            // Draw a bar representing the ADC value
            int bar_length = (adc_value * 128) / 4095;
            gfx_draw_line(&display, 0, 20, bar_length, 20);

            // Update the display using DMA
            gfx_show_dma(&display);
        }
    }
}

int main() {
    stdio_init_all();

    // Create a queue to hold ADC values
    adcQueue = xQueueCreate(10, sizeof(uint16_t));

    // Create tasks
    TaskHandle_t adcTaskHandle;
    TaskHandle_t oledTaskHandle;

    xTaskCreate(adc_task, "ADC Task", 256, NULL, 1, &adcTaskHandle);
    xTaskCreate(oled_task, "OLED Task", 1024, NULL, 1, &oledTaskHandle);

    // Set task affinities
    vTaskCoreAffinitySet(adcTaskHandle, CORE_0);
    vTaskCoreAffinitySet(oledTaskHandle, CORE_1);

    // Start the scheduler
    vTaskStartScheduler();

    while (1) {
        // Should never reach here
    }
}
