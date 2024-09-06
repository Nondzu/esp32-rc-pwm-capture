#include <stdio.h>
#include <inttypes.h>  // Makra do formatowania uint32_t
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define RC_PWM_PIN 2       // Ustaw pin do odbioru PWM
#define LED_PIN 21         // Ustaw pin do migania LED
static const char *TAG = "RC_PWM";

// Callback wywoływany po zakończeniu odbioru
bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    if (edata->received_symbols && edata->num_symbols > 0) {
        uint32_t duration = edata->received_symbols[0].duration0;
        ESP_LOGI(TAG, "Odczytano PWM: %" PRIu32 " µs", duration);  // PRIu32 dla uint32_t
    }
    return true;
}

// Funkcja do migania LED
void led_blink_task(void *pvParameter) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // gpio_pad_select_gpio(LED_PIN);
    // gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (true) {
        // Włącz LED
        gpio_set_level(LED_PIN, 1);
        // vTaskDelay(pdMS_TO_TICKS(500));  // Czekaj 500 ms
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Wyłącz LED
        gpio_set_level(LED_PIN, 0);
        // vTaskDelay(pdMS_TO_TICKS(500));  // Czekaj 500 ms
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// void app_main(void) {
//     // Tworzymy zadanie do migania LED
//     xTaskCreate(led_blink_task, "led_blink_task", 2048, NULL, 5, NULL);
//     // czekamy 2s na uruchomienie migania LED
//     vTaskDelay(pdMS_TO_TICKS(1700));
//     printf("starting rmt...n");
//     vTaskDelay(pdMS_TO_TICKS(300));

//     // Konfiguracja kanału RMT
//     rmt_rx_channel_config_t rx_chan_config = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,    // Domyślne źródło zegara
//         .resolution_hz = 1 * 1000 * 1000,  // 1 MHz, 1 tick = 1 µs
//         .mem_block_symbols = 64,           // Rozmiar bloku pamięci
//         .gpio_num = RC_PWM_PIN,            // GPIO do odczytu sygnału PWM
//         .flags.invert_in = false,          // Nie odwracaj sygnału
//         .flags.with_dma = false,           // Nie używamy DMA
//     };

//     rmt_channel_handle_t rx_chan = NULL;
//     ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));  // Tworzymy kanał RX

//     // Rejestracja callbacka na zakończenie odbioru
//     rmt_rx_event_callbacks_t cbs = {
//         .on_recv_done = rmt_rx_done_callback,  // Ustawiamy callback
//     };
//     ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL));

//     // Włącz kanał do odbioru
//     ESP_ERROR_CHECK(rmt_enable(rx_chan));  // Włączamy kanał RMT



//     while (true) {
//         // Odbiór sygnału PWM
//         ESP_ERROR_CHECK(rmt_receive(rx_chan, NULL, 1, NULL));  // Rozpocznij odbiór
//         vTaskDelay(pdMS_TO_TICKS(20));  // Czekaj 20 ms (typowy okres PWM dla RC)
//     }
// }



void app_main(void) {
    // Tworzymy zadanie do migania LED
    xTaskCreate(led_blink_task, "led_blink_task", 4096, NULL, 5, NULL);
    // czekamy 2s na uruchomienie migania LED
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("starting rmt...n");
    vTaskDelay(300 / portTICK_PERIOD_MS);

    // Konfiguracja kanału RMT
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // Domyślne źródło zegara
        .resolution_hz = 1 * 1000 * 1000,  // 1 MHz, 1 tick = 1 µs
        .mem_block_symbols = 512,           // Rozmiar bloku pamięci
        .gpio_num = RC_PWM_PIN,            // GPIO do odczytu sygnału PWM
        .flags.invert_in = false,          // Nie odwracaj sygnału
        .flags.with_dma = false,           // Nie używamy DMA
    };

    rmt_channel_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));  // Tworzymy kanał RX

    // Rejestracja callbacka na zakończenie odbior
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,  // Ustawiamy callback
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL));

    // Włącz kanał do odbioru
    ESP_ERROR_CHECK(rmt_enable(rx_chan));  // Włączamy kanał RMT
    // Bufor na odbierane symbole
    rmt_symbol_word_t receive_buffer[512];  // Bufor na 64 symbole (zgodnie z konfiguracją mem_block_symbols)

    while (true) {
        // Odbiór sygnału PWM - przekazujemy odpowiedni bufor
        ESP_ERROR_CHECK(rmt_receive(rx_chan, receive_buffer, sizeof(receive_buffer), NULL));  // Odbieramy sygnały do bufora
        vTaskDelay(pdMS_TO_TICKS(20));  // Czekaj 20 ms (typowy okres PWM dla RC)
    }
}
