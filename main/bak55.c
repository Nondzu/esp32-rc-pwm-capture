#include <stdio.h>
#include <inttypes.h>  // Makra do formatowania uint32_t
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <math.h> 

#define LED_PIN 21           // Ustaw pin do migania LED
#define RC_PWM1_PIN 2        // Ustaw pin do odbioru PWM na GPIO 2 (Throttle)
#define RC_PWM2_PIN 3        // Ustaw pin do odbioru PWM na GPIO 3 (Steering)
#define MOTOR1_DIR_PIN 4     // Pin kierunku dla silnika 1 (DIR)
#define MOTOR1_PWM_PIN 5     // Pin PWM dla silnika 1 (PWM)
#define MOTOR2_DIR_PIN 6     // Pin kierunku dla silnika 2 (DIR)
#define MOTOR2_PWM_PIN 7     // Pin PWM dla silnika 2 (PWM)
#define MEM_BLOCK_SYMBOLS 64 // Liczba symboli RMT dla kanałów
#define PWM_MIN 1000         // Minimalna wartość PWM z odbiornika (1 ms)
#define PWM_MAX 2000         // Maksymalna wartość PWM z odbiornika (2 ms)

static const char *TAG = "RC_PWM";

QueueHandle_t receive_queue1 = NULL;  // Kolejka do odbierania danych z kanału 1
QueueHandle_t receive_queue2 = NULL;  // Kolejka do odbierania danych z kanału 2

// Callback wywoływany po zakończeniu odbioru (dla obu kanałów)
bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // Wyślij odebrane symbole do odpowiedniej kolejki
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

// Funkcja do migania LED
void led_blink_task(void *pvParameter) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (true) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gpio_set_level(LED_PIN, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // print time
        printf("Time: %u \n", (unsigned int)xTaskGetTickCount());
    }
}

// Funkcja sterująca silnikami na podstawie wartości z kanałów
void control_motors(int throttle, int steering) {
    int pwm_left = throttle + steering;  // Sterowanie lewym silnikiem
    int pwm_right = throttle - steering; // Sterowanie prawym silnikiem

    // Sterowanie lewym silnikiem (Motor 1)
    if (pwm_left > 0) {
        gpio_set_level(MOTOR1_DIR_PIN, 0); // Przód
        gpio_set_level(MOTOR1_PWM_PIN, pwm_left);
    } else {
        gpio_set_level(MOTOR1_DIR_PIN, 1); // Tył
        gpio_set_level(MOTOR1_PWM_PIN, -pwm_left);
    }

    // Sterowanie prawym silnikiem (Motor 2)
    if (pwm_right > 0) {
        gpio_set_level(MOTOR2_DIR_PIN, 0); // Przód
        gpio_set_level(MOTOR2_PWM_PIN, pwm_right);
    } else {
        gpio_set_level(MOTOR2_DIR_PIN, 1); // Tył
        gpio_set_level(MOTOR2_PWM_PIN, -pwm_right);
    }
}

// Funkcja konwertująca wartość PWM na skalę dla sterowania silnikami
int map_pwm_to_motor_speed(uint32_t pwm_value) {
    return (int)((pwm_value - PWM_MIN) * 255 / (PWM_MAX - PWM_MIN));  // Normalizacja do zakresu 0-255
}

void app_main(void) {
    // Tworzymy zadanie do migania LED
    xTaskCreate(led_blink_task, "led_blink_task", 4096, NULL, 5, NULL);

    // Konfiguracja kanału 1 RMT do odbioru PWM (GPIO 2)
    rmt_rx_channel_config_t rx_chan_config1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // Domyślne źródło zegara
        .resolution_hz = 1 * 1000 * 1000,  // 1 MHz, 1 tick = 1 µs
        .mem_block_symbols = MEM_BLOCK_SYMBOLS,  // Ustawiona wartość mem_block_symbols
        .gpio_num = RC_PWM1_PIN,           // GPIO do odczytu sygnału PWM (GPIO 2)
        .flags.invert_in = false,          // Nie odwracaj sygnału
        .flags.with_dma = false,           // Nie używamy DMA
    };

    rmt_channel_handle_t rx_chan1 = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config1, &rx_chan1));  // Tworzymy kanał RX 1

    // Konfiguracja odbioru sygnałów PWM dla kanału 1
    rmt_receive_config_t receive_config1 = {
        .signal_range_min_ns = 1000,      // Minimalny czas trwania impulsu
        .signal_range_max_ns = 3000000,   // Maksymalny czas trwania impulsu (3 ms)
    };

    // Rejestracja callbacka na zakończenie odbioru dla kanału 1
    receive_queue1 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs1 = {
        .on_recv_done = rmt_rx_done_callback,  // Ustawiamy callback
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan1, &cbs1, receive_queue1));

    // Włącz kanał 1 do odbioru
    ESP_ERROR_CHECK(rmt_enable(rx_chan1));  // Włączamy kanał RMT

    // --- Konfiguracja dla kanału 2 (GPIO 3) ---
    
    // Konfiguracja kanału 2 RMT do odbioru PWM (GPIO 3)
    rmt_rx_channel_config_t rx_chan_config2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // Domyślne źródło zegara
        .resolution_hz = 1 * 1000 * 1000,  // 1 MHz, 1 tick = 1 µs
        .mem_block_symbols = MEM_BLOCK_SYMBOLS,  // Ustawiona wartość mem_block_symbols
        .gpio_num = RC_PWM2_PIN,           // GPIO do odczytu sygnału PWM (GPIO 3)
        .flags.invert_in = false,          // Nie odwracaj sygnału
        .flags.with_dma = false,           // Nie używamy DMA
    };

    rmt_channel_handle_t rx_chan2 = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config2, &rx_chan2));  // Tworzymy kanał RX 2

    // Konfiguracja odbioru sygnałów PWM dla kanału 2
    rmt_receive_config_t receive_config2 = {
        .signal_range_min_ns = 1000,      // Minimalny czas trwania impulsu
        .signal_range_max_ns = 3000000,   // Maksymalny czas trwania impulsu (3 ms)
    };

    // Rejestracja callbacka na zakończenie odbioru dla kanału 2
    receive_queue2 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs2 = {
        .on_recv_done = rmt_rx_done_callback,  // Ustawiamy callback
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan2, &cbs2, receive_queue2));

    // Włącz kanał 2 do odbioru
    ESP_ERROR_CHECK(rmt_enable(rx_chan2));  // Włączamy kanał RMT

    // Bufory na odebrane symbole
    rmt_symbol_word_t raw_symbols1[MEM_BLOCK_SYMBOLS];
    rmt_symbol_word_t raw_symbols2[MEM_BLOCK_SYMBOLS];
    char log_buffer[128];  // Bufor na sformatowany tekst

    while (true) {
        // Odbieranie z kanału 1 (GPIO 2)
        ESP_ERROR_CHECK(rmt_receive(rx_chan1, raw_symbols1, sizeof(raw_symbols1), &receive_config1));

        // Oczekuj na zakończenie odbioru z kanału 1
        rmt_rx_done_event_data_t rx_data1;
        xQueueReceive(receive_queue1, &rx_data1, portMAX_DELAY);

        // Przetwarzanie odebranych symboli z kanału 1
        ESP_LOGI(TAG, "Kanał 1: Odebrano %d symboli", rx_data1.num_symbols);
        for (int i = 0; i < rx_data1.num_symbols; i++) {
            sprintf(log_buffer, "Kanał 1 - Symbol %d: duration0=%u duration1=%u", 
                    i, (unsigned int)rx_data1.received_symbols[i].duration0, (unsigned int)rx_data1.received_symbols[i].duration1);
            ESP_LOGI(TAG, "%s", log_buffer);
        }

        // Odbieranie z kanału 2 (GPIO 3)
        ESP_ERROR_CHECK(rmt_receive(rx_chan2, raw_symbols2, sizeof(raw_symbols2), &receive_config2));

        // Oczekuj na zakończenie odbioru z kanału 2
        rmt_rx_done_event_data_t rx_data2;
        xQueueReceive(receive_queue2, &rx_data2, portMAX_DELAY);

        // Przetwarzanie odebranych symboli z kanału 2
        ESP_LOGI(TAG, "Kanał 2: Odebrano %d symboli", rx_data2.num_symbols);
        for (int i = 0; i < rx_data2.num_symbols; i++) {
            sprintf(log_buffer, "Kanał 2 - Symbol %d: duration0=%u duration1=%u", 
                    i, (unsigned int)rx_data2.received_symbols[i].duration0, (unsigned int)rx_data2.received_symbols[i].duration1);
            ESP_LOGI(TAG, "%s", log_buffer);
        }

        // Czekaj 20 ms (typowy okres PWM dla RC)
        // vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
