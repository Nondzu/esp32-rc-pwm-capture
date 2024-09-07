#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rmt_capture.h"

#define RC_PWM1_PIN 2
#define RC_PWM2_PIN 3
#define MEM_BLOCK_SYMBOLS 64
#define PWM_MIN 1000         // Minimalna wartość PWM z odbiornika (1 ms)
#define PWM_MAX 2000         // Maksymalna wartość PWM z odbiornika (2 ms)

static const char *TAG = "RC_PWM";

// Kolejki dla kanałów
QueueHandle_t receive_queue1 = NULL;
QueueHandle_t receive_queue2 = NULL;

// Kanały RMT
rmt_channel_handle_t rx_chan1 = NULL;
rmt_channel_handle_t rx_chan2 = NULL;

// Callback po zakończeniu odbioru PWM
bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}
// Konfiguracja odbioru sygnałów PWM z kanałów RMTs
rmt_receive_config_t receive_config = {
    .signal_range_min_ns = 1000,
    .signal_range_max_ns = 3000000,
   };
    

// // Funkcja inicjalizująca kanały RMT dla odbioru PWMz
// void setup_rmt_channels() {
//     // Konfiguracja kanału 1 (Throttle)
//     rmt_rx_channel_config_t rx_chan_config1 = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = 1 * 1000 * 1000,
//         .mem_block_symbols = MEM_BLOCK_SYMBOLS,
//         .gpio_num = RC_PWM1_PIN,
//         .flags.invert_in = false,
//         .flags.with_dma = false,
//     };
//     ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config1, &rx_chan1));

//     receive_queue1 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
//     rmt_rx_event_callbacks_t cbs1 = { .on_recv_done = rmt_rx_done_callback };
//     ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan1, &cbs1, receive_queue1));
//     ESP_ERROR_CHECK(rmt_enable(rx_chan1));

//     // Konfiguracja kanału 2 (Steering)
//     rmt_rx_channel_config_t rx_chan_config2 = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = 1 * 1000 * 1000,
//         .mem_block_symbols = MEM_BLOCK_SYMBOLS,
//         .gpio_num = RC_PWM2_PIN,
//         .flags.invert_in = false,
//         .flags.with_dma = false,
//     };
//     ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config2, &rx_chan2));


//     receive_queue2 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
//     rmt_rx_event_callbacks_t cbs2 = { .on_recv_done = rmt_rx_done_callback };
//     ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan2, &cbs2, receive_queue2));
//     ESP_ERROR_CHECK(rmt_enable(rx_chan2));

    
//     xTaskCreate(process_pwm_signals, "process_pwm_signals", 4096, NULL, 5, NULL);
// }

void process_pwm_signals_with_args(void *pvParameter) {
    int *throttle = ((int *)pvParameter);  // Throttle pointer
    int *steering = throttle + 1;          // Steering pointer (next int after throttle)

    rmt_symbol_word_t raw_symbols1[MEM_BLOCK_SYMBOLS];
    rmt_symbol_word_t raw_symbols2[MEM_BLOCK_SYMBOLS];

    while (true) {
        // Receive and process from channel 1 (Throttle)
        ESP_ERROR_CHECK(rmt_receive(rx_chan1, raw_symbols1, sizeof(raw_symbols1), &receive_config));
        rmt_rx_done_event_data_t rx_data1;
        xQueueReceive(receive_queue1, &rx_data1, portMAX_DELAY);
        if (rx_data1.num_symbols > 0) {
            // Assume the first symbol's duration is the PWM value
            *throttle = rx_data1.received_symbols[0].duration0;
            // ESP_LOGI(TAG, "Throttle PWM: %d", *throttle);
        }

        // Receive and process from channel 2 (Steering)
        ESP_ERROR_CHECK(rmt_receive(rx_chan2, raw_symbols2, sizeof(raw_symbols2), &receive_config));
        rmt_rx_done_event_data_t rx_data2;
        xQueueReceive(receive_queue2, &rx_data2, portMAX_DELAY);
        if (rx_data2.num_symbols > 0) {
            // Assume the first symbol's duration is the PWM value
            *steering = rx_data2.received_symbols[0].duration0;
            // ESP_LOGI(TAG, "Steering PWM: %d", *steering);
        }
    }
}


void setup_rmt_channels(int *throttle, int *steering) {
    // Configure RMT Channel 1 (Throttle)
    rmt_rx_channel_config_t rx_chan_config1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1 * 1000 * 1000,  // 1 MHz resolution
        .mem_block_symbols = MEM_BLOCK_SYMBOLS,
        .gpio_num = RC_PWM1_PIN,
        .flags.invert_in = false,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config1, &rx_chan1));

    receive_queue1 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs1 = { .on_recv_done = rmt_rx_done_callback };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan1, &cbs1, receive_queue1));
    ESP_ERROR_CHECK(rmt_enable(rx_chan1));

    // Configure RMT Channel 2 (Steering)
    rmt_rx_channel_config_t rx_chan_config2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1 * 1000 * 1000,  // 1 MHz resolution
        .mem_block_symbols = MEM_BLOCK_SYMBOLS,
        .gpio_num = RC_PWM2_PIN,
        .flags.invert_in = false,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config2, &rx_chan2));

    receive_queue2 = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs2 = { .on_recv_done = rmt_rx_done_callback };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan2, &cbs2, receive_queue2));
    ESP_ERROR_CHECK(rmt_enable(rx_chan2));

    // Create task for processing PWM signals with argument passing
    xTaskCreate(process_pwm_signals_with_args, "process_pwm_signals_with_args", 4096, (void *)throttle, 5, NULL);
}

char log_buffer[328];
// Funkcja do przetwarzania sygnałów PWM z obu kanałów
void process_pwm_signals(void *pvParameter) {
    rmt_symbol_word_t raw_symbols1[MEM_BLOCK_SYMBOLS];
    rmt_symbol_word_t raw_symbols2[MEM_BLOCK_SYMBOLS];
    
    while (true) {    
        // Odbieranie i przetwarzanie z kanału 1 (Throttle)
        ESP_ERROR_CHECK(rmt_receive(rx_chan1, raw_symbols1, sizeof(raw_symbols1), &receive_config));
        rmt_rx_done_event_data_t rx_data1;
        xQueueReceive(receive_queue1, &rx_data1, portMAX_DELAY);
        ESP_LOGI(TAG, "Kanal 1: Odebrano %d symboli", rx_data1.num_symbols);
        for (int i = 0; i < rx_data1.num_symbols; i++) {
            sprintf(log_buffer, "Kanal 1 - Symbol %d: duration0=%u duration1=%u", 
                    i, (unsigned int)rx_data1.received_symbols[i].duration0, (unsigned int)rx_data1.received_symbols[i].duration1);
            ESP_LOGI(TAG, "%s", log_buffer);
        }

        // Odbieranie i przetwarzanie z kanału 2 (Steering)
        ESP_ERROR_CHECK(rmt_receive(rx_chan2, raw_symbols2, sizeof(raw_symbols2), &receive_config));
        rmt_rx_done_event_data_t rx_data2;
        xQueueReceive(receive_queue2, &rx_data2, portMAX_DELAY);
        ESP_LOGI(TAG, "Kanal 2: Odebrano %d symboli", rx_data2.num_symbols);
        for (int i = 0; i < rx_data2.num_symbols; i++) {
            sprintf(log_buffer, "Kanal 2 - Symbol %d: duration0=%u duration1=%u", 
                    i, (unsigned int)rx_data2.received_symbols[i].duration0, (unsigned int)rx_data2.received_symbols[i].duration1);
            ESP_LOGI(TAG, "%s", log_buffer);
        }
    }
}