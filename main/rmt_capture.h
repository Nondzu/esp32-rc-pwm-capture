#ifndef RMT_CAPTURE_H
#define RMT_CAPTURE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"

// Zewnętrzne deklaracje zmiennych
extern QueueHandle_t receive_queue1;
extern QueueHandle_t receive_queue2;
extern rmt_channel_handle_t rx_chan1;
extern rmt_channel_handle_t rx_chan2;

// Deklaracje funkcji
void setup_rmt_channels(int *throttle, int *steering);
void process_pwm_signals_with_args(void *pvParameter);
void process_pwm_signals(void *pvParameter);
void control_motors(int throttle, int steering);
#endif // RMT_CAPTURE_H
