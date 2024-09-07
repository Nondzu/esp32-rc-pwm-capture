#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"
#include "rmt_capture.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#define MEM_BLOCK_SYMBOLS 64

#define LED_PIN 21           // Ustaw pin do migania LED
#define RC_PWM1_PIN 2        // Ustaw pin do odbioru PWM na GPIO 2 (Throttle)
#define RC_PWM2_PIN 3        // Ustaw pin do odbioru PWM na GPIO 3 (Steering)

#define MEM_BLOCK_SYMBOLS 64 // Liczba symboli RMT dla kanałów
#define PWM_MIN 1000         // Minimalna wartość PWM z odbiornika (1 ms)
#define PWM_MAX 2000         // Maksymalna wartość PWM z odbiornika (2 ms)



static const char *TAG = "RC_PWM";

// Deklaracje funkcji z innych plików
void setup_motor_pwm();
void setup_rmt_channels();
void update_motor_pwm(uint32_t duty_motor1, uint32_t duty_motor2);

// Funkcja konwertująca wartość PWM na skalę dla sterowania silnikami
int map_pwm_to_motor_speed(uint32_t pwm_value) {
    return (int)((pwm_value - PWM_MIN) * 255 / (PWM_MAX - PWM_MIN));  // Normalizacja do zakresu 0-255
}

// Funkcja do migania LED
void led_blink_task(void *pvParameter) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (true) {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(111 / portTICK_PERIOD_MS);

        gpio_set_level(LED_PIN, 1);
        vTaskDelay(1845 / portTICK_PERIOD_MS);

        // print time
        printf("Time: %u \n", (unsigned int)xTaskGetTickCount());
    }
}
// Główna funkcja
void app_main(void) {

    vTaskDelay(2000 / portTICK_PERIOD_MS);  

    xTaskCreate(led_blink_task, "led_blink_task", 4096, NULL, 5, NULL);

    // Konfiguracja PWM dla silników
    setup_motor_pwm();
    
    volatile int throttle = 0;
    volatile int steering = 0;

    // Konfiguracja kanałów RMT
    setup_rmt_channels(&throttle, &steering);

    int pwm = 0;
    int cnt = 0;
    while(true) {

        pwm = map_pwm_to_motor_speed(cnt);

        if (cnt < 2000) {
            cnt+=100;
        } else {
            cnt = 1500;
        }



        // Odbieranie i aktualizowanie sygnałów PWM...
        
        control_motors(throttle, steering);
        vTaskDelay(20 / portTICK_PERIOD_MS);  // Czekaj 20 ms
        

    }
    // Tutaj można dodać logikę do przetwarzania danych z kanałów RMT i sterowania PWM
    
        // Odbieranie i aktualizowanie sygnałów PWM...
        // while (true) {
        //     // Przetwarzanie sygnałów PWM z kanałów
        //     // process_pwm_signals();
        //     vTaskDelay(20 / portTICK_PERIOD_MS);  // Czekaj 20 ms
        //     }
}
