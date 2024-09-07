#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"
#include "rmt_capture.h"

#define MEM_BLOCK_SYMBOLS 64


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
#define MCPWM_TIMER_RESOLUTION_HZ 1000000  // 1 MHz = 1 µs per tick


static const char *TAG = "RC_PWM";

// Deklaracje funkcji z innych plików
void setup_motor_pwm();
void setup_rmt_channels();
void update_motor_pwm(uint32_t duty_motor1, uint32_t duty_motor2);

// Funkcja konwertująca wartość PWM na skalę dla sterowania silnikami
int map_pwm_to_motor_speed(uint32_t pwm_value) {
    return (int)((pwm_value - PWM_MIN) * 255 / (PWM_MAX - PWM_MIN));  // Normalizacja do zakresu 0-255
}


// Główna funkcja
void app_main(void) {
    // Konfiguracja PWM dla silników
    setup_motor_pwm();
    
    // Konfiguracja kanałów RMT
    setup_rmt_channels();

    // Tutaj można dodać logikę do przetwarzania danych z kanałów RMT i sterowania PWM
    
        // Odbieranie i aktualizowanie sygnałów PWM...
        while (true) {
            // Przetwarzanie sygnałów PWM z kanałów
            process_pwm_signals();
            // vTaskDelay(20 / portTICK_PERIOD_MS);  // Czekaj 20 ms
            }
}
