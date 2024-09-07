#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"

#define MOTOR1_DIR_PIN 4
#define MOTOR1_PWM_PIN 5
#define MOTOR2_DIR_PIN 6
#define MOTOR2_PWM_PIN 7
#define MCPWM_TIMER_RESOLUTION_HZ 1000000  // 1 MHz = 1 µs per tick

mcpwm_cmpr_handle_t comparator_motor1;
mcpwm_cmpr_handle_t comparator_motor2;
void setup_motor_pwm() {
    // Konfiguracja MCPWM Timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MWDT_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = 20000,  // 20 ms dla sygnału PWM
    };
    mcpwm_timer_handle_t motor1_timer;
    mcpwm_timer_handle_t motor2_timer;
    
    // Motor 1 (lewy silnik)
    mcpwm_oper_handle_t operator_motor1;
    mcpwm_operator_config_t operator_config = {};
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor1_timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_motor1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_motor1, motor1_timer));
    
    mcpwm_comparator_config_t cmp_config = { .flags.update_cmp_on_tep = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_motor1, &cmp_config, &comparator_motor1));
    
    mcpwm_generator_config_t gen_config = { .gen_gpio_num = MOTOR1_PWM_PIN };
    mcpwm_gen_handle_t generator_motor1;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_motor1, &gen_config, &generator_motor1));

    // Motor 2 (prawy silnik)
    mcpwm_oper_handle_t operator_motor2;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor2_timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_motor2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_motor2, motor2_timer));
    
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_motor2, &cmp_config, &comparator_motor2));
    
    mcpwm_generator_config_t gen_config_motor2 = { .gen_gpio_num = MOTOR2_PWM_PIN };
    mcpwm_gen_handle_t generator_motor2;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_motor2, &gen_config_motor2, &generator_motor2));

    // Ustawienie akcji na zdarzenia timera
    mcpwm_gen_timer_event_action_t ev_action_up = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_EMPTY,  // Początek okresu
        .action = MCPWM_GEN_ACTION_HIGH
    };
    
    mcpwm_gen_timer_event_action_t ev_action_down = {
        .direction = MCPWM_TIMER_DIRECTION_DOWN,
        .event = MCPWM_TIMER_EVENT_FULL,   // Koniec okresu
        .action = MCPWM_GEN_ACTION_LOW
    };

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_motor1, ev_action_up));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_motor1, ev_action_down));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_motor2, ev_action_up));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_motor2, ev_action_down));

    // Włączanie timerów
    ESP_ERROR_CHECK(mcpwm_timer_enable(motor1_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor1_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_ERROR_CHECK(mcpwm_timer_enable(motor2_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor2_timer, MCPWM_TIMER_START_NO_STOP));
}


void update_motor_pwm(uint32_t duty_motor1, uint32_t duty_motor2) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor1, duty_motor1));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor2, duty_motor2));
}
