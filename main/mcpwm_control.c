#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"

#define MOTOR1_DIR_PIN 4
#define MOTOR1_PWM_PIN 5
#define MOTOR2_DIR_PIN 6
#define MOTOR2_PWM_PIN 7
#define MCPWM_TIMER_RESOLUTION_HZ 10000000  // 10 MHz = 1 µs per tick
#define MCPWM_TIMEBASE_PERIOD 1000          // 1000 ticks, 20ms

#define PWM_MAX 1000
#define PWM_MIN -1000

#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(a) ((a) > 0 ? (a) : -(a))

static const char *TAG = "RC_PWM";
mcpwm_cmpr_handle_t comparator_motor1;
mcpwm_cmpr_handle_t comparator_motor2;

// Function to initialize motor PWM control
void setup_motor_pwm() {
    // Configure motor control pins
    gpio_set_direction(MOTOR1_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR1_PWM_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR2_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR2_PWM_PIN, GPIO_MODE_OUTPUT);

    // MCPWM timer configuration
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MWDT_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = MCPWM_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    
    // Motor 1 (left motor)
    mcpwm_oper_handle_t operator_motor1;
    mcpwm_timer_handle_t motor1_timer;
    mcpwm_operator_config_t operator_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor1_timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_motor1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_motor1, motor1_timer));

    mcpwm_comparator_config_t cmp_config = { .flags.update_cmp_on_tep = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_motor1, &cmp_config, &comparator_motor1));

    mcpwm_generator_config_t gen_config = { .gen_gpio_num = MOTOR1_PWM_PIN };
    mcpwm_gen_handle_t generator_motor1;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_motor1, &gen_config, &generator_motor1));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor1, 0));

    // Motor 2 (right motor)
    mcpwm_oper_handle_t operator_motor2;
    mcpwm_timer_handle_t motor2_timer;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor2_timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_motor2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_motor2, motor2_timer));
    
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_motor2, &cmp_config, &comparator_motor2));

    mcpwm_generator_config_t gen_config_motor2 = { .gen_gpio_num = MOTOR2_PWM_PIN };
    mcpwm_gen_handle_t generator_motor2;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_motor2, &gen_config_motor2, &generator_motor2));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor2, 0));

    // Set PWM actions
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_motor1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_motor1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_motor1, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_motor2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_motor2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_motor2, MCPWM_GEN_ACTION_LOW)));

    // Enable and start timers
    ESP_ERROR_CHECK(mcpwm_timer_enable(motor1_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor1_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_ERROR_CHECK(mcpwm_timer_enable(motor2_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor2_timer, MCPWM_TIMER_START_NO_STOP));
}

// Update PWM duty cycle for both motors
void update_motor_pwm(uint32_t duty_motor1, uint32_t duty_motor2) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor1, duty_motor1));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor2, duty_motor2));
}

// Function to clamp PWM values within the specified range
int clamp_pwm_value(int value, int min, int max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// Function to control motors based on throttle and steering input
void control_motors(int throttle, int steering) {
    if (throttle < 1000 || throttle > 2000) {
        ESP_LOGE(TAG, "Throttle value out of range: %d", throttle);
        return;
    }

    if (steering < 1000 || steering > 2000) {
        ESP_LOGE(TAG, "Steering value out of range: %d", steering);
        return;
    }

    ESP_LOGI(TAG, "Throttle: %d, Steering: %d", throttle, steering);

    // Normalize throttle and steering values to -1000 to 1000
    throttle = (throttle - 1500) * 2;
    steering = (steering - 1500) * 2;

    throttle = clamp_pwm_value(throttle, PWM_MIN, PWM_MAX);
    steering = clamp_pwm_value(steering, PWM_MIN, PWM_MAX);

    // Stop motors if values are too small
    if (abs(throttle) < 20) throttle = 0;
    if (abs(steering) < 20) steering = 0;

    // Calculate PWM for left and right motors
    int pwm_left = throttle + steering;
    int pwm_right = throttle - steering;

    // Scale PWM values if exceeding the allowable range
    int max_pwm = max(abs(pwm_left), abs(pwm_right));
    if (max_pwm > PWM_MAX) {
        pwm_left = (pwm_left * PWM_MAX) / max_pwm;
        pwm_right = (pwm_right * PWM_MAX) / max_pwm;
    }

    ESP_LOGI(TAG, "PWM Left: %d, PWM Right: %d", pwm_left, pwm_right);

    // Set motor direction and PWM
    gpio_set_level(MOTOR1_DIR_PIN, pwm_left > 0 ? 0 : 1);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor1, abs(pwm_left)));

    gpio_set_level(MOTOR2_DIR_PIN, pwm_right > 0 ? 0 : 1);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_motor2, abs(pwm_right)));
}
