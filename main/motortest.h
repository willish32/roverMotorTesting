#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"
#include "freertos/queue.h"


#define FL_PCNT_UNIT 0
#define FR_PCNT_UNIT 1
#define ML_PCNT_UNIT 2
#define MR_PCNT_UNIT 3
#define BL_PCNT_UNIT 4
#define BR_PCNT_UNIT 5


typedef struct {
    QueueHandle_t pid_queue;
    int FL_accumu_count;
    int ML_accumu_count;
    int BL_accumu_count;
    int FR_accumu_count;
    int MR_accumu_count;
    int BR_accumu_count;
    pcnt_unit_handle_t FL_pcnt_encoder;
    pcnt_unit_handle_t ML_pcnt_encoder;
    pcnt_unit_handle_t BL_pcnt_encoder;
    pcnt_unit_handle_t FR_pcnt_encoder;
    pcnt_unit_handle_t MR_pcnt_encoder;
    pcnt_unit_handle_t BR_pcnt_encoder;
} motor_ctrl_timer_context_t; //context struct for pcnt overrun cbs and encoder timer cb

typedef struct {
    int FL_pulses;
    int ML_pulses;
    int BL_pulses;
    int FR_pulses;
    int MR_pulses;
    int BR_pulses;
} pulse_count_t; //holder struct to pass counts from timer ISR to PID task through QueueHandle

typedef struct {
    QueueHandle_t pid_feedback_queue;
    // pid_ctrl_block_handle_t FL_pid_ctrl;
    // pid_ctrl_block_handle_t ML_pid_ctrl;
    // pid_ctrl_block_handle_t BL_pid_ctrl;
    // pid_ctrl_block_handle_t FR_pid_ctrl;
    // pid_ctrl_block_handle_t MR_pid_ctrl;
    // pid_ctrl_block_handle_t BR_pid_ctrl;
} motor_ctrl_task_context_t;

void gpio_initialize();
static void test_timer_init(motor_ctrl_timer_context_t *my_timer_ctx);
static bool FL_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx);
static bool ML_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx);
static bool BL_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx);
static bool FR_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx);
static bool MR_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx);
static bool BR_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx);
void pwm_pin_initialize();
void pwm_initialize();
void encoder_init(motor_ctrl_timer_context_t *my_timer_ctx);
void left_encoder_init(motor_ctrl_timer_context_t *my_timer_ctx);
void right_encoder_init(motor_ctrl_timer_context_t *my_timer_ctx);
static bool motor_ctrl_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
void set_direction_forward();
void set_direction_backward();
float calculate_duty_cycle(int pulse_counts, float duty);
static void set_duty_cycles();

