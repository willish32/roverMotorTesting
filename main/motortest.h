#include "rotary_encoder.h"
#include "motor_ctrl_timer.h"
#define FL_PCNT_UNIT 0
#define FR_PCNT_UNIT 1
#define ML_PCNT_UNIT 2
#define MR_PCNT_UNIT 3
#define BL_PCNT_UNIT 4
#define BR_PCNT_UNIT 5

//wrapper struct to pass all encoders at once through Timer ISR
typedef struct {
    rotary_encoder_t *fr_encoder;
    rotary_encoder_t *fl_encoder;
    rotary_encoder_t *mr_encoder;
    rotary_encoder_t *ml_encoder;
    rotary_encoder_t *br_encoder;
    rotary_encoder_t *bl_encoder;
} all_encoder_t;

typedef struct {
    all_encoder_t *encoders;                //encoders
    motor_ctrl_timer_info_t *timer_info;    //timer stuff
    QueueHandle_t timer_evt_que;            //queue to pass timer data (pulse counts) across threads
    int pulses_in_one_period[6];    //holder for timer ISR feedback data in thread loop
    int FL_accumu_count;
    int ML_accumu_count;
    int BL_accumu_count;
    int FR_accumu_count;
    int MR_accumu_count;
    int BR_accumu_count;

} motor_control_t;

void gpio_initialize();
static void test_timer_init();
void pwm_pin_initialize();
void pwm_initialize();
void encoder_initialize();
float calculate_duty_cycle(int pulse_counts, float duty);
static void set_duty_cycles();

