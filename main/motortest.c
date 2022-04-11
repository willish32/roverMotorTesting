#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
//#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "assignments.h"
#include "motortest.h"


#define GPIO_LEFT_PWM0A_OUT FL_MOTOR_PWM   //Set GPIO 32 as PWM0A -- Front Left forward
#define GPIO_LEFT_0_DIR FL_MOTOR_DIR   //Set GPIO 33 as PWM0B -- front left backward
#define GPIO_LEFT_PWM1A_OUT ML_MOTOR_PWM   //Set GPIO 25 as PWM1A -- middle left forward 
#define GPIO_LEFT_1_DIR ML_MOTOR_DIR   //Set GPIO 26 as PWM1B -- middle left backward
#define GPIO_LEFT_PWM2A_OUT BL_MOTOR_PWM   //Set GPIO 12 as PWM2A -- back left forward
#define GPIO_LEFT_2_DIR BL_MOTOR_DIR   //Set GPIO 13 as PWM2B -- back left backward
#define GPIO_RIGHT_PWM0A_OUT FR_MOTOR_PWM   //Set GPIO  as PWM0A -- Front right forward
#define GPIO_RIGHT_0_DIR FR_MOTOR_DIR   //Set GPIO  as PWM0B -- front right backward
#define GPIO_RIGHT_PWM1A_OUT MR_MOTOR_PWM  //Set GPIO  as PWM1A -- middle right backward
#define GPIO_RIGHT_1_DIR MR_MOTOR_DIR   //Set GPIO  as PWM1B -- middle right forward
#define GPIO_RIGHT_PWM2A_OUT BR_MOTOR_PWM   //Set GPIO  as PWM2A -- back right backward
#define GPIO_RIGHT_2_DIR BR_MOTOR_DIR   //Set GPIO  as PWM2B -- back right forward


//INDEX references for duty cycle global
#define FRONT_LEFT    0
#define MIDDLE_LEFT   1
#define BACK_LEFT     2
#define FRONT_RIGHT   3
#define MIDDLE_RIGHT  4
#define BACK_RIGHT    5
//INDEX references for set points
#define LEFT_SIDE     0
#define RIGHT_SIDE    1
//run each side of motors off a single 3 pwm unit
#define LEFT_MOTOR_UNIT MCPWM_UNIT_0
#define RIGHT_MOTOR_UNIT MCPWM_UNIT_1

//All pwm pins routed through operator A, see above
#define OPERATOR MCPWM_OPR_A

#define PID_CALCULATION_PERIOD_US 10000
#define PID_FEEDBACK_QUEUE_LEN 10

//direction masks
#define FORWARD 93
#define BACKWARD 123

#define TEST_SPEED 30.0
#define CTRL_PERIOD 10

#define SET_POINT_COUNTS_PER_PERIOD 15 //desired wheel encoder count per period WILL figure out what this is
#define INCREMENT_VALUE 1.0 //increment value for stop gap incremental controller --> will be supplanted by PIDs
#define RANGE 5 //acceptable range

#define BDC_PCNT_ENCODER_HIGH_LIMIT 100
#define BDC_PCNT_ENCODER_LOW_LIMIT -100
#define ENCODER_CORRECTION_FACTOR 1.2


 
float duty_cycles[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float set_point[2] = {50.0,50.0}; //motor RPM setpoint
float temp_set_point[2] = {0.0,0.0}; //temporary setpoint for changing direction
bool CHANGE_DIR_FLAG = false;


void init_all(motor_ctrl_timer_context_t *my_timer_ctx) {
  //setup direction pins
  gpio_initialize();
  //set up pwn pins
  pwm_pin_initialize();
  //set up pwm drivers
  pwm_initialize();
  //set up pcnt registers
  encoder_init(my_timer_ctx);
  //Initialize timer and interrupts
  test_timer_init(my_timer_ctx);
}


static void test_timer_init(motor_ctrl_timer_context_t *my_timer_ctx) {
  //init timer w/ desired config
  gptimer_handle_t gptimer;
  gptimer_config_t config = {
    .clk_src = GPTIMER_CLK_SRC_APB,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, // 1 MHz, 1 tick = 1 us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&config, &gptimer));

  //Register timer callback
  gptimer_event_callbacks_t gptimer_cbs = {
    .on_alarm = motor_ctrl_timer_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &gptimer_cbs, my_timer_ctx));

  //Configure callback alarm
  gptimer_alarm_config_t alarm_config = {
    .reload_count = 0,
    .alarm_count = PID_CALCULATION_PERIOD_US,
    .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  //start timer
  ESP_ERROR_CHECK(gptimer_start(gptimer));
}

//encoder stuff
static bool FL_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx)
{
  //cast user context into motor control context
  motor_ctrl_timer_context_t *timer_ctx = (motor_ctrl_timer_context_t *) user_ctx;
  //add count change into accumu count for motor
  timer_ctx->FL_accumu_count += edata->watch_point_value;
  return false;
}

static bool ML_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx)
{
  //cast user context into motor control context
  motor_ctrl_timer_context_t *timer_ctx = (motor_ctrl_timer_context_t *) user_ctx;
  //add count change into accumu count for motor
  timer_ctx->ML_accumu_count += edata->watch_point_value;
  return false;
}

static bool BL_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx)
{
  //cast user context into motor control context
  motor_ctrl_timer_context_t *timer_ctx = (motor_ctrl_timer_context_t *) user_ctx;
  //add count change into accumu count for motor
  timer_ctx->BL_accumu_count += edata->watch_point_value;
  return false;
}

static bool FR_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx)
{
  //cast user context into motor control context
  motor_ctrl_timer_context_t *timer_ctx = (motor_ctrl_timer_context_t *) user_ctx;
  //add count change into accumu count for motor
  timer_ctx->FR_accumu_count += edata->watch_point_value;
  return false;
}

static bool MR_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx)
{
  //cast user context into motor control context
  motor_ctrl_timer_context_t *timer_ctx = (motor_ctrl_timer_context_t *) user_ctx;
  //add count change into accumu count for motor
  timer_ctx->MR_accumu_count += edata->watch_point_value;
  return false;
}

static bool BR_pcnt_on_reach(pcnt_unit_handle_t unit, pcnt_watch_event_data_t *edata, void *user_ctx)
{
  //cast user context into motor control context
  motor_ctrl_timer_context_t *timer_ctx = (motor_ctrl_timer_context_t *) user_ctx;
  //add count change into accumu count for motor
  timer_ctx->BR_accumu_count += edata->watch_point_value;
  return false;
}

/*
  @brief - initialize GPIO pins for direction switching on new drivers
*/
void gpio_initialize() {
  //pin resets
  gpio_reset_pin(GPIO_LEFT_0_DIR);
  gpio_reset_pin(GPIO_LEFT_1_DIR);
  gpio_reset_pin(GPIO_LEFT_2_DIR);
  gpio_reset_pin(GPIO_RIGHT_0_DIR);
  gpio_reset_pin(GPIO_RIGHT_1_DIR);
  gpio_reset_pin(GPIO_RIGHT_2_DIR);
  //set all output
  gpio_set_direction(GPIO_LEFT_0_DIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_LEFT_1_DIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_LEFT_2_DIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_RIGHT_0_DIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_RIGHT_1_DIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_RIGHT_2_DIR, GPIO_MODE_OUTPUT);

  //adjust for direction (?)
}

void pwm_pin_initialize() {
  //Left motor IO pins
  mcpwm_gpio_init(LEFT_MOTOR_UNIT, MCPWM0A, GPIO_LEFT_PWM0A_OUT);
  
  mcpwm_gpio_init(LEFT_MOTOR_UNIT, MCPWM1A, GPIO_LEFT_PWM1A_OUT);
  
  mcpwm_gpio_init(LEFT_MOTOR_UNIT, MCPWM2A, GPIO_LEFT_PWM2A_OUT);
  
  
  //Right motor IO pins
  mcpwm_gpio_init(RIGHT_MOTOR_UNIT, MCPWM0A, GPIO_RIGHT_PWM0A_OUT);
  
  mcpwm_gpio_init(RIGHT_MOTOR_UNIT, MCPWM1A, GPIO_RIGHT_PWM1A_OUT);
  
  mcpwm_gpio_init(RIGHT_MOTOR_UNIT, MCPWM2A, GPIO_RIGHT_PWM2A_OUT);
  
}


void encoder_init(motor_ctrl_timer_context_t *my_timer_ctx) {
  left_encoder_init(my_timer_ctx);
  right_encoder_init(my_timer_ctx);
}

void left_encoder_init(motor_ctrl_timer_context_t *my_timer_ctx) {
  //make config struct
  pcnt_unit_config_t config = {
    .high_limit = 100,
    .low_limit = -100,
  };
  //establish actual unit handler object(s)?
  pcnt_unit_handle_t FL_pcnt_unit = NULL;
  pcnt_unit_handle_t ML_pcnt_unit = NULL;
  pcnt_unit_handle_t BL_pcnt_unit = NULL;

  //Actually make units for each encoder
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &FL_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &ML_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &BL_pcnt_unit));

  //set up glitch filters for each
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
  };

  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(FL_pcnt_unit, &filter_config));
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(ML_pcnt_unit, &filter_config));
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(BL_pcnt_unit, &filter_config));
  
  //Channel A config
  pcnt_chan_config_t FL_chan_a_config = {
    .edge_gpio_num = FL_ENCODER_A,
    .level_gpio_num = FL_ENCODER_B,
  };
  pcnt_chan_config_t ML_chan_a_config = {
    .edge_gpio_num = ML_ENCODER_A,
    .level_gpio_num = ML_ENCODER_B,
  };
  pcnt_chan_config_t BL_chan_a_config = {
    .edge_gpio_num = BL_ENCODER_A,
    .level_gpio_num = BL_ENCODER_B,
  };

  //Create actual A channels
  pcnt_channel_handle_t FL_pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(FL_pcnt_unit, &FL_chan_a_config, &FL_pcnt_chan_a));
  pcnt_channel_handle_t ML_pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(ML_pcnt_unit, &ML_chan_a_config, &ML_pcnt_chan_a));
  pcnt_channel_handle_t BL_pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(BL_pcnt_unit, &BL_chan_a_config, &BL_pcnt_chan_a));

  //channel B configs
  pcnt_chan_config_t FL_chan_b_config = {
    .edge_gpio_num = FL_ENCODER_B,
    .level_gpio_num = FL_ENCODER_A,
  };
  pcnt_chan_config_t ML_chan_b_config = {
    .edge_gpio_num = ML_ENCODER_B,
    .level_gpio_num = ML_ENCODER_A,
  };
  pcnt_chan_config_t BL_chan_b_config = {
    .edge_gpio_num = BL_ENCODER_B,
    .level_gpio_num = BL_ENCODER_A,
  };

  //create actual B channels
  pcnt_channel_handle_t FL_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(FL_pcnt_unit, &FL_chan_b_config, &FL_pcnt_chan_b));
  pcnt_channel_handle_t ML_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(ML_pcnt_unit, &ML_chan_b_config, &ML_pcnt_chan_b));
  pcnt_channel_handle_t BL_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(BL_pcnt_unit, &BL_chan_b_config, &BL_pcnt_chan_b));

  //Set up edge & channel events for each channel
  //A Channels:
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(FL_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ML_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(BL_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(FL_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ML_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(BL_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  //B channels:
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(FL_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ML_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(BL_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(FL_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ML_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(BL_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(FL_pcnt_unit, BDC_PCNT_ENCODER_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(FL_pcnt_unit, BDC_PCNT_ENCODER_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ML_pcnt_unit, BDC_PCNT_ENCODER_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ML_pcnt_unit, BDC_PCNT_ENCODER_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(BL_pcnt_unit, BDC_PCNT_ENCODER_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(BL_pcnt_unit, BDC_PCNT_ENCODER_LOW_LIMIT));
  
  pcnt_event_callbacks_t FL_pcnt_cbs = {
    .on_reach = FL_pcnt_on_reach,
  };
  pcnt_event_callbacks_t ML_pcnt_cbs = {
    .on_reach = ML_pcnt_on_reach,
  };
  pcnt_event_callbacks_t BL_pcnt_cbs = {
    .on_reach = BL_pcnt_on_reach,
  };

//-----NEED TO CHECK my_timer_ctx

  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(FL_pcnt_unit, &FL_pcnt_cbs, my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(ML_pcnt_unit, &ML_pcnt_cbs, my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(BL_pcnt_unit, &BL_pcnt_cbs, my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(FL_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(ML_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(BL_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(FL_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(ML_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(BL_pcnt_unit));


  //put units into timer context struct for callback pointer
  my_timer_ctx->FL_pcnt_encoder = FL_pcnt_unit;
  my_timer_ctx->ML_pcnt_encoder = ML_pcnt_unit;
  my_timer_ctx->BL_pcnt_encoder = BL_pcnt_unit;

}


//right side initialization
void right_encoder_init(motor_ctrl_timer_context_t *my_timer_ctx) {
pcnt_unit_config_t config = {
    .high_limit = 100,
    .low_limit = -100,
  };
  //establish actual unit handler object(s)?
  pcnt_unit_handle_t FR_pcnt_unit = NULL;
  pcnt_unit_handle_t MR_pcnt_unit = NULL;
  pcnt_unit_handle_t BR_pcnt_unit = NULL;

  //Actually make units for each encoder
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &FR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &MR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &BR_pcnt_unit));

  //set up glitch filters for each
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
  };

  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(FR_pcnt_unit, &filter_config));
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(MR_pcnt_unit, &filter_config));
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(BR_pcnt_unit, &filter_config));
  
  //Channel A config
  pcnt_chan_config_t FR_chan_a_config = {
    .edge_gpio_num = FR_ENCODER_A,
    .level_gpio_num = FR_ENCODER_B,
  };
  pcnt_chan_config_t MR_chan_a_config = {
    .edge_gpio_num = MR_ENCODER_A,
    .level_gpio_num = MR_ENCODER_B,
  };
  pcnt_chan_config_t BR_chan_a_config = {
    .edge_gpio_num = BR_ENCODER_A,
    .level_gpio_num = BR_ENCODER_B,
  };

  //Create actual A channels
  pcnt_channel_handle_t FR_pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(FR_pcnt_unit, &FR_chan_a_config, &FR_pcnt_chan_a));
  pcnt_channel_handle_t MR_pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(MR_pcnt_unit, &MR_chan_a_config, &MR_pcnt_chan_a));
  pcnt_channel_handle_t BR_pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(BR_pcnt_unit, &BR_chan_a_config, &BR_pcnt_chan_a));

  //channel B configs
  pcnt_chan_config_t FR_chan_b_config = {
    .edge_gpio_num = FR_ENCODER_B,
    .level_gpio_num = FR_ENCODER_A,
  };
  pcnt_chan_config_t MR_chan_b_config = {
    .edge_gpio_num = MR_ENCODER_B,
    .level_gpio_num = MR_ENCODER_A,
  };
  pcnt_chan_config_t BR_chan_b_config = {
    .edge_gpio_num = BR_ENCODER_B,
    .level_gpio_num = BR_ENCODER_A,
  };

  //create actual B channrels
  pcnt_channel_handle_t FR_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(FR_pcnt_unit, &FR_chan_b_config, &FR_pcnt_chan_b));
  pcnt_channel_handle_t MR_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(MR_pcnt_unit, &MR_chan_b_config, &MR_pcnt_chan_b));
  pcnt_channel_handle_t BR_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(BR_pcnt_unit, &BR_chan_b_config, &BR_pcnt_chan_b));


  //TODO: Verify directionality of these for right side vs. left side
  //Set up edge & channel events for each channel
  //A Channels:
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(FR_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(MR_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(BR_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(FR_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(MR_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(BR_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  //B channels:
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(FR_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(MR_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(BR_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(FR_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(MR_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(BR_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(FR_pcnt_unit, BDC_PCNT_ENCODER_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(FR_pcnt_unit, BDC_PCNT_ENCODER_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(MR_pcnt_unit, BDC_PCNT_ENCODER_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(MR_pcnt_unit, BDC_PCNT_ENCODER_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(BR_pcnt_unit, BDC_PCNT_ENCODER_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(BR_pcnt_unit, BDC_PCNT_ENCODER_LOW_LIMIT));
  
  pcnt_event_callbacks_t FR_pcnt_cbs = {
    .on_reach = FR_pcnt_on_reach,
  };
  pcnt_event_callbacks_t MR_pcnt_cbs = {
    .on_reach = MR_pcnt_on_reach,
  };
  pcnt_event_callbacks_t BR_pcnt_cbs = {
    .on_reach = BR_pcnt_on_reach,
  };

//-----NEED TO CHECK my_timer_ctx

  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(FR_pcnt_unit, &FR_pcnt_cbs, my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(MR_pcnt_unit, &MR_pcnt_cbs, my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(BR_pcnt_unit, &BR_pcnt_cbs, my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(FR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(MR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(BR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(FR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(MR_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(BR_pcnt_unit));

  my_timer_ctx->FR_pcnt_encoder = FR_pcnt_unit;
  my_timer_ctx->MR_pcnt_encoder = MR_pcnt_unit;
  my_timer_ctx->BR_pcnt_encoder = BR_pcnt_unit;
}

//setup PWM clocks
void pwm_initialize() {
  mcpwm_config_t pwm_config;
  //set initial settings
  pwm_config.frequency = 1000;
  pwm_config.cmpr_a = 0.0; //set motors to stop initially (0% duty cycle)
  pwm_config.cmpr_b = 0.0; //set second pin also to 0% cycle
  pwm_config.counter_mode = MCPWM_UP_COUNTER; // set timer to count up (I don't really know what changing this will do)
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; //set duty cycle to % active high

  //Set config onto all 3 "left" timer units, which default map onto the same numbered pwm operator unit
  mcpwm_init(LEFT_MOTOR_UNIT, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(LEFT_MOTOR_UNIT, MCPWM_TIMER_1, &pwm_config);
  mcpwm_init(LEFT_MOTOR_UNIT, MCPWM_TIMER_2, &pwm_config);

  // same for "right units"
  mcpwm_init(RIGHT_MOTOR_UNIT, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(RIGHT_MOTOR_UNIT, MCPWM_TIMER_1, &pwm_config);
  mcpwm_init(RIGHT_MOTOR_UNIT, MCPWM_TIMER_2, &pwm_config);
}

static void motor_ctrl_thread(void *arg) {
  motor_ctrl_task_context_t *user_ctx = (motor_ctrl_task_context_t *) arg;
  pulse_count_t actual_pulses = {};
  #define pulse_queue_size 10
  int FL_pulse_queue[pulse_queue_size] = {0};
  int ML_pulse_queue[pulse_queue_size] = {0};
  int BL_pulse_queue[pulse_queue_size] = {0};
  int FR_pulse_queue[pulse_queue_size] = {0};
  int MR_pulse_queue[pulse_queue_size] = {0};
  int BR_pulse_queue[pulse_queue_size] = {0};
  int ENCODER_SUM[6] = {0};
  float ENCODER_AVG[6] = {0.0};
  int i = 0;
  int j = 0;
  bool flag = false;
  float adjusted_length = 1.0;
  while(1) {
    xQueueReceive(user_ctx->pid_feedback_queue, &actual_pulses, portMAX_DELAY);
    if(i >= pulse_queue_size) {
      i = 0;
      flag = true;
    } 
      //subtract oldest encoder counts from sum
      ENCODER_SUM[FRONT_LEFT] = ENCODER_SUM[FRONT_LEFT] - FL_pulse_queue[i];
      ENCODER_SUM[MIDDLE_LEFT] = ENCODER_SUM[MIDDLE_LEFT] - ML_pulse_queue[i];
      ENCODER_SUM[BACK_LEFT] = ENCODER_SUM[BACK_LEFT] - BL_pulse_queue[i];
      ENCODER_SUM[FRONT_RIGHT] = ENCODER_SUM[FRONT_RIGHT] - FR_pulse_queue[i];
      ENCODER_SUM[MIDDLE_RIGHT] = ENCODER_SUM[MIDDLE_RIGHT] - MR_pulse_queue[i];
      ENCODER_SUM[BACK_RIGHT] = ENCODER_SUM[BACK_RIGHT] - BR_pulse_queue[i];

      //retrieve new encoder counts
      FL_pulse_queue[i] = actual_pulses.FL_pulses;
      ML_pulse_queue[i] = actual_pulses.ML_pulses;
      BL_pulse_queue[i] = actual_pulses.BL_pulses;
      FR_pulse_queue[i] = actual_pulses.FR_pulses;
      MR_pulse_queue[i] = actual_pulses.MR_pulses;
      BR_pulse_queue[i] = actual_pulses.BR_pulses;
      
      //calculate new sum
      ENCODER_SUM[FRONT_LEFT] += FL_pulse_queue[i];
      ENCODER_SUM[MIDDLE_LEFT] += ML_pulse_queue[i];
      ENCODER_SUM[BACK_LEFT] += BL_pulse_queue[i];
      ENCODER_SUM[FRONT_RIGHT] += FR_pulse_queue[i];
      ENCODER_SUM[MIDDLE_RIGHT] += MR_pulse_queue[i];
      ENCODER_SUM[BACK_RIGHT] += BR_pulse_queue[i];

      if(flag) {
        adjusted_length = (float)pulse_queue_size;
      } else {
        adjusted_length = (float)(i+1);
      }

      //calculate new average
      ENCODER_AVG[FRONT_LEFT] = (float)ENCODER_SUM[FRONT_LEFT]*ENCODER_CORRECTION_FACTOR/(adjusted_length);
      ENCODER_AVG[MIDDLE_LEFT] = (float)ENCODER_SUM[MIDDLE_LEFT]*ENCODER_CORRECTION_FACTOR/(adjusted_length);
      ENCODER_AVG[BACK_LEFT] = (float)ENCODER_SUM[BACK_LEFT]*ENCODER_CORRECTION_FACTOR/(adjusted_length);
      ENCODER_AVG[FRONT_RIGHT] = (float)ENCODER_SUM[FRONT_RIGHT]*ENCODER_CORRECTION_FACTOR/(adjusted_length);
      ENCODER_AVG[MIDDLE_RIGHT] = (float)ENCODER_SUM[MIDDLE_RIGHT]*ENCODER_CORRECTION_FACTOR/(adjusted_length);
      ENCODER_AVG[BACK_RIGHT] = (float)ENCODER_SUM[BACK_RIGHT]*ENCODER_CORRECTION_FACTOR/(adjusted_length);

      //printf("Front Left rpm: %f\n", (float)FL_avg*ENCODER_CORRECTION_FACTOR/((float)pulse_queue_size));
      //printf("Middle Left rpm: %f\n", (float)ML_avg*ENCODER_CORRECTION_FACTOR/((float)pulse_queue_size));
      //printf("Back Left rpm: %f\n", (float)BL_avg*ENCODER_CORRECTION_FACTOR/((float)pulse_queue_size));
      //printf("Front Right rpm: %f\n", (float)FR_avg*ENCODER_CORRECTION_FACTOR/((float)pulse_queue_size));
      //printf("Middle Right rpm: %f\n", (float)MR_avg*ENCODER_CORRECTION_FACTOR/((float)pulse_queue_size));
      //printf("Back Right rpm: %f\n", (float)BR_avg*ENCODER_CORRECTION_FACTOR/((float)pulse_queue_size));
      printf("%f,", ENCODER_AVG[FRONT_LEFT]);
      printf("%f,", ENCODER_AVG[MIDDLE_LEFT]);
      printf("%f,", ENCODER_AVG[BACK_LEFT]);
      printf("%f,", ENCODER_AVG[FRONT_RIGHT]);
      printf("%f,", ENCODER_AVG[MIDDLE_RIGHT]);
      printf("%f\n", ENCODER_AVG[BACK_RIGHT]);
      i++;
    if (j < 10) {
      j++;
      continue;
    }
    j = 0;
    //compare duty cycles to set point to check for direction change here
    //this may break, if it does we need to store previous setpoints... but i think it will be fine
    //if necessary we can offload some of this logic to the change_direction fx
    //only compare the FL and FR odoms for SPEED
    for (int w = 0; w < 4; w = w + 3){
      int k = 0;  //need a different index for set points
      //right now, the change direction fx is called when the setpoint has an opposite sign as the odom reading, and if odom is zero and the setpoint is nonzero
      if ((((set_point[k] > 0.0) != (ENCODER_AVG[w] > 0.0) && (set_point[k] != 0.0)) || ((ENCODER_AVG[w] == 0.0) && (set_point[k] != ENCODER_AVG[w]))) && (!CHANGE_DIR_FLAG)){
      CHANGE_DIR_FLAG = true;
      temp_set_point[LEFT_SIDE] = set_point[LEFT_SIDE];
      temp_set_point[RIGHT_SIDE] = set_point[RIGHT_SIDE];
      set_point[LEFT_SIDE] = 0.0;
      set_point[RIGHT_SIDE] = 0.0;
    }
    k++;
    }
    //only change direction once all encoders read 0
    if ((CHANGE_DIR_FLAG) && (ENCODER_AVG[FRONT_LEFT]==0.0) && (ENCODER_AVG[MIDDLE_LEFT]==0.0) && (ENCODER_AVG[BACK_LEFT]==0.0) && (ENCODER_AVG[FRONT_RIGHT]==0.0) && (ENCODER_AVG[MIDDLE_RIGHT]==0.0) && (ENCODER_AVG[BACK_RIGHT]==0.0)){
      change_direction(temp_set_point[LEFT_SIDE], temp_set_point[RIGHT_SIDE]);
    }

    //calculate new duty cycles
    duty_cycles[FRONT_LEFT] = calculate_duty_cycle(ENCODER_AVG[FRONT_LEFT], duty_cycles[FRONT_LEFT], set_point[LEFT_SIDE]);
    duty_cycles[MIDDLE_LEFT] = calculate_duty_cycle(ENCODER_AVG[MIDDLE_LEFT], duty_cycles[MIDDLE_LEFT], set_point[LEFT_SIDE]);
    duty_cycles[BACK_LEFT] = calculate_duty_cycle(ENCODER_AVG[BACK_LEFT], duty_cycles[BACK_LEFT], set_point[LEFT_SIDE]);
    duty_cycles[FRONT_RIGHT] = calculate_duty_cycle(ENCODER_AVG[FRONT_RIGHT], duty_cycles[FRONT_RIGHT], set_point[RIGHT_SIDE]);
    duty_cycles[MIDDLE_RIGHT] = calculate_duty_cycle(ENCODER_AVG[MIDDLE_RIGHT], duty_cycles[MIDDLE_RIGHT], set_point[RIGHT_SIDE]);
    duty_cycles[BACK_RIGHT] = calculate_duty_cycle(ENCODER_AVG[BACK_RIGHT], duty_cycles[BACK_RIGHT], set_point[RIGHT_SIDE]);
    set_duty_cycles();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

//we could add a direction flag to not run through the logic every time... might be important as im not sure
//if writing direction pins continuously while encoder is 0 would be good
void change_direction(float left_setpoint, float right_setpoint) {
  //stop unit
  //change direction
  //restart unit 
  //make sure all duty cycles are zero just to be sure... but dont call me shirley
  if ( 
  (mcpwm_get_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_0, OPERATOR) == 0.0) &&
  (mcpwm_get_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_1, OPERATOR) == 0.0) &&
  (mcpwm_get_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_2, OPERATOR) == 0.0) &&
  (mcpwm_get_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_0, OPERATOR) == 0.0) &&
  (mcpwm_get_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_1, OPERATOR) == 0.0) &&
  (mcpwm_get_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_2, OPERATOR) == 0.0)) {
    if ((left_setpoint > 0.0) && (right_setpoint < 0.0)){
      set_direction_right();
    }
    else if ((left_setpoint < 0.0) && (right_setpoint > 0.0)){
      set_direction_left();
    }
    else if ((left_setpoint < 0.0) && (right_setpoint < 0.0)){
      set_direction_backward();
    }
    else if ((left_setpoint >=0.0) && (right_setpoint >= 0.0)){
      set_direction_forward();
    }
    CHANGE_DIR_FLAG = false;
    set_point[LEFT_SIDE] = left_setpoint;
    set_point[RIGHT_SIDE] = right_setpoint;
  }
}

static void set_duty_cycles() {
  //TODO: fix directional blindness 
  //      - allow for duty cycles floats to be positive or negative, adjust in calculations by setting GPIO pins
  //      - check direction or set direction?  
  //set left side PWMs
  mcpwm_set_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_0, OPERATOR, duty_cycles[FRONT_LEFT]);
  mcpwm_set_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_1, OPERATOR, duty_cycles[MIDDLE_LEFT]);
  mcpwm_set_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_2, OPERATOR, duty_cycles[BACK_LEFT]);

  //set right side PWMs
  mcpwm_set_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_0, OPERATOR, duty_cycles[FRONT_RIGHT]);
  mcpwm_set_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_1, OPERATOR, duty_cycles[MIDDLE_RIGHT]);
  mcpwm_set_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_2, OPERATOR, duty_cycles[BACK_RIGHT]);

}

static bool motor_ctrl_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg)
{
  static int FL_last_pulse_count = 0;
  static int ML_last_pulse_count = 0;
  static int BL_last_pulse_count = 0;
  static int FR_last_pulse_count = 0;
  static int MR_last_pulse_count = 0;
  static int BR_last_pulse_count = 0;
  BaseType_t high_task_awoken = pdFALSE;
  motor_ctrl_timer_context_t *user_ctx = (motor_ctrl_timer_context_t *)arg;
  pcnt_unit_handle_t FL_pcnt_unit = user_ctx->FL_pcnt_encoder;
  pcnt_unit_handle_t ML_pcnt_unit = user_ctx->ML_pcnt_encoder;
  pcnt_unit_handle_t BL_pcnt_unit = user_ctx->BL_pcnt_encoder;
  pcnt_unit_handle_t FR_pcnt_unit = user_ctx->FR_pcnt_encoder;
  pcnt_unit_handle_t MR_pcnt_unit = user_ctx->MR_pcnt_encoder;
  pcnt_unit_handle_t BR_pcnt_unit = user_ctx->BR_pcnt_encoder;

  int FL_curr_pulse_count = 0;  //may be able to recycle thruout fx
  pcnt_unit_get_count(FL_pcnt_unit, &FL_curr_pulse_count);
  FL_curr_pulse_count += user_ctx->FL_accumu_count; //need to look into this
  int ML_curr_pulse_count = 0;
  pcnt_unit_get_count(ML_pcnt_unit, &ML_curr_pulse_count);
  ML_curr_pulse_count += user_ctx->ML_accumu_count;
  int BL_curr_pulse_count = 0;
  pcnt_unit_get_count(BL_pcnt_unit, &BL_curr_pulse_count);
  BL_curr_pulse_count += user_ctx->BL_accumu_count;
  int FR_curr_pulse_count = 0;  //may be able to recycle thruout fx
  pcnt_unit_get_count(FR_pcnt_unit, &FR_curr_pulse_count);
  FR_curr_pulse_count += user_ctx->FR_accumu_count; //need to look into this
  int MR_curr_pulse_count = 0;
  pcnt_unit_get_count(MR_pcnt_unit, &MR_curr_pulse_count);
  MR_curr_pulse_count += user_ctx->MR_accumu_count;
  int BR_curr_pulse_count = 0;
  pcnt_unit_get_count(BR_pcnt_unit, &BR_curr_pulse_count);
  BR_curr_pulse_count += user_ctx->BR_accumu_count;

  // Load pulse counts from this check into passing struct, set last pulse count to current pulse count for next calculation
  pulse_count_t resp;
  resp.FL_pulses = FL_curr_pulse_count - FL_last_pulse_count; //recycled delta
  FL_last_pulse_count = FL_curr_pulse_count;
  resp.ML_pulses = ML_curr_pulse_count - ML_last_pulse_count;
  ML_last_pulse_count = ML_curr_pulse_count;
  resp.BL_pulses = BL_curr_pulse_count - BL_last_pulse_count;
  BL_last_pulse_count = BL_curr_pulse_count;
  resp.FR_pulses = FR_curr_pulse_count - FR_last_pulse_count;
  FR_last_pulse_count = FR_curr_pulse_count;
  resp.MR_pulses = MR_curr_pulse_count - MR_last_pulse_count;
  MR_last_pulse_count = MR_curr_pulse_count;
  resp.BR_pulses = BR_curr_pulse_count - BR_last_pulse_count;
  BR_last_pulse_count = BR_curr_pulse_count;
  

  //send data into queue for retrieval by PID
  xQueueSendFromISR(user_ctx->pid_queue, &resp, &high_task_awoken);
  //task management response for RTOS
  return high_task_awoken == pdTRUE;
}

void set_direction_forward() {
  //fix -- change to #defined flags
  gpio_set_level(GPIO_LEFT_0_DIR, 1);
  gpio_set_level(GPIO_LEFT_1_DIR, 1);
  gpio_set_level(GPIO_LEFT_2_DIR, 1);
  gpio_set_level(GPIO_RIGHT_0_DIR, 0);
  gpio_set_level(GPIO_RIGHT_1_DIR, 0);
  gpio_set_level(GPIO_RIGHT_2_DIR, 0);
}

void set_direction_backward() {
  gpio_set_level(GPIO_LEFT_0_DIR, 0);
  gpio_set_level(GPIO_LEFT_1_DIR, 0);
  gpio_set_level(GPIO_LEFT_2_DIR, 0);
  gpio_set_level(GPIO_RIGHT_0_DIR, 1);
  gpio_set_level(GPIO_RIGHT_1_DIR, 1);
  gpio_set_level(GPIO_RIGHT_2_DIR, 1);
}

void set_direction_left() {
  gpio_set_level(GPIO_LEFT_0_DIR, 0);
  gpio_set_level(GPIO_LEFT_1_DIR, 0);
  gpio_set_level(GPIO_LEFT_2_DIR, 0);
  gpio_set_level(GPIO_RIGHT_0_DIR, 0);
  gpio_set_level(GPIO_RIGHT_1_DIR, 0);
  gpio_set_level(GPIO_RIGHT_2_DIR, 0);
}

void set_direction_right() {
  gpio_set_level(GPIO_LEFT_0_DIR, 1);
  gpio_set_level(GPIO_LEFT_1_DIR, 1);
  gpio_set_level(GPIO_LEFT_2_DIR, 1);
  gpio_set_level(GPIO_RIGHT_0_DIR, 1);
  gpio_set_level(GPIO_RIGHT_1_DIR, 1);
  gpio_set_level(GPIO_RIGHT_2_DIR, 1);
}

float calculate_duty_cycle(float pulse_counts, float duty, float set_point) {  //probs going to need to change pulse_counts to a float
  //#define set_point 50.0  //placeholder cause im lazy
  #define PAD 4.0  //+- for RPM acceptance
  #define MAX_RPM 60.0  //maximum allowable RPM

  //incremental control
  if((abs(pulse_counts) < (set_point + PAD)) && (abs(pulse_counts) > set_point - PAD)){
    return duty;
  }
  if(abs(pulse_counts) > (set_point + PAD)){
    if ((duty - 1.0) <= 0.0){
      return 0.0;
    }
    return duty - 1.0;
  }
  if (abs(pulse_counts) < (set_point - PAD)){
    if ((duty + 1.0) >= MAX_RPM){
      return duty;
    }
    return duty + 1.0;
  }

  return duty;
}
  
/*
 - Rewrite encoder initializers, potentially accessor interrupt functions
 - Add in PID functionalilty
 - Add in rosserial interface, set point injection

*/

void app_main(void)
{
  QueueHandle_t pid_fb_queue = xQueueCreate(PID_FEEDBACK_QUEUE_LEN, sizeof(pulse_count_t));
  assert(pid_fb_queue);
  
  static motor_ctrl_timer_context_t my_timer_ctx = {};
  my_timer_ctx.pid_queue = pid_fb_queue;
  init_all(&my_timer_ctx);
  //for test, hard set direction to forward for both sides
  set_direction_forward();
  //set_direction_backward();
  //set_direction_left();
  //set_direction_right();
  
  static motor_ctrl_task_context_t my_task_ctx = {};
  my_task_ctx.pid_feedback_queue = pid_fb_queue;
  // gpio_set_level(GPIO_LEFT_0_DIR, 1);
  // gpio_set_level(GPIO_LEFT_1_DIR, 1);
  // gpio_set_level(GPIO_LEFT_2_DIR, 1);
  // gpio_set_level(GPIO_RIGHT_0_DIR, 0);
  // gpio_set_level(GPIO_RIGHT_1_DIR, 0);
  // gpio_set_level(GPIO_RIGHT_2_DIR, 0);
  xTaskCreate(motor_ctrl_thread, "motor_ctrl_thread", 4096, &my_task_ctx, 5, NULL);
  //TODO: add thread to set expected pulses 
}