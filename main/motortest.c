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
#include "driver/pcnt.h"

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
//run each side of motors off a single 3 pwm unit
#define LEFT_MOTOR_UNIT MCPWM_UNIT_0
#define RIGHT_MOTOR_UNIT MCPWM_UNIT_1

//All pwm pins routed through operator A, see above
#define OPERATOR MCPWM_OPR_A


//direction masks
#define FORWARD 93
#define BACKWARD 123

#define TEST_SPEED 30.0
#define CTRL_PERIOD 10

#define SET_POINT_COUNTS_PER_PERIOD 15 //desired wheel encoder count per period WILL figure out what this is
#define INCREMENT_VALUE 1.0 //increment value for stop gap incremental controller --> will be supplanted by PIDs
#define RANGE 5 //acceptable range


static motor_control_t motor_ctrl;

// 
float duty_cycles[6] = {0.0,0.0,0.0,0.0,0.0,0.0};


void init_all() {
  //setup direction pins
  gpio_initialize();
  //set up pwn pins
  pwm_pin_initialize();
  //set up pwm drivers
  pwm_initialize();
  //set up pcnt registers
  //encoder_initialize();
  //Initialize timer and interrupts
  //test_timer_init();
}


//write pulse getter callback func -- TODO: change to match changes in encoder code 
static int * pcnt_get_pulses_callback(void *args) {
  //static ints get held across func calls, these will store value even after scope exit
  static unsigned int fl_pulses = 0;
  static unsigned int fr_pulses = 0;
  static unsigned int ml_pulses = 0;
  static unsigned int mr_pulses = 0;
  static unsigned int bl_pulses = 0;
  static unsigned int br_pulses = 0;
  static int ret[6];

  //extract actual data from args struct w/ casting
  all_encoder_t *encoders = (all_encoder_t *)args;
  
  //get current counpulse_countt
  unsigned int fl_temp = encoders->fl_encoder->get_counter_value(encoders->fl_encoder);
  unsigned int fr_temp = encoders->fr_encoder->get_counter_value(encoders->fr_encoder);
  unsigned int ml_temp = encoders->ml_encoder->get_counter_value(encoders->ml_encoder);
  unsigned int mr_temp = encoders->mr_encoder->get_counter_value(encoders->mr_encoder);
  unsigned int bl_temp = encoders->bl_encoder->get_counter_value(encoders->bl_encoder);
  unsigned int br_temp = encoders->br_encoder->get_counter_value(encoders->br_encoder);
  
  //fill data into return array that will be parsed into timer_event_struct, processed in thread loop
  ret[0] = abs(fl_temp - fl_pulses);
  ret[1] = abs(fr_temp - fr_pulses);
  ret[2] = abs(ml_temp - ml_pulses);
  ret[3] = abs(mr_temp - mr_pulses);
  ret[4] = abs(bl_temp - bl_pulses);
  ret[5] = abs(br_temp - br_pulses);
  return ret;
}

static void test_timer_init() {
  //timer alarm event queue
  motor_ctrl.timer_evt_que = xQueueCreate(10, sizeof(motor_ctrl_timer_info_t));

  pulse_info_t pulse_info = 
  {
    .callback_args = motor_ctrl.encoders,
    .get_pulse_callback = pcnt_get_pulses_callback
  };
  motor_ctrl_new_timer(&motor_ctrl.timer_info, motor_ctrl.timer_evt_que, CTRL_PERIOD, pulse_info);
}

//

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

// //initialize PCNT rotary encoders
void encoder_initialize() {
  //make configs
  rotary_encoder_config_t fl_config = ROTARY_ENCODER_DEFAULT_CONFIG(
                                    (rotary_encoder_dev_t)FL_PCNT_UNIT,
                                    FL_ENCODER_A, FL_ENCODER_B);
  rotary_encoder_config_t fr_config = ROTARY_ENCODER_DEFAULT_CONFIG(
                                    (rotary_encoder_dev_t)FR_PCNT_UNIT,
                                    FR_ENCODER_A, FR_ENCODER_B);
  rotary_encoder_config_t ml_config = ROTARY_ENCODER_DEFAULT_CONFIG(
                                    (rotary_encoder_dev_t)ML_PCNT_UNIT,
                                    ML_ENCODER_A, ML_ENCODER_B);
  rotary_encoder_config_t mr_config = ROTARY_ENCODER_DEFAULT_CONFIG(
                                    (rotary_encoder_dev_t)MR_PCNT_UNIT,
                                    MR_ENCODER_A, MR_ENCODER_B);
  rotary_encoder_config_t bl_config = ROTARY_ENCODER_DEFAULT_CONFIG(
                                    (rotary_encoder_dev_t)BL_PCNT_UNIT,
                                    BL_ENCODER_A, BL_ENCODER_B);
  rotary_encoder_config_t br_config = ROTARY_ENCODER_DEFAULT_CONFIG(
                                    (rotary_encoder_dev_t)BR_PCNT_UNIT,
                                    BR_ENCODER_A, BR_ENCODER_B);

  //stand up encoder objects with configs                                  
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&fl_config, &motor_ctrl.encoders->fl_encoder));
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&fr_config, &motor_ctrl.encoders->fr_encoder));                                
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&ml_config, &motor_ctrl.encoders->ml_encoder));
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&mr_config, &motor_ctrl.encoders->mr_encoder));
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&bl_config, &motor_ctrl.encoders->bl_encoder));
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&br_config, &motor_ctrl.encoders->br_encoder));

  //set up glitch filters - 1 us 
  ESP_ERROR_CHECK(motor_ctrl.encoders->fl_encoder->set_glitch_filter(motor_ctrl.encoders->fl_encoder, 1));
  ESP_ERROR_CHECK(motor_ctrl.encoders->fr_encoder->set_glitch_filter(motor_ctrl.encoders->fr_encoder, 1));
  ESP_ERROR_CHECK(motor_ctrl.encoders->ml_encoder->set_glitch_filter(motor_ctrl.encoders->ml_encoder, 1));
  ESP_ERROR_CHECK(motor_ctrl.encoders->mr_encoder->set_glitch_filter(motor_ctrl.encoders->mr_encoder, 1));
  ESP_ERROR_CHECK(motor_ctrl.encoders->bl_encoder->set_glitch_filter(motor_ctrl.encoders->bl_encoder, 1));
  ESP_ERROR_CHECK(motor_ctrl.encoders->br_encoder->set_glitch_filter(motor_ctrl.encoders->br_encoder, 1));

  //start encoders
  ESP_ERROR_CHECK(motor_ctrl.encoders->fl_encoder->start(motor_ctrl.encoders->fl_encoder));
  ESP_ERROR_CHECK(motor_ctrl.encoders->fr_encoder->start(motor_ctrl.encoders->fr_encoder));
  ESP_ERROR_CHECK(motor_ctrl.encoders->ml_encoder->start(motor_ctrl.encoders->ml_encoder));
  ESP_ERROR_CHECK(motor_ctrl.encoders->mr_encoder->start(motor_ctrl.encoders->mr_encoder));
  ESP_ERROR_CHECK(motor_ctrl.encoders->bl_encoder->start(motor_ctrl.encoders->bl_encoder));
  ESP_ERROR_CHECK(motor_ctrl.encoders->br_encoder->start(motor_ctrl.encoders->br_encoder));

  //clear PCNTs
  pcnt_counter_clear((pcnt_unit_t)FL_PCNT_UNIT);
  pcnt_counter_clear((pcnt_unit_t)FR_PCNT_UNIT);
  pcnt_counter_clear((pcnt_unit_t)ML_PCNT_UNIT);
  pcnt_counter_clear((pcnt_unit_t)MR_PCNT_UNIT);  
  pcnt_counter_clear((pcnt_unit_t)BL_PCNT_UNIT);
  pcnt_counter_clear((pcnt_unit_t)BR_PCNT_UNIT);
}

void encoder_init() {
  left_encoder_init();
  right_encoder_init();
}

void left_encoder_init() {
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
  ESP_ERROR_CHECK(pcnt_new_channel(FL_pcnt_unit, &FL_chan_b_config, FL_pcnt_chan_b));
  pcnt_channel_handle_t ML_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(ML_pcnt_unit, &ML_chan_b_config, ML_pcnt_chan_b));
  pcnt_channel_handle_t FL_pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(BL_pcnt_unit, &BL_chan_b_config, BL_pcnt_chan_b));

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

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(FL_pcnt_unit, BDC_ENCODER_PCNT_HIGHT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(FL_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ML_pcnt_unit, BDC_ENCODER_PCNT_HIGHT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ML_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(BL_pcnt_unit, BDC_ENCODER_PCNT_HIGHT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(BL_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
  
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

  ESP_ERROR_CHECK(FL_pcnt_unit_register_event_callbacks(FL_pcnt_unit, &pcnt_cbs, &my_timer_ctx));
  ESP_ERROR_CHECK(ML_pcnt_unit_register_event_callbacks(ML_pcnt_unit, &pcnt_cbs, &my_timer_ctx));
  ESP_ERROR_CHECK(ML_pcnt_unit_register_event_callbacks(BL_pcnt_unit, &pcnt_cbs, &my_timer_ctx));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(FL_PCNT_UNIT));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(ML_PCNT_UNIT));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(BL_PCNT_UNIT));
  ESP_ERROR_CHECK(pcnt_unit_start(FL_PCNT_UNIT));
  ESP_ERROR_CHECK(pcnt_unit_start(ML_PCNT_UNIT));
  ESP_ERROR_CHECK(pcnt_unit_start(BL_PCNT_UNIT));
}

// void right_encoder_init() {

// }

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
  motor_ctrl_timer_info_t recv_info;
  while(1) {
    //xQueueReceive(motor_ctrl.timer_evt_que, &recv_info, portMAX_DELAY);
    //can't assign arrays in C, copies information from queue struct into ctrl struct
    //memcpy(motor_ctrl.pulses_in_one_period, recv_info.pulse_info.pulse_counts, 6);

    //compare all pulse counts to setpoint, increment undershooters, decrement overshooters, flatten equivs
    printf("BL duty (1): %f\n", duty_cycles[BACK_LEFT]);
    for(int i = 0; i <= 5; i++) {
      duty_cycles[i] = calculate_duty_cycle(motor_ctrl.pulses_in_one_period[i], duty_cycles[i]);
    }
    printf("BL duty (2): %f\n", duty_cycles[BACK_LEFT]);
    set_duty_cycles();
    printf("BL duty (3): %f\n", duty_cycles[BACK_LEFT]);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

static void set_duty_cycles() {
  //set left side PWMs
  mcpwm_set_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_0, OPERATOR, duty_cycles[FRONT_LEFT]);
  mcpwm_set_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_1, OPERATOR, duty_cycles[MIDDLE_LEFT]);
  mcpwm_set_duty(LEFT_MOTOR_UNIT, MCPWM_TIMER_2, OPERATOR, duty_cycles[BACK_LEFT]);

  //set right side PWMs
  mcpwm_set_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_0, OPERATOR, duty_cycles[FRONT_RIGHT]);
  mcpwm_set_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_1, OPERATOR, duty_cycles[MIDDLE_RIGHT]);
  mcpwm_set_duty(RIGHT_MOTOR_UNIT, MCPWM_TIMER_2, OPERATOR, duty_cycles[BACK_RIGHT]);

}

void set_direction_forward() {
  //fix -- change to #defined flags
  gpio_set_level(GPIO_LEFT_0_DIR, 0);
  gpio_set_level(GPIO_LEFT_1_DIR, 0);
  gpio_set_level(GPIO_LEFT_2_DIR, 0);
  gpio_set_level(GPIO_RIGHT_0_DIR, 1);
  gpio_set_level(GPIO_RIGHT_1_DIR, 1);
  gpio_set_level(GPIO_RIGHT_2_DIR, 1);
}

void set_direction_backward() {
  gpio_set_level(GPIO_LEFT_0_DIR, 1);
  gpio_set_level(GPIO_LEFT_1_DIR, 1);
  gpio_set_level(GPIO_LEFT_2_DIR, 1);
  gpio_set_level(GPIO_RIGHT_0_DIR, 0);
  gpio_set_level(GPIO_RIGHT_1_DIR, 0);
  gpio_set_level(GPIO_RIGHT_2_DIR, 0);
}

float calculate_duty_cycle(int pulse_counts, float duty) {
  //return 15.0;
  if(duty >= 35.0) {
    return 35.0;
  }
  return duty + 1.0;
}
  
/*
 - Rewrite encoder initializers, potentially accessor interrupt functions
 - Add in PID functionalilty
 - Add in rosserial interface, set point injection

*/

void app_main(void)
{
  printf("Beginning Motor Test\n");
  printf("Encoder Tick target per control period: %d", SET_POINT_COUNTS_PER_PERIOD);
  init_all();
  //for test, hard set direction to forward for both sides
  //set_direction_forward();
  set_direction_backward();
  xTaskCreate(motor_ctrl_thread, "motor_ctrl_thread", 4096, NULL, 3, NULL);
}