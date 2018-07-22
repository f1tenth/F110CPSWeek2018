#include <Arduino.h>
//#include <Bounce2.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <rericha_racing/drive_values.h>
#include <rericha_racing/pwm_high.h>

/* Input capture FTM register values */
#define FTM_SC_VALUE (FTM_SC_TOIE | FTM_SC_CLKS(1) | FTM_SC_PS(0))
#define FTM_CSC_RAISING (FTM_CSC_CHIE | FTM_CSC_ELSA)
#define FTM_CSC_FALLING (FTM_CSC_CHIE | FTM_CSC_ELSB)

/* Input capture helper variables */
static volatile uint32_t duty_cycle_c0 = 0;
static volatile uint32_t duty_cycle_c1 = 0;
static volatile uint32_t c0_done = 0;
static volatile uint32_t c1_done = 0;

f1tenth_race::pwm_high pwm_high_msg;

/* Forward declarations */
void messageDrive(const f1tenth_race::drive_values& pwm);
void messageEmergencyStop(const std_msgs::Bool& flag);

static volatile boolean flagStop = false;
static volatile boolean flagManualOverride = false;

/* Measured values (V2):
    TRIM STEERING - CALM STATE
        left: 8247, right: 9622
        => str_center_value = avg() = 8934; using 9361
        => str boundaries: using corrected center +- 300

    TRIM THROTTLE - CALM STATE
        backward: 9618, forward: 8240
        => thr_center_value = avg() = 8929; not driving for +- 250
        => thr boundaries: using center +- 200

    TRIM STEERING left
        RATE MIN
            left: 7799, right: 8950
        RATE MAX
            left: 6000, right: 11674
        => str_lowerlimit = 6000

    TRIM STEERING right
        RATE MIN
            left: 8901, right: 10600
        RATE MAX
            left: 6014, right: 11764
        => str_upperlimit = 11764

    TRIM THROTTLE backward
        backward: 11312, forward: 6567
            thr_upperlimit = 11312

    TRIM THROTTLE forward
        backward: 11312, forward: 6567
            thr_lowerlimit = 6567
*/

/* Measured important PWM duty cycle values */
#define PWM_FREQUENCY	91

#define pwm_str_center_value  9361  //  15% duty cycle - V1 8611
#define pwm_str_lowerlimit    6000  //  10% duty cycle - V1 7000
#define pwm_str_upperlimit   11764  //  20% duty cycle - V1 10684

#define pwm_thr_center_value  8929  //  15% duty cycle - V1 8850
#define pwm_thr_lowerlimit    6567  //  10% duty cycle - V1 6458
#define pwm_thr_upperlimit   11312  //  20% duty cycle - V1 11241

/* ROS Publishers/Subscribers */
ros::NodeHandle  nh;

ros::Subscriber<f1tenth_race::drive_values> sub_drive("drive_pwm", &messageDrive );
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );
ros::Publisher pwm_high("pwm_high", &pwm_high_msg);

elapsedMillis modeSwitchBtnElapsed;
elapsedMillis manual_blink_elapsed;
elapsedMillis drive_msg_recv_elapsed;
elapsedMillis ftm_irq_elapsed;

void messageDrive(const f1tenth_race::drive_values& pwm) {

  if(flagStop == false && flagManualOverride == false) {
    if(pwm.pwm_drive < pwm_thr_lowerlimit) {
      analogWrite(5,pwm_thr_lowerlimit);    //  Safety lower limit
    } else if(pwm.pwm_drive > pwm_thr_upperlimit) {
      analogWrite(5,pwm_thr_upperlimit);    //  Safety upper limit
    } else {
      analogWrite(5,pwm.pwm_drive);     //  Incoming data
    }

    if(pwm.pwm_angle < pwm_str_lowerlimit) {
      analogWrite(6,pwm_str_lowerlimit);    //  Safety lower limit
    } else if(pwm.pwm_angle > pwm_str_upperlimit) {
      analogWrite(6,pwm_str_upperlimit);    //  Safety upper limit
    } else {
      analogWrite(6,pwm.pwm_angle);     //  Incoming data
    }

    drive_msg_recv_elapsed = 0;
  }
}

void messageEmergencyStop(const std_msgs::Bool& flag) {
  flagStop = flag.data;

  if(flagStop == true && flagManualOverride == false) {
    digitalWrite(13,HIGH);

    analogWrite(6,pwm_str_center_value);
    analogWrite(5,pwm_thr_center_value);
  }

  if(flagStop == false) {
    flagStop = false;
    flagManualOverride = false;
    digitalWrite(13,LOW);
  }
}

void ftm1_isr(void) {
  static uint16_t capture_ovf_bits = 0;
  static uint32_t prev_cap_c0 = 0;
  static uint32_t prev_cap_c1 = 0;
  static bool next_edge_falling_c0 = false;
  static bool next_edge_falling_c1 = false;
  static int LED_state = LOW;

  bool overflowed = false;
  uint32_t cap_val, status_reg_copy = FTM1_STATUS;
  uint32_t pwm_high_c0, pwm_high_c1;

  if (FTM1_SC & FTM_SC_TOF) {
    FTM1_SC = FTM_SC_VALUE;
    capture_ovf_bits++;
    overflowed = true;
  }

  /* Event on channel 0 */
  if (status_reg_copy & 0b01) {
    cap_val = FTM1_C0V;
    if (cap_val <= 0x8000 || !overflowed)
      cap_val |= capture_ovf_bits << 16;
    else
      cap_val |= (capture_ovf_bits - 1) << 16;
    next_edge_falling_c0 = !next_edge_falling_c0;
    FTM1_C0SC = next_edge_falling_c0 ? FTM_CSC_FALLING : FTM_CSC_RAISING;
    if (next_edge_falling_c0) {
      prev_cap_c0 = cap_val;
    }
    else {
      pwm_high_c0 = cap_val - prev_cap_c0;
//      duty_cycle_c0 = (PWM_FREQUENCY*pwm_high_c0*65535) / F_BUS;
      duty_cycle_c0 = (pwm_high_c0*12424)/100000;

      // Steering
      if (flagManualOverride == false && (duty_cycle_c0 > 9661 || duty_cycle_c0 < 9061)) { // V1 - > 9000; < 8300
        flagManualOverride = true;
      }

      if (flagManualOverride == true) {
        analogWrite(6,(int) duty_cycle_c0);
      }

      c0_done = 1;
    }
  }

  /* Event on channel 1 */
  if (status_reg_copy & 0b10) {
    cap_val = FTM1_C1V;
    if (cap_val <= 0x8000 || !overflowed)
      cap_val |= capture_ovf_bits << 16;
    else
      cap_val |= (capture_ovf_bits - 1) << 16;
    next_edge_falling_c1 = !next_edge_falling_c1;
    FTM1_C1SC = next_edge_falling_c1 ? FTM_CSC_FALLING : FTM_CSC_RAISING;
    if (next_edge_falling_c1) {
      prev_cap_c1 = cap_val;
    }
    else {
      pwm_high_c1 = cap_val - prev_cap_c1;
//      duty_cycle_c1 = (PWM_FREQUENCY*pwm_high_c1*65535) / F_BUS;
      duty_cycle_c1 = (pwm_high_c1*12424)/100000;

      // Throttle
      if (flagManualOverride == false && (duty_cycle_c1 > 9129 || duty_cycle_c1 < 8729)) { // V1 - > 9100; < 8800
        flagManualOverride = true;
      }

      if (flagManualOverride == true) {
        analogWrite(5,(int) duty_cycle_c1);
      }

      c1_done = 1;
    }
  }

  if (flagManualOverride == true && manual_blink_elapsed > 250) {
    if (LED_state == LOW)
      LED_state = HIGH;
    else
      LED_state = LOW;

    digitalWrite(13, LED_state);
    manual_blink_elapsed = 0;
  }

  ftm_irq_elapsed = 0;
}

/*
 * FTM1 channel 0 ... Teensy pin 3 ... STR
 * FTM1 channel 1 ... Teensy pin 4 ... THR
 */
void setupFTM(void) {
  NVIC_DISABLE_IRQ(IRQ_FTM1);

  CORE_PIN3_CONFIG = PORT_PCR_MUX(3);
  CORE_PIN4_CONFIG = PORT_PCR_MUX(3);

  FTM1_SC = 0;
  FTM1_CNT = 0;
  FTM1_MOD = 0xFFFF;
  FTM1_SC = FTM_SC_VALUE;
  FTM1_MODE = FTM_MODE_WPDIS;

  FTM1_FILTER = FTM_FILTER_CH0FVAL(2) | FTM_FILTER_CH1FVAL(2);
  FTM1_C0SC = FTM_CSC_RAISING;
  FTM1_C1SC = FTM_CSC_RAISING;

  NVIC_SET_PRIORITY(IRQ_FTM1, 1);
  NVIC_ENABLE_IRQ(IRQ_FTM1);
}


void setup() {

  analogWriteFrequency(5, PWM_FREQUENCY);  // THR
  analogWriteFrequency(6, PWM_FREQUENCY);  // STR
  analogWriteResolution(16);
  analogWrite(6,pwm_str_center_value);
  analogWrite(5,pwm_thr_center_value);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pinMode(2,INPUT);
  setupFTM();

  nh.getHardware()->setBaud(115200);

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_stop);

  nh.advertise(pwm_high);
}

void loop() {
  if (c0_done && c1_done) {
    pwm_high_msg.period_thr = duty_cycle_c1;
    pwm_high_msg.period_str = duty_cycle_c0;
    pwm_high.publish(&pwm_high_msg);

    c0_done = 0;
    c1_done = 0;
  }

  nh.spinOnce();

  // drive command timeout
  if ((flagManualOverride == false && flagStop == false && drive_msg_recv_elapsed > 300)
      || (flagManualOverride == true && ftm_irq_elapsed > 100)) {
    analogWrite(6,pwm_str_center_value);
    analogWrite(5,pwm_thr_center_value);
  }
}

int main()
{
  setup();
  while(1) {
    loop();
    yield();
  }

  return 0;
}
