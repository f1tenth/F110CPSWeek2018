/*
 * 
 * 
 * 
 *   
    Author: Philipp Rothenhäusler
    Royal Institute of Technology, Stockholm 2018
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *
 *
 *
 */

 
#include <ros.h>   
#include <string>      
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h> // Change for higher encoder precision --> might cause rosserial issues
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <control/pwm_interface.h>
#include <math.h>

// PIN-Definition
#define PIN_VEL 5 
#define PIN_STEER 6
#define PIN_ENC 9
#define PIN_SIGNAL 13

// Output-Definition
#define SERIAL_BAUDRATE 57600 
#define PWM_FREQUENCY 100
#define PWM_RESOLUTION 16

// CTRL-Signal in 16bit resolution Saturation and Steady State Definition                                                                                     
#define DUTY_CYCLE_MIN_BIT 6554                //  10% duty cycle in 16bit res (100% negative)
#define DUTY_CYCLE_CENTER_BIT 9830             //  15% duty cycle in 16bit res
#define DUTY_CYCLE_MAX_BIT 13108               //  20% duty cycle in 16bit res (100% positive)

/*
####################################################################################
#      Definition of Constants and Initialization of Variables                     #
####################################################################################
*/
ros::NodeHandle  nh;
//std_msgs::Int32 enc_velocity;
std_msgs::Float64 enc_velocity;
std_msgs::Int32 diag_msg;
int msg_code = 200;

ros::Publisher pub_diagnosis("pwm_interface_diagnosis", &diag_msg);
ros::Publisher pub_encoder("pwm_interface_encoder", &enc_velocity);

boolean INITIALIZED                 = false;
boolean EMERGENCY                   = false;
boolean ENC_PARAM_UPDATED           = false;
boolean ENC_PARAMETERS_INITIALIZED  = false;
boolean RUN_ENC                     = true;
boolean ENC_REINITIALIZE_PARAMETERS = false;
boolean IDLE_ACTIVE                 = false;

int PWM_VEL                         = 0;                          // in Percent = +-100
int PWM_STEER                       = 0;
int PWM_VEL_SAT                     = 0;
int PWM_STEER_SAT                   = 0;
int SAT_VEL_MIN                     = -100;
int SAT_VEL_MAX                     = 100;
int SAT_STEER_MIN                   = -100;
int SAT_STEER_MAX                   = 100;
int VEL_DUTY_CYCLE_BIT              = DUTY_CYCLE_CENTER_BIT;      // 16 bit res = 2exp(16)*0.5
int STEER_DUTY_CYCLE_BIT            = DUTY_CYCLE_CENTER_BIT;
        
unsigned int ENC_TS             = 5;               // Sampling Time for publishing encoder readings
unsigned int ENC_T_ZERO         = 100;              // 500ms without pulses sets velocity to zero
unsigned int ENC_T_PER_REV      = 8;
float WHEEL_RADIUS              = 0.08/2;
unsigned int k_enc              = 1;                // Amount of Zero Loops without detecting rising edge
unsigned int enc_pulses         = 0;
float velocity_filtered[100]    = {0};              // Might include lp filter for velocity before publishing with 25ms (40Hz)

unsigned long pub_enc_dT        = 25;             
unsigned long pub_diag_dT       = 500;     
unsigned long pub_enc_time0     = millis();           // PUB_ENC TIMER
unsigned long pub_diag_time0    = millis();           // PUB_DIAG TIMER
unsigned long enc_time0         = millis();           // ENC TIMER
unsigned long cb_ctrl_time0     = millis();           // IDLE TIMER (CB_CTRL)
unsigned long led_time0         = millis();           // LED TIMER

unsigned int LED_STATE          = 1;
boolean LED_VALUE               = HIGH;
unsigned long IDLE_TIME         = 250;    
unsigned long IDLE_LED_DT       = 500;              // Time period for LED to indicate IDLE condition



/*
####################################################################################
#        PWM_INTERFACE MOTOR and Servo TOPIC CALLBACK FCN                          #
####################################################################################
*/
void pwm_interface_callback( const control::pwm_interface& pwm ) {
  cb_ctrl_time0 = millis();
  VEL_DUTY_CYCLE_BIT = pwm.velocity;
  STEER_DUTY_CYCLE_BIT = pwm.steering;
}

/*
####################################################################################
#                       EMERGENCY TOPIC CALLBACK FCN                               #
####################################################################################
*/
void emergency_callback( const std_msgs::Bool& state ) {
  EMERGENCY = state.data;
}

/*
####################################################################################
#                             ACTUATE                                              #
####################################################################################
*/
void actuate(void) {
  msg_code = 0x00;
  
  if(!EMERGENCY) {
    /*
    ####################################################################################
    #                APPLY SATURATION FOR VELOCITY CONTROLLER (VXL 3s)                 #
    ####################################################################################
    */
    if(VEL_DUTY_CYCLE_BIT < DUTY_CYCLE_MIN_BIT) {
      analogWrite(PIN_VEL, DUTY_CYCLE_MIN_BIT);
      msg_code |= 1;                                                
    }
    else if(VEL_DUTY_CYCLE_BIT > DUTY_CYCLE_MAX_BIT) {
      analogWrite(PIN_VEL, DUTY_CYCLE_MAX_BIT);
      msg_code |= 2;                                                     
    }
    else{
      analogWrite(PIN_VEL, VEL_DUTY_CYCLE_BIT);
      msg_code |= 4;                                                                                 
    }
    
    /*
    ####################################################################################
    #                APPLY SATURATION FOR STEERING CONTROLLER (Servo)                  #
    ####################################################################################
    */
    if(STEER_DUTY_CYCLE_BIT < DUTY_CYCLE_MIN_BIT) {
      analogWrite(PIN_STEER, DUTY_CYCLE_MIN_BIT);
      msg_code |= 8;                                                          
    }
    else if(STEER_DUTY_CYCLE_BIT > DUTY_CYCLE_MAX_BIT){
      analogWrite(PIN_STEER, DUTY_CYCLE_MAX_BIT);
      msg_code |= 16;                                                 
    }
    else{
      analogWrite(PIN_STEER, STEER_DUTY_CYCLE_BIT);
      msg_code |= 32;                                                                      
    }
  }
  else{   
    analogWrite(PIN_VEL, DUTY_CYCLE_CENTER_BIT);  
    analogWrite(PIN_STEER, DUTY_CYCLE_CENTER_BIT);
      msg_code = 0;        
  }
  diag_msg.data = msg_code;
}

/*
####################################################################################
#                Publish Encoder Velocity based on Sampling Time                   #
####################################################################################
*/
void enc_check_sampling() {
  if(RUN_ENC){
    if((millis() - enc_time0) > ENC_TS){
      enc_time0 = millis();
      if (enc_pulses > 0){
        enc_velocity.data = float(2*M_PI*WHEEL_RADIUS)*float(enc_pulses/float(ENC_T_PER_REV))/float(k_enc*float(ENC_TS)*0.001);
        Serial.println(enc_velocity.data);
        enc_pulses  = 0;
        k_enc       = 1; 
      }
      else {
        if (k_enc*ENC_TS > ENC_T_ZERO) {
          Serial.println(enc_velocity.data);
          enc_velocity.data = 0;
          k_enc=5;
        }
        else {
          k_enc++;
         }
        //Serial.println("ENC_CHK_");
        //Serial.println(k_enc);
      }
    }
    else{
      ;
    } 
  }
  else{
    ;
  }
}


/*
####################################################################################
#                              ENCODER INTERRUPT                                   #
####################################################################################
*/
void isr_pwm_interface_encoder () {
  //INCLUDE SOFTWARE FILTER
  Serial.println("INTERRUPT");
  if(RUN_ENC){
    enc_pulses++;
  }
  else {;}
}

void blink_LED(){
  if(LED_STATE==1){
    digitalWrite(PIN_SIGNAL, HIGH);
  }
  else if(LED_STATE==2){
    if(millis() - led_time0 > IDLE_LED_DT){                                                               
      LED_VALUE         = !LED_VALUE;
      digitalWrite(PIN_SIGNAL, LED_VALUE);
      led_time0 = millis();
    }
    else {;}
  }
  else{;}
}


void update_timers(){
  if (!INITIALIZED){
      pub_enc_time0     = millis(); 
      pub_diag_time0    = millis();
      cb_ctrl_time0     = millis(); 
      enc_time0         = millis();
      led_time0         = millis();
      INITIALIZED       = true;
  }
  else{     
  }
}

void check_states() {
  if((millis() - cb_ctrl_time0) > IDLE_TIME) {
    EMERGENCY = true;
    IDLE_ACTIVE = true;
    LED_STATE = 2;
  }
  else{
    LED_STATE = 1;
    IDLE_ACTIVE = false;
  }
}

void run_teensy(){
      enc_check_sampling();
      //update_parameters();
      publish_data();
      blink_LED();
      nh.spinOnce();
      update_timers();
      check_states();
      actuate();
}

void publish_data(){
  if((millis()-pub_enc_time0) > pub_enc_dT){
    pub_enc_time0 = millis();
    pub_encoder.publish(&enc_velocity);
  }
  else {;}
  
  if((millis()-pub_diag_time0) > pub_diag_dT){
    pub_diag_time0 = millis();
    pub_diagnosis.publish(&diag_msg);
    //Serial.println("Diagnosis Status");
  }
  else{;}
}


ros::Subscriber<control::pwm_interface> sub_pwm_interface("pwm_interface", &pwm_interface_callback );                                         
ros::Subscriber<std_msgs::Bool> sub_pwm_interface_emergency("pwm_interface_emergency", &emergency_callback ); 
  
void setup() {
/*
####################################################################################
#       Setup Analog Pins and configure LED + Create ROS node handles              #
####################################################################################
*/
  //Serial.begin(SERIAL_BAUDRATE);
  analogWriteFrequency(PIN_VEL, PWM_FREQUENCY);                                                                            
  analogWriteFrequency(PIN_STEER, PWM_FREQUENCY); 
  analogWriteResolution(PWM_RESOLUTION);
  analogWrite(PIN_VEL, DUTY_CYCLE_CENTER_BIT);                                                                         
  analogWrite(PIN_STEER, DUTY_CYCLE_CENTER_BIT);
  
  pinMode(PIN_ENC, INPUT_PULLUP); 
  pinMode(PIN_SIGNAL, OUTPUT);                                                                              
  digitalWrite(PIN_SIGNAL, HIGH);  
  attachInterrupt(PIN_ENC, isr_pwm_interface_encoder, RISING);        
                                                                                                          
  /*
  ####################################################################################
  #                   Initialize ROS node and Subscribers                            #
  ####################################################################################
  */
  nh.initNode();                                                                                            
  nh.subscribe(sub_pwm_interface);                                                                          
  nh.subscribe(sub_pwm_interface_emergency);
   
  nh.advertise(pub_diagnosis);          
  nh.advertise(pub_encoder);
  diag_msg.data     = 0;
  enc_velocity.data = 0;
  update_timers();
}


/*
####################################################################################
#                              Main Loop for control                               #
####################################################################################
*/
void loop() {
  run_teensy();  
}




