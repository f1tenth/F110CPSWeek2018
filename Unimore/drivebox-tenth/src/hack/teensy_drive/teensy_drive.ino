#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <race/drive_param.h>
ros::NodeHandle  nh;
 


boolean flagStop = false;
int pwm_center_value = 9830;  //  15% duty cycle
int pwm_lowerlimit = 6554;    //  10% duty cycle
int pwm_upperlimit = 13108;   //  20% duty cycle

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);
race::drive_param drv_msg;
ros::Publisher drive("drive_act", &drv_msg);

int kill_pin = 2;
unsigned long duration = 0;

boolean manual = true;

void writepwm(int pwm_angle, int pwm_drive)
{
  if (pwm_drive < pwm_lowerlimit) {
    analogWrite(5, pwm_lowerlimit);   //  Safety lower limit

  } else if (pwm_drive > pwm_upperlimit) {
    analogWrite(5, pwm_upperlimit);   //  Safety upper limit

  } else {
    analogWrite(5, pwm_drive);    //  Incoming data
  }

  if (pwm_angle < pwm_lowerlimit) {
    analogWrite(6, pwm_lowerlimit);   //  Safety lower limit

  } else if (pwm_angle > pwm_upperlimit) {
    analogWrite(6, pwm_upperlimit);   //  Safety upper limit

  } else {
    analogWrite(6, pwm_angle);    //  Incoming data
  }
}


void messageDrive( const race::drive_param& par )
{
  if (manual)
    return;

  send_drive_act(par.velocity, par.angle);

  if (flagStop == false) {
    int16_t pwm_drive = map(par.velocity, -100, 100, pwm_lowerlimit, pwm_upperlimit);
    int16_t pwm_angle = map(par.angle, -100, 100, pwm_lowerlimit, pwm_upperlimit);
    writepwm(pwm_angle, pwm_drive);
    //str_msg.data = pwm_drive;
    //chatter.publish( &str_msg );

  } else {
    analogWrite(5, pwm_center_value);
    analogWrite(6, pwm_center_value);
  }
}

void messageEmergencyStop( const std_msgs::Bool& flag )
{
  flagStop = flag.data;
  if (flagStop == true) {
    analogWrite(5, pwm_center_value);
    analogWrite(6, pwm_center_value);
  } else {
    manual = false;
    str_msg.data = manual + 1000;
    chatter.publish( &str_msg );
  }
}

void send_drive_act(int velocity, int angle) {

    drv_msg.velocity = velocity;
    drv_msg.angle = angle;
    drive.publish(&drv_msg);
}

ros::Subscriber<race::drive_param> sub_drive("drive_parameters", &messageDrive );
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );

void setup() {

  analogWriteFrequency(5, 100);
  analogWriteFrequency(6, 100);
  analogWriteResolution(16);

  pinMode(14, INPUT);
  pinMode(15, INPUT);

  analogWrite(5, pwm_center_value);
  analogWrite(6, pwm_center_value);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_stop);

  nh.advertise(chatter);
  nh.advertise(drive);

}

void loop() {
  unsigned long pwm_angle = pulseIn(14, HIGH, 30000);
  unsigned long pwm_drive = pulseIn(15, HIGH, 30000);

    str_msg.data = manual;
    chatter.publish( &str_msg );

    if ( (pwm_angle > 500 && pwm_angle < 2500) && (pwm_angle > 1550 || pwm_drive > 1550 || pwm_angle < 1450 || pwm_drive < 1450)) {
      manual = true;
    }
    if(manual) {
      int angle = map(pwm_angle, 1000, 2000, pwm_lowerlimit, pwm_upperlimit);
      int drive = map(pwm_drive, 1000, 2000, pwm_lowerlimit, pwm_upperlimit);
      writepwm(angle, drive);

      send_drive_act( map(drive, pwm_lowerlimit, pwm_upperlimit, -100, 100),
                      map(angle, pwm_lowerlimit, pwm_upperlimit, -100, 100));
    }

    nh.spinOnce();

    if(manual)
      digitalWrite(2, LOW);
    else
      digitalWrite(2, HIGH);
}


