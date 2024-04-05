#include <ros.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

#include "MapFloat.h"

RMCS2303 rmcs;  // creation of motor driver object
// slave ids to be set on the motor driver refer to the manual in the reference section
byte slave_id1 = 1;
byte slave_id2 = 2;
byte slave_id3 = 3;
byte slave_id4 = 4;

// byte slave_id1 = 2;
// byte slave_id2 = 4;
// byte slave_id3 = 1;
// byte slave_id4 = 3;

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist

std_msgs::Int32 wheel1;  // for storing left encoder value
std_msgs::Int32 wheel2;  // for storing right encoder value

std_msgs::Int32 wheel3;  // for storing left encoder value
std_msgs::Int32 wheel4;  // for storing right encoder value

std_msgs::Int32MultiArray motor_rev;

// Publisher object with topic names wheel1_ticks and wheel2_ticks for publishing Enc Values
ros::Publisher wheel1_ticks("wheel1_ticks", &wheel1);
ros::Publisher wheel2_ticks("wheel2_ticks", &wheel2);
ros::Publisher wheel3_ticks("wheel3_ticks", &wheel3);
ros::Publisher wheel4_ticks("wheel4_ticks", &wheel4);


ros::Publisher value_wheels("value_wheels", &motor_rev);


// Make sure to specify the correct values here
//*******************************************
double wheel_rad = 0.05, wheel_sep = 0.250;  // wheel radius and wheel sepration in meters.
//******************************************
double w1 = 0, w2 = 0, w3 = 0, w4 = 0;
double rad_wheel = 0.05, rad_bot = 0.25;
double speed_ang;
double speed_lin_x;
double speed_lin_y;
double msg_value;

// double leftPWM;
// double rightPWM;

double w1_pwm, w2_pwm, w3_pwm, w4_pwm;

void messageCb(const geometry_msgs::Twist& msg)  // cmd_vel callback function definition
{
  speed_lin_x = max(min(msg.linear.x, 1.0f), -1.0f);  // limits the linear x value from -1 to 1
  speed_lin_y = max(min(msg.linear.y, 1.0f), -1.0f);
  speed_ang = max(min(msg.angular.z, 1.0f), -1.0f);  // limits the angular z value from -1 to 1

  w1 = 20 * ((-0.707 * speed_lin_x) + (-0.707 * speed_lin_y) + (rad_bot * speed_ang));
  w2 = 20 * ((0.707 * speed_lin_x) + (-0.707 * speed_lin_y) + (rad_bot * speed_ang));
  w3 = 20 * ((0.707 * speed_lin_x) + (0.707 * speed_lin_y) + (rad_bot * speed_ang));
  w4 = 20 * ((-0.707 * speed_lin_x) + (0.707 * speed_lin_y) + (rad_bot * speed_ang));

  Serial3.print(w1);
  Serial3.print(" || ");
  Serial3.print(w2);
  Serial3.print(" || ");
  Serial3.print(w3);
  Serial3.print(" || ");
  Serial3.println(w4);

  if (w1 == 0 && w2 == 0 && w3 == 0 && w4 == 0) {
    w1_pwm = 0;
    w2_pwm = 0;
    w3_pwm = 0;
    w4_pwm = 0;
    rmcs.Disable_Digital_Mode(slave_id1, 0);
    rmcs.Disable_Digital_Mode(slave_id2, 0);
    rmcs.Disable_Digital_Mode(slave_id3, 0);
    rmcs.Disable_Digital_Mode(slave_id4, 0);
  } else {
    w1_pwm = mapFloat(fabs(w1), 0.0, 18.0, 1500, 17200);  // mapping the right wheel velocity with respect to Motor PWM values
    w2_pwm = mapFloat(fabs(w2), 0.0, 18.0, 1500, 17200);  // mapping the right wheel velocity with respect to Motor PWM values
    w3_pwm = mapFloat(fabs(w3), 0.0, 18.0, 1500, 17200);  // mapping the right wheel velocity with respect to Motor PWM values
    w4_pwm = mapFloat(fabs(w4), 0.0, 18.0, 1500, 17200);  // mapping the right wheel velocity with respect to Motor PWM values
  }

  rmcs.Speed(slave_id1, w1_pwm);
  rmcs.Speed(slave_id2, w2_pwm);
  rmcs.Speed(slave_id3, w3_pwm);
  rmcs.Speed(slave_id4, w4_pwm);

  if ((w1 < 0 && w2 > 0) && (w3 > 0 && w4 < 0)) {
    // CW - 0 || CCW - 1
    Serial3.println("Frwd");
    rmcs.Enable_Digital_Mode(slave_id1, 1);
    rmcs.Enable_Digital_Mode(slave_id2, 1);  // forward condition
    rmcs.Enable_Digital_Mode(slave_id3, 0);
    rmcs.Enable_Digital_Mode(slave_id4, 0);
  }

  else if ((w1 > 0 && w2 < 0) && (w3 < 0 && w4 > 0)) {
    Serial3.println("Back");
    rmcs.Enable_Digital_Mode(slave_id1, 0);
    rmcs.Enable_Digital_Mode(slave_id2, 0);  // backward condition
    rmcs.Enable_Digital_Mode(slave_id3, 1);
    rmcs.Enable_Digital_Mode(slave_id4, 1);
  } else if ((w1 > 0 && w2 > 0) && (w3 > 0 && w4 > 0)) {
    Serial3.println("Turn left");
    rmcs.Enable_Digital_Mode(slave_id1, 0);
    rmcs.Enable_Digital_Mode(slave_id2, 0);  // Leftward condition
    rmcs.Enable_Digital_Mode(slave_id3, 0);
    rmcs.Enable_Digital_Mode(slave_id4, 0);
  }

  else if ((w1 < 0 && w2 < 0) && (w3 < 0 && w4 < 0)) {
    Serial3.println("Trun right");
    rmcs.Enable_Digital_Mode(slave_id1, 1);
    rmcs.Enable_Digital_Mode(slave_id2, 1);
    rmcs.Enable_Digital_Mode(slave_id3, 1);
    rmcs.Enable_Digital_Mode(slave_id4, 1);
  } else if ((w1 < 0 && w2 < 0) && (w3 > 0 && w4 > 0)) {
    Serial3.println("Left");
    rmcs.Enable_Digital_Mode(slave_id1, 0);
    rmcs.Enable_Digital_Mode(slave_id2, 1);
    rmcs.Enable_Digital_Mode(slave_id3, 1);
    rmcs.Enable_Digital_Mode(slave_id4, 0);
  } else if ((w1 > 0 && w2 > 0) && (w3 < 0 && w4 < 0)) {
    Serial3.println("Right");
    rmcs.Enable_Digital_Mode(slave_id1, 1);
    rmcs.Enable_Digital_Mode(slave_id2, 0);
    rmcs.Enable_Digital_Mode(slave_id3, 0);
    rmcs.Enable_Digital_Mode(slave_id4, 1);
  } else if ((w1 < 0 && w2 == 0) && (w3 > 0 && w4 == 0)) {
    Serial3.println("II Quadrant");
    rmcs.Brake_Motor(slave_id1, 0);
    rmcs.Enable_Digital_Mode(slave_id2, 1);
    rmcs.Brake_Motor(slave_id3, 0);
    rmcs.Enable_Digital_Mode(slave_id4, 0);
  } else if ((w1 == 0 && w2 > 0) && (w3 == 0 && w4 < 0)) {
    Serial3.println("I Quadrant");
    rmcs.Enable_Digital_Mode(slave_id1, 1);
    rmcs.Brake_Motor(slave_id2, 0);
    rmcs.Enable_Digital_Mode(slave_id3, 0);
    rmcs.Brake_Motor(slave_id4, 0);
  } else if ((w1 == 0 && w2 < 0) && (w3 == 0 && w4 > 0)) {
    Serial3.println("III Quadrant");
    rmcs.Enable_Digital_Mode(slave_id1, 0);
    rmcs.Brake_Motor(slave_id2, 0);
    rmcs.Enable_Digital_Mode(slave_id3, 1);
    rmcs.Brake_Motor(slave_id4, 0);
  } else if ((w1 > 0 && w2 == 0) && (w3 < 0 && w4 == 0)) {
    Serial3.println("IV Quadrant");
    rmcs.Brake_Motor(slave_id1, 0);
    rmcs.Enable_Digital_Mode(slave_id2, 0);
    rmcs.Brake_Motor(slave_id3, 0);
    rmcs.Enable_Digital_Mode(slave_id4, 1);
  } else {
    rmcs.Brake_Motor(slave_id1, 0);
    rmcs.Brake_Motor(slave_id2, 0);
    rmcs.Brake_Motor(slave_id3, 0);
    rmcs.Brake_Motor(slave_id4, 0);  // if none of the above break the motors both in clockwise n anti-clockwise direction
    rmcs.Brake_Motor(slave_id1, 1);
    rmcs.Brake_Motor(slave_id2, 1);
    rmcs.Brake_Motor(slave_id3, 1);
    rmcs.Brake_Motor(slave_id4, 1);
  }
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);  // creation of subscriber object sub for recieving the cmd_vel

void setup() {
  rmcs.Serial_selection(0);  // 0 -> for Harware serial tx1 rx1 of arduino mega
  rmcs.Serial0(9600);
  rmcs.begin(&Serial1, 9600);

  Serial3.begin(9600);

  nh.initNode();      // initialzing the node handle object
                      // nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);  // subscribing to cmd vel with sub object

  nh.advertise(wheel1_ticks);  // advertise the wheel1_ticks topic
  nh.advertise(wheel2_ticks);  // advertise the wheel1_ticks topic

  nh.advertise(wheel3_ticks);  // advertise the wheel1_ticks topic
  nh.advertise(wheel4_ticks);  // advertise the wheel1_ticks topic
  nh.advertise(value_wheels);  // advertise the wheel1_ticks topic
}

void loop() {

  wheel1.data = rmcs.Position_Feedback(slave_id1);
  wheel2.data = -rmcs.Position_Feedback(slave_id2);
  wheel3.data = rmcs.Position_Feedback(slave_id3);
  wheel4.data = -rmcs.Position_Feedback(slave_id4);

  motor_rev.data[0] = wheel1.data;
  motor_rev.data[1] = wheel2.data;
  motor_rev.data[2] = wheel3.data;
  motor_rev.data[3] = wheel4.data;
  motor_rev.data_length = 4;

  wheel1_ticks.publish(&wheel1);  // publish left enc values
  wheel2_ticks.publish(&wheel2);  // publish right enc values
  wheel3_ticks.publish(&wheel3);
  wheel4_ticks.publish(&wheel4);

  value_wheels.publish(&motor_rev);

  nh.spinOnce();
  delay(1);
}