#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <OdomCompacted.h>
#include <geometry_msgs/Twist.h>

const float WHEEL_DISTANCE = 0.45;
const float WHEEL_RADIUS = 0.065;
const float ENCODER_RESOLUTION = 24.0; // counts/revolution
const float GEAR_RATIO = 98.78;

const int CW = 0;
const int CCW = 1;

float time_last_encoder_measure = 0;

int INA[2] = {39, 42};
int INB[2] = {40, 43};
int PWMpins[2] = {45, 46};
int ENC1[2] = {25, 27};
int ENC2[2] = {26, 28};
int stb = 29;

volatile long MCount[2] = {0, 0};
float vel[2] = {0,0};
float acc_error[2] = {0,0};
float previous_vel[2] = {0,0};
int cPWM[2] = {0,0};
int flag = 0;

struct POSE{
  float x = 0;
  float y = 0;
  float theta = 0;
  };
POSE pose;

struct PID {
  float kd = 0.015;
  float ki = 0.0015;
  float kp = 0.15;
};
PID pid;

struct TargetVelocity {
  float linear = 0;
  float angular = 0;
};
TargetVelocity target_vel;

ros::NodeHandle  nh;

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg){
  target_vel.linear = cmd_vel_msg.linear.x;
  target_vel.angular = cmd_vel_msg.angular.z;
}
std_msgs::Float32 msg;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Publisher chatter("chatter", &msg);

// Odometria
zordon_msgs::OdomCompacted odometry_msg;
//const std::string odometry_msg_name = PREFIX_MSG + "/wheels_odom";
//const std::string odometry_msg_child_frame_id = PREFIX_MSG + "_base";
//const std::string odometry_msg_frame_id = PREFIX_MSG + "_odom";
ros::Publisher odometry_msg_pub("base_controller/odom", &odometry_msg);

void setup() {
  int count;
  for(count = 0; count<4; count++){
    pinMode(ENC1[count], INPUT);
    pinMode(ENC2[count], INPUT);
    pinMode(INA[count], OUTPUT);
    pinMode(INB[count], OUTPUT);
    pinMode(PWMpins[count], PWM);
  }
  pinMode(stb, OUTPUT);
  digitalWrite(stb, LOW);
  attachInterrupt(digitalPinToInterrupt(ENC1[0]), M1EncoderEvent, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC1[1]), M2EncoderEvent, RISING);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(cmd_vel_sub);
  configureRosMsg();

//  // Publica mensagens dos encoders
//  odometry_msg.header.frame_id = odometry_msg_frame_id.c_str();
//  odometry_msg.child_frame_id = odometry_msg_child_frame_id.c_str();
//  setOdomMsgCovariance(odometry_msg.twist.covariance, 0.01);
  nh.advertise(odometry_msg_pub);
}

void loop() {
  if(!nh.connected()){
    digitalWrite(stb, LOW);    
  }
  else {
    digitalWrite(stb, HIGH);
    compute_wheel_velocity();
    control();
  }
  nh.spinOnce();
  
  delay(2);
}

void configureRosMsg() {
//  odometry_msg.header.frame_id = "odom";
 // odometry_msg.child_frame_id = "base_link";
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if(direct == 0 && pwm > 25){
    digitalWrite(INA[motor], LOW);
    digitalWrite(INB[motor], HIGH);
    analogWrite(PWMpins[motor], pwm);
  }
  else if(direct == 1 && pwm > 25){
    digitalWrite(INA[motor], HIGH);
    digitalWrite(INB[motor], LOW);
    analogWrite(PWMpins[motor], pwm);
  }
  else{
    digitalWrite(INA[motor], LOW);
    digitalWrite(INB[motor], LOW);
    analogWrite(PWMpins[motor], 0);
    acc_error[motor] = 0;
    previous_vel[motor] = 0;
  }
}

void control() {
  int count;
  
  float left_targ_vel = (2*target_vel.linear - target_vel.angular*WHEEL_DISTANCE)/(2 * WHEEL_RADIUS);
  cPWM[0] = run_pid(0, left_targ_vel);

  float right_targ_vel = -(2*target_vel.linear + target_vel.angular*WHEEL_DISTANCE)/(2 * WHEEL_RADIUS);
  cPWM[1] = run_pid(2, right_targ_vel);

  for(count = 0; count<2; count++){
    if(cPWM[count] > 0) motorGo(count, CW, cPWM[count]);
    else motorGo(count, CCW, -cPWM[count]);
  }
  
}

int run_pid(int count, float target_vel)
{
  float error = target_vel - vel[count];
  float deriv_error = vel[count] - previous_vel[count];
  previous_vel[count] = vel[count];
  acc_error[count] += error;
//  if(acc_error[count] > 0.5/pid.ki) acc_error[count] = 0.5/pid.ki;
//  if(acc_error[count] < 0 && acc_error[count] <  -0.5/pid.ki) acc_error[count] = -0.5/pid.ki;
  return constrain(pid.kp * error + pid.ki * acc_error[count] - pid.kd * deriv_error,-1,1)*65535;
}

void compute_wheel_velocity()
{
  int count;
  float time_now = millis()/1000.0; // segundos
  float elapsed_time = time_now - time_last_encoder_measure;

  // Calcula a cada 10ms
  if(elapsed_time < 0.025) return;
  time_last_encoder_measure = time_now;
  
  // Calcula as velocidades de cada roda (rad/s)
  for(count = 0; count < 4; count++){
    float velo = ((float)MCount[count]*2.0*PI)/(ENCODER_RESOLUTION  * GEAR_RATIO * elapsed_time);
    if(velo > 0){
      if(velo < 5)
        vel[count] = velo;
    }
    if(velo < 0){
      if(velo > -5)
        vel[count] = velo; 
    }
    MCount[count] = 0;
  }

  float _speed = (-vel[0] - vel[1] + vel[2] + vel[3]) * WHEEL_RADIUS/4.0;
  float Dth = (-vel[0] - vel[1] - (vel[2] + vel[3])) * WHEEL_RADIUS/WHEEL_DISTANCE;
  pose.theta = pose.theta + elapsed_time*Dth;
  pose.x = pose.x - elapsed_time*_speed*cos(pose.theta);
  pose.y = pose.y - elapsed_time*_speed*sin(pose.theta);
  float left_vel = (vel[0] + vel[1]) * WHEEL_RADIUS/2.0;
  float right_vel = -(vel[2] + vel[3]) * WHEEL_RADIUS/2.0;


//  odometry_msg.twist.twist.linear.x = _speed;
//  odometry_msg.twist.twist.angular.z = Dth;
//  //odometry_msg.twist.twist.angular.x = left_wheel.velocity;
//  //odometry_msg.twist.twist.angular.y = right_wheel.velocity;
// 
//  
//  odometry_msg.pose.pose.position.x = pose.x;
//  odometry_msg.pose.pose.position.y = pose.y;
//  odometry_msg.pose.pose.position.z = 0.0;//pose.theta *180/3.1415;
//  odometry_msg.pose.pose.orientation.x = 0.0;//goal_vel.angular[2];
//  odometry_msg.pose.pose.orientation.y = 0.0;//dxl_wb.itemRead(DXL_ID, "Present_Position");
//  odometry_msg.pose.pose.orientation.z = sin(pose.theta/2.0);
//  odometry_msg.pose.pose.orientation.w = cos(pose.theta/2.0);
//  odometry_msg.pose.covariance[0]  = 0.2; ///< x
//  odometry_msg.pose.covariance[7]  = 0.2; ///< y
//  odometry_msg.pose.covariance[35] = 0.4; ///< yaw
//  odometry_msg.header.stamp = nh.now();


    odometry_msg.x=pose.x;
    odometry_msg.y=pose.y;
    odometry_msg.wl=left_vel;
    odometry_msg.wr=right_vel;
    odometry_msg.yaw=pose.theta;
    odometry_msg_pub.publish(&odometry_msg);
    }

void setOdomMsgCovariance(float* cov, float diagonal_value)
{
  for(int i = 0; i < 36; i++) {
    if(i == 0 || i == 7 || i == 14 ||
       i == 21 || i == 28 || i == 35)
      cov[i] = diagonal_value;
    else
      cov[i] = 0;
  }
}

void M1EncoderEvent(){
  if (digitalRead(ENC1[0]) == HIGH) {
    if (digitalRead(ENC2[0]) == LOW) {
      MCount[0]++;
    } else {
      MCount[0]--;
    }
  } else {
    if (digitalRead(ENC2[0]) == LOW) {
      MCount[0]--;
    } else {
      MCount[0]++;
    }
  }
}

void M2EncoderEvent(){
  if (digitalRead(ENC1[1]) == HIGH) {
    if (digitalRead(ENC2[1]) == LOW) {
      MCount[1]++;
    } else {
      MCount[1]--;
    }
  } else {
    if (digitalRead(ENC2[1]) == LOW) {
      MCount[1]--;
    } else {
      MCount[1]++;
    }
  }
}
