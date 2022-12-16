#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <string.h>
#include <Servo.h>
#include <SharpIR.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

// Define model and input pin:
#define IRPin A0
#define model 1080
SharpIR SharpSensor = SharpIR(IRPin, model);

#define debug 1
#define base_port 11411 //usar outros números para ligar mais robôs na mesma porta

//----------------------
#define top_cmd_vel "/bob/cmd_vel"
#define top_servo   "/bob/ir_motor"
#define top_sharp   "/bob/ir_sensor"
#define top_imu     "/bob/heading"
#define top_renc    "/bob/raw/right_ticks"
#define top_lenc    "/bob/raw/left_ticks"
//----------------------

Servo servo; 
int pos = 0;
int dir = 0;
//----------------------
//Ponte-H
int IN1 = 3;
int IN2 = 1;
int IN3 = 16;
int IN4 = 13;
int LED = 2;
int SRV = 0;
int EC1 = 12;
int EC2 = 14;
//----------------------

int EC1_count = 0;
int EC2_count = 0;


struct config_t
{
  const char* ssid = "nome_da_rede_wifi";
  const char* password = "senha_da_rede_wifi";
  
  uint16_t serverPort = base_port; 
  int serverIP[4] = {192, 168, 68, 113}; // "ip do computador com roscore"
  int robot = 0;
  char* topic_servo = top_servo;
  char* topic_lenc = top_lenc;
  char* topic_renc = top_renc;
  char* topic_cmd_vel = top_cmd_vel;
  char* topic_sharp = top_sharp;
  char* topic_imu = top_imu;
} configuration;

std_msgs::Float32 sharp_msg, lenc_msg, renc_msg, yaw_msg, imu_msg;
ros::Publisher pub_lenc(configuration.topic_lenc, &lenc_msg);
ros::Publisher pub_renc(configuration.topic_renc, &renc_msg);
ros::Publisher pub_sharp(configuration.topic_sharp, &sharp_msg);
ros::Publisher pub_imu(configuration.topic_imu, &imu_msg);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel(configuration.topic_cmd_vel, &odometry_cb);
ros::Subscriber<std_msgs::UInt16> sub_servo(configuration.topic_servo, servo_cb);

void odometry_cb(const geometry_msgs::Twist& msg) {
  float forward, lateral, wheelL, wheelR, wlength = 0.115, wradius = 0.0685/2, reduction = 48;
  forward = msg.linear.x;
  lateral = msg.angular.z;

  wheelR = ((forward / wradius) + (lateral * wlength) / (2 * wradius)) * reduction;
  wheelL = ((forward / wradius) - (lateral * wlength) / (2 * wradius)) * reduction;

  sharp_msg.data = wheelR;

  
  if (wheelR >= 0) {
    analogWrite(IN2, wheelR);
    analogWrite(IN1, 0);
  }
  else {
    analogWrite(IN2, 0);
    analogWrite(IN1, abs(wheelR));
  }
  if (wheelL >= 0) {
    analogWrite(IN3, wheelL);
    analogWrite(IN4, 0);
  }
  else {
    analogWrite(IN3, 0);
    analogWrite(IN4, abs(wheelL));
  }  
}

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void setupWiFi() {                    // connect to ROS server as as a client
  if (debug) {
    Serial.print("Connecting wifi to ");
    Serial.println(configuration.ssid);
  }
  WiFi.begin(configuration.ssid, configuration.password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debug)
      Serial.print(".");
  }
  if (debug) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

ros::NodeHandle nh;


void IRAM_ATTR ISR_EC1()
{
  EC1_count++;
}
void IRAM_ATTR ISR_EC2()
{
  EC2_count++;
}

void setup() {

  configuration.serverPort=configuration.serverPort+configuration.robot;  
  IPAddress server(configuration.serverIP[0], configuration.serverIP[1], configuration.serverIP[2], configuration.serverIP[3]);   // Set the rosserial socket server IP address
  setupWiFi();
  delay(2000);
  nh.getHardware()->setConnection(server, configuration.serverPort); 
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_servo);
  nh.advertise(pub_lenc);
  nh.advertise(pub_renc);
  nh.advertise(pub_sharp);
  nh.advertise(pub_imu);

  // IMU Initialization
  Wire.begin();
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050

   delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero

  // configure GPIO's
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED, OUTPUT);
  servo.attach(SRV);
  servo.write(90);
  pinMode(EC1, INPUT);
  attachInterrupt(EC1, ISR_EC1, RISING);
  pinMode(EC2, INPUT);
  attachInterrupt(EC2, ISR_EC2, RISING);

}


void loop() {
  delay(50);
  digitalWrite(LED, LOW);
  delay(50);                 
  digitalWrite(LED, HIGH);
  
  lenc_msg.data = EC1_count;
  pub_lenc.publish(&lenc_msg);
  
  renc_msg.data = EC2_count;
  pub_renc.publish(&renc_msg);
  
  sharp_msg.data = SharpSensor.distance()*1.8;
  pub_sharp.publish(&sharp_msg);

  mpu.update();
  imu_msg.data = mpu.getAngleZ();
  pub_imu.publish(&imu_msg);


  nh.spinOnce();
}