#define ROSSERIAL_ARDUINO_TCP

#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <string.h>
#include <Servo.h>
#include <SharpIR.h>
#include "Wire.h"

// ---- biblioteca mpu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container

unsigned long timer = 0;

// ---------- sensor config
// Define model and input pin:
#define IRPin A0
#define model 1080
SharpIR SharpSensor = SharpIR(IRPin, model);

// ---------- config network
#define debug 1
#define base_port 11411 //usar outros números para ligar mais robôs na mesma porta

// ---------- config servo motor
Servo servo; 
int pos = 0;
int dir = 0;
int SRV = 0;

// ---------- driver motor pins
int IN1 = 3;
int IN2 = 1;
int IN3 = 16;
int IN4 = 13;

// ---------- encoder pin
int EC1 = 12;
int EC2 = 14;

// ---------- tick count 
int EC1_count = 0;
int EC2_count = 0;

// ---------- LED pin for debug 
int LED = 2;

// kinematics variables 
float wheelL;
float wheelR;

//----------- topics name 
#define top_cmd_vel "/bob/cmd_vel"
#define top_servo   "/bob/ir_motor"
#define top_sharp   "/bob/ir_sensor"
#define top_imu     "/bob/heading"
#define top_renc    "/bob/raw/right_ticks"
#define top_lenc    "/bob/raw/left_ticks"
#define top_pwm_lm  "/bob/pwm/left"
#define top_pwm_rm  "/bob/pwm/right"
#define top_write_pwm_lm "/bob/write/pwm/left"
#define top_write_pwm_rm "/bob/write/pwm/right"

// wifi config 
struct config_t
{

  // roteador Gui
  // const char* ssid = "Dinossauro_Conectado";
  // const char* password = "Dinossauro_Conectado";

  const char* ssid = "Le";
  const char* password = "leticia21";
  
  uint16_t serverPort = base_port; 
  int serverIP[4] = {192, 168, 13, 213}; // "ip do computador com roscore"
  int robot = 0;
  char* topic_servo = top_servo;
  char* topic_lenc = top_lenc;
  char* topic_renc = top_renc;
  char* topic_cmd_vel = top_cmd_vel;
  char* topic_sharp = top_sharp;
  char* topic_imu = top_imu;
  char* topic_pwml = top_pwm_lm; 
  char* topic_pwmr = top_pwm_rm;
  char* topic_write_pwml = top_write_pwm_lm;
  char* topic_write_pwmr = top_write_pwm_rm; 
} configuration;

// ----- messages
std_msgs::Float32 sharp_msg; 
std_msgs::Float32 lenc_msg;     // left encoder
std_msgs::Float32 renc_msg;     // right encoder
std_msgs::Float32 imu_msg;
std_msgs::Float32 pwmL_msg; 
std_msgs::Float32 pwmR_msg; 

// ---- publishers 
ros::Publisher pub_lenc(configuration.topic_lenc, &lenc_msg);
ros::Publisher pub_renc(configuration.topic_renc, &renc_msg);
ros::Publisher pub_sharp(configuration.topic_sharp, &sharp_msg);
ros::Publisher pub_imu(configuration.topic_imu, &imu_msg);
ros::Publisher pub_pwml(configuration.topic_pwml, &pwmL_msg); 
ros::Publisher pub_pwmr(configuration.topic_pwmr, &pwmR_msg); 

// ---- subscribers 
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel(configuration.topic_cmd_vel, &odometry_cb);
ros::Subscriber<std_msgs::UInt16> sub_servo(configuration.topic_servo, &servo_cb);
ros::Subscriber<std_msgs::UInt16> sub_write_pwml(configuration.topic_write_pwml, &write_leftPWM_cb); 
ros::Subscriber<std_msgs::UInt16> sub_write_pwmr(configuration.topic_write_pwmr, &write_rightPWM_cb); 

void write_leftPWM_cb(const std_msgs::UInt16& msg){
  analogWrite(IN3, msg.data);
  analogWrite(IN4, 0);
}

void write_rightPWM_cb(const std_msgs::UInt16& msg){
  analogWrite(IN2, msg.data);
  analogWrite(IN1, 0);
}

void odometry_cb(const geometry_msgs::Twist& msg) {

  float linear_velocity;
  float angular_velocity; 
  float width_robot = 0.140; 
  float wradius = 0.0660/2;

  float gain_linear = 10; 
  float gain_angular = 100;

  int offset = 10; 

  linear_velocity = msg.linear.x;
  angular_velocity = msg.angular.z;

  float min_vel = 0;
  float left_max_vel = 250 - offset; 
  float right_max_vel = 250;

  wheelR = (gain_linear*(linear_velocity) + gain_angular*(angular_velocity * width_robot/2)) / (wradius);
  wheelL = (gain_linear*(linear_velocity) - gain_angular*(angular_velocity * width_robot/2)) / (wradius);

  if (wheelR != 0){
  wheelR = wheelR + offset; 
  }

  sharp_msg.data = wheelR;

  // saturate max velocity for the right wheel
  if (wheelR > right_max_vel){
    wheelR = right_max_vel; 
  }

  // saturate max velocity for the left wheel
  if (wheelL > left_max_vel){
    wheelL = left_max_vel; 
  }

  // saturate min velocity for the right wheel
  if (wheelR < min_vel) {
    wheelR = min_vel; 
  }

  // saturate mi velocity for the left wheel
  if(wheelL < min_vel) {
    wheelL = min_vel; 
  }

  // write velocity for the right motor
  analogWrite(IN2, wheelR);
  analogWrite(IN1, 0);

  // write velocity for the left motor 
  analogWrite(IN3, wheelL);
  analogWrite(IN4, 0);
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

void setup_imu()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
        
    // turn on the DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
    
    else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

float get_yaw()
{
    
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    // display Euler angles in radians
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t"); // print yaw
    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);  // print pitch 
    Serial.print("\t");
    Serial.println(ypr[2]); // print roll 
  }
  // return yaw
  return ypr[0];

}

void setup() {

  configuration.serverPort=configuration.serverPort+configuration.robot;  
  IPAddress server(configuration.serverIP[0], configuration.serverIP[1], configuration.serverIP[2], configuration.serverIP[3]);   // Set the rosserial socket server IP address
  setupWiFi();

  // configure ros communication
  nh.getHardware()->setConnection(server, configuration.serverPort); 
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_servo);
  nh.subscribe(sub_write_pwml); 
  nh.subscribe(sub_write_pwmr);

  nh.advertise(pub_lenc);
  nh.advertise(pub_renc);
  nh.advertise(pub_sharp);
  nh.advertise(pub_imu);
  nh.advertise(pub_pwml); 
  nh.advertise(pub_pwmr); 

  // configure GPIO's
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED, OUTPUT);

  // configure servo motor and put it at "position 0"
  servo.attach(SRV);
  servo.write(55);

  // configure interruption pins for encoder
  pinMode(EC1, INPUT);
  attachInterrupt(EC1, ISR_EC1, RISING);
  pinMode(EC2, INPUT);
  attachInterrupt(EC2, ISR_EC2, RISING);

  // initialize serial communication  
  Serial.begin(115200);
  while (!Serial);

  // setup and calibration of the imu sensor 
  setup_imu();
}

void loop() {
  
  lenc_msg.data = EC1_count;
  pub_lenc.publish(&lenc_msg);
  
  renc_msg.data = EC2_count;
  pub_renc.publish(&renc_msg);
  
  sharp_msg.data = SharpSensor.distance()*1.8;
  pub_sharp.publish(&sharp_msg);

  imu_msg.data = get_yaw(); 
  pub_imu.publish(&imu_msg); 

  pwmL_msg.data = wheelL; 
  pwmR_msg.data = wheelR;
  pub_pwml.publish(&pwmL_msg); 
  pub_pwmr.publish(&pwmR_msg); 

  nh.spinOnce();
}
