//Janus Brushless Controller Firmware 022 (Includes current sensing)

//SimpleFOC Version 2.0.1
#include <SimpleFOC.h>
#include <SPI.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <Wire.h>
#include <AS5600.h>

//#######_USER VARIABLES_#######
byte pp = 7;                  //BLDC motor number of pole pairs
float phaseRes = 4.8;       //Phase winding resistance [ohms]
float sourceVoltage = 12.5;      //Voltage of your power source [Volts]
float maxCurrent = 2;          //Rough approximation of max current [Amps]

//#######_CONTROLLER PARAMETERS_#######
float ki = 0.28;             //Velocity control loop PROPORTIONAL gain value
float ti = 2.12;               //Velocity control loop INTEGRAL gain value
float lpFilter = 0.05;
float kp = 0;                //Position control loop PROPORTIONAL gain value
//float voltageRamp = 100;       //Change in voltage allowed [Volts per sec]
float voltageLimit = phaseRes*maxCurrent;
float velocityLimit = 200;   //Velocity limit [rpm]
 
//#######_DRV8305_########
//Datasheet: www.ti.com/lit/ds/symlink/drv8305.pdf
#define enGate 17       //Chip Enable
#define nFault 14       //Fault reading
#define cs 5           //DRV8305 Chip-select
#define so1 36
#define so2 35
#define so3 34
bool faultTrig = false;

//####_CURRENT READING_####
/*
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
static portMUX_TYPE mcpwm_spinlock = portMUX_INITIALIZER_UNLOCKED;
int a1, a2, a3;         //Current readings from internal current sensor amplifiers
static void IRAM_ATTR isr_handler(void*);
byte currentState = 1;
*/

//######_TEMPERATURE SENSOR_######
#define vTemp 39
byte maxTemp = 80;      //Maximum temperature [°C]
float temp;

//#####_TIME MANAGEMENT_#####
float runTime, prevT = 0, timeDif, stateT, sampleTime;
int timeInterval = 1000, totalTempTime;
long swingDownTime = 0, swingUpTime = 0;

//####_SIMPLEFOC INSTANCES_####
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);
MagneticSensorAnalog pend = MagneticSensorAnalog(12, 15, 4095);
float zero_pend = 0;


BLDCMotor motor = BLDCMotor(pp, phaseRes, 496, 0.00145488);   //BLDCMotor instance
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, enGate);     //3PWM Driver instance
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }



//#####_CONTROL VARIABLES_#####
float target_voltage;         // Voltage applied to the motor depending on the corresponding control algorithm
bool overTurn = false;        // if many turn have occured (encoder cable chocking with no slip ring)
float cosFourth = 0;          // Term used for the Energy Shaping Controller
int energy = 0;               // decided wether to use evergy shaping for swing-up or swing-down (currently swing-down is not used)
bool sSwitch = false;         // decide if stability has been achieved
float balanceAngle = 1.1; 
int pendulumControl = 1;


//####_GAINS OF THE CONTROLLERS_#####
// LQR
float k1 = 23.7547/10;
float k2 = 2.0320/10;
float k3 = -0.800/10;
float k4 = -0.1754/10;                                         
// P controller (return from overturning)
float k5 = 1;
// Energy Shaping
float k6 = 0.00365;

LowPassFilter LPFpend{0.1};
LowPassFilter LPFmot{0.1};
LowPassFilter LPFswitch{0.02};
LowPassFilter LPFvoltage{0.02};

//--------------------------------------------Set-up--------------------------------------------
void setup() {
  Serial.begin(115200);


  //Pinmodes
  pinMode(so1, INPUT);
  pinMode(so2, INPUT);
  pinMode(so3, INPUT);
  //pinMode(nFault, INPUT);
  pinMode(enGate, OUTPUT);
  digitalWrite(enGate, LOW);
  //SPI start up
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  //Motor driver initialization
  delay(250);
  Serial.println("DRV8305 INIT");
  drv_init();
  delay(500);
  Serial.println("enGate Enabled");
  digitalWrite(enGate, HIGH);
  delay(500);

  
  // velocity PI controller parameters, default K=0.5 Ti = 0.01
  motor.PID_velocity.P = ki;
  motor.PID_velocity.I = ti;


  motor.PID_current_q.P = 3;                       
  motor.PID_current_q.I = 30000;                  
  //motor.PID_current_q.ramp = 1000;                  
  // Low pass filtering - default 
  motor.LPF_current_q.Tf= 0.01;                         
  // D axis
  // PID parameters - default 
  motor.PID_current_d.P = 3;                       
  motor.PID_current_d.I = 30000;                   
  //motor.PID_current_d.ramp = 1000;                  
  // Low pass filtering - default 
  motor.LPF_current_d.Tf= 0.01;   
  
  pend.init();
  _delay(500);

  Serial.println("Pendulum Sensor Ready.");


  //Initialise magnetic sensor hardware
  sensor.init();
  motor.linkSensor(&sensor);
  motor.velocity_limit = 100; 
  driver.voltage_power_supply = sourceVoltage;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.voltage_sensor_align = voltageLimit;
  motor.current_limit = maxCurrent;
  motor.phase_resistance = phaseRes;
  command.add('M',doMotor,"motor");
  motor.useMonitoring(Serial);      // use monitoring functionality
  motor.init();                     // initialise motor
  motor.initFOC();            // align sensor/ encoder and start FOC
  Serial.println("Ready BLDC.");
  
  _delay(1000);
  Serial.println(motor.zero_electric_angle);
  motor.zero_electric_angle = 5.35; 

  zero_pend = pend.getAngle();
  
  //Current sensing interrupt routines
  /*
  MCPWM[MCPWM_UNIT_0]->int_ena.timer0_tep_int_ena = true;  //A PWM timer 0 TEP event will trigger this interrupt
  MCPWM[MCPWM_UNIT_0]->int_ena.timer1_tep_int_ena = true;//A PWM timer 1 TEP event will trigger this interrupt
  MCPWM[MCPWM_UNIT_0]->int_ena.timer2_tep_int_ena = true;//A PWM timer 2 TEP event will trigger this interrupt
  mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
  */
  
}

//---------------------------------------Loop-------------------------------------------
void loop() {
  motor.loopFOC();
  serialEvent();
  command.run();
  
  //Time managment
  runTime = micros();
  timeDif = runTime - prevT;
  prevT = runTime;
  stateT += timeDif;
  sampleTime += timeDif;

  // control loop each ~10ms (100 Hz)
  if (sampleTime >= 10000){

    sampleTime = 0;
    // calculate the pendulum angle and velocity, and motor angle and velocity with respective filtering
    pend.update();
    float pendulum_angle = constrainAngle(pend.getAngle() - zero_pend + M_PI);
    float pendulum_velocity = LPFpend( pend.getVelocity() );
    float motor_velocity = LPFmot(motor.shaftVelocity() );
    float motor_angle = sensor.getAngle();

    float pendulum_velocity_switch = LPFswitch( pend.getVelocity() );

      if ( abs(pendulum_angle) < balanceAngle ) {
        target_voltage = controllerLQR( pendulum_angle, pendulum_velocity, motor_velocity, motor_angle, k1, k2, k3, k4 );
      }
      else
      {
        target_voltage = 0; 
      }
  }
  motor.move(-target_voltage);
  
  // 5 Hz function caller for Janus Controller security features. 
  if(stateT >= 200000){
    stateT = 0;
     float pendulum_angle = constrainAngle(pend.getAngle() - zero_pend + M_PI);
      // Serial.println(pendulum_angle) ;
  }
  
}

//----------------------------------Control Functions-----------------------------------

// Function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return (x - M_PI);
}

// LQR stabilization controller functions
// calculates the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel, float m_pos, float c1, float c2, float c3, float c4){
  // calculate the control law 
  // LQR controller u = k*x
  //  - k = [c1, c2, c3, c4]
  //  - x = [pendulum angle, pendulum velocity, motor velocity, motor position]' 
  float u =  c1*p_angle + c2*p_vel + c3*m_vel + c4*m_pos;
  u = -u; 
  
  // limit the voltage set to the motor (saturation)
  if(abs(u) > voltageLimit) u = _sign(u)*voltageLimit;
 /* Serial.println("Controller LQR : ");
  Serial.println(p_angle);
  Serial.println(u);*/
  
  return u;
}

// Energy shaping controller
// calculates the voltage that needs to be set to the motor in order to add/ remove energy to the pendulum
float energyShaping(float p_angle, float p_vel, float c6){
  
  float cosFourth = cos( p_angle )*cos( p_angle )*cos( p_angle )*cos( p_angle );
  float u = c6*cosFourth*(p_vel)*(9.81*(1-cos(p_angle)-(0.0075*(p_vel)*(p_vel))));

  if ( abs(target_voltage) > voltageLimit*1 ) target_voltage = _sign(target_voltage)*voltageLimit*1;

  return u;
}


//Temperature status and manager
void tempStatus(){
  static int tFlag;

  //Read voltage from temperature sensor and transform it to °C
  float vOut = analogRead(vTemp);
  temp = (((vOut*3.3)/4095)-0.4)/0.0195;
  //Serial.println(temp,2);
  
  if (temp >= 80 && tFlag == false){
    int tempTime = micros();
    totalTempTime += tempTime;

    //If temperature is high for 3 seconds disable DRV
    if(totalTempTime >= 3000000){
      tFlag = true;
      digitalWrite(enGate, LOW);
      Serial.print("enGate Disabled - Temperature protection: ");
      Serial.println(temp);
    }
    
  }
  else if (temp <= 80 && tFlag == false){
    totalTempTime = 0;
  }
  
}

//Configure DRV8305 to desired operation mode
void drv_init(){
  
  //Set to three PWM inputs mode
  digitalWrite(cs, LOW);
  byte resp1 = SPI.transfer(B00111010);
  byte resp2 = SPI.transfer(B10000110);
  digitalWrite(cs, HIGH);

  Serial.println(resp1, BIN);
  Serial.println(resp2, BIN);
  
}

//Fault status and manager for the DRV8305
//Datahseet pages 37 and 38
void faultStatus(){
  //Read nFault pin from DRV8305 - LOW == error / HIGH == normal operation
  int fault = digitalRead(nFault);

  Serial.println(nFault); 
  Serial.println(faultTrig); 
 // faultTrig = false ;

  
  if(fault == LOW && faultTrig == false){
    Serial.println("Fault detected");
    faultTrig = true;
    //Check warning and watchdog reset (Address = 0x1)
    digitalWrite(cs, LOW);
    byte ft1 = SPI.transfer(B10001000);
    byte ft2 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x1");
    Serial.println(ft1);
    Serial.println(ft2);

    //Check OV/VDS Faults (Address = 0x2)
    digitalWrite(cs, LOW);
    byte ft3 = SPI.transfer(B10010000);
    byte ft4 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x2");
    Serial.println(ft3,BIN);
    Serial.println(ft4,BIN);

    //Check IC Faults (Address = 0x3)
    digitalWrite(cs, LOW);
    byte ft5 = SPI.transfer(B10011000);
    byte ft6 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x3");
    Serial.println(ft5,BIN);
    Serial.println(ft6,BIN);

    //Check VGS Faults (Address = 0x4)
    digitalWrite(cs, LOW);
    byte ft7 = SPI.transfer(B10100000);
    byte ft8 = SPI.transfer(B00000000);
    digitalWrite(cs, HIGH);
    Serial.println("Address = 0x4");
    Serial.println(ft7,BIN);
    Serial.println(ft8,BIN);
  }
}

/*
static void IRAM_ATTR isr_handler(void*){
  uint32_t mcpwm_intr_status_0;
  uint32_t mcpwm_intr_status_1;
  uint32_t mcpwm_intr_status_2;
  mcpwm_intr_status_0 = MCPWM[MCPWM_UNIT_0]->int_st.timer0_tep_int_st;
  mcpwm_intr_status_1 = MCPWM[MCPWM_UNIT_0]->int_st.timer1_tep_int_st;
  mcpwm_intr_status_2 = MCPWM[MCPWM_UNIT_0]->int_st.timer2_tep_int_st;
  if(mcpwm_intr_status_0 > 0 && currentState == 1){
    a1 = analogRead(so1);
    currentState = 2; 
  }
  else if(mcpwm_intr_status_1 > 0 && currentState == 2){
    a2 = analogRead(so2);
    currentState = 3;
  }
  else if(mcpwm_intr_status_2 > 0 && currentState == 3){
    a3 = analogRead(so3);  
    currentState = 1;
  }
  MCPWM[MCPWM_UNIT_0]->int_clr.timer0_tep_int_clr = mcpwm_intr_status_0;
  MCPWM[MCPWM_UNIT_0]->int_clr.timer1_tep_int_clr = mcpwm_intr_status_1;
  MCPWM[MCPWM_UNIT_0]->int_clr.timer2_tep_int_clr = mcpwm_intr_status_2;
}
*/


