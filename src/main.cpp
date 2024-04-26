#include <Arduino.h>
#include <SimpleFOC.h>
// #include "encoders/esp32hwencoder/ESP32HWEncoder.h"
#include "current_sense/hardware_specific/esp32/esp32_i2s_driver.h"

#include "board.h"

HFIBLDCMotor motor2 = HFIBLDCMotor(7, 0.5f, 640.0f);

// BLDCDriver3PWM driver2 = BLDCDriver3PWM(A1, B1, C1); // normal channel 2
// LowsideCurrentSense current_sensor2 = LowsideCurrentSense(0.01f, 50.0f, I_A1, I_B1); // normal channel 2

BLDCDriver3PWM driver2 = BLDCDriver3PWM(A1, B1, B0); // channel 2, but phase c connected to B1, to use 3x current sensing
LowsideCurrentSense current_sensor2 = LowsideCurrentSense(0.01f, 50.0f, I_A1, I_B1, I_B0); // channel 2, but phase c connected to B1, to use 3x current sensing

// ESP32HWEncoder sensor2 = ESP32HWEncoder(SCL_1, SDA_1, 1024, I_1); 

// Commander
Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor2,cmd);}

float lqtarget = 0.5 * 3.38e-3f;
float ldq_div = (0.5 * 2.75e-3f) / lqtarget;
float dohfivolts = 1.0f;
#define MIN_L (2e-6f)
#define MAX_L (50e-3f)
#define MAX_DIFF 1.5f
#define MIN_DIFF 0.5f
#define MAX_HFI_V 10.0f
#define MIN_HFI_V 0.01f

// Set motor2 Lq directly and set Ld to the diffs percentage of Lq
void doLq(char* cmd) { 
  command.scalar(&lqtarget, cmd); 
  if (lqtarget <= MAX_L && lqtarget >= MIN_L && (lqtarget * ldq_div) <= MAX_L && (lqtarget * ldq_div) >= MIN_L)
  {
    motor2.Lq = lqtarget;
    motor2.Ld = lqtarget * ldq_div;
  } 
}

// Set Lq to the same value as before and change the percentage of Lq that Ld is, (Parameter = Ld/Lq)
void doLd_div_Lq(char* cmd) { 
  command.scalar(&ldq_div, cmd); 
  if(ldq_div <= MAX_DIFF && ldq_div >= MIN_DIFF) {
    if (lqtarget <= MAX_L && lqtarget >= MIN_L && (lqtarget * ldq_div) <= MAX_L && (lqtarget * ldq_div) >= MIN_L)
    {
      motor2.Lq = lqtarget;
      motor2.Ld = lqtarget * ldq_div;
    }
  }
}

// Set motor2 Lq directly and set Ld to the diffs percentage of Lq
void doHfiV(char* cmd) { 
  command.scalar(&dohfivolts, cmd); 
  if (dohfivolts <= MAX_HFI_V && dohfivolts >= MIN_HFI_V)
  {
    motor2.hfi_v = dohfivolts;
  }
}

// end Commander

void IRAM_ATTR process_hfi(){
  motor2.process_hfi();
}

void setup() {
  Serial.begin(921600);
  _delay(1000);
  SimpleFOCDebug::enable();
  // sensor2.init();
  Serial.println("Boot. Linking sensor.");
  // motor2.linkSensor(&sensor2);
  // driver config
  // power supply voltage [V]
  driver2.voltage_power_supply = 12;
  driver2.pwm_frequency = 18000;
  driver2.voltage_limit = 12;
  motor2.current_limit = 1.5;
  motor2.voltage_limit = 1;
  motor2.voltage_sensor_align = 0.2;


  Serial.println("Init driver");
  driver2.init();

  Serial.println("Link Driver");
  motor2.linkDriver(&driver2);

  Serial.println("Link current sense to motor");
  motor2.linkCurrentSense(&current_sensor2);

  Serial.println("Link driver to current sense");
  current_sensor2.linkDriver(&driver2);
  current_sensor2.skip_align = false;

  Serial.println("Init Current sense");
  current_sensor2.init();

  // add custom commands to Commander
  command.add('M',&onMotor,"motor");
  command.add('Q', doLq, "set Lq = arg ; set Ld = arg * diff");
  command.add('D', doLd_div_Lq, "Change diff: set Lq = Lq ; set Ld = Lq * arg");
  command.add('H', doHfiV, "change hfi injection voltage between 0.01v and 10v");
  command.decimal_places = 6;

  // set torque mode:
  // TorqueControlType::dc_current
  // TorqueControlType::voltage
  // TorqueControlType::foc_current
  motor2.torque_controller = TorqueControlType::foc_current;
  // set motion control loop to be used
  motor2.controller = MotionControlType::torque;

  
  motor2.LPF_current_q.Tf = 0.001f; // 1ms default
  motor2.LPF_current_d.Tf = 0.001f; // 1ms default
  

  motor2.PID_velocity.P = 0.2f;
  motor2.PID_velocity.I = 0;
  motor2.PID_velocity.D = 0;

  motor2.P_angle.P = 3;
  motor2.P_angle.I = 0;
  motor2.P_angle.D = 0.01;

  motor2.P_angle.limit = 10.0f;
  motor2.velocity_limit = 100.0f;
  
  motor2.LPF_velocity.Tf = 0;
  motor2.LPF_angle.Tf = 0;

  motor2.Ld = 0.06e-3f;
  motor2.Lq = 0.04e-3f;

  Serial.println("Init Motor");
  motor2.init();

  Serial.println("Init FOC");
  motor2.initFOC();

  Serial.println("Init complete, starting loop");
  
  motor2.sensor_direction = Direction::CW;
  motor2.hfi_on = true;
  motor2.current_setpoint.d = 0.0f;
  motor2.hfi_v = 2.0f;

  motor2.target = 0.0f;

  // preset Commander variables to actual values
  dohfivolts = motor2.hfi_v;
  lqtarget = motor2.Lq;
  ldq_div = motor2.Ld / motor2.Lq;

}

int cnt = 0;
int t0 = micros();
int t1 = micros();

float lastangle = 0.0f;


void loop() {
  t0 = t1;
  t1 = micros();
  cnt++;
  
  motor2.move();
  // motor2.move(_2PI*_sin(float(t1)/1000000.0f));
  motor2.loopFOC();

  if(cnt > 1000){
    // printdbg(); // ADC Debug

    command.run(); // Commander

    // sensor2.update();

    // Teleplot printing:

    // Serial.printf(">M2A:%f\n", motor2.shaft_angle);
    // Serial.printf(">M2Vrad:%f\n", motor2.shaft_velocity);
    
    // Serial.printf(">sensor2Angle:%f\n", sensor2.getAngle());
   
    // Serial.printf(">Iq2:%f\n", (motor2.current_high.q + motor2.current_low.q)/2);
    // Serial.printf(">Id2:%f\n", (motor2.current_high.d + motor2.current_low.d)/2);
    
    // PhaseCurrent_s currents = current_sensor2.getPhaseCurrents();
    // Serial.printf(">Ia2:%f\n", currents.a);
    // Serial.printf(">Ib2:%f\n", currents.b);
    // Serial.printf(">Ic2:%f\n", currents.c);

    // Serial.printf(">Ua2:%f\n", motor2.Ua);
    // Serial.printf(">Ub2:%f\n", motor2.Ub);
    // Serial.printf(">Uc2:%f\n", motor2.Uc);

    // Serial.printf(">deltaId2:%f\n", motor2.delta_current.d);
    // Serial.printf(">deltaIq2:%f\n", motor2.delta_current.q);

    // Serial.printf(">loopFreq:%f\n", 1000000.0f/(t1 - t0));
    cnt = 0;
  }
}
