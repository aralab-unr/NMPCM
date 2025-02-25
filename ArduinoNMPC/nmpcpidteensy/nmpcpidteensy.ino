#include <Wire.h>
#include <Arduino.h>
#include <sbus.h> 
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define SBUS_BAUD_RATE 100000
bfs::SbusRx sbus_rx(&Serial5);

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN (MICOLINK_MAX_PAYLOAD_LEN + 7)
enum {
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,  // Range Sensor
};
// MICOLINK Message Structure
typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

// MICOLINK Payload for Range Sensor
#pragma pack(push, 1)
typedef struct {
    uint32_t time_ms;
    uint32_t distance;
    uint8_t strength;
    uint8_t precision;
    uint8_t dis_status;
    uint8_t reserved1;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)

static_assert(sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t) <= MICOLINK_MAX_PAYLOAD_LEN, "Payload size exceeds maximum limit");
#define NX          ACADO_NX    // Number of differential states
#define NXA         ACADO_NXA   // Number of algebraic states
#define NU          ACADO_NU    // Number of control inputs
#define N           ACADO_N     // Number of control intervals
#define NOD         ACADO_NOD   // Number of online data
#define NY          ACADO_NY    // Number of references, nodes 0..N - 1
#define NYN         ACADO_NYN   // Number of references for node N
#define NUM_STEPS   1           // Number of simulation steps
#define VERBOSE     1           // Show iterations: 1, Silent: 0

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class PID {
public:
    PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), previous_error_(0), integral_(0) {}

    double calculate(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        double Pout = kp_ * error;
        integral_ += error * dt;
        integral_ = constrain(integral_, -1.25, 1.25);
        double Iout = ki_ * integral_;
        double derivative = (error - previous_error_) / dt;
        double Dout = kd_ * derivative;
        double output = Pout + Iout + Dout;
        previous_error_ = error;
        return output;
    }

private:
    double kp_, ki_, kd_;
    double previous_error_, integral_;
};

float vphi, vtheta, vpsi;
float vphicalibration, vthetacalibration, vpsicalibration;
int RateCalibrationNumber;
float AccX, AccY, AccZ;

float phi, theta, psi;
uint32_t LoopTimer;

float phikalman = 0, phiunkalman = 4;
float thetakalman = 0, thetaunkalman = 4;
float psikalman = 0, psiunkalman = 4;
float Kalman1DOutput[2];
float ch0 = 0.0, ch1 = 0.0, ch2 = 0.0, ch3 = 0.0, ch4 = 0.0, ch5 = 0.0;

float phir, thetar, phidr, thetadr, psidr;
float pwm_value1, pwm_value2, pwm_value3, pwm_value4;
double x = 0, y = 0, z = 0;
double xd = 0, yd = 0, zd = 0;
double m = 1.83, g = 9.8;  
double degtorad = 0.01745;

unsigned long lastOpticalTime = 0;

PID controlphirate(1.05, 0.05, 0.025);   
PID controlthetarate(1.05, 0.05, 0.025);   
PID controlpsirate(7.75, 12.75, 0.0); 

void setup() {
  Serial.begin(115200);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    vphicalibration += vphi;
    vthetacalibration += vtheta;
    vpsicalibration += vpsi;
    delay(1);
  }
  
  vphicalibration /= 2000;
  vthetacalibration /= 2000;
  vpsicalibration /= 2000;

  Serial5.begin(SBUS_BAUD_RATE, SERIAL_8E2);
  sbus_rx.Begin();
  Serial3.begin(115200);   // Optical flow sensor
  
  acado_initializeSolver();

  analogWriteFrequency(3, 100);
  analogWriteFrequency(4, 100);
  analogWriteFrequency(5, 100);
  analogWriteFrequency(6, 100);
  analogWriteResolution(12);

  analogWrite(3, 409); // motor 1
  analogWrite(4, 409); //motor 2
  analogWrite(5, 409); // motor 3
  analogWrite(6, 409); //motor 4
  delay(3000);

  LoopTimer=micros();
}

void loop() {

  gyro_signals();
  read_receiver();
  getoptical();
  double zdes = map(ch2, 74, 1668, 0.0, 2.0);
  double xdes = x / 1000 - map(ch1, 540, 1925, -2.0, 2.0);
  double ydes = y / 1000 - map(ch0, 541, 1924, -2.0, 2.0);
  double psides = 0.005 * (ch3 - 1001);

  vphi -= vphicalibration;
  vtheta -= vthetacalibration;
  vpsi -= vpsicalibration;

  kalman_1d(phikalman, phiunkalman, vphi, phi);
  kalman_1d(thetakalman, thetaunkalman, vtheta, theta);
  kalman_1d(psikalman, psiunkalman, vpsi, psi);  

  phir = 0.025 * (ch0 - 1237);
  thetar = -0.025 * (ch1 - 1242);
  psidr = 0.05 * (ch3 - 972);

  phidr = 4 * (phir - phikalman);
  thetadr = 4 * (thetar - thetakalman);

  double U1 = map(ch2, 74, 1668, 1000, 1800);
  double U2 = constrain(controlphirate.calculate(phidr, vphi, 0.01), -125, 125);
  double U3 = constrain(controlthetarate.calculate(thetadr, vtheta, 0.01), -125, 125);
  double U4 = constrain(controlpsirate.calculate(psidr, vpsi, 0.01), -105.0, 105.0);

  unsigned int i;

  // Reset all solver memory
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Prepare a consistent initial guess
  for (i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + 0] = x / 1000;
      acadoVariables.x[i * NX + 1] = y / 1000;
      acadoVariables.x[i * NX + 2] = z / 1000;
      acadoVariables.x[i * NX + 3] = phikalman * degtorad;
      acadoVariables.x[i * NX + 4] = thetakalman * degtorad;
      acadoVariables.x[i * NX + 5] = psi * degtorad;
      acadoVariables.x[i * NX + 6] = xd / 1000;
      acadoVariables.x[i * NX + 7] = yd / 1000;
      acadoVariables.x[i * NX + 8] = zd / 1000;
      acadoVariables.x[i * NX + 9] = vphi * degtorad / 7.5;
      acadoVariables.x[i * NX + 10] = vtheta * degtorad / 7.5;
      acadoVariables.x[i * NX + 11] = vpsi * degtorad / 7.5;
  }

  // Prepare references
  for (i = 0; i < N; ++i) {
      acadoVariables.y[i * NY + 0] = zdes;
      acadoVariables.y[i * NY + 1] = phir * degtorad; // phides (placeholder)
      acadoVariables.y[i * NY + 2] = thetar * degtorad; // thetades (placeholder)
      acadoVariables.y[i * NY + 3] = psides;
      acadoVariables.y[i * NY + 4] = m * g;
      acadoVariables.y[i * NY + 5] = U2 / 12.5 * degtorad;
      acadoVariables.y[i * NY + 6] = U3 / 12.5 * degtorad;
      acadoVariables.y[i * NY + 7] = U4 / 10.5 * degtorad;
  }

  acadoVariables.yN[0] = zdes;
  acadoVariables.yN[1] = phir * degtorad; // phides (placeholder)
  acadoVariables.yN[2] = thetar * degtorad; // thetades (placeholder)
  acadoVariables.yN[3] = psides;

  for (i = 0; i < NX; ++i) {
      acadoVariables.x0[i] = acadoVariables.x[i];
  }

  // Perform ACADO steps
  acado_preparationStep();
  acado_feedbackStep();
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  // Retrieve control variables
  real_t* u = acado_getVariablesU();
  double U1r = map(u[0], 12.5, 22.5, 1000, 1800);
  double U2r = 12.75 * u[1] / degtorad;
  double U3r = 12.75 * u[2] / degtorad;
  double U4r = 10.75 * u[3] / degtorad;

  if (ch4 < 700 && ch5 < 700) {
    pwm_value1 = constrain(0.409 * (U1r - U2r - U3r - U4r), 409, 800);
    pwm_value2 = constrain(0.409 * (U1r + U2r - U3r + U4r), 409, 800);
    pwm_value3 = constrain(0.409 * (U1r - U2r + U3r + U4r), 409, 800);
    pwm_value4 = constrain(0.409 * (U1r + U2r + U3r - U4r), 409, 800);
  }
  else {
    pwm_value1 = 409;
    pwm_value2 = 409;
    pwm_value3 = 409;
    pwm_value4 = 409; 
  }
  analogWrite(3, pwm_value1); // motor 1
  analogWrite(5, pwm_value2); //motor 2
  analogWrite(4, pwm_value3); // motor 3
  analogWrite(6, pwm_value4); //motor 4

  float dt = (micros() - LoopTimer) / 1000000.0;
  psi = psi + dt * vpsi;

  //Serial.print(x); Serial.print(","); 
  //Serial.print(y); Serial.print(",");
  //Serial.print(z); Serial.print(",");
  //Serial.print(phi); Serial.print(",");
  //Serial.print(theta); Serial.print(",");
  //Serial.print(psi); Serial.print(",");
  //Serial.print(U1r); Serial.print(",");
  //Serial.print(U2r); Serial.print(",");
  //Serial.print(U3r); Serial.print(",");
  //Serial.print(U4r); Serial.print(",");
  //Serial.print(pwm_value1); Serial.print(",");
  //Serial.print(pwm_value2); Serial.print(",");
  //Serial.print(pwm_value3); Serial.print(",");
  //Serial.print(pwm_value4); Serial.print(",");
  //Serial.print(phir); Serial.print(",");
  //Serial.print(thetar); Serial.print(",");
  //Serial.print(psides); Serial.println(";");

  while (micros() - LoopTimer < 10000);
  LoopTimer = micros();
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.01 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.01 * 0.01 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); 
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  vphi = (float)GyroX / 65.5;
  vtheta = (float)GyroY / 65.5;
  vpsi = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096 - 0.02;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096 - 0.22;
  
  phi = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180 / PI);
  theta = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180 / PI);
}

void read_receiver() {
    if (sbus_rx.Read()) {
        bfs::SbusData data = sbus_rx.data();
        ch0 = data.ch[0];
        ch1 = data.ch[1];
        ch2 = data.ch[2];
        ch3 = data.ch[3];
        ch4 = data.ch[8];
        ch5 = data.ch[9];
    }
}

void getoptical() {
    while (Serial3.available()) {
        uint8_t sensor_data = Serial3.read();
        micolink_decode(sensor_data);
    }
}

void micolink_decode(uint8_t data) {
    static MICOLINK_MSG_t msg;

    if (!micolink_parse_char(&msg, data)) {
        return;
    }

    if (msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
        MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
        memcpy(&payload, msg.payload, msg.len);
        unsigned long currentOpticalTime = millis();
        float dtOptical = (currentOpticalTime - lastOpticalTime) / 1000.0;

        if (payload.distance < 5000 && abs(payload.flow_vel_x) < 1250 && abs(payload.flow_vel_y) < 1250) {
            if (dtOptical > 0) {
                zd = (payload.distance - z) / dtOptical;
                z = payload.distance;
                yd = payload.flow_vel_y * z * 0.01;
                xd = -payload.flow_vel_x * z * 0.01;

                x += xd * dtOptical;
                y += yd * dtOptical;
                lastOpticalTime = currentOpticalTime;
            }
        }
    }
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
    switch (msg->status) {
        case 0: 
            if (data == MICOLINK_MSG_HEAD) {
                msg->head = data;
                msg->status = 1;
            }
            break;
        case 1: msg->dev_id = data; msg->status = 2; break;
        case 2: msg->sys_id = data; msg->status = 3; break;
        case 3: msg->msg_id = data; msg->status = 4; break;
        case 4: msg->seq = data; msg->status = 5; break;
        case 5:
            msg->len = data;
            if (msg->len == 0) {
                msg->status = 7;
            } else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) {
                msg->status = 0;
            } else {
                msg->status = 6;
            }
            break;
        case 6:
            msg->payload[msg->payload_cnt++] = data;
            if (msg->payload_cnt == msg->len) {
                msg->payload_cnt = 0;
                msg->status = 7;
            }
            break;
        case 7: 
            msg->checksum = data;
            msg->status = 0;
            return true;
        default: 
            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }
    return false;
}

