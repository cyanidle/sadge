#define BAUD_RATE 115200
#define MAX_SERVOS 15
#define MOTORS_COUNT 3
#define MSG_BUFFER 256
#define ODOM_DELAY_MS 30
#define ECHO_MSGS 1
#define START_PIN 48

#include <Arduino.h>
#include "motor.hpp"


static Motor motors[MOTORS_COUNT] = {};

struct PinState {
  explicit PinState(int pin, int neededCount) : pin(pin), neededCount(neededCount) {}
  bool measure{ false };
  bool current{ false };
  int neededCount{};
  int currentCount{ 0 };
  int pin{};
};

bool debounce(PinState& state) {
  state.measure = digitalRead(state.pin);
  if (state.measure == state.current) {
    state.currentCount = 0;
  } else {
    state.currentCount++;
  }
  if (state.currentCount > state.neededCount) {
    state.currentCount = state.neededCount;
    state.current = state.measure;
  }
  return state.current;
}

static void Update() {
  for (auto i = 0; i < MOTORS_COUNT; ++i) {
    motors[i].Update();
  }
}

static void Move(float x, float y, float z) {
  for (auto i = 0; i < MOTORS_COUNT; ++i) {
    motors[i].SpeedCallback(x, y, z);
  }
}

static void Forward(float x) {
  Move(x, 0, 0);
}

static void Stop() {
  Move(0, 0, 0);
}

static void Turn(float z) {
  Move(0, 0, z);
}

static void Rigth(float y) {
  Move(0, y, 0);
}

static void WaitPin() {
  static PinState pin(START_PIN, 3);
  while (!debounce(pin)) {
    delay(5);
    Update();
  }
}

template<size_t I>
void MotorCb() {
  if (digitalRead(motors[I].pinout.encoderB) == HIGH) {
    motors[I].X++;
  } else {
    motors[I].X--;
  }
}

static void Freeze(bool state) {
    for (auto i = 0; i < MOTORS_COUNT; ++i) {
      motors[i].enabled = !state;
    }
    Update();
}

const int P = 450, I = 600, D = 1;

void Route();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  pinMode(START_PIN, INPUT_PULLUP);
  Serial.begin(BAUD_RATE);
  ConfigPinout pinout = {};
  pinout.encoderA = 18;
  pinout.encoderB = 31;
  pinout.enable = 12;
  pinout.fwd = 34;
  pinout.back = 35;
  motors[0].SetPinout(MotorCb<0>, pinout);
  pinout.encoderA = 19;
  pinout.encoderB = 38;
  pinout.enable = 8;
  pinout.fwd = 37;
  pinout.back = 36;
  motors[1].SetPinout(MotorCb<1>, pinout);
  pinout.encoderA = 3;
  pinout.encoderB = 49;
  pinout.enable = 9;
  pinout.fwd = 43;
  pinout.back = 42;
  motors[2].SetPinout(MotorCb<2>, pinout);
  ConfigMotor params = {};
  params.propCoeff = P;
  params.interCoeff = I;
  params.diffCoeff = D;
  params.radius = 0.18;
  params.maxSpeed = 0.55;
  params.turnMaxSpeed = 0.25;
  params.ticksPerRotation = 360;
  params.coeff = 1;
  params.angleDegrees = 90;
  motors[0].SetParams(params);
  params.angleDegrees = 210;
  motors[1].SetParams(params);
  params.angleDegrees = 330;
  motors[2].SetParams(params);
  Freeze(true);
  WaitPin();
}

void loop() {
  Freeze(false);
  Route();
  Freeze(true);
  while(1){}
}

static void Sleep(int ms) {
  auto start = millis();
  while (millis() - start < ms) {
    delay(10);
    Update();
  }
}

void Route() {

  Forward(1);
  Sleep(2000);
  Turn(1);
  Sleep(2000);
  Turn(-1);
  Sleep(2000);
  Stop();


}
