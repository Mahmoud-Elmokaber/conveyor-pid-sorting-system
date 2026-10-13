#include <Servo.h>
#include <Arduino.h>
// =====================================================
//  Conveyor PID speed control + height-based sorting
//  Wire the Proteus schematic EXACTLY like this:
//    D2  <- MOTOR-ENCODER pulse output
//    D3  -> L293D IN2  (pin 7)
//    D4  -> L293D IN1  (pin 2)
//    D5  -> L293D EN1  (pin 1)  (PWM)
//    D8  <- HC-SR04 ECHO
//    D9  -> HC-SR04 TRIG
//    D10 -> Servo signal
//    D0/D1 <-> Virtual Terminal (crossed: D0->TXD, D1->RXD), 9600 baud
// =====================================================

// --- Pins ---
const byte IR_PULSE_PIN = 2;
const byte IN2_PIN = 3;
const byte IN1_PIN = 4;
const byte ENA_PIN = 5;
const byte ECHO_PIN = 8;
const byte TRIG_PIN = 9;
const byte SERVO_PIN = 10;

// --- Calibration mode ---
// Set to 1: no PID. Sweeps PWM 0..255 (step 15, 4 s each) and prints ">>> PWM x settled at RPM y".
// Use the max RPM you see to pick a realistic targetRPM (about 50-70% of max).
#define OPEN_LOOP_TEST 0
const byte TEST_PWM = 255;

Servo sortingGate;

// --- Speed measurement ---
volatile unsigned long pulseCount = 0;
const float PULSES_PER_REV = 24.0; // measured: matches the Proteus encoder (69.2 RPM display)
const float RPM_FILTER = 0.3;      // 0..1, lower = smoother
float currentRPM = 0.0;

// --- PID ---
float targetRPM = 100; // must be BELOW the motor's max RPM (see calibration)
float Kp = 5;
float Ki = 7;
float Kd = 6; // keep 0 until P and I behave
float error = 0.0, lastPV = 0.0, integralTerm = 0.0, controlSignal = 0.0;

const unsigned long SAMPLE_MS = 100; // longer window = less pulse quantization
unsigned long lastPIDTime = 0;

// --- Sorting ---
const float SENSOR_TO_BELT_DIST = 100.0;     // simulation "cm": set to the D: reading with the pot at "empty belt"
const float HEIGHT_THRESHOLD = 30.0;         // simulation "cm": boxes at least this tall get rejected
const float MIN_OBJECT_HEIGHT = 5.0;         // simulation "cm"
const unsigned long ECHO_TIMEOUT_US = 60000; // lower to ~30000 once the pot is set to a short distance
const unsigned long ULTRA_MS = 60;
const unsigned long DIVERT_HOLD_MS = 1500; // keep gate open until box reaches it
const byte GATE_PASS = 0, GATE_REJECT = 90;

unsigned long lastUltraTime = 0;
unsigned long divertUntil = 0;
byte tallCount = 0;
byte gatePos = 255;
float objectHeight = 0.0;
float lastDistance = -1.0; // -1 means "no echo received"

void countPulse() { pulseCount++; }

void setGate(byte pos)
{
  if (pos != gatePos)
  {
    sortingGate.write(pos);
    gatePos = pos;
  }
}

float readUltrasonic()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US); // the Proteus TEP model reports long distances
  if (duration == 0)
    return -1.0; // no echo received
  return (duration * 0.0343) / 2.0;
}

void setup()
{
  Serial.begin(9600);

  pinMode(IR_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PULSE_PIN), countPulse, RISING);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  sortingGate.attach(SERVO_PIN, 1000, 2000); // 1-2 ms range: write(90) = 1500 us = 0 deg in Proteus
  setGate(GATE_PASS);
}

void loop()
{
  unsigned long now = millis();

  // ---------- 1. PID loop (fixed interval) ----------
  if (now - lastPIDTime >= SAMPLE_MS)
  {
    float dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float rawRPM = ((float)pulses / PULSES_PER_REV) * (60.0 / dt);
    currentRPM += RPM_FILTER * (rawRPM - currentRPM); // low-pass filter

    int pwmOutput;

#if OPEN_LOOP_TEST
    // Sweep PWM 0..255 in steps of 15, 4 s per step, repeating.
    // At each step change, print the RPM the motor settled at.
    static unsigned long lastStep = 0;
    unsigned long step = (now / 4000UL) % 18;
    if (step != lastStep)
    {
      Serial.print(">>> PWM ");
      Serial.print(lastStep * 15);
      Serial.print(" settled at RPM ");
      Serial.println(currentRPM, 1);
      lastStep = step;
    }
    pwmOutput = (int)(step * 15);
    error = targetRPM - currentRPM;
#else
    error = targetRPM - currentRPM;

    integralTerm += Ki * error * dt;
    integralTerm = constrain(integralTerm, 0.0, 255.0); // anti-windup in output units

    float derivative = -(currentRPM - lastPV) / dt; // on measurement: no kick
    lastPV = currentRPM;

    controlSignal = Kp * error + integralTerm + Kd * derivative;
    pwmOutput = constrain((int)controlSignal, 0, 255);
#endif

    analogWrite(ENA_PIN, pwmOutput);

    Serial.print("SP:");
    Serial.print(targetRPM, 1);
    Serial.print(" PV:");
    Serial.print(currentRPM, 1);
    Serial.print(" ERR:");
    Serial.print(error, 1);
    Serial.print(" PWM:");
    Serial.print(pwmOutput);
    Serial.print(" D:");
    Serial.print(lastDistance, 1); // raw cm, -1.0 = no echo
    Serial.print(" T:");
    Serial.print(ECHO_TIMEOUT_US / 1000); // build marker: echo timeout in ms
    Serial.print(" H:");
    Serial.println(objectHeight, 1);
  }

  // ---------- 2. Height inspection (non-blocking cadence) ----------
  if (now - lastUltraTime >= ULTRA_MS)
  {
    lastUltraTime = now;

    lastDistance = readUltrasonic();
    objectHeight = (lastDistance < 0) ? 0.0 : (SENSOR_TO_BELT_DIST - lastDistance);

    if (objectHeight >= HEIGHT_THRESHOLD)
    {
      if (tallCount < 255)
        tallCount++;
      if (tallCount >= 2)
        divertUntil = now + DIVERT_HOLD_MS; // debounce: 2 reads in a row
    }
    else
    {
      tallCount = 0;
    }
  }

  // ---------- 3. Gate: stays open for a hold time, then returns ----------
  setGate((long)(divertUntil - now) > 0 ? GATE_REJECT : GATE_PASS);
}
