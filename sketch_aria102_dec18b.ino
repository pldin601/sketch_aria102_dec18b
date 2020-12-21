#define HALL_SENSOR_1              A0
#define HALL_SENSOR_2              A1

#define HALL_SENSOR_1_MIN          190
#define HALL_SENSOR_1_MAX          850
#define HALL_SENSOR_2_MIN          190
#define HALL_SENSOR_2_MAX          850

#define COIL_1_POSITIVE       3
#define COIL_1_NEGATIVE       9
#define COIL_2_POSITIVE       10
#define COIL_2_NEGATIVE       11

struct SensorCheckpoint {
  byte sensorId;
  int triggerLevel;
  char triggerDirection;
  byte offsetPercent;
};

struct HallSensorsValues {
  int sensor1;
  int sensor2;
};

const byte sensorCheckpointsCount = 80;
SensorCheckpoint checkpoints[sensorCheckpointsCount] = {
  { 1, 0,  1, 110 }, { 1,  100,  1, 114 }, { 1,  200,  1, 118 }, { 0, -200,  1,  98 }, { 0, -100,  1, 129 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 118 }, { 0,  200,  1, 122 }, { 1,  200, -1, 161 }, { 1,  100, -1, 118 },
  { 1, 0, -1, 114 }, { 1, -100, -1, 114 }, { 1, -200, -1, 122 }, { 0,  200, -1, 149 }, { 0,  100, -1, 126 },
  { 0, 0, -1, 118 }, { 0, -100, -1, 118 }, { 0, -200, -1, 122 }, { 1,  -200, 1, 181 }, { 1, -100,  1, 118 },

  { 1, 0,  1, 110 }, { 1,  100,  1, 110 }, { 1,  200,  1, 114 }, { 0, -200,  1, 114 }, { 0, -100,  1, 126 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 114 }, { 0,  200,  1, 118 }, { 1,  200, -1, 196 }, { 1,  100, -1, 114 },
  { 1, 0, -1, 110 }, { 1, -100, -1, 110 }, { 1, -200, -1, 110 }, { 0,  200, -1, 173 }, { 0,  100, -1, 118 },
  { 0, 0, -1, 114 }, { 0, -100, -1, 110 }, { 0, -200, -1, 122 }, { 1, -200,  1, 232 }, { 1, -100,  1, 106 },

  { 1, 0,  1, 110 }, { 1,  100,  1, 102 }, { 1,  200,  1, 114 }, { 0, -200,  1, 122 }, { 0, -100,  1, 122 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 110 }, { 0,  200,  1, 118 }, { 1,  200, -1, 181 }, { 1,  100, -1, 114 },
  { 1, 0, -1, 110 }, { 1, -100, -1, 110 }, { 1, -200, -1, 110 }, { 0,  200, -1, 169 }, { 0,  100, -1, 122 },
  { 0, 0, -1, 118 }, { 0, -100, -1, 106 }, { 0, -200, -1, 126 }, { 1, -200,  1, 189 }, { 1, -100,  1, 110 },

  { 1, 0,  1, 110 }, { 1,  100,  1, 102 }, { 1,  200,  1, 114 }, { 0, -200,  1, 126 }, { 0, -100,  1, 118 },
  { 0, 0,  1, 110 }, { 0,  100,  1, 110 }, { 0,  200,  1, 114 }, { 1,  200, -1, 204 }, { 1,  100, -1, 114 },
  { 1, 0, -1, 106 }, { 1, -100, -1, 110 }, { 1, -200, -1, 114 }, { 0,  200, -1, 173 }, { 0,  100, -1, 118 },
  { 0, 0, -1, 114 }, { 0, -100, -1, 106 }, { 0, -200, -1, 126 }, { 1, -200,  1, 189 }, { 1, -100,  1, 118 },
};

int expectedRoundTime = 1000 * (33 + 1.0 / 3) / 60;

void setup() {
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  Serial.begin(9600);
}

void loop() {
  HallSensorsValues sensorsValues = readSensors();
  float rpm = measureRpm(sensorsValues);
  int sig = computeSig(rpm, 33.333);
  writeToCoils(sensorsValues, sig);
}

HallSensorsValues readSensors() {
  int sensor1 = analogRead(HALL_SENSOR_1);
  sensor1 = map(sensor1, HALL_SENSOR_1_MIN, HALL_SENSOR_1_MAX, -255, 255);
  sensor1 = constrain(sensor1, -255, 255);

  int sensor2 = analogRead(HALL_SENSOR_2);
  sensor2 = map(sensor2, HALL_SENSOR_2_MIN, HALL_SENSOR_2_MAX, -255, 255);
  sensor2 = constrain(sensor2, -255, 255);
  
  return { sensor1, sensor2 };
}

void writeToCoils(HallSensorsValues sensors, int level) {
  float mul = 1.0 / 255 * level;

  int sensor1Pos = constrain(max(0, sensors.sensor1), 0, 255);
  int sensor1Neg = constrain(-min(0, sensors.sensor1), 0, 255);
  int sensor2Pos = constrain(max(0, sensors.sensor2), 0, 255);
  int sensor2Neg = constrain(-min(0, sensors.sensor2), 0, 255);

  analogWrite(COIL_2_NEGATIVE, sensor1Neg * mul);
  analogWrite(COIL_1_NEGATIVE, sensor2Neg * mul);
  analogWrite(COIL_1_POSITIVE, sensor1Pos * mul);
  analogWrite(COIL_2_POSITIVE, sensor2Pos * mul);
}


float measureRpm(HallSensorsValues sensors) {
  static unsigned long lastResetMillis = 0;
  static float rpm = 0.0;
  static byte currentCheckpoint = 0;
  static unsigned long previousLaps[sensorCheckpointsCount] = {0};
  
  SensorCheckpoint sc = checkpoints[currentCheckpoint];

  int level = sc.triggerLevel;
  int sign = sc.triggerDirection;
  int sensorValue = sc.sensorId == 0 ? sensors.sensor1 : sensors.sensor2;
  unsigned long now = millis();

  if (sign * sensorValue >= sign * level) {
    unsigned long previousRound = previousLaps[currentCheckpoint];

    if (previousRound > 0) {
      rpm = 1000.0 / (now - previousRound) * 60.0;
    }
    else if (currentCheckpoint >= 20) {
      unsigned long previousQuad = previousLaps[currentCheckpoint - 20];
      rpm = 1000.0 / ((now - previousQuad) * 4) * 60.0;
    }

    previousLaps[currentCheckpoint] = now;
    currentCheckpoint = (currentCheckpoint + 1) % sensorCheckpointsCount;
    lastResetMillis = now;
  } else if (now - lastResetMillis > 1000) {
    rpm = 0.0;
    lastResetMillis = now;
  }

  return rpm;
}

int computeSig(float rpm, float expected) {
  static unsigned long prevMillis = 0;
  static int sig = 0;
  unsigned long now = millis();
  if (now - prevMillis >= 100) {
    prevMillis = now;
    sig = pid(rpm, expected, 8.0, 1.2, 0.1, 0.1);
    Serial.print(expected);
    Serial.print(' ');
    Serial.println(rpm);
  }
  return sig;
}

int pid(float actual, float expected, float kp, float ki, float kd, float dt) {
  float diff = expected - actual;
  static float I = 0, prevDiff = 0;
  I += diff * dt;
  float D = (diff - prevDiff) / dt;
  prevDiff = diff;
  return constrain(diff * kp + I * ki + D * kd, 0, 255);
}
