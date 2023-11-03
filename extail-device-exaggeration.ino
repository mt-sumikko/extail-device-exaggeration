// Extail source ver2023.10

#include "M5Atom.h"

// *--- 9-axis sensor BNO055 ---
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // sample frequency

// I2C device address
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

// *--- Stepper ---
// Driver PIN
#define MODE 32 // MODE HIGH:PHASE／ENABLE Mode
#define VCC 26  // VCC 0V:Sleep Mode

// Motor PIN
int APHASE = 22;
int AENBL = 19;
int BPHASE = 23;
int BENBL = 33;

int dir = 0; // Direction of rotation. 1:counterclockwise, 0:none, -1:clockwise

// delayMicroseconds between Steps
int stepDelay = 1200;
int stepDelay_max = 800;  // fastest
int stepDelay_min = 3000; // slowest

// Motor Specifications
int stepAngle_pure = 18;                    // Step angle of the motor (not taking into account the gear ratio)
int gearRatio = 20;                         // Gear ratio 20:1
float stepPer1Rotation = 102.5f;            // Number of steps per revolution (visually confirmed)
float stepAngle = 360.0 / stepPer1Rotation; // Angle per step (step angle) 3.51219... 約3.5度

// For calculating the number of steps
int initialPosition = 0;  // Initial position (axis angle)
int targetSteps = 0;      // Number of steps to move according to sensor value
int shaftSteps_total = 0; // Total number of steps (current location)
int shaftSteps_max = 25;  // Maximum amplitude (for one side)

// *--- Sensor value ---
// Acceleration
double accX = 0.0;
double accX_th_min = 1.5; // Minimum threshold subject to exaggeration
double accX_th_max = 8;   // Maximum threshold for applying proportionality between input values and motor rotation angle

// Attitude angle
double roll;     // Angle of head tilt（-90~90, but the human neck usually tilts only -45~45 at most.）
int roll_roundf; // Integer value rounded to the nearest roll
int roll_old;    // roll value before 1loop
int roll_diff;   // Difference between the current roll value and the roll value before 1 loop  (absolute value, 0~180)

// diffの閾値 絶対値がこれ以上の場合回転させる roll
int roll_diff_th_min = 3;  // Minimum threshold subject to exaggeration
int roll_diff_th_max = 20; // Maximum threshold for applying proportionality between input values and motor rotation angle

// --------------------------
// *--- Multithread tasks ---

// task1：Turn the stepper according to the sensor value
void task0(void *arg)
{
  while (1)
  {
    // stepRoll();
    stepAccX();

    vTaskDelay(5);
  }
}

// --------------------------
void setup()
{

  M5.begin(true, false, false); // Init Atom-Matrix(Initialize serial port, LED).  初始化 ATOM-Matrix(初始化串口、LED点阵)

  pinMode(APHASE, OUTPUT);
  pinMode(AENBL, OUTPUT);
  pinMode(BPHASE, OUTPUT);
  pinMode(BENBL, OUTPUT);
  digitalWrite(AENBL, HIGH);
  digitalWrite(BENBL, HIGH);

  pinMode(MODE, OUTPUT);
  digitalWrite(MODE, HIGH);
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH); // Current Start

  // xTaskCreatePinnedToCore(func,"name",Stuck size,NULL,priority,Task pointer,Core ID)
  // Core ID: 0 or 1 or tskNO_AFFINITY
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, tskNO_AFFINITY);

  Serial.begin(115200);

  Wire.begin(25, 21);
  while (!Serial)
    delay(50); // wait for serial port to open!

  // Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  printEvent(&orientationData);
  Serial.print("roll: ");
  Serial.println(roll);
  roll_old = roundf(roll);
  delay(1000);
  Serial.println("Ready.");
}

// Retrieve sensor values
void loop(void)
{

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  printEvent(&orientationData);
  printEvent(&linearAccelData);

  // String str = "x:" + String(accX) + " target:" + String(targetSteps) + " shaftSteps_total:" + String(shaftSteps_total);
  // Serial.println(str);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// --------------------------
// *--- Functions ---

// *--- Stepper Basics ---
void turn_forward(int stepNum, int stepDelay) // 反時計回り
{
  for (int i = 0; i < stepNum; i++)
  {
    digitalWrite(APHASE, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(BPHASE, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(APHASE, LOW);
    delayMicroseconds(stepDelay);
    digitalWrite(BPHASE, LOW);
    delayMicroseconds(stepDelay);
    shaftSteps_total++;
  }
}

void turn_reverse(int stepNum, int stepDelay) // 時計回り
{

  for (int i = 0; i < stepNum; i++)
  {
    digitalWrite(BPHASE, LOW);
    delayMicroseconds(stepDelay);
    digitalWrite(APHASE, LOW);
    delayMicroseconds(stepDelay);
    digitalWrite(BPHASE, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(APHASE, HIGH);
    delayMicroseconds(stepDelay);
    shaftSteps_total--;
  }
}

// *--- tools ---
// Outputs RPM to serial monitor
void printSteps_roll(int dir, int roundf_, int old, int diff)
{
  if (targetSteps != 0)
  {
    Serial.print(", roundf: ");
    Serial.print(roundf_);
    Serial.print(", old: ");
    Serial.print(old);
    Serial.print(", diff: ");
    if (dir == -1)
    {
      Serial.print("-");
    }
    Serial.print(diff);

    Serial.print(", targetSteps: ");
    if (dir == -1)
    {
      Serial.print("-");
    }
    Serial.print(targetSteps);

    Serial.print(", total: ");
    Serial.print(shaftSteps_total);
    Serial.print("steps (");
    Serial.print(shaftSteps_total * stepAngle);
    Serial.print("°)");

    Serial.println();
  }
}

void printSteps_accX(int dir, double accX_absolute)
{
  if (targetSteps != 0)
  {
    String str;
    if (dir == 1)
    {
      str = "accX: " + String(accX_absolute) + " target: " + String(targetSteps) + " shaftSteps_total:" + String(shaftSteps_total);
    }
    else
    {
      str = "accX:-" + String(accX_absolute) + " target:- " + String(targetSteps) + " shaftSteps_total:" + String(shaftSteps_total);
    }
    Serial.println(str);
  }
}

// *--- About Sernsor Value ---

// Turn the stepper motor according to the sensor value and current status
void rotateWithSensorValue(int dir, int &targetSteps, int stepDelay)
{

  if (dir == 1) // forward
  {
    // 振幅が既に最大値に至っている場合は回転しない
    if (shaftSteps_total < shaftSteps_max)
    {
      // ex 37<40
      // 回転量をそのまま足すと最大値を突破する場合は、最大値に収まる値を足す
      if (shaftSteps_total + targetSteps > shaftSteps_max)
      {
        // shaftSteps_total + targetSteps <= shaftSteps_max にしておきたいから
        // ex）37 + 5 > 42 これはオーバーなので補正する
        targetSteps = shaftSteps_max - shaftSteps_total;
        // 3 = 40 - 37
        // Serial.print("Corrected ");
      }
      turn_forward(targetSteps, stepDelay);
    }
    else
    {
      targetSteps = 0;
    }
  }
  else if (dir == -1) // reverse
  {
    // 振幅が既に最小値に至っている場合は回転しない
    if (shaftSteps_total > 0 - shaftSteps_max)
    {
      // ex -37 > -40
      // 回転量をそのまま足すと最小値を突破する場合は、最小値に収まる値を足す
      if (shaftSteps_total - targetSteps < 0 - shaftSteps_max)
      {
        // shaftSteps_total - targetSteps > 0 - shaftSteps_max にしておきたいから
        // ex -37 -5 < -40
        targetSteps = shaftSteps_max + shaftSteps_total;
        // 3= 40+(-37)
        // Serial.print("Corrected ");
      }
      turn_reverse(targetSteps, stepDelay);
    }
    else
    {
      targetSteps = 0;
    }
  }
  else // dir==0
  {
    targetSteps = 0;
  }
}

//  Process of roll value
void stepRoll()
{
  // 最新のroll値を確認
  roll_roundf = roundf(roll); // rollの四捨五入値
  // どのくらい動かせば良いかを知るために、1つ前の角度との差を計算
  calculateDirAndAbsDiff(roll_roundf, roll_old, dir, roll_diff); // ここでroll_oldからrollに動く方向がが正負のいずれかを見る

  // roll_diffが一定以上の場合は回転する
  if (roll_diff > roll_diff_th_min)
  {
    // 必要な回転量（ステップ数）を計算
    // 差を絶対値になるようにしているため、roll値が-90~90度をとるところを0~180度で考えている
    // 例）1ステップ3.44の場合90度動くには 90/3.44 = 26.16ステップ
    targetSteps = roundf(map(roll_diff, roll_diff_th_min, 20, 0, shaftSteps_max * 2)); // 目標角度をステップ数に変換
    rotateWithSensorValue(dir, targetSteps, stepDelay);                                // memo: 多分この3つはローカル変数にしておかないとごちゃごちゃになる
    // if (targetSteps != 0) {
    printSteps_roll(dir, roll_roundf, roll_old, roll_diff);
    //}
    roll_old = roll_roundf; // 次のループに向けて古い値として保存 回転した時のみ更新
  }
  else
  {
    targetSteps = 0;
  }
}

// Process of accX value
void stepAccX()
{
  // 回転方向決める
  if (accX < 0)
  {
    dir = -1; // reverse
  }
  else if (accX > 0)
  {
    dir = 1; // forward
  }
  else
  {
    dir = 0;
    return; // 方向づけをしないため、ここで終了
  }

  // accX_absolute の計算
  float accX_absolute = abs(accX);

  // accX_absolute の値に応じて targetSteps, stepDelay を割り当てるコードを書く
  if (accX_absolute > accX_th_min)
  {

    if (accX_absolute > accX_th_max) // 上限を8m/s^2として、それ以上の扱いは8にまるめる
    {
      accX_absolute = accX_th_max;
    }
    targetSteps = roundf(map(accX_absolute, accX_th_min, accX_th_max, 3, 10));
    stepDelay = 1000;

    rotateWithSensorValue(dir, targetSteps, stepDelay);

    if (targetSteps != 0)
    {
      printSteps_accX(dir, accX_absolute);
    }
  }
  else
  {
    targetSteps = 0;
  }
}

// Calculate differences, determine direction of rotation
void calculateDirAndAbsDiff(int roll_roundf, int roll_old, int &dir, int &roll_diff)
{
  // 値の範囲を -90 から 90 に制限
  roll_roundf = std::max(-90, std::min(90, roll_roundf));
  roll_old = std::max(-90, std::min(90, roll_old));

  if (roll_roundf > roll_old)
  {
    dir = 1;
    roll_diff = roll_roundf - roll_old;
  }
  else if (roll_roundf < roll_old)
  {
    dir = -1;
    roll_diff = roll_old - roll_roundf;
  }
  else
  {
    dir = 0;
    roll_diff = 0;
  }

  if (roll_diff > roll_diff_th_max)
  {
    roll_diff = roll_diff_th_max; //
  }
}

// Acquisition of sensor values
void printEvent(sensors_event_t *event)
{
  double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    // Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    accX = x;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    // Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    roll = y;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    // Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE)
  {
    // Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    // Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    // Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    accX = x;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY)
  {
    // Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    // accX = x;
  }
  else
  {
    Serial.print("Unk:");
  }

  String str = "X:" + String(x) + "," + "Y:" + String(y) + "," + "Z:" + String(z);
  // Serial.println(str);
}
