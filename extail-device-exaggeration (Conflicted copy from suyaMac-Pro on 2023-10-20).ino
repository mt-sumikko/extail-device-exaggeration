// Extail source ver2023.10 これを使ったので本家のgithubおよび論文の説明を書き換える

#include "M5Atom.h"

uint8_t DisBuff[2 + 5 * 5 * 3]; //Used to store RBG color values.  用来存储RBG色值
uint8_t FSM = 0;    //Store the number of key presses.  存储按键按下次数

// *--- 9軸センサ BNO055 ---
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // サンプル取得間のdelay

// I2C device address
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

// *--- Stepper ---
// Driver PIN
#define MODE 32 // MODE HIGHでPHASE／ENABLEモード
#define VCC 26  // VCC 0Vでスリープモード

bool VCC_HIGH = false; // モーターに電源供給しているか

// Motor PIN
int APHASE = 22;
int AENBL = 19;
int BPHASE = 23;
int BENBL = 33;

int dir = 0; // 回転方向

// ステップ間のdelayMicroseconds
int stepDelay = 1200;
int stepDelay_max = 800;  // 最速
int stepDelay_min = 3000; // 最遅

// モーターの仕様
int stepAngle_pure = 18;                    // モーターのステップ角（ギア比を考慮してない）
int gearRatio = 20;                         // ギア比 20:1
float stepPer1Rotation = 102.5f;            // 1回転あたりのステップ数（目視で確認した）
float stepAngle = 360.0 / stepPer1Rotation; // 1ステップの角度（ステップ角）3.51219... 約3.5度

// ステップ数の計算用
int initialPosition = 0;  // 初期位置（軸の角度）
int targetAngle = 0;      // 目標角度（-45〜45度の範囲）
int targetSteps = 0;      // センサ値に応じて動かすステップ数（角度からステップ数に変換）
int shaftSteps_total = 0; // のべステップ数（現在地）
int shaftSteps_max = 25;  // 振幅の最大値（片側分）
int step_param = 10;      // ステップ

// *--- センサ値関係 ---
// 加速度
double accX = 0.0;
double accX_th_min = 1.5; // 閾値
double accX_th_max = 8;

// 姿勢角
double roll;     //-90~90の値を取る ただ普段人間の首はせいぜい-45~45くらいしか傾げないので、設計上は-45~45外は丸める
int roll_roundf; // rollを四捨五入した整数値
int roll_old;    // 1loop前のroll値
int roll_diff;   // 現roll値と1loop前のroll値の差（絶対値）0-180をとる

double yaw;     // 0~360の値を取る
int yaw_roundf; // yawを四捨五入した整数値
int yaw_old;
int yaw_diff;

// diffの閾値 絶対値がこれ以上の場合回転させる roll
int roll_diff_th_min = 3;
int roll_diff_th_max = 20;

// 誇張係数（未使用）
int expand = 1;

bool back = false; // ホームポジションに戻す状態か

// --------------------------
// *--- Multithread tasks ---

// task1：センサ値に応じてステッパーを回す
void task0(void *arg)
{
  while (1)
  {
    // stepRoll();
    stepAccX();
    vTaskDelay(5); // ステッパーがセンサ値に応じて回転する1セット分を待つインターバル
    // 0でもうごきはするが挙動が不安定になりやすいので5か10くらいにしておく。50だと流石にカクつく
  }
}

// task2：回りっぱなしの軸をじわじわ中央へ戻す  / モード切り替え
void task1(void *arg)
{
  while (1)
  {
    // stepBack();

    if (M5.Btn.wasPressed()) {  //Check if the key is pressed.  检测按键是否被按下
      switch (FSM) {
        case 0: //roll
          setBuff(0x40, 0x00, 0x00);
          Serial.println("0")
          break;
        case 1: //ax
          setBuff(0x00, 0x40, 0x00);
           Serial.println("1")
          break;
        case 2: // furifuri
          setBuff(0x00, 0x00, 0x40);
           Serial.println("2")
          break;
        case 3: // stop
          setBuff(0x20, 0x20, 0x20);
           Serial.println("3")
          break;
        default:
          break;
      }
      M5.dis.displaybuff(DisBuff);

      FSM++;
      if (FSM >= 4) {
        FSM = 0;
      }
    }
    delay(50);
    M5.update();


    vTaskDelay(50);
  }
}




void setBuff(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata) { //Set the colors of LED, and save the relevant data to DisBuff[].  设置RGB灯的颜色
  DisBuff[0] = 0x05;
  DisBuff[1] = 0x05;
  for (int i = 0; i < 25; i++) {
    DisBuff[2 + i * 3 + 0] = Rdata;
    DisBuff[2 + i * 3 + 1] = Gdata;
    DisBuff[2 + i * 3 + 2] = Bdata;
  }
}





// --------------------------
void setup()
{

  M5.begin(true, false, true); //Init Atom-Matrix(Initialize serial port, LED).  初始化 ATOM-Matrix(初始化串口、LED点阵)
  delay(10);  //delay10ms.  延迟10ms
  setBuff(0xff, 0x00, 0x00);
  M5.dis.displaybuff(DisBuff);    //Display the DisBuff color on the LED.  同步所设置的颜色

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
  xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 2, NULL, tskNO_AFFINITY); // ホームポジションに戻す方が優先度低い

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

// センサ値を常時取得
void loop(void)
{

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  printEvent(&orientationData);
  printEvent(&linearAccelData);
  // printEvent(&accelerometerData);

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
// 回転数をシリアルモニタに出力
void printSteps(int dir, int roundf_, int old, int diff)
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

// モーターへの電源供給状態切り替え
void switchVCCtoHIGH()
{
  // モーターに電源供給されてなかったとき、電源供給する
  if (VCC_HIGH == false)
  {
    digitalWrite(VCC, HIGH);
    VCC_HIGH = true;
  }
}
void switchVCCtoLOW()
{
  // モーターに電源供給されてたとき、電源供給を断つ
  if (VCC_HIGH == true)
  {
    digitalWrite(VCC, LOW);
    VCC_HIGH = false;
  }
}

// 回りっぱなしの軸をじわじわ中央へ戻す
void stepBack()
{
  // ステッパーを回すセンサ値になってるターンから最低1000msおいてから戻す
  //  じわじわ戻すために、戻す感覚は時間を空けたいが じわじわ戻すほど大変で最悪っぽいのでほどほどにする
  int diff = shaftSteps_total - 0;
  if (shaftSteps_total > 0)
  {
    // Serial.print("back+ IN ");

    if (back == true)
    {
      Serial.print("差+");
      Serial.print(diff);
      turn_reverse(1, 1000);
      Serial.print(" 戻した- total ");
      Serial.println(shaftSteps_total);
    }
    else
    {
      delay(1000); // total==0でもここには入っているっぽいのはなぜだろう??気のせい
      Serial.print(" delayedOnly");
    }
    back = true;
  }
  else if (shaftSteps_total < 0)
  {
    // Serial.print("back- IN ");

    if (back == true)
    {
      Serial.print("差");
      Serial.print(diff);
      turn_forward(1, 1000);
      Serial.print(" 戻した+ total ");
      Serial.println(shaftSteps_total);
    }
    else
    {
      delay(1000);
      Serial.println(" delayedOnly");
    }
    back = true;
  }
}

// *--- センサー値 ---

// センサー値と現状に応じてステッパーを回す
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
      // switchVCCtoHIGH();
      turn_forward(targetSteps, stepDelay);
      back = false;
    }
    else
    {
      targetSteps = 0;
      // switchVCCtoLOW();
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
      // switchVCCtoHIGH();
      turn_reverse(targetSteps, stepDelay);
      back = false;
    }
    else
    {
      targetSteps = 0;
      // switchVCCtoLOW();
    }
  }
  else // dir==0
  {
    targetSteps = 0;
    // switchVCCtoLOW();
  }
}

//  roll値の処理
void stepRoll()
{
  // 最新のroll値を確認
  roll_roundf = roundf(roll); // rollの四捨五入値
  // どのくらい動かせば良いかを知るために、1つ前の角度との差を計算
  // targetAngle = calcDiff(roll_roundf, roll_old); // ここでroll_oldからrollに動く方向がが正負のいずれかを見る
  calculateDirAndAbsDiff(roll_roundf, roll_old, dir, roll_diff);

  // roll_diffが一定以上の場合は回転する
  if (roll_diff > roll_diff_th_min)
  {
    // 必要な回転量（ステップ数）を計算
    // 差を絶対値になるようにしているため、roll値が-90~90度をとるところを0~180度で考えている
    // 例）1ステップ3.44の場合90度動くには 90/3.44 = 26.16ステップ
    targetSteps = roundf(map(roll_diff, roll_diff_th_min, 20, 0, shaftSteps_max * 2)); // 目標角度をステップ数に変換
    rotateWithSensorValue(dir, targetSteps, stepDelay);                                // memo: 多分この3つはローカル変数にしておかないとごちゃごちゃになる
    // if (targetSteps != 0) {
    printSteps(dir, roll_roundf, roll_old, roll_diff);
    //}
    roll_old = roll_roundf; // 次のループに向けて古い値として保存 回転した時のみ更新
  }
  else
  {
    targetSteps = 0;
  }
}

// x軸方向の加速度値の処理
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
      // accXには回ってる間にかなりの確率で次の値が代入される。printをaccXですると回転した時の値とずれるので、accX_absoluteでする。
      printSteps_accX(dir, accX_absolute);
    }
  }
  else
  {
    targetSteps = 0;
  }
}

// 差分計算・回転方向確定 暫定roll用の関数
// memo：値がひっくり返ってしまった時に無視する処理を入れておきたいが、なくても大丈夫そうかも
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

// センサ値の取得
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
