// =====================================================================
// AutoPeriodicSwitch.ino
// Arduino UNO R3 + マイクロサーボ SG92R
// 定期的にある装置の押し釦スイッチを自動で押下するツールの制御コード
//
// ■ 動作
// ・約59.3分に1回、サーボが押し釦スイッチを押下する。
//   使用開始直後は、対象装置の電源が切れていることが前提なので、
//   電源を入れるための1回だけ押す。以降の周期では、電源が入って
//   いる装置をいったん切って入れ直すため、1周期で2回押す。
// ・押下は、待機角度（ホーム）から押下角度までサーボをゆっくり
//   回転させ、0.2秒保持してから待機角度に戻すことで行う。
// ・待機中はマイコンをディープスリープ（最も消費電力の小さい
//   パワーダウンモード）に入れる。スリープは1回で最大8秒まで
//   しか眠れないため、445回繰り返して約59.3分を確保している。
//   8秒ごとにLEDを一瞬だけ点滅させ、動作中であることを示す。
//
// ■ 配線
// ・サーボの信号線（橙）: D9
// ・サーボの電源（赤）: 5V、GND（茶）: GND
// ・LED: D13（基板上のLEDでよい）
//
// ■ 必要なライブラリ
// ・Servo（Arduino標準ライブラリ）
// ・LowPower（rocketscream版。ライブラリマネージャから導入）
//
// ■ 角度の調整
// サーボの取り付け状態によって、押し釦をちょうど良く押せる角度は
// 変わる。下記の設定項目 startAngle（待機角度）と targetAngleMove
// （押下角度）を、実機に合わせて微調整すること。
//
// ■ スリープ中の誤回転への対策（設計上の工夫）
// サーボは、信号線に届くパルスの長さで角度を判断する。マイコンが
// ディープスリープに入るとパルス生成用のタイマも止まるため、
// サーボを接続したまま眠ると、信号ピンがHIGHのまま固定されて
// 「異常に長いパルス」と解釈され、サーボが端まで回ってしまう
// ことがある（固まるタイミング次第で、起きたり起きなかったり
// する）。そこで本コードでは、
//   1) 眠る前にサーボ制御を切り離し（detach）、信号ピンを
//      LOWに固定してから眠る
//   2) 起きた後は「先に現在の角度を指示してから」制御を再開し、
//      再開直後にデフォルト角度へ飛ぶのを防ぐ
// という手順にしている。
//
// ■ リセット要因の診断
// 起動時に、直前のリセット要因を基板上のLEDの点滅回数で知らせる。
// サーボの動作電流で電源電圧が落ちてマイコンがリセットすると、
// 起動処理の再実行でサーボが待機角度に戻るため、「勝手に回った」
// ように見える。その切り分けのための機能である。
//   点滅1回=電源投入 / 2回=外部リセット /
//   3回=電圧低下（ブラウンアウト） / 4回=ウォッチドッグ
// 不要なら DIAG_RESET_BLINK を false にすること。
//
// ■ 電源まわりの注意
// サーボの動作電流で電圧が落ちる場合は、サーボの電源をマイコン
// ボードの5Vと共用せず、別の5V電源（1A以上）から供給し、GND
// だけを共通にすると安定する。サーボ電源の両端に470〜1000uFの
// 電解コンデンサを入れることも有効である。
// また、下記 SERVO_POWER_PIN にMOSFETのゲートを割り当てると、
// スリープ中はサーボ自体の電源も切れる。誤動作が原理的に起き
// なくなり、サーボの待機電流も消えるため省電力効果が大きい。
// 使わない場合は -1 のままにすること（通常の配線でそのまま動く）。
// =====================================================================

#include <Servo.h>
#include <stdbool.h>
#include <LowPower.h>

// --- ピン配置 ---
const int SERVO_PIN = 9;
const int LED_PIN = 13;
// サーボ電源を MOSFET で入切する場合のゲートピン。-1 = 使わない
const int SERVO_POWER_PIN = -1;
// 8秒 × 445回 ≒ 3558秒 (約59.3分)
const uint16_t SLEEP_COUNT_TARGET = 445;

// --- 設定項目（ここを調整してください） ---
float startAngle = 56.0;      // 初期角度（ホームポジション）
float targetAngleMove = 30.0; // スイッチ押下方向への目標角度
float moveStep = 0.2;         // 滑らかさの係数（小さいほどゆっくり）
int moveDelay = 40;           // 移動の間の待ち時間(ms)

// リセット要因を LED 点滅で知らせる診断（起動時のみ）
const bool DIAG_RESET_BLINK = true;

// --- 変数 ---
Servo myservo;
float currentAngle = startAngle;
bool is1stPush;
uint8_t resetCause = 0;

// 起動時にリセット要因を点滅で知らせる（切り分け用）
void reportResetCause() {
  int blinks = 0;
  if (resetCause & (1 << PORF))  blinks = 1; // 電源投入
  if (resetCause & (1 << EXTRF)) blinks = 2; // 外部リセット
  if (resetCause & (1 << BORF))  blinks = 3; // ブラウンアウト
  if (resetCause & (1 << WDRF))  blinks = 4; // ウォッチドッグ
  for (int i = 0; i < blinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
}

// 動作開始前の準備：サーボ電源ON（MOSFET使用時）→ 角度を先に書く → attach
void servoAttachForAction() {
  if (SERVO_POWER_PIN >= 0) {
    digitalWrite(SERVO_POWER_PIN, HIGH); // サーボ電源ON
    delay(100);                          // サーボの起動待ち
  }
  // detach 中でも write() はライブラリ内部の目標値を更新するだけ。
  // attach より先に現在角度を書いておくことで、再開直後に
  // デフォルトの 90° へ飛ぶのを防ぐ。
  myservo.write((int)currentAngle);
  myservo.attach(SERVO_PIN);
  delay(300); // 現在角度で安定するまで待つ
}

// スリープ前の後片付け：detach して信号を確実に止め、ピンを LOW に固定
void servoDetachForSleep() {
  myservo.write((int)currentAngle); // 念のため現在角度を保持
  delay(50);
  myservo.detach();
  // スリープ中にピンが HIGH で凍結しないよう、パルスを止めたうえで
  // ピンを LOW に固定する（誤回転対策の核心）
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  if (SERVO_POWER_PIN >= 0) {
    digitalWrite(SERVO_POWER_PIN, LOW); // サーボ電源OFF（省電力＋誤動作防止）
  }
}

void setup() {
  // リセット要因は setup の最初に読んでクリアする
  resetCause = MCUSR;
  MCUSR = 0;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 最初は消灯
  if (SERVO_POWER_PIN >= 0) {
    pinMode(SERVO_POWER_PIN, OUTPUT);
    digitalWrite(SERVO_POWER_PIN, LOW);
  }

  // 未使用ピンのフローティング対策（SERVO_PINとLED_PIN以外を入力プルアップ）
  for (int i = 2; i <= 13; i++) {
    if (i != SERVO_PIN && i != LED_PIN && i != SERVO_POWER_PIN) {
      pinMode(i, INPUT_PULLUP);
    }
  }

  currentAngle = startAngle;
  servoAttachForAction(); // attach + 現在角度で開始

  if (DIAG_RESET_BLINK) {
    reportResetCause();
    digitalWrite(LED_PIN, LOW);
  }

  is1stPush = true;
  // 起動直後に待機
  delay(2000);
}

// 指定した角度まで滑らかに動かす関数
void moveToAngle(float target) {
  digitalWrite(LED_PIN, HIGH); // 一連の動作中のみ点灯

  while (abs(currentAngle - target) > 0.1) {
    if (currentAngle < target) {
      currentAngle += moveStep;
    } else {
      currentAngle -= moveStep;
    }
    myservo.write((int)currentAngle);
    delay(moveDelay); // 滑らかさのコントロール用インターバル
  }

  // 完全に目標に到達したことを保証（端数処理）
  currentAngle = target;
  myservo.write((int)currentAngle);
}

void loop() {
  // --- 起床：サーボを現在角度（ホーム）で安全に再開 ---
  servoAttachForAction();

  // --- シーケンス実行 ---

  if (!is1stPush) { // 使用開始時は１回のみボタンを押す

    // ① スイッチ押下方向に回転して一旦停止
    moveToAngle(targetAngleMove);
    delay(200); // 一時停止のインターバル

    // ② 初期角度にスムーズに回転して戻す
    moveToAngle(startAngle);
    delay(500); // インターバル
  } else {
    is1stPush = false;
  }

  // ③ スイッチ押下方向に回転して一旦停止
  moveToAngle(targetAngleMove);
  delay(200); // 一時停止のインターバル

  // ④ 初期角度にスムーズに回転して戻す
  moveToAngle(startAngle);
  digitalWrite(LED_PIN, LOW); // 動作終了で消灯

  // --- スリープ準備：パルスを止め、ピンを LOW に固定してから眠る ---
  servoDetachForSleep();

  // 指定回数（約59.3分）ディープスリープを繰り返す
  // ※この間サーボは detach 済みで信号は LOW 固定。誤回転は起きない
  for (uint16_t i = 0; i < SLEEP_COUNT_TARGET; i++) {
    // Power-downモード、ADCオフ、BODオフで8秒間スリープ
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    // 8秒ごとに20ミリ秒だけ一瞬点滅させて即消灯（極小の電力消費で動作中確認）
    digitalWrite(LED_PIN, HIGH);
    delay(20);
    digitalWrite(LED_PIN, LOW);
  }
}
