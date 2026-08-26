/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    /* 速度PID用：モーター0、1 */
    volatile float gosagoukei;
    volatile float maenogosa;
    float Kp, Ki, Kd;
    volatile int16_t v;
    uint16_t can_id;
    volatile int16_t mokuhyou;

    /* 位置制御用：主にモーター2 */
    volatile uint16_t encoder_raw;
    volatile uint16_t previous_encoder_raw;
    volatile int32_t position_count;
    volatile int32_t target_position_count;

    volatile float position_integral;
    volatile float previous_position_error;

    float position_Kp;
    float position_Ki;
    float position_Kd;

    volatile uint8_t encoder_initialized;
    volatile uint8_t homed;
} Motor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M2006_ENCODER_CPR          8192L

/* 72 × 36 = 2592度 = 7.2回転 */
#define MOTOR2_MOVE_DEGREE         (72.0f * 36.0f)

#define MOTOR2_MOVE_COUNT \
    ((int32_t)(M2006_ENCODER_CPR * MOTOR2_MOVE_DEGREE / 360.0f))

/* 8192 × 7.2 = 58982.4カウント */
#define MOTOR2_MAX_CURRENT         1000

/*
 * CubeMXで設定したフォトインタラプタの名前に合わせて変更する。
 *
 * 例：
 * #define MOTOR2_PHOTO_GPIO_Port GPIOC
 * #define MOTOR2_PHOTO_Pin       GPIO_PIN_2
 */
#define motor2_photointerrupter_GPIO_Port     GPIOC
#define motor2_photointerrupter_Pin           GPIO_PIN_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;
#include <math.h> // 数学
#include <stdlib.h> // 絶対値
#define PI 3.14159265359

/* USER CODE BEGIN PV */

Motor motors[4];
int16_t g_shusoku[4] = {0,0,0,0};
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

FDCAN_TxHeaderTypeDef TxHeader_com;
FDCAN_RxHeaderTypeDef RxHeader_com;

/*
 * 操作側マイコンから受信したボタン状態
 *
 * RxData[2]：モーター0 上昇
 * RxData[3]：モーター0 下降
 * RxData[4]：モーター1 上昇
 * RxData[5]：モーター1 下降
 */
volatile uint8_t motor0_up = 0;
volatile uint8_t motor0_down = 0;
volatile uint8_t motor1_up = 0;
volatile uint8_t motor1_down = 0;

/*
 * モーター2, RxData[1]から受信する
 */
volatile uint8_t motor2_command = 0;
volatile uint8_t motor2_command_previous = 0;

/* フォトインタラプタの前回状態 */
volatile GPIO_PinState motor2_photo_previous = GPIO_PIN_SET;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader)
{
  htxheader->Identifier = CANID;
  if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata))
  {
    printf("addmessage error\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}


void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  Hfdcan_Filter_Settings->FilterID1 = 0x200;
  Hfdcan_Filter_Settings->FilterID2 = 0x410;
}

void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x200;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_CLASSIC_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_OFF;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  motor_CAN_filter_init(&FDCAN_Filter_settings);
  motor_CAN_txheader_init(Htxheader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
    printf("fdcan_configglobalfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan3))
  {
    printf("fdcan_start is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
  {
    printf("fdcan_activatenotification is error\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef communication_CAN_init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /*
     * 標準ID 0x205だけを受信し、
     * Rx FIFO1に格納する
     */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = 0x205;
    filter.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * 0x205以外の標準IDおよび拡張IDを拒否する
     */
    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * FIFO1にメッセージが入ったとき
     * 受信割り込みを発生させる
     */
    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
            0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * FDCAN1の通信を開始する
     */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
// 	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

//     /* Retrieve Rx messages from RX FIFO0 */
// 		uint8_t RxData_motor[8] = {};
//     FDCAN_RxHeaderTypeDef RxHeader_motor;
// 		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader_motor, RxData_motor)) {
// 			printf("fdcan_getrxmessage_motor is error\r\n");
// 			Error_Handler();
// 		}
// 		/*receive robomas's status*/
// 		for (int i=0; i < 8; i++){
// 			if (RxHeader_motor.Identifier == (robomas[i].CANID)) {
// 				robomas[i].actangle = (int16_t)((RxData_motor[0] << 8) | RxData_motor[1]);
// 				robomas[i].actVel = (int16_t)((RxData_motor[2] << 8) | RxData_motor[3]);
// 				robomas[i].actCurrent = (int16_t)((RxData_motor[4] << 8) | RxData_motor[5]);
// 			}
// 		}
// 	}
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim == &htim6)
{
/*
 * モーター2のフォトインタラプタ処理(合ってるかわからん)
 * GPIO_PULLUPを使用しているため、検出時GPIO_PIN_RESETを想定
  */
GPIO_PinState motor2_photo_now =
    HAL_GPIO_ReadPin(
        motor2_photointerrupter_GPIO_Port,
        motor2_photointerrupter_Pin);

/*
 * SETからRESETへ変わった瞬間だけ処理する
 */
if ((motor2_photo_previous == GPIO_PIN_SET) &&
    (motor2_photo_now == GPIO_PIN_RESET))
{
    
  // 座標系をずらして現在位置を0にする。
  // 目標位置も同じ量だけずらすことで、フォトインタラプタ検出前後で
  // 残りの移動量が変わらないようにする。homedってなんだ
    
    motors[2].target_position_count -=
        motors[2].position_count;

    motors[2].position_count = 0;

    motors[2].position_integral = 0.0f;
    motors[2].previous_position_error = 0.0f;

    motors[2].homed = 1U;
}

// 次回の判定用に現在状態を保存する
motor2_photo_previous = motor2_photo_now;

/* =========================================================
 * モーター2の72度回転指令
 * ========================================================= */

//ボタンが0から1へ変わった瞬間を検出する
if ((motor2_command != 0U) &&
    (motor2_command_previous == 0U) &&
    (motors[2].encoder_initialized != 0U))
  {
  //現在位置から72度先を目標位置にする
  
    motors[2].target_position_count =
        motors[2].position_count +
        MOTOR2_MOVE_COUNT;

    /*
     * 新しい動作を開始するのでPIDをリセット
     */
    motors[2].position_integral = 0.0f;
    motors[2].previous_position_error = 0.0f;
}
// 次回の判定用に現在のボタン状態を保存
motor2_command_previous = motor2_command;

// モーター0、1の目標速度決定
// GPIO_PULLUPを使用しているため、
// リミットスイッチ押下時はGPIO_PIN_RESETとして判定する
 
/* ---------- モーター0のリミット状態 ---------- */

uint8_t motor0_upper_limit =
    (HAL_GPIO_ReadPin(
        GPIOC,
        motor0_upper_Pin) == GPIO_PIN_RESET);

uint8_t motor0_lower_limit =
    (HAL_GPIO_ReadPin(
        GPIOC,
        motor0_lower_Pin) == GPIO_PIN_RESET);


/* ---------- モーター1のリミット状態 ---------- */

uint8_t motor1_upper_limit =
    (HAL_GPIO_ReadPin(
        GPIOC,
        motor1_upper_Pin) == GPIO_PIN_RESET);

uint8_t motor1_lower_limit =
    (HAL_GPIO_ReadPin(
        GPIOC,
        motor1_lower_Pin) == GPIO_PIN_RESET);


/* =========================================================
 * モーター0の目標速度決定
 * ========================================================= */

/*
 * 上昇と下降が同時に押されたときは停止する
 */
if (motor0_up && motor0_down)
{
    motors[0].mokuhyou = 0;
    motors[0].gosagoukei = 0.0f;
}
/*
 * 下ボタンが押され、下限に達していなければ下降
 */
else if (motor0_down && !motor0_lower_limit)
{
    motors[0].mokuhyou = 4000;
}
/*
 * 上ボタンが押され、上限に達していなければ上昇
 */
else if (motor0_up && !motor0_upper_limit)
{
    motors[0].mokuhyou = -4000;
}
/*
 * ボタンを押していない場合、
 * またはリミットに達している場合は停止
 */
else
{
    motors[0].mokuhyou = 0;
    motors[0].gosagoukei = 0.0f;
}


/* ===================================================
 * モーター1の目標速度決定
 * =================================================== */

/*
 * 上昇と下降が同時に押されたときは停止する
 */
if (motor1_up && motor1_down)
{
    motors[1].mokuhyou = 0;
    motors[1].gosagoukei = 0.0f;
}
/*
 * 下ボタンが押され、下限に達していなければ下降
 */
else if (motor1_down && !motor1_lower_limit)
{
    motors[1].mokuhyou = 4000;
}
/*
 * 上ボタンが押され、上限に達していなければ上昇
 */
else if (motor1_up && !motor1_upper_limit)
{
    motors[1].mokuhyou = -4000;
}
/*
 * ボタンを押していない場合、
 * またはリミットに達している場合は停止
 */
else
{
    motors[1].mokuhyou = 0;
    motors[1].gosagoukei = 0.0f;
}

/* ------------ PID計算 ------------ */

TxHeader.Identifier = 0x200;
uint8_t TxData[8] = {0};

for(int h = 0; h < 2; h++)
{
float current = (float)motors[h].v;

float error =
(float)motors[h].mokuhyou - current;

motors[h].gosagoukei += error;

if(motors[h].gosagoukei > 100)
motors[h].gosagoukei = 100;

if(motors[h].gosagoukei < -100)
motors[h].gosagoukei = -100;

float derivative =
(error - motors[h].maenogosa) / 0.001f;

float output =
motors[h].Kp * error
+ motors[h].Ki * motors[h].gosagoukei
+ motors[h].Kd * derivative;

motors[h].maenogosa = error;

if(output > 32767)
output = 32767;

if(output < -32767)
output = -32767;

int16_t current_cmd = (int16_t)output;

TxData[2*h] = (uint8_t)(current_cmd >> 8);
TxData[2*h + 1] = (uint8_t)(current_cmd & 0xFF);
}

//モーター2の位置PID
int16_t motor2_current_cmd = 0;

//エンコーダを1回以上受信している場合だけ制御する(?)
if (motors[2].encoder_initialized != 0U)
{
  //位置誤差
    float motor2_position_error =
        (float)(
            motors[2].target_position_count -
            motors[2].position_count);

  //積分
    motors[2].position_integral +=
        motor2_position_error * 0.001f;

  //積分値の上限下限

    if (motors[2].position_integral > 10000.0f)
    {
        motors[2].position_integral = 10000.0f;
    }

    if (motors[2].position_integral < -10000.0f)
    {
        motors[2].position_integral = -10000.0f;
    }

  //微分値
    float motor2_position_derivative =
        (motor2_position_error -
         motors[2].previous_position_error) /
        0.001f;

  //位置PID出力
    float motor2_output =
        motors[2].position_Kp *
            motor2_position_error +
        motors[2].position_Ki *
            motors[2].position_integral +
        motors[2].position_Kd *
            motor2_position_derivative;

    motors[2].previous_position_error =
        motor2_position_error;

  //電流を±MOTOR2_MAX_CURRENTに制限
    if (motor2_output >
        (float)MOTOR2_MAX_CURRENT)
    {
        motor2_output =
            (float)MOTOR2_MAX_CURRENT;
    }

    if (motor2_output <
        -(float)MOTOR2_MAX_CURRENT)
    {
        motor2_output =
            -(float)MOTOR2_MAX_CURRENT;
    }

  //目標位置から±50以内なら停止、積分値もリセット

    if ((motor2_position_error < 50.0f) &&
        (motor2_position_error > -50.0f))
    {
      motor2_output = 0.0f;
      motors[2].position_integral = 0.0f;
    }

    motor2_current_cmd =
        (int16_t)motor2_output;
}

/*
 * モーター2、CAN ID 0x203の電流指令
 */
uint16_t motor2_current_data =
    (uint16_t)motor2_current_cmd;

TxData[4] =
    (uint8_t)(motor2_current_data >> 8);

TxData[5] =
    (uint8_t)(motor2_current_data & 0xFF);

/*
 * モーター3はまだ未実装なので0
 */
TxData[6] = 0;
TxData[7] = 0;

HAL_FDCAN_AddMessageToTxFifoQ(
&hfdcan3,
&TxHeader,
TxData
);
}
}
    // リミットスイッチ処理↑
    
//FDCAN3：ロボマスモーターのフィードバック受信
void HAL_FDCAN_RxFifo0Callback(
  FDCAN_HandleTypeDef *hfdcan,
  uint32_t RxFifo0ITs)
{
  //FDCAN3以外なら処理しない
    if (hfdcan != &hfdcan3)
    {
        return;
    }

  //FIFO0に新しいメッセージがない場合は処理しない
    if ((RxFifo0ITs &
         FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    uint8_t RxData[8] = {0};

    /*
     * FIFO0からデータを取得する
     */
    if (HAL_FDCAN_GetRxMessage(
        &hfdcan3,
        FDCAN_RX_FIFO0,
        &RxHeader,
        RxData) != HAL_OK)
    {
        printf("fdcan_getrxmessage is error\r\n");
        Error_Handler();
    }

    /*
     * モーター0、1、2を確認する
     */
    for (int i = 0; i < 3; i++)
    {
        if (RxHeader.Identifier == motors[i].can_id)
        {
            /*
             * Byte 0、1：エンコーダ角度
             * 範囲は0～8191
             */
            uint16_t new_encoder =
                (uint16_t)(
                    ((uint16_t)RxData[0] << 8) |
                    RxData[1]);

            /*
             * Byte 2、3：現在速度
             */
            motors[i].v =
                (int16_t)(
                    ((uint16_t)RxData[2] << 8) |
                    RxData[3]);

            /*
             * モーター2だけ累積位置を計算する
             */
            if (i == 2)
            {
                /*
                 * 初めてエンコーダ値を受信した場合
                 */
                if (motors[2].encoder_initialized == 0U)
                {
                    motors[2].encoder_raw =
                        new_encoder;

                    motors[2].previous_encoder_raw =
                        new_encoder;

                    motors[2].position_count = 0;
                    motors[2].target_position_count = 0;

                    motors[2].encoder_initialized = 1U;
                }
                else
                {
                    /*
                     * 前回値から何カウント動いたか
                     */
                    int32_t encoder_difference =
                        (int32_t)new_encoder -
                        (int32_t)motors[2]
                            .previous_encoder_raw;

                    /*
                     * 8191から0をまたいで
                     * 正方向へ回転した場合
                     */
                    if (encoder_difference < -4096)
                    {
                        encoder_difference +=
                            M2006_ENCODER_CPR;
                    }
                    /*
                     * 0から8191をまたいで
                     * 負方向へ回転した場合
                     */
                    else if (encoder_difference > 4096)
                    {
                        encoder_difference -=
                            M2006_ENCODER_CPR;
                    }

                    /*
                     * 累積位置へ加算する
                     */
                    motors[2].position_count +=
                        encoder_difference;

                    motors[2].encoder_raw =
                        new_encoder;

                    motors[2].previous_encoder_raw =
                        new_encoder;
                }
            }
        }
    }
}

// マイコン間通信
// FDCAN1：操作側マイコンからの指令受信
void HAL_FDCAN_RxFifo1Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo1ITs)
{
    /*
     * FDCAN1以外のコールバックだった場合は処理しない
     */
    if (hfdcan != &hfdcan1)
    {
        return;
    }

    /*
     * FIFO1に新しいメッセージが入った場合だけ処理する
     */
    if ((RxFifo1ITs &
         FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)
    {
        return;
    }

    uint8_t RxData[8] = {0};

    /*
     * FIFO1から受信データを取り出す
     */
    if (HAL_FDCAN_GetRxMessage(
            &hfdcan1,
            FDCAN_RX_FIFO1,
            &RxHeader_com,
            RxData) != HAL_OK)
    {
        printf("fdcan_getrxmessage is error\r\n");
        Error_Handler();
    }

    /*
     * 操作側マイコンからのIDが0x205の場合だけ
     * ボタン状態を保存する
     */
    if (RxHeader_com.Identifier == 0x205)
    {
        motor0_up = RxData[2];
        motor0_down = RxData[3];

        motor1_up = RxData[4];
        motor1_down = RxData[5];
        
        motor2_command = RxData[1];
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_FDCAN3_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

 int16_t _v=0;
  //モーター0
  motors[0].Kp = 10;
  motors[0].Ki = 0;
  motors[0].Kd = 0;
  motors[0].gosagoukei = 0;
  motors[0].maenogosa = 0;
  motors[0].v = 0;
  motors[0].can_id = 0x201;
  motors[0].mokuhyou = _v;
  
  //モーター1
  motors[1].Kp = 10;
  motors[1].Ki = 0;
  motors[1].Kd = 0;
  motors[1].gosagoukei = 0;
  motors[1].maenogosa = 0;
  motors[1].v = 0;
  motors[1].can_id = 0x202;
  motors[1].mokuhyou= _v;

  //モーター2
  motors[2].Kp = 0.0f;
  motors[2].Ki = 0.0f;
  motors[2].Kd = 0.0f;
  motors[2].gosagoukei = 0.0f;
  motors[2].maenogosa = 0.0f;
  motors[2].v = 0;
  motors[2].can_id = 0x203;
  motors[2].mokuhyou = 0;

  /* エンコーダ情報 */
  motors[2].encoder_raw = 0;
  motors[2].previous_encoder_raw = 0;
  motors[2].encoder_initialized = 0;

/* 現在位置と目標位置 */
motors[2].position_count = 0;
motors[2].target_position_count = 0;

/* 位置PID */
motors[2].position_integral = 0.0f;
motors[2].previous_position_error = 0.0f;

motors[2].position_Kp = 0.05f;
motors[2].position_Ki = 0.0f;
motors[2].position_Kd = 0.0f;

/* フォトインタラプタ検出済みフラグ */
motors[2].homed = 0;

    FDCAN_FilterTypeDef  FDCAN_Filter_settings;
  FDCAN_Filter_settings.IdType=FDCAN_STANDARD_ID;
  FDCAN_Filter_settings.FilterIndex=0;
  FDCAN_Filter_settings.FilterType=FDCAN_FILTER_RANGE;
  FDCAN_Filter_settings.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
  FDCAN_Filter_settings.FilterID1=0x200;
  FDCAN_Filter_settings.FilterID2=0x410;
  TxHeader.IdType=FDCAN_STANDARD_ID;
  TxHeader.TxFrameType=FDCAN_DATA_FRAME;
  TxHeader.DataLength=FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch=FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker=0;
  
  HAL_TIM_Base_Start_IT(&htim6);
  if (HAL_OK != motor_CAN_RxTxSettings_init(&TxHeader)) Error_Handler();

  if (HAL_OK != communication_CAN_init())
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : motor0_upper_Pin motor0_lower_Pin motor1_upper_Pin motor1_lower_Pin
                           motor3_upper_Pin motor3_lower_Pin motor2_photointerrupter_Pin */
  GPIO_InitStruct.Pin = motor0_upper_Pin|motor0_lower_Pin|motor1_upper_Pin|motor1_lower_Pin
                          |motor3_upper_Pin|motor3_lower_Pin|motor2_photointerrupter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
