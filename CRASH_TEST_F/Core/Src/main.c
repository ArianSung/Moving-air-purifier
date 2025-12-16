/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//================================================================================
// 1. 시스템 상태 및 제어 플래그 (System State & Control Flags)
// - 로봇의 현재 동작 상태(시작, 정지, 충전 등)를 관리하는 변수들입니다.
//================================================================================
volatile uint8_t  start = 0;         // 자율 주행 시작 플래그 (0: 정지, 1: 시작)
volatile uint8_t  start_charge = 0;  // 자동 충전 시작 플래그 (0: 비활성, 1: 시작)
volatile uint8_t  stop_flag = 0;     // 현재 로봇의 행동을 결정하는 상태 플래그
volatile uint8_t  finsh_crash = 0;   // 장애물 회피 동작이 진행 중인지 나타내는 플래그

//================================================================================
// 2. 모터 및 액추에이터 제어 (Motor & Actuator Control)
// - 바퀴 모터와 서보 모터의 PWM듀티 사이클 값을 저장합니다.
//================================================================================
volatile uint16_t duty_right = 0;   // 오른쪽 바퀴 모터 PWM 듀티
volatile uint16_t duty_left = 0;    // 왼쪽 바퀴 모터 PWM 듀티
volatile uint32_t duty_sensor = 0;  // 초음파 센서용 서보 모터 PWM 듀티

//================================================================================
// 3. 초음파 센서 관련 (Ultrasonic Sensor)
// - HC-SR04 초음파 센서의 트리거, 에코 신호 처리 및 거리 계산에 사용됩니다.
//================================================================================
volatile uint8_t  trig_flag = 0;       // 초음파 발사 요청 플래그
volatile uint8_t  pwm_flag = 0;        // 초음파 측정 및 서보 스캔 제어 플래그
volatile uint8_t  waiting_fall = 0;    // Echo 핀의 Falling Edge를 기다리는 상태 플래그
volatile uint8_t  distance_ready = 0;  // 거리 계산 완료 플래그
volatile float    distance_cm = 0;     // 최종 계산된 거리 값 (cm)

volatile uint8_t  trig_rise = 0;     // Trig 핀의 상태를 관리하는 변수
volatile uint32_t t_rise = 0;        // Echo 핀 Rising Edge 시점의 타이머 값
volatile uint32_t t_fall = 0;        // Echo 핀 Falling Edge 시점의 타이머 값
volatile uint32_t echo_width = 0;    // (t_fall - t_rise) 결과, Echo 펄스 폭
volatile uint32_t trig_start = 0;    // Trig 펄스 시작 시점 타이머 값
volatile uint32_t trig_end = 0;      // Trig 펄스 종료 시점 타이머 값

volatile uint8_t trig_flag_b = 0;
volatile uint8_t waiting_fall_b = 0;
volatile uint8_t distance_ready_b = 0;
volatile float distance_cm_b = 0;

volatile uint8_t trig_rise_b = 0;
volatile uint32_t trig_end_b = 0;
volatile uint32_t trig_start_b = 0;
volatile uint32_t t_rise_b = 0;
volatile uint32_t t_fall_b = 0;
volatile uint32_t echo_width_b = 0;


//================================================================================
// 4. 장애물 회피 및 스캔 (Obstacle Avoidance & Scanning)
// - 장애물 감지 후 주변을 스캔하여 경로를 결정하는 데 사용됩니다.
//================================================================================
volatile uint8_t  scan_step = 0;  // 스캔 동작의 현재 단계를 나타내는 변수
volatile uint16_t scan_L = 0;     // 스캔 시 측정된 왼쪽 거리
volatile uint16_t scan_R = 0;     // 스캔 시 측정된 오른쪽 거리
volatile uint16_t scan_C = 0;     // 스캔 시 측정된 중앙 거리
volatile uint8_t sweep_dir = 0;  // 서보모터 스위핑 방향 (0: 증가, 1: 감소)
volatile uint16_t dist_L = 0;
volatile uint16_t dist_C = 0;
volatile uint16_t dist_R = 0;

//================================================================================
// 5. 먼지 센서 관련 (Dust Sensor)
// - UART 통신으로 먼지 센서 데이터를 수신하고 처리합니다.
//================================================================================
volatile float dust_vol = 0.0f;
volatile float dust_ug = 0.0f;
volatile uint8_t dust_state = 0;
volatile uint32_t dust_start = 0;
volatile uint32_t dust_adc = 0;
volatile uint32_t dust_10msec = 0;
volatile uint8_t dust_flag = 0;


//================================================================================
// 6. 타이머 및 카운터 (Timers & Counters)
// - 시간 기반 동작을 위해 Systick 인터럽트에서 관리됩니다.
//================================================================================
volatile uint16_t sec_count = 0;     // 1초마다 증가하는 카운터
volatile uint16_t pwm_10msec = 0;    // 10ms마다 증가하는 카운터
volatile uint32_t button_block = 0;
volatile uint32_t start_block = 0;
volatile uint32_t charge_block = 0; // 버튼 디바운싱(채터링 방지)을 위한 시간 기록
volatile uint32_t echo_10msec = 0;

//================================================================================
// 7. 자동 충전 관련 변수 (Auto Charging Variables)
//================================================================================
volatile uint32_t ir_10msec = 0;
volatile uint32_t adc_time = 0;
volatile uint32_t best_time = 0;
volatile uint32_t old_adc = 0;
volatile uint32_t new_adc = 0;
volatile uint32_t best_adc = 0;
volatile uint8_t  charge_state = 0;
volatile float vol = 0;
volatile uint8_t docking_count = 0;


//================================================================================
// 8. 디버그 및 기타 변수
//================================================================================
uint32_t count=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#define DUST_ADC_Channel  ADC_CHANNEL_2    // 먼지 센서 (IN2)
#define IR_ADC_Channel    ADC_CHANNEL_0    // IR 센서 (IN0)

void dust_sensor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	ADC_ChannelConfTypeDef sConfig = {0}; // ADC 설정용 변수

	uint32_t block = HAL_GetTick(); // 버튼 디바운싱 시간 체크

	if (GPIO_Pin == start_key_Pin)
	{
		if(block - start_block < 500) return;
		start_block = block;

		if(start_charge == 1)
		{
			start_charge = 0;
			charge_state = 0;
		}

    start = !start;

    if (start == 1)
    {
		sConfig.Channel = DUST_ADC_Channel;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		trig_flag = 1;
		duty_sensor = 750;
		sec_count = 0;
		stop_flag = 0;
		finsh_crash = 0;
		trig_flag_b = 0;

		charge_state = 0;
		adc_time = 0;
		best_adc = 0;
		best_time = 0;
    }

    else
    {
		duty_left = 0;
		duty_right = 0;
		duty_sensor = 750;

		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim3.Instance->CCR2 = duty_sensor;

		stop_flag = 4;
		finsh_crash = 0;
		scan_step = 0;
		trig_flag = 0;
		pwm_flag = 0;
    }
  }

  if (GPIO_Pin == start_charge_Pin)
  {
	  if (block - charge_block < 500) return;
	  charge_block = block;

	  count++;

	  if(start == 1)
	  {
		start = 0;
		stop_flag = 4;
	  }

	  duty_left = 0;
	  duty_right = 0;
	  htim1.Instance->CCR1 = 0;
	  htim1.Instance->CCR2 = 0;

	  start_charge = !start_charge;

	  sConfig.Channel = IR_ADC_Channel;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  if (start_charge == 1)
	  {
		duty_sensor = 750;
		htim3.Instance->CCR2 = duty_sensor;
		sec_count = 0;
		stop_flag = 6;

		trig_flag = 0;
		pwm_flag = 0;
		waiting_fall = 0;
		trig_rise = 0;
		distance_ready = 0;

	    trig_flag_b = 1;
	    waiting_fall_b = 0;
	    trig_rise_b = 0;
	    distance_ready_b = 0;
	  }

    else
    {
       charge_state = 0;
    }
  }

  if (GPIO_Pin == Echo_Pin)
  {
    if (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1)
    {
      t_rise = TIM2->CNT;
      waiting_fall = 1;
    }

    else
    {
      if (waiting_fall == 1)
      {
        t_fall = TIM2->CNT;

        if (t_fall >= t_rise)
        {
          echo_width = t_fall - t_rise;
        }

        else
        {
          echo_width = (0xFFFFFFFF - t_rise) + t_fall + 1;
        }

        distance_ready = 1;
        waiting_fall = 0;
      }
    }
  }

  if (GPIO_Pin == Echo_b_Pin)
  {
	  if(HAL_GPIO_ReadPin(Echo_b_GPIO_Port, Echo_b_Pin) == 1)
	  {
		  t_rise_b = TIM2->CNT;
		  waiting_fall_b = 1;
	  }

	  else
	  {
		  if (waiting_fall_b == 1)
		  {
			  t_fall_b = TIM2->CNT;

			  if(t_fall_b >= t_rise_b)
			  {
				  echo_width_b = t_fall_b - t_rise_b;
			  }

			  else
			  {
				  echo_width_b = (0xFFFFFFFF - t_rise_b) + t_fall_b + 1;
			  }

			  distance_ready_b = 1;
			  waiting_fall_b = 0;
		  }
	  }
  }
}

void HAL_IncTick(void)
{
  uwTick++;

  if ((uwTick % 10) == 0)
  {
    pwm_10msec++;
    dust_10msec++;
    echo_10msec++;

    if (charge_state == 2) ir_10msec++;

    if (charge_state == 1 || charge_state == 4)
    {
    	HAL_ADC_Start_IT(&hadc1);
    	adc_time++;
    }

    if ((uwTick % 1000) == 0)
    {
		sec_count++;
		dust_flag = 1;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

  if (hadc->Instance == ADC1)
  {
	  if (start_charge == 1)
	  {
		  old_adc = new_adc;
		  new_adc = HAL_ADC_GetValue(hadc);
		  vol=(3.295*new_adc)/4096;

		  if (new_adc >= old_adc)
		  {
			  if (new_adc > best_adc)
			  {
				  best_adc = new_adc;
				  best_time = adc_time;
			  }
		  }
	  }

	  else
	  {
		  dust_adc = HAL_ADC_GetValue(hadc);
		  dust_vol = (3.295*dust_adc) / 4096;
		  dust_ug = (dust_vol - 0.1)/ 0.005;

		  if(dust_ug < 0) dust_ug = 0;
	  }
  }
}

void dust_sensor(void)
{
    uint32_t now = TIM2->CNT;                        // 현재 타이머 카운터 값
    uint16_t elapsed = (uint16_t)(now - dust_start); // 오버플로 안전한 시간 차이 계산

    if (dust_state == 0)
    {
        dust_10msec = 0;
        dust_state = 1;
        HAL_GPIO_WritePin(Dust_GPIO_Port, Dust_Pin, 0);
        dust_start = now;
    }

    else if (dust_state == 1)
    {
        if (elapsed > 280)
        {
            HAL_ADC_Start_IT(&hadc1);
            dust_state = 2;
        }
    }

    else if (dust_state == 2)
    {

        if (elapsed > 330)
        {
            HAL_GPIO_WritePin(Dust_GPIO_Port, Dust_Pin, 1);
        }

        if (elapsed >= 10000)
        {
            dust_state = 0;
            dust_flag = 0;
        }
    }
}


void trigger(void)
{
  if (trig_flag == 1 && trig_rise == 0)
  {
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 1);
    trig_start = TIM2->CNT;
    trig_rise = 2;
    trig_flag = 0;
  }

  else if (trig_rise == 2)
  {
    trig_end = TIM2->CNT;

    if ((trig_end - trig_start) >= 10)
    {
      HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
      trig_start = TIM2->CNT;
      trig_rise = 1;
    }
  }

  else if (trig_rise == 1)  //응답 없음
  {

	  //1. 60ms 이상
	  if ((TIM2->CNT - trig_start) > 60000)
	  {
		  trig_rise = 0;
		  waiting_fall = 0;
		  pwm_flag = 1;
		  distance_ready = 2;
	  }
  }
}

void trigger_back (void)
{
	if (trig_flag_b == 1 && trig_rise_b == 0)
	  {
	    HAL_GPIO_WritePin(Trig_b_GPIO_Port, Trig_b_Pin, 1);
	    trig_start_b = TIM2->CNT;
	    trig_rise_b = 2;
	    trig_flag_b = 0;
	  }

	  else if (trig_rise_b == 2)
	  {
	    trig_end_b = TIM2->CNT;

	    if ((trig_end_b - trig_start_b) >= 10)
	    {
	      HAL_GPIO_WritePin(Trig_b_GPIO_Port, Trig_b_Pin, 0);
	      trig_start_b = TIM2->CNT;
	      trig_rise_b = 1;
	    }
	  }

	  else if (trig_rise_b == 1)  //응답 없음
	  {

		  //1. 60ms 이상
		  if ((TIM2->CNT - trig_start_b) > 60000)
		  {
			  trig_rise_b = 0;
			  waiting_fall_b = 0;
			  distance_ready_b = 2;
		  }
	  }
}

void crash_PWM(void)
{
  if (stop_flag != 0)
    return;

  if (pwm_flag == 1)
  {
    if (duty_sensor <= 750 + 50 * 8 && sweep_dir == 0)
      duty_sensor += 25;
    if (duty_sensor > 750 + 50 * 8 && sweep_dir == 0)
      sweep_dir = 1;
    if (duty_sensor >= 750 - 50 * 8 && sweep_dir == 1)
      duty_sensor -= 25;
    if (duty_sensor < 750 - 50 * 8 && sweep_dir == 1)
      sweep_dir = 0;
    pwm_flag = 2;
  }

  if (pwm_10msec >= 10 && pwm_flag == 2)
  {
    pwm_flag = 0;
    trig_flag = 1;
  }
}

void scan(void)
{
  duty_right = 0;
  duty_left = 0;
  finsh_crash = 1;

  if (scan_step == 1)
  {
    duty_sensor = 350;
    pwm_10msec = 0;
    pwm_flag = 0;
    scan_step = 2;
  }

  else if (scan_step == 2)
  {
    if (pwm_10msec > 60)
    {
      trig_flag = 1;
      pwm_flag = 0;
      scan_step = 3;
    }
  }

  else if (scan_step == 3)
  {
    if (pwm_flag == 1)
    {
      scan_L = distance_cm;
      duty_sensor = 750;
      pwm_10msec = 0;
      pwm_flag = 0;
      scan_step = 4;
    }
  }

  else if (scan_step == 4)
  {
    if (pwm_10msec > 60)
    {
      trig_flag = 1;
      pwm_flag = 0;
      scan_step = 5;
    }
  }

  else if (scan_step == 5)
  {
    if (pwm_flag == 1)
    {
      scan_C = distance_cm;
      duty_sensor = 1150;
      pwm_10msec = 0;
      pwm_flag = 0;
      scan_step = 6;
    }
  }

  else if (scan_step == 6)
  {
    if (pwm_10msec > 60)
    {
      trig_flag = 1;
      pwm_flag = 0;
      scan_step = 7;
    }
  }

  else if (scan_step == 7)
  {
    if (pwm_flag == 1)
    {
      scan_R = distance_cm;
      duty_sensor = 750;
      pwm_10msec = 0;
      pwm_flag = 0;
      scan_step = 8;
    }
  }

  else if (scan_step == 8)  //저장 해놓은 값 비교해서 가장 큰 쪽으로 구동하도록 함
  {
    sec_count = 0;

    if (scan_L < 10 && scan_C < 10 && scan_R < 10)  //좌 중 우가 모두 20 이하일 때
    {
      stop_flag = 1;  // turn around
    }

    else if (scan_L > scan_C && scan_L > scan_R)  //왼쪽이 제일 클 때
    {
      stop_flag = 2;  // turn left
    }

    else if (scan_R > scan_C && scan_R >= scan_L)  // 오른쪽이 제일 클 때
    {
      stop_flag = 3;  // turn right
    }

    else
    {
      if (scan_L >= scan_R)
      {
        stop_flag = 7;
      }

      else
      {
        stop_flag = 8;
      }
    }

    finsh_crash = 0;
    scan_step = 0;
    pwm_flag = 1;
  }
}

void move_forward(void)
{
  duty_right = 500;
  duty_left = 550;

  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);  // IN1 , 4가 1이면 후진
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);  // IN2 , 3이 1이면 전진

  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

void move_backward(void)
{
  duty_right = 650;
  duty_left = 600;

  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);  // IN1 , 4가 1이면 후진
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);  // IN2 , 3이 1이면 전진

  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}

void stop(void)
{
  duty_right = 0;
  duty_left = 0;
  finsh_crash = 0;

  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

void turn_around(void)
{
  if (sec_count < 2)
  {
    finsh_crash = 1;
    duty_right = 500;
    duty_left = 500;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  }

  else if (sec_count < 8)
  {
    duty_right = 800;
    duty_left = 800;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  }

  else
  {
    sec_count = 0;
    stop_flag = 0;
    finsh_crash = 0;
    scan_step = 0;
    pwm_flag = 1;
  }
}

void turn_right(void)
{
  if (sec_count < 2)
  {
    finsh_crash = 1;

    duty_right = 500;
    duty_left = 500;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  }

  else if (sec_count < 5)
  {
    duty_right = 650;
    duty_left = 650;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
  }

  else
  {
    sec_count = 0;
    stop_flag = 0;
    finsh_crash = 0;
    scan_step = 0;
    pwm_flag = 1;
  }
}

void turn_left(void)
{
  if (sec_count < 2)
  {
    finsh_crash = 1;

    duty_right = 500;
    duty_left = 500;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  }

  else if (sec_count < 5)
  {
    duty_right = 650;
    duty_left = 650;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  }
  else
  {
    sec_count = 0;
    stop_flag = 0;
    finsh_crash = 0;
    scan_step = 0;
    pwm_flag = 1;
  }
}

void small_turn_left(void)
{
  if (sec_count < 2)
  {
    finsh_crash = 1;

    duty_right = 500;
    duty_left = 500;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
  }

  else
  {
    sec_count = 0;
    stop_flag = 0;
    finsh_crash = 0;
    scan_step = 0;
    pwm_flag = 1;
  }
}

void small_turn_right(void)
{
  if (sec_count < 2)
  {
    duty_right = 500;
    duty_left = 500;

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
  }

  else
  {
    sec_count = 0;
    stop_flag = 0;
    finsh_crash = 0;
    scan_step = 0;
    pwm_flag = 1;
  }
}

void ir_turn_around(void)
{
  duty_right = 800;
  duty_left = 800;

  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

void charge(void)
{
  static uint8_t charge_flag = 0;
  static uint8_t fail_count = 0;

  if (start_charge == 1 && charge_flag == 0)
  {
    charge_state = 0;
    fail_count = 0;
    ir_10msec = 0;
  }

  charge_flag = start_charge;

  if (charge_state == 0)  // 잠시 멈춤
  {
    stop();
    ir_10msec = 0;
   adc_time  = 0;
   best_time = 0;
   old_adc   = 0;
   new_adc   = 0;
   best_adc  = 0;

    if (sec_count >= 2)
    {
      sec_count = 0;
      charge_state = 1;
    }
  }
  else if (charge_state == 1)  //한바퀴 돌면서 ir값 받고, 가장 강했던 초 저장
  {
    ir_turn_around();

    if (sec_count >= 11)  // 한바퀴 돌면서 값이 일정 이상 없었다? 다시
    {
      stop();
      sec_count = 0;

      if (best_adc > 300)
      {
        fail_count = 0;
        ir_10msec = 0;
        charge_state = 2;
      }
      else
      {
        fail_count++;
        sec_count = 0;

        if (fail_count == 1)
        {
           charge_state = 10;
        }

        else if (fail_count == 2)
        {
           charge_state = 11;
        }

        else if (fail_count >= 3)
        {
          start_charge = 0;
          start = 1;
          stop_flag = 0;
          fail_count = 0;
          charge_state = 0;
          trig_flag = 1;
          return;
        }
        else
        {
          charge_state = 0;
          new_adc = 0;
        }
      }
    }
  }

  else if (charge_state == 2)  //가장 강했던 만큼 다시 돌고 방향 잡기
  {
    ir_turn_around();

    if (ir_10msec >= best_time)
    {
      stop();
      sec_count = 0;
      charge_state = 3;
    }
  }

  else if (charge_state == 3)  // n초 후진
  {
    move_backward();

    if (sec_count >= 5)
    {
      stop();
      sec_count = 0;
      charge_state = 4;
    }
  }

  else if (charge_state == 4)  // ir 값이 일정 값 이하라면 다시
  {
    if (best_adc > 3000)
    {
      charge_state = 5;
    }

    else
    {
      charge_state = 0;
    }
    sec_count = 0;
  }

  else if (charge_state == 5)
  {

	  if (distance_cm_b <= 20)
	  {
		  if (docking_count < 255)
		  {
			  docking_count ++;
		  }
	  }

	  else
	  {
		  if(docking_count > 0)
		  {
			  docking_count --;
		  }
	  }

	  if (best_adc > 3000 && docking_count >= 5)
	  {
		  stop();
		  charge_state = 6;
		  docking_count = 0;
		  sec_count = 0;
		  return;
	  }

	  if (sec_count < 5)
	  {
       move_backward();
	  }

	  else
	  {
      stop();
      charge_state = 0;
      sec_count = 0;
	  }
  }

  else if (charge_state == 6)
  {
    stop();
  }

  else if (charge_state == 10)
  {
    move_forward();

    if(sec_count >= 3)
    {
       stop();
       sec_count = 0;
       charge_state = 0;
    }
  }

  else if (charge_state == 11)
  {
     turn_right();

     if(sec_count >= 3)
     {
       sec_count = 0;
       charge_state = 12;
     }
  }

  else if (charge_state == 12)
  {
     move_forward();

     if(sec_count >= 3)
     {
        stop();
        sec_count = 0;
        charge_state = 0;
     }
  }
}

void distance_data(void)
{
	if(start_charge == 0 && finsh_crash == 0)
	{
	    if (dust_ug > 600)
	    {
	    	stop_flag = 4;
	    }

	    else if (dust_ug < 200 && stop_flag == 4)
	    {
	    	stop_flag = 0;
	    }
	}

	if (distance_ready == 1)
	{
		distance_cm = (float) echo_width / 58.0f;

    if (start_charge == 0 && stop_flag != 5 && finsh_crash == 0 && distance_cm < 17 && dust_ug < 200)
    {
      stop_flag = 5;
      scan_step = 1;
      pwm_10msec = 0;
    }

    distance_ready = 0;
  }

  else if (distance_ready == 2)
  {
	  distance_cm = 400;
	  distance_ready = 0;
  }
}

/*void distance_data(void)
{
	if(distance_ready == 1)
	{
		distance_cm = (float) echo_width / 58.0f;

		if(duty_sensor < 600)
		{
			dist_L = distance_cm;

			if (start_charge == 0 && stop_flag != 5 && finsh_crash == 0 && dust_ug < 200)
			{
				if(distance_cm < 20)
				{
					stop_flag = 5;
					scan_step = 1;
					pwm_10msec = 0;
				}
			}
		}

		else if(duty_sensor >= 600 && duty_sensor <= 800)
		{
			dist_C = distance_cm;
			if (start_charge == 0 && stop_flag != 5 && finsh_crash == 0 && dust_ug < 200)
			{
				if(distance_cm < 15)
				{
					stop_flag = 5;
					scan_step = 1;
					pwm_10msec = 0;
				}
			}
		}

		else if(duty_sensor > 800)
		{
			dist_R = distance_cm;

			if (start_charge == 0 && stop_flag != 5 && finsh_crash == 0 && dust_ug < 200)
			{
				if(distance_cm < 20)
				{
					stop_flag = 5;
					scan_step = 1;
					pwm_10msec = 0;
				}
			}
		}
		distance_ready = 0;
	}

	else if (distance_ready == 2)
	{
		if(duty_sensor < 600)
		{
			dist_L = 400;
		}

		else if (duty_sensor > 800)
		{
			dist_R = 400;
		}

		else
		{
			dist_C = 400;
		}

		distance_cm = 400;
		distance_ready = 0;
	}
}*/

void distance_data_back (void)
{
	if (distance_ready_b == 1)
	{
		distance_cm_b = (float) echo_width_b / 58.0f;
	    distance_ready_b = 0;
	    trig_flag_b = 1;
	}
	else if (distance_ready_b == 2)
	{
		distance_cm_b = 400;
		distance_ready_b = 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
  HAL_GPIO_WritePin(Trig_b_GPIO_Port, Trig_b_Pin, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  htim1.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  htim1.Instance->CCR2 = 0;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  htim3.Instance->CCR2 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (waiting_fall == 1)
	  {
	      uint32_t echo_now = TIM2->CNT;
	      if ((echo_now - t_rise) > 30000)
	      {
	          waiting_fall = 0;
	          pwm_flag = 1;
	          distance_ready = 2;
	          trig_rise = 0;
	      }
	  }

	  if (start == 1 && start_charge == 0 && dust_flag == 1)
	  {
		  dust_sensor();
	  }

	  if (start == 1 || start_charge == 1)
	  {
		  htim1.Instance->CCR1 = duty_left;
		  htim1.Instance->CCR2 = duty_right;
		  htim3.Instance->CCR2 = duty_sensor;

		  distance_data();

		  if (start_charge == 0)
		  {
			  crash_PWM();
			  trigger();
		  }

		  if (start_charge == 1)
		  {
			  if(trig_flag_b == 0)
			  {
				  trig_flag_b = 1;
			  }

			  if(echo_10msec % 6 == 0)
			  {
				  trigger_back();
			  }

			  distance_data_back();
		  }

		  if (stop_flag == 0) move_forward();
		  else if (stop_flag == 1) turn_around();
		  else if (stop_flag == 2) turn_left();
		  else if (stop_flag == 3) turn_right();
          else if (stop_flag == 4) stop();
          else if (stop_flag == 5) scan();
          else if (stop_flag == 6) charge();
          else if (stop_flag == 7) small_turn_left();
          else if (stop_flag == 8) small_turn_right();

    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Trig_b_Pin|IN1_Pin|IN2_Pin|IN3_Pin
                          |IN4_Pin|Dust_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Trig_Pin */
  GPIO_InitStruct.Pin = Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : start_charge_Pin start_key_Pin */
  GPIO_InitStruct.Pin = start_charge_Pin|start_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_b_Pin */
  GPIO_InitStruct.Pin = Echo_b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_b_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_b_Pin Dust_Pin */
  GPIO_InitStruct.Pin = Trig_b_Pin|Dust_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
