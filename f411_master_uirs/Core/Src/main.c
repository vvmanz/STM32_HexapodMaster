/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "stdlib.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265
#define WHEEL_DIAMETER 11.0
#define EFFECTIVE_ANGLE 30.0
#define AXIS_DISTANCE 22.0
#define MIDDLE_AXIS_LENGTH 37.0
#define OUTER_AXIS_LENGTH 28.0
#define BODY_LENGTH 46.0
#define BODY_WIDTH 20.0
#define ERROR_CHECK_TIMER 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
// SPI
volatile int spi_tick;
int32_t master_transmit_1[9], master_receive_1[9], master_transmit_2[9],
		master_receive_2[9], master_transmit_3[9], master_receive_3[9];

int32_t rc_spi2_angle_1, rc_spi2_speed_1, rc_spi2_current_1, rc_spi2_angle_2,
		rc_spi2_speed_2, rc_spi2_current_2, tr_spi_2_command1,
		tr_spi_2_command2, tr_spi_2_command3, tr_spi_2_command4,
		tr_spi_2_command5, tr_spi_2_command6, tr_spi_2_protocol,
		rc_spi2_protocol, rc_spi2_currangle1, rc_spi2_currangle2,
		tr_spi_2_command7, tr_spi_2_command8;

int32_t rc_spi1_angle_1, rc_spi1_speed_1, rc_spi1_current_1, rc_spi1_angle_2,
		rc_spi1_speed_2, rc_spi1_current_2, tr_spi_1_command1,
		tr_spi_1_command2, tr_spi_1_command3, tr_spi_1_command4,
		tr_spi_1_command5, tr_spi_1_command6, tr_spi_1_protocol,
		rc_spi1_protocol, rc_spi1_currangle1, rc_spi1_currangle2,
		tr_spi_1_command7, tr_spi_1_command8;

int32_t rc_spi3_angle_1, rc_spi3_speed_1, rc_spi3_current_1, rc_spi3_angle_2,
		rc_spi3_speed_2, rc_spi3_current_2, tr_spi_3_command1,
		tr_spi_3_command2, tr_spi_3_command3, tr_spi_3_command4,
		tr_spi_3_command5, tr_spi_3_command6, tr_spi_3_protocol,
		rc_spi3_protocol, rc_spi3_currangle1, rc_spi3_currangle2,
		tr_spi_3_command7, tr_spi_3_command8;
;

float angle_mot1, angle_mot2, angle_mot3, angle_mot4, angle_mot5, angle_mot6,
		speed_mot1, speed_mot2, speed_mot3, speed_mot4, speed_mot5, speed_mot6,
		current_mot1, current_mot2, current_mot3, current_mot4, current_mot5,
		current_mot6, currangle1, currangle2, currangle3, currangle4,
		currangle5, currangle6;
float current_angles[6];
volatile int8_t ifready = 0;
volatile int8_t stop_com;
volatile int32_t protocol;
volatile int spi1_cond, spi2_cond, spi3_cond;

int spi1_error, spi1_success, spi2_error, spi2_success, spi3_error,
		spi3_success;
volatile uint32_t error_flag_counter;
volatile uint32_t last_error_check;
volatile uint32_t last_spi1_error;
volatile uint32_t last_spi2_error;
volatile uint32_t last_spi3_error;
int counter_spi;
int16_t spi_suc_1, spi_suc_2, spi_suc_3;
int16_t spi_f1, spi_f2, spi_f3;

// UART (Bluetooth, Raspberry Pi)
int32_t uart_transmit[18];
int32_t uart_recieve[6];
int32_t tr_uart_1, tr_uart_2, tr_uart_3, tr_uart_4, tr_uart_5, tr_uart_6;
volatile int uart_tick;
uint8_t message_r = 20;
uint32_t error_counter_uart = 0;

// Joystick
volatile uint16_t adcValBuf[2];
volatile int adc_flag;
volatile int vx_joy, vy_joy;
volatile int up_down, left_right;
volatile int control_direction;
volatile int info1, info2, info3;

// Control
bool first_condition_met = false;
bool second_condition_met = true;
int tolerance = 120;
int first_condition_met_1 = 0;
int second_condition_met_1 = 1;
int df_spi1_angle1, df_spi1_angle2, df_spi2_angle1, df_spi2_angle2,
		df_spi3_angle1, df_spi3_angle2;
int com_spi1_angle1, com_spi1_angle2, com_spi2_angle1, com_spi2_angle2,
		com_spi3_angle1, com_spi3_angle2;
bool first_wheel = true;
int spi1_adaptive_1_var1, spi2_adaptive_1_var1, spi3_adaptive_1_var1,
		spi1_adaptive_2_var1, spi2_adaptive_2_var1, spi3_adaptive_2_var1;

int spi1_adaptive_1_var2, spi2_adaptive_1_var2, spi3_adaptive_1_var2,
		spi1_adaptive_2_var2, spi2_adaptive_2_var2, spi3_adaptive_2_var2;
int start = 1;

// Forward kinematics
float linear_velocity_x = 0; // Задайте желаемую линейную скорость по X
float linear_velocity_y = 0; // Задайте желаемую линейную скорость по Y
float angular_velocity = 0;
float delta_angles[6];
float v[6];
float ta1, ta2, ta3, ta4, ta5, ta6;
float average_left;
float average_right;
float average_sum;
int delay = 5;
volatile int odometry;
float radius = 0.11;
float beneficial_percentage = 0.2;
float distance = 0.2;
float distance_robot;
float all_distance;
float time_calcu = 0.1;
float derx_1, derx_2, derx_3, derx_4, derx_5, derx_6;
#define NUM_MOTORS 6
int angle_mot[NUM_MOTORS];
int angle_circ[NUM_MOTORS];
float diff_angle[NUM_MOTORS];
float derx[NUM_MOTORS];
int angle_circ_old[NUM_MOTORS];
float average_speed_left, average_speed_right, average_speed_sum,
		average_speed_diff, main_speed_vector, cos_phi, sin_phi, sum_phi;
float vector_speed_x, vector_speed_y;
float path_x, path_y;
volatile int reset = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		message_r = 0;
		HAL_UART_MspDeInit(&huart1);
		HAL_UART_MspInit(&huart1);
		HAL_UART_Receive_DMA(&huart1, &message_r, 1);
		error_counter_uart++;
	}
}
void current_wheel_distance() {
	average_left = ((speed_mot1 + speed_mot3 + speed_mot5) / 3) * (3.14 / 30)
			/ (27);
	average_right = ((-speed_mot2 + -speed_mot4 + -speed_mot6) / 3)
			* (3.14 / 30) / (27);
	average_sum = ((average_left + average_right) / 2) * radius;

	distance_robot = average_sum * beneficial_percentage * time_calcu;
}
void all_wheel_distance() {
	all_distance += distance_robot;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spi1_send();
void spi2_send();
void spi3_send();
void spi2_clear();
void spi1_clear();
void spi3_clear();
void joystick_control();
void joystick_manual();
void com_forward();
void com_backward();
void com_left();
void com_right();
void com_stop();
void com_a();
void com_b();
void com_x();
void com_y();
void com_pause();
void com_start();
void transmit_info();
void flag_on();
void received_transform();
void spi_to_uart();
void calculate_wheel_angles(float linear_velocity_x, float linear_velocity_y,
		float angular_velocity, float current_angles[6], float delta_angles[6],
		float dt);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

//kinematics
void angle_motor_calc();
void f_kinematics();

//bluetooth connection
void bluetooth_control();
void bluetooth_uart();
void spi_check();
int set_to_zero;
int allow_work = 0;
int set_speed_limit;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_USART6_UART_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	protocol = 666;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcValBuf, 2);
	HAL_TIM_Base_Start_IT(&htim2);
	allow_work = 1;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (allow_work == 1) {
			joystick_manual();
		}
		flag_on();
		transmit_info();
		if (spi_tick) {
			spi_check();
			if (adc_flag) {
				adc_flag = 0;
				vx_joy = adcValBuf[0];
				vy_joy = adcValBuf[1];
				if (message_r == 20) {
					joystick_control();
				}
				if (message_r != 20) {
					bluetooth_control();
				}
			}
			spi_tick = 0;
			spi1_send();
			spi2_send();
			spi3_send();
			received_transform();
		}
		if (odometry) {
			odometry = 0;
//			angle_motor_calc();
//			f_kinematics();
			bluetooth_uart();
			spi_to_uart();
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1000 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1000 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	/* DMA2_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, RED_LED_SPI_Pin | GREEN_LED_SPI_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI_2_Pin | SPI_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : RED_LED_SPI_Pin GREEN_LED_SPI_Pin */
	GPIO_InitStruct.Pin = RED_LED_SPI_Pin | GREEN_LED_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : IFREADY3_Pin */
	GPIO_InitStruct.Pin = IFREADY3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IFREADY3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI_2_Pin SPI_1_Pin */
	GPIO_InitStruct.Pin = SPI_2_Pin | SPI_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void com_forward() {
//	if (start != 0) {
//		ifready = 1;
//		int rc_angle_1 =
//				(first_wheel) ?
//						rc_spi1_angle_1 / 100.0 : rc_spi1_angle_2 / 100.0;
//		int rc_angle_2 =
//				(first_wheel) ?
//						rc_spi2_angle_2 / 100.0 : rc_spi2_angle_1 / 100.0;
//		int rc_angle_3 =
//				(first_wheel) ?
//						rc_spi3_angle_1 / 100.0 : rc_spi3_angle_2 / 100.0;
//
//		if ((abs(
//				rc_angle_1
//						- ((first_wheel) ? com_spi1_angle1 : com_spi1_angle2))
//				< tolerance)
//				&& (abs(
//						rc_angle_2
//								- ((first_wheel) ?
//										com_spi2_angle2 : com_spi2_angle1))
//						< tolerance)
//				&& (abs(
//						rc_angle_3
//								- ((first_wheel) ?
//										com_spi3_angle1 : com_spi3_angle2))
//						< tolerance)) {
//			first_wheel = !first_wheel;
//			if (first_wheel) {
//				com_spi1_angle1 += 360;
//				com_spi2_angle2 += 360;
//				com_spi3_angle1 += 360;
//			} else {
//				com_spi1_angle2 += 360;
//				com_spi2_angle1 += 360;
//				com_spi3_angle2 += 360;
//			}
//		}
//	}
//}
//
//void com_backward() {
//	if (start != 0) {
//		ifready = 1;
//		int rc_angle_1 =
//				(first_wheel) ?
//						rc_spi1_angle_1 / 100.0 : rc_spi1_angle_2 / 100.0;
//		int rc_angle_2 =
//				(first_wheel) ?
//						rc_spi2_angle_2 / 100.0 : rc_spi2_angle_1 / 100.0;
//		int rc_angle_3 =
//				(first_wheel) ?
//						rc_spi3_angle_1 / 100.0 : rc_spi3_angle_2 / 100.0;
//
//		if ((abs(
//				rc_angle_1
//						- ((first_wheel) ? com_spi1_angle1 : com_spi1_angle2))
//				< tolerance)
//				&& (abs(
//						rc_angle_2
//								- ((first_wheel) ?
//										com_spi2_angle2 : com_spi2_angle1))
//						< tolerance)
//				&& (abs(
//						rc_angle_3
//								- ((first_wheel) ?
//										com_spi3_angle1 : com_spi3_angle2))
//						< tolerance)) {
//			first_wheel = !first_wheel;
//			if (first_wheel) {
//				com_spi1_angle1 -= 360;
//				com_spi2_angle2 -= 360;
//				com_spi3_angle1 -= 360;
//			} else {
//				com_spi1_angle2 -= 360;
//				com_spi2_angle1 -= 360;
//				com_spi3_angle2 -= 360;
//			}
//		}
//	}
//}
void com_forward() {
	if (start != 0) {
		ifready = 1;
		int rc_angle_1 =
				(first_wheel) ?
						rc_spi1_angle_1 / 100.0 : rc_spi1_angle_2 / 100.0;
		int rc_angle_2 =
				(first_wheel) ?
						rc_spi2_angle_1 / 100.0 : rc_spi2_angle_2 / 100.0;
		int rc_angle_3 =
				(first_wheel) ?
						rc_spi3_angle_1 / 100.0 : rc_spi3_angle_2 / 100.0;

		if ((abs(
				rc_angle_1
						- ((first_wheel) ? com_spi1_angle2 : com_spi1_angle1))
				< tolerance)
				&& (abs(
						rc_angle_2
								- ((first_wheel) ?
										com_spi2_angle1 : com_spi2_angle2))
						< tolerance)
				&& (abs(
						rc_angle_3
								- ((first_wheel) ?
										com_spi3_angle1 : com_spi3_angle2))
						< tolerance)) {
			first_wheel = !first_wheel;
			if (first_wheel) {
				com_spi1_angle1 += 360;
				com_spi2_angle1 += 360;
				com_spi3_angle1 += 360;

				spi1_adaptive_1_var1 = 1;
				spi2_adaptive_1_var1 = 1;
				spi3_adaptive_1_var1 = 1;

				spi1_adaptive_2_var1 = 0;
				spi2_adaptive_2_var1 = 0;
				spi3_adaptive_2_var1 = 0;

			} else {
				com_spi1_angle2 += 360;
				com_spi2_angle2 += 360;
				com_spi3_angle2 += 360;

				spi1_adaptive_2_var1 = 1;
				spi2_adaptive_2_var1 = 1;
				spi3_adaptive_2_var1 = 1;

				spi1_adaptive_1_var1 = 0;
				spi2_adaptive_1_var1 = 0;
				spi3_adaptive_1_var1 = 0;
			}
		}
	}
}

void com_backward() {
	if (start != 0) {
		ifready = 1;
		int rc_angle_1 =
				(first_wheel) ?
						rc_spi1_angle_1 / 100.0 : rc_spi1_angle_2 / 100.0;
		int rc_angle_2 =
				(first_wheel) ?
						rc_spi2_angle_1 / 100.0 : rc_spi2_angle_2 / 100.0;
		int rc_angle_3 =
				(first_wheel) ?
						rc_spi3_angle_1 / 100.0 : rc_spi3_angle_2 / 100.0;

		if ((abs(
				rc_angle_1
						- ((first_wheel) ? com_spi1_angle2 : com_spi1_angle1))
				< tolerance)
				&& (abs(
						rc_angle_2
								- ((first_wheel) ?
										com_spi2_angle1 : com_spi2_angle2))
						< tolerance)
				&& (abs(
						rc_angle_3
								- ((first_wheel) ?
										com_spi3_angle1 : com_spi3_angle2))
						< tolerance)) {
			first_wheel = !first_wheel;
			if (first_wheel) {
				com_spi1_angle1 -= 360;
				com_spi2_angle1 -= 360;
				com_spi3_angle1 -= 360;

				spi1_adaptive_2_var1 = 0;
				spi2_adaptive_2_var1 = 0;
				spi3_adaptive_2_var1 = 0;

				spi1_adaptive_1_var1 = 2;
				spi2_adaptive_1_var1 = 2;
				spi3_adaptive_1_var1 = 2;
			} else {
				com_spi1_angle2 -= 360;
				com_spi2_angle2 -= 360;
				com_spi3_angle2 -= 360;

				spi1_adaptive_2_var1 = 2;
				spi2_adaptive_2_var1 = 2;
				spi3_adaptive_2_var1 = 2;

				spi1_adaptive_1_var1 = 0;
				spi2_adaptive_1_var1 = 0;
				spi3_adaptive_1_var1 = 0;
			}
		}
	}
}
void com_left() {
	if (start != 0) {
		ifready = 1;
		int rc_angle_1 =
				(first_wheel) ?
						rc_spi1_angle_1 / 100.0 : rc_spi1_angle_2 / 100.0;
		int rc_angle_2 =
				(first_wheel) ?
						rc_spi2_angle_1 / 100.0 : rc_spi2_angle_2 / 100.0;
		int rc_angle_3 =
				(first_wheel) ?
						rc_spi3_angle_1 / 100.0 : rc_spi3_angle_2 / 100.0;

		if ((abs(
				rc_angle_1
						- ((first_wheel) ? com_spi1_angle2 : com_spi1_angle1))
				< tolerance)
				&& (abs(
						rc_angle_2
								- ((first_wheel) ?
										com_spi2_angle1 : com_spi2_angle2))
						< tolerance)
				&& (abs(
						rc_angle_3
								- ((first_wheel) ?
										com_spi3_angle1 : com_spi3_angle2))
						< tolerance)) {
			first_wheel = !first_wheel;
			if (first_wheel) {
				com_spi1_angle1 += 360;
				com_spi2_angle1 -= 360;
				com_spi3_angle1 += 360;

				spi1_adaptive_1_var1 = 1;
				spi2_adaptive_1_var1 = 2;
				spi3_adaptive_1_var1 = 1;

				spi1_adaptive_2_var1 = 0;
				spi2_adaptive_2_var1 = 0;
				spi3_adaptive_2_var1 = 0;
			} else {
				com_spi1_angle2 -= 360;
				com_spi2_angle2 += 360;
				com_spi3_angle2 -= 360;

				spi1_adaptive_2_var1 = 2;
				spi2_adaptive_2_var1 = 1;
				spi3_adaptive_2_var1 = 2;

				spi1_adaptive_1_var1 = 0;
				spi2_adaptive_1_var1 = 0;
				spi3_adaptive_1_var1 = 0;

			}
		}
	}
}
void com_right() {
	if (start != 0) {
		ifready = 1;
		int rc_angle_1 =
				(first_wheel) ?
						rc_spi1_angle_2 / 100.0 : rc_spi1_angle_1 / 100.0;
		int rc_angle_2 =
				(first_wheel) ?
						rc_spi2_angle_1 / 100.0 : rc_spi2_angle_2 / 100.0;
		int rc_angle_3 =
				(first_wheel) ?
						rc_spi3_angle_1 / 100.0 : rc_spi3_angle_2 / 100.0;

		if ((abs(
				rc_angle_1
						- ((first_wheel) ? com_spi1_angle1 : com_spi1_angle2))
				< tolerance)
				&& (abs(
						rc_angle_2
								- ((first_wheel) ?
										com_spi2_angle1 : com_spi2_angle2))
						< tolerance)
				&& (abs(
						rc_angle_3
								- ((first_wheel) ?
										com_spi3_angle1 : com_spi3_angle2))
						< tolerance)) {
			first_wheel = !first_wheel;
			if (first_wheel) {
				com_spi1_angle1 -= 360;
				com_spi2_angle1 += 360;
				com_spi3_angle1 -= 360;

				spi1_adaptive_1_var1 = 2;
				spi2_adaptive_1_var1 = 1;
				spi3_adaptive_1_var1 = 2;

				spi1_adaptive_2_var1 = 0;
				spi2_adaptive_2_var1 = 0;
				spi3_adaptive_2_var1 = 0;
			} else {
				com_spi1_angle2 += 360;
				com_spi2_angle2 -= 360;
				com_spi3_angle2 += 360;

				spi1_adaptive_2_var1 = 1;
				spi2_adaptive_2_var1 = 2;
				spi3_adaptive_2_var1 = 1;

				spi1_adaptive_1_var1 = 0;
				spi2_adaptive_1_var1 = 0;
				spi3_adaptive_1_var1 = 0;
			}
		}
	}
}
void com_stop() {
//	if (!first_condition_met && second_condition_met) {
//		first_condition_met = true;
//		second_condition_met = false;
//	} else if (!second_condition_met && first_condition_met) {
//		first_condition_met = false;
//		second_condition_met = true;
//	}
//
}
int flag_com_a;
int com_a_timer;
void com_a() {
	if (flag_com_a == 1) {
		com_a_timer++;
		com_left();
		if (control_direction != 9) {
			flag_com_a = 0;
		}
	}
	if (com_a_timer >= 15000) {
		flag_com_a = 0;
	}

}
void com_b() {
	set_to_zero = 1;
}
void com_x() {

}
void com_y() {

}
void com_pause() {

}
void com_start() {

}
void joystick_manual() {
	if (master_receive_1[6] == 666 && master_receive_2[6] == 666
			&& master_receive_3[6] == 666) {

		set_to_zero = 0;
		if (flag_com_a == 0) {
			if (control_direction == 1) { // ВПЕРЕД
				com_forward();
			}
			if (control_direction == 3) { // ВПРАВО
				com_right();
			}
			if (control_direction == 5) { // НАЗАД
				com_backward();
			}
			if (control_direction == 7) { // ВЛЕВО
				com_left();
			}
			if (control_direction == 9) { // STOP
				com_stop();
			}
			if (control_direction == 10) { // A
				set_speed_limit = 1;
			}
			if (control_direction == 11) { // B
				set_speed_limit = 2;
			}
			if (control_direction == 12) { // X
				set_speed_limit = 3;
			}
			if (control_direction == 13) { // Y
				tolerance = 200;
			}
			if (control_direction == 14) { // PAUSE
				tolerance = 120;
			}
			if (control_direction == 15) { // START
				tolerance = 100;
			}
		}
	}
}
void joystick_control() {
	if (vx_joy < 600) {
		left_right = 1;
	} else if (vx_joy > 3000) {
		left_right = 0;
	} else {
		left_right = 2;
	}
	if (vy_joy < 600) {
		up_down = 0;
	} else if (vy_joy > 3000) {
		up_down = 1;
	} else {
		up_down = 2;
	}
	if (left_right == 2 && up_down == 0) {
		control_direction = 1;
	} else if (left_right == 0 && up_down == 0) {
		control_direction = 2;
	} else if (left_right == 0 && up_down == 2) {
		control_direction = 3;
	} else if (left_right == 0 && up_down == 1) {
		control_direction = 4;
	} else if (left_right == 2 && up_down == 1) {
		control_direction = 5;
	} else if (left_right == 1 && up_down == 1) {
		control_direction = 6;
	} else if (left_right == 1 && up_down == 2) {
		control_direction = 7;
	} else if (left_right == 1 && up_down == 0) {
		control_direction = 8;
	} else {
		control_direction = 9;
	}
}
void bluetooth_control() {
	if (message_r == 70) { // FORWARD
		control_direction = 1;
	} else if (message_r == 82) { //RIGHT
		control_direction = 3;
	} else if (message_r == 66) { // BACKWARD
		control_direction = 5;
	} else if (message_r == 76) { // LEFT
		control_direction = 7;
	} else if (message_r == 48) { // STOP
		control_direction = 9;
	} else if (message_r == 88) { // A
		control_direction = 10;
	} else if (message_r == 67) { // B
		control_direction = 11;
	} else if (message_r == 83) { // X
		control_direction = 12;
	} else if (message_r == 84) { // Y
		control_direction = 13;
	} else if (message_r == 65) { // START
		control_direction = 14;
	} else if (message_r == 80) { // PAUSE
		control_direction = 15;
	}
}
void calculate_wheel_angles(float linear_velocity_x, float linear_velocity_y,
		float angular_velocity, float current_angles[6], float delta_angles[6],
		float dt) {
	float angle_offset = EFFECTIVE_ANGLE / 360.0 * 2 * PI;
	float wheel_radius = WHEEL_DIAMETER / 2.0;
	float middle_to_center = MIDDLE_AXIS_LENGTH / 2.0;
	float outer_to_center = OUTER_AXIS_LENGTH / 2.0;

	float l1 = sqrt(pow(middle_to_center, 2) + pow(BODY_WIDTH / 2, 2));
	float l2 = sqrt(pow(outer_to_center, 2) + pow(BODY_WIDTH / 2, 2));
	float alpha1 = atan2(middle_to_center, BODY_WIDTH / 2);
	float alpha2 = atan2(outer_to_center, BODY_WIDTH / 2);

	v[0] = linear_velocity_x - l1 * angular_velocity * cos(alpha1);
	v[1] = linear_velocity_x + l1 * angular_velocity * cos(alpha1);
	v[2] = linear_velocity_x - l2 * angular_velocity * cos(alpha2);
	v[3] = linear_velocity_x + l2 * angular_velocity * cos(alpha2);
	v[4] = linear_velocity_x - l2 * angular_velocity * cos(alpha2);
	v[5] = linear_velocity_x + l2 * angular_velocity * cos(alpha2);

	float max_angle_change = 720.0 * dt; // Учесть ограничение времени

	for (int i = 0; i < 6; ++i) {
		float current_angle_rad = current_angles[i] / 360.0 * 2 * PI;
		float v_wheel = v[i]
				/ (wheel_radius * cos(current_angle_rad + angle_offset));
		float delta_angle = v_wheel / wheel_radius;
		delta_angles[i] = delta_angle * 180.0 / PI;

		// Ограничить изменение угла
		if (delta_angles[i] > max_angle_change) {
			delta_angles[i] = max_angle_change;
		} else if (delta_angles[i] < -max_angle_change) {
			delta_angles[i] = -max_angle_change;
		}
	}
}
void spi1_send() {
	if (spi1_cond == 1) {
		rc_spi1_angle_1 = master_receive_1[0];
		rc_spi1_speed_1 = master_receive_1[1];
		rc_spi1_current_1 = master_receive_1[2];
		rc_spi1_angle_2 = master_receive_1[3];
		rc_spi1_speed_2 = master_receive_1[4];
		rc_spi1_current_2 = master_receive_1[5];
		rc_spi1_protocol = master_receive_1[6];
		rc_spi1_currangle1 = master_receive_1[7];
		rc_spi1_currangle2 = master_receive_1[8];
	}
	if (spi1_cond == 2) {
		master_transmit_1[0] = 0;
		master_transmit_1[1] = 0;
		master_transmit_1[2] = 0;
		master_transmit_1[3] = 0;
		master_transmit_1[4] = 0;
		master_transmit_1[5] = 0;
		master_transmit_1[6] = 0;
		master_transmit_1[7] = 0;
		master_transmit_1[8] = 0;

		master_receive_1[0] = 0;
		master_receive_1[1] = 0;
		master_receive_1[2] = 0;
		master_receive_1[3] = 0;
		master_receive_1[4] = 0;
		master_receive_1[5] = 0;
		master_receive_1[6] = 0;
		master_receive_1[7] = 0;
		master_receive_1[8] = 0;
		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*) master_transmit_1,
				(uint8_t*) master_receive_1, 36);
		spi1_cond = 0;
	}
	HAL_StatusTypeDef status1 = HAL_SPI_TransmitReceive_DMA(&hspi1,
			(uint8_t*) master_transmit_1, (uint8_t*) master_receive_1, 36);
	if (status1 == HAL_OK) {
		HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*) master_transmit_2,
				(uint8_t*) master_receive_2, 36);
	}
}

void spi2_send() {
	if (spi2_cond == 1) {
		rc_spi2_angle_1 = master_receive_2[0];
		rc_spi2_angle_2 = master_receive_2[3];

		rc_spi2_speed_1 = master_receive_2[1];
		rc_spi2_speed_2 = master_receive_2[4];
		rc_spi2_current_1 = master_receive_2[2];

		rc_spi2_current_2 = master_receive_2[5];
		rc_spi2_protocol = master_receive_2[6];

		rc_spi2_currangle1 = master_receive_2[7];
		rc_spi2_currangle2 = master_receive_2[8];
	}
	if (spi2_cond == 2) {
		master_transmit_2[0] = 0;
		master_transmit_2[1] = 0;
		master_transmit_2[2] = 0;
		master_transmit_2[3] = 0;
		master_transmit_2[4] = 0;
		master_transmit_2[5] = 0;
		master_transmit_2[6] = 0;
		master_transmit_2[7] = 0;
		master_transmit_2[8] = 0;

		master_receive_2[0] = 0;
		master_receive_2[1] = 0;
		master_receive_2[2] = 0;
		master_receive_2[3] = 0;
		master_receive_2[4] = 0;
		master_receive_2[5] = 0;
		master_receive_2[6] = 0;
		master_receive_2[7] = 0;
		master_receive_2[8] = 0;
		HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*) master_transmit_2,
				(uint8_t*) master_receive_2, 36);
		spi2_cond = 0;
	}
	HAL_StatusTypeDef status2 = HAL_SPI_TransmitReceive_DMA(&hspi2,
			(uint8_t*) master_transmit_2, (uint8_t*) master_receive_2, 36);
	if (status2 == HAL_OK) {
		HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*) master_transmit_2,
				(uint8_t*) master_receive_2, 36);
	}
}
void spi3_send() {
	if (spi3_cond == 1) {
		rc_spi3_angle_1 = master_receive_3[0];
		rc_spi3_speed_1 = master_receive_3[1];
		rc_spi3_current_1 = master_receive_3[2];
		rc_spi3_angle_2 = master_receive_3[3];
		rc_spi3_speed_2 = master_receive_3[4];
		rc_spi3_current_2 = master_receive_3[5];
		rc_spi3_protocol = master_receive_3[6];
		rc_spi3_currangle1 = master_receive_3[7];
		rc_spi3_currangle2 = master_receive_3[8];
	}
	if (spi3_cond == 2) {
		master_transmit_3[0] = 0;
		master_transmit_3[1] = 0;
		master_transmit_3[2] = 0;
		master_transmit_3[3] = 0;
		master_transmit_3[4] = 0;
		master_transmit_3[5] = 0;
		master_transmit_3[6] = 0;
		master_transmit_3[7] = 0;
		master_transmit_3[8] = 0;

		master_receive_3[0] = 0;
		master_receive_3[1] = 0;
		master_receive_3[2] = 0;
		master_receive_3[3] = 0;
		master_receive_3[4] = 0;
		master_receive_3[5] = 0;
		master_receive_3[6] = 0;
		master_receive_3[7] = 0;
		master_receive_3[8] = 0;

		HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) master_transmit_3,
				(uint8_t*) master_receive_3, 36);
		spi3_cond = 0;
	}
	HAL_StatusTypeDef status3 = HAL_SPI_TransmitReceive_DMA(&hspi3,
			(uint8_t*) master_transmit_3, (uint8_t*) master_receive_3, 36);
	if (status3 == HAL_OK) {
		HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) master_transmit_3,
				(uint8_t*) master_receive_3, 36);
	}
}
void flag_on() {
	if (HAL_GPIO_ReadPin(IFREADY3_GPIO_Port, IFREADY3_Pin) == 1) {
		spi1_clear();
		spi2_clear();
		spi3_clear();
	}
}
void transmit_info() {
	master_transmit_1[0] = com_spi1_angle1;
	master_transmit_1[1] = com_spi1_angle2;
	master_transmit_1[2] = ifready;
//	master_transmit_1[2] = 1;
//	master_transmit_1[3] = stop_com;
	master_transmit_1[3] = 0;
	master_transmit_1[4] = spi1_adaptive_1_var1;
	master_transmit_1[5] = spi1_adaptive_2_var1;
	master_transmit_1[6] = protocol;
	master_transmit_1[7] = set_to_zero;
	master_transmit_1[8] = set_speed_limit;

	master_transmit_2[0] = com_spi2_angle1;
	master_transmit_2[1] = com_spi2_angle2;
	master_transmit_2[2] = ifready;
//	master_transmit_2[3] = stop_com
//	master_transmit_2[2] = 1;
	master_transmit_2[3] = 0;
	master_transmit_2[4] = spi2_adaptive_1_var1;
	master_transmit_2[5] = spi2_adaptive_2_var1;
	master_transmit_2[6] = protocol;
	master_transmit_2[7] = set_to_zero;
	master_transmit_2[8] = set_speed_limit;

	master_transmit_3[0] = com_spi3_angle1;
	master_transmit_3[1] = com_spi3_angle2;
	master_transmit_3[2] = ifready;
//	master_transmit_3[3] = stop_com;
//	master_transmit_3[2] = 1;
	master_transmit_3[3] = 0;
	master_transmit_3[4] = spi3_adaptive_1_var1;
	master_transmit_3[5] = spi3_adaptive_2_var1;
	master_transmit_3[6] = protocol;
	master_transmit_3[7] = set_to_zero;
	master_transmit_3[8] = set_speed_limit;

	tr_spi_1_command1 = master_transmit_1[0];
	tr_spi_1_command2 = master_transmit_1[1];
	tr_spi_1_command3 = master_transmit_1[2];
	tr_spi_1_command4 = master_transmit_1[3];
	tr_spi_1_command5 = master_transmit_1[4];
	tr_spi_1_command6 = master_transmit_1[5];
	tr_spi_1_protocol = master_transmit_1[6];
	tr_spi_1_command7 = master_transmit_1[7];
	tr_spi_1_command8 = master_transmit_1[8];

	tr_spi_2_command1 = master_transmit_2[0];
	tr_spi_2_command2 = master_transmit_2[1];
	tr_spi_2_command3 = master_transmit_2[2];
	tr_spi_2_command4 = master_transmit_2[3];
	tr_spi_2_command5 = master_transmit_2[4];
	tr_spi_2_command6 = master_transmit_2[5];
	tr_spi_2_protocol = master_transmit_2[6];
	tr_spi_2_command7 = master_transmit_2[7];
	tr_spi_2_command8 = master_transmit_2[8];

	tr_spi_3_command1 = master_transmit_3[0];
	tr_spi_3_command2 = master_transmit_3[1];
	tr_spi_3_command3 = master_transmit_3[2];
	tr_spi_3_command4 = master_transmit_3[3];
	tr_spi_3_command5 = master_transmit_3[4];
	tr_spi_3_command6 = master_transmit_3[5];
	tr_spi_3_protocol = master_transmit_3[6];
	tr_spi_3_command7 = master_transmit_3[7];
	tr_spi_3_command8 = master_transmit_3[8];
}

void received_transform() {
	angle_mot1 = (float) rc_spi1_angle_1 / 100.0f;
	angle_mot2 = (float) rc_spi1_angle_2 / 100.0f;
	angle_mot3 = (float) rc_spi2_angle_1 / 100.0f;
	angle_mot4 = (float) rc_spi2_angle_2 / 100.0f;
	angle_mot5 = (float) rc_spi3_angle_1 / 100.0f;
	angle_mot6 = (float) rc_spi3_angle_2 / 100.0f;

	speed_mot1 = (float) rc_spi1_speed_1 / 100.0f;
	speed_mot2 = (float) rc_spi1_speed_2 / 100.0f;
	speed_mot3 = (float) rc_spi2_speed_1 / 100.0f;
	speed_mot4 = (float) rc_spi2_speed_2 / 100.0f;
	speed_mot5 = (float) rc_spi3_speed_1 / 100.0f;
	speed_mot6 = (float) rc_spi3_speed_2 / 100.0f;

	current_mot1 = (float) rc_spi1_current_1 / 10000.0f;
	current_mot2 = (float) rc_spi1_current_2 / 10000.0f;
	current_mot3 = (float) rc_spi2_current_1 / 10000.0f;
	current_mot4 = (float) rc_spi2_current_2 / 10000.0f;
	current_mot5 = (float) rc_spi3_current_1 / 10000.0f;
	current_mot6 = (float) rc_spi3_current_2 / 10000.0f;

	current_angles[0] = angle_mot1;
	current_angles[1] = angle_mot2;
	current_angles[2] = angle_mot3;
	current_angles[3] = angle_mot4;
	current_angles[4] = angle_mot5;
	current_angles[5] = angle_mot6;
}

void spi_to_uart() {
	uart_transmit[0] = rc_spi1_angle_1;
	uart_transmit[1] = rc_spi1_speed_1;
	uart_transmit[2] = rc_spi1_current_1;
	uart_transmit[3] = rc_spi1_angle_2;
	uart_transmit[4] = rc_spi1_speed_2;
	uart_transmit[5] = rc_spi1_current_2;

	uart_transmit[6] = rc_spi2_angle_1;
	uart_transmit[7] = rc_spi2_speed_1;
	uart_transmit[8] = rc_spi2_current_1;
	uart_transmit[9] = rc_spi2_angle_2;
	uart_transmit[10] = rc_spi2_speed_2;
	uart_transmit[11] = rc_spi2_current_2;

	uart_transmit[12] = rc_spi3_angle_1;
	uart_transmit[13] = rc_spi3_speed_1;
	uart_transmit[14] = rc_spi3_current_1;
	uart_transmit[15] = rc_spi3_angle_2;
	uart_transmit[16] = rc_spi3_speed_2;
	uart_transmit[17] = rc_spi3_current_2;

	if (uart_tick) {
		uart_tick = 0;
		spi_to_uart();
		HAL_UART_Transmit_DMA(&huart6, (uint8_t*) uart_transmit, 72);
		HAL_UART_Receive_DMA(&huart6, (uint8_t*) uart_recieve, 24);
		tr_uart_1 = uart_recieve[0];
		tr_uart_2 = uart_recieve[1];
		tr_uart_3 = uart_recieve[2];
		tr_uart_4 = uart_recieve[3];
		tr_uart_5 = uart_recieve[4];
		tr_uart_6 = uart_recieve[5];
	}
	if (uart_tick) {

		uart_tick = 0;
		spi_to_uart();
		HAL_UART_Transmit_DMA(&huart6, (uint8_t*) uart_transmit, 72);
	}
//		linear_velocity_x = 0.5;
//		linear_velocity_y = 0;
//		angular_velocity = 0;
//
//		calculate_wheel_angles(linear_velocity_x, linear_velocity_y,
//				angular_velocity, current_angles, delta_angles, 0.5);
//
//		for (int i = 0; i < 6; ++i) {
//			target_angles[i] = current_angles[i] + delta_angles[i];
//			if (target_angles[i] < 0)
//				target_angles[i] += 720;
//			if (target_angles[i] >= 720)
//				target_angles[i] -= 720;
//		}
//		ta1 = target_angles[0];
//		ta2 = target_angles[1];
//		ta3 = target_angles[2];
//		ta4 = target_angles[3];
//		ta5 = target_angles[4];
//		ta6 = target_angles[5];
}
void spi1_clear() {
	rc_spi1_angle_1 = 0;
	rc_spi1_speed_1 = 0;
	rc_spi1_current_1 = 0;
	rc_spi1_angle_2 = 0;
	rc_spi1_speed_2 = 0;
	rc_spi1_current_2 = 0;
	rc_spi1_protocol = 0;
	rc_spi1_currangle1 = 0;
	rc_spi1_currangle2 = 0;
}
void spi2_clear() {
	rc_spi2_angle_1 = 0;
	rc_spi2_speed_1 = 0;
	rc_spi2_current_1 = 0;
	rc_spi2_angle_2 = 0;
	rc_spi2_speed_2 = 0;
	rc_spi2_current_2 = 0;
	rc_spi2_protocol = 0;
	rc_spi2_currangle1 = 0;
	rc_spi2_currangle2 = 0;
}
void spi3_clear() {
	rc_spi3_angle_1 = 0;
	rc_spi3_speed_1 = 0;
	rc_spi3_current_1 = 0;
	rc_spi3_angle_2 = 0;
	rc_spi3_speed_2 = 0;
	rc_spi3_current_2 = 0;
	rc_spi3_protocol = 0;
	rc_spi3_currangle1 = 0;
	rc_spi3_currangle2 = 0;
}

void spi_check() {
	if (spi1_success != spi_suc_1) {
		spi_f1 = 1;
	} else {
		spi_f1 = 0;
	}
	if (spi2_success != spi_suc_2) {
		spi_f2 = 1;
	} else {
		spi_f2 = 0;
	}
	if (spi3_success != spi_suc_3) {
		spi_f3 = 1;
	} else {
		spi_f3 = 0;
	}
	if (spi_f1 == 1 && spi_f2 == 1 && spi_f3 == 1) {
		HAL_GPIO_WritePin(GREEN_LED_SPI_GPIO_Port, GREEN_LED_SPI_Pin, 1);
		HAL_GPIO_WritePin(RED_LED_SPI_GPIO_Port, RED_LED_SPI_Pin, 0);
	} else {
		HAL_GPIO_WritePin(GREEN_LED_SPI_GPIO_Port, GREEN_LED_SPI_Pin, 0);
		HAL_GPIO_WritePin(RED_LED_SPI_GPIO_Port, RED_LED_SPI_Pin, 1);
	}
	spi_suc_1 = spi1_success;
	spi_suc_2 = spi2_success;
	spi_suc_3 = spi3_success;
}
void angle_motor_calc() {
	angle_mot[0] = angle_mot1;
	angle_mot[1] = angle_mot2;
	angle_mot[2] = angle_mot3;
	angle_mot[3] = angle_mot4;
	angle_mot[4] = angle_mot5;
	angle_mot[5] = angle_mot6;

	for (int i = 0; i < NUM_MOTORS; i++) {
		angle_circ[i] = angle_mot[i] % 360;
		if (angle_circ[i] < 0) {
			angle_circ[i] += 360;
		}

		diff_angle[i] = (float) (angle_circ[i] - angle_circ_old[i]) / 100.0f;

		if (angle_circ[i] > 0 && angle_circ[i] < 180) {
			derx[i] = -(0.11 * cosf((float) angle_circ[i]) - 0.11)
					* diff_angle[i];
		} else {
			derx[i] = -(2 * 0.11 * cosf((float) angle_circ[i])) * diff_angle[i];
		}

		angle_circ_old[i] = angle_circ[i];
	}
	derx_1 = derx[0];
	derx_2 = derx[1];
	derx_3 = derx[2];
	derx_4 = derx[3];
	derx_5 = derx[4];
	derx_6 = derx[5];
}

void f_kinematics() {
	average_speed_left = (derx[0] + derx[2] + derx[4]) / 3.00f;
	average_speed_right = (derx[1] + derx[3] + derx[5]) / 3.00f;
	average_speed_sum = (average_speed_left + average_speed_right);
	average_speed_diff = (average_speed_left - average_speed_right);
	main_speed_vector = average_speed_sum / 2.00f;
	sum_phi += average_speed_diff * 0.01;
	cos_phi = cosf(sum_phi);
	sin_phi = sinf(sum_phi);

	vector_speed_y = main_speed_vector * sin_phi;
	vector_speed_x = main_speed_vector * cos_phi;

	path_x += vector_speed_x * 0.01;
	path_y += vector_speed_y * 0.01;
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == hspi1.Instance) {
		spi1_error++;
		HAL_SPI_DMAStop(hspi);
		__HAL_SPI_CLEAR_OVRFLAG(hspi);
		__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
		__HAL_SPI_CLEAR_MODFFLAG(hspi);
		__HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, DMA_FLAG_TCIF2_6);
		__HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, DMA_FLAG_TCIF0_4);

		spi1_cond = 2;

		master_receive_1[0] = 0;
		master_receive_1[1] = 0;
		master_receive_1[2] = 0;
		master_receive_1[3] = 0;
		master_receive_1[4] = 0;
		master_receive_1[5] = 0;
		master_receive_1[6] = 0;
		master_receive_1[7] = 0;
		master_receive_1[8] = 0;
		spi1_clear();
	}
	if (hspi->Instance == hspi2.Instance) {
		spi2_error++;
		HAL_SPI_DMAStop(hspi);
		__HAL_SPI_CLEAR_OVRFLAG(hspi);
		__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
		__HAL_SPI_CLEAR_MODFFLAG(hspi);
		__HAL_DMA_CLEAR_FLAG(hspi2.hdmatx, DMA_FLAG_TCIF0_4); // Updated line for SPI2_TX DMA1 Stream 4
		__HAL_DMA_CLEAR_FLAG(hspi2.hdmarx, DMA_FLAG_TCIF3_7);

		spi2_cond = 2;

		master_receive_2[0] = 0;
		master_receive_2[1] = 0;
		master_receive_2[2] = 0;
		master_receive_2[3] = 0;
		master_receive_2[4] = 0;
		master_receive_2[5] = 0;
		master_receive_2[6] = 0;
		master_receive_2[7] = 0;
		master_receive_2[8] = 0;
		spi2_clear();
	}
	if (hspi->Instance == hspi3.Instance) {
		spi3_error++;
		HAL_SPI_DMAStop(hspi);
		__HAL_SPI_CLEAR_OVRFLAG(hspi);
		__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
		__HAL_SPI_CLEAR_MODFFLAG(hspi);
		__HAL_DMA_CLEAR_FLAG(hspi3.hdmatx, DMA_FLAG_TCIF0_4); // Updated line for SPI3_TX DMA1 Stream 0
		__HAL_DMA_CLEAR_FLAG(hspi3.hdmarx, DMA_FLAG_TCIF1_5);

		spi3_cond = 2;

		master_receive_3[0] = 0;
		master_receive_3[1] = 0;
		master_receive_3[2] = 0;
		master_receive_3[3] = 0;
		master_receive_3[4] = 0;
		master_receive_3[5] = 0;
		master_receive_3[6] = 0;
		master_receive_3[7] = 0;
		master_receive_3[8] = 0;
		spi3_clear();
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == hspi1.Instance) {
		if (master_receive_1[6] != 666) {
			__HAL_SPI_CLEAR_OVRFLAG(hspi);
			__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
			__HAL_SPI_CLEAR_MODFFLAG(hspi);
			__HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, DMA_FLAG_TCIF2_6);
			__HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, DMA_FLAG_TCIF0_4);
			HAL_SPI_ErrorCallback(&hspi1);
		}
		if (master_receive_1[6] == 666) {
			if (HAL_SPI_GetError(hspi) == HAL_SPI_ERROR_NONE) {
				spi1_cond = 1;
				spi1_success++;
			}
		}
	}
	if (hspi->Instance == hspi2.Instance) {
		if (master_receive_2[6] != 666) {
			__HAL_SPI_CLEAR_OVRFLAG(hspi);
			__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
			__HAL_SPI_CLEAR_MODFFLAG(hspi);
			__HAL_DMA_CLEAR_FLAG(hspi2.hdmatx, DMA_FLAG_TCIF0_4); // Updated line for SPI2_TX DMA1 Stream 4
			__HAL_DMA_CLEAR_FLAG(hspi2.hdmarx, DMA_FLAG_TCIF3_7);
			HAL_SPI_ErrorCallback(&hspi2);
		}
		if (master_receive_2[6] == 666) {
			if (HAL_SPI_GetError(hspi) == HAL_SPI_ERROR_NONE) {
				spi2_cond = 1;
				spi2_success++;
			}
		}
	}
	if (hspi->Instance == hspi3.Instance) {
		if (master_receive_3[6] != 666) {
			__HAL_SPI_CLEAR_OVRFLAG(hspi);
			__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
			__HAL_SPI_CLEAR_MODFFLAG(hspi);
			__HAL_DMA_CLEAR_FLAG(hspi3.hdmatx, DMA_FLAG_TCIF0_4); // Updated line for SPI3_TX DMA1 Stream 0
			__HAL_DMA_CLEAR_FLAG(hspi3.hdmarx, DMA_FLAG_TCIF1_5);
			HAL_SPI_ErrorCallback(&hspi3);
		}
		if (master_receive_3[6] == 666) {
			if (HAL_SPI_GetError(hspi) == HAL_SPI_ERROR_NONE) {
				spi3_cond = 1;
				spi3_success++;
			}
		}
	}
}
void bluetooth_uart() {
	HAL_StatusTypeDef status1 = HAL_UART_Receive_DMA(&huart1, &message_r, 1);
	if (status1 == HAL_OK) {
		HAL_UART_Receive_DMA(&huart1, &message_r, 1);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
