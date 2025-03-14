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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads1220.h"
#include "ee.h"
#include "eeConfig.h"
#include "printf.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct cal_point_struct {
  float reading;
  float value;
} CalPoint_t;

enum cal_state_enum {
  CAL_DEFAULT,
  CAL_IN_PROGRESS,
  CAL_GOOD
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DISP_CHAR_COUNT     4

#define RFL_PWR_LO_RNG_MUX  ADS1220_MUX_P0_NVSS
#define RFL_PWR_HI_RNG_MUX  ADS1220_MUX_P1_NVSS
#define FWD_PWR_LO_RNG_MUX  ADS1220_MUX_P2_NVSS
#define FWD_PWR_HI_RNG_MUX  ADS1220_MUX_P3_NVSS

#define ADC_INIT_MUX_SET    RFL_PWR_LO_RNG_MUX

/* Set up for 20 SPS, which allows the 50/60 Hz FIR filter to be used */
#define ADC_INIT_CONFIG   \ 
  ((ADC_INIT_MUX_SET << 24) | \
  ((ADS1220_MODE_NORMAL | ADS1220_DR_0) << 16) | \
  (ADS1220_VREF_EXT << 8) | (ADS1220_FIR_50_60))

#define VREF_IDEAL          4.096

#define CAL_POINTS          26
#define FLASH_FLAG          0xA5A5A5A5
#define UART_RX_WORD_SIZE   4
#define UART_BEGIN_CAL      0x12345678
#define UART_CAL_FWD_LO     0x11111111
#define UART_CAL_FWD_HI     0x22222222
#define UART_CAL_RFL_LO     0x33333333
#define UART_CAL_RFL_HI     0x44444444
#define UART_SAVE_CAL       0xDEADBEEF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static const uint8_t seven_seg_lut[12][7] = {
  {1,1,1,1,1,1,0}, // 0
  {0,1,1,0,0,0,0}, // 1
  {1,1,0,1,1,0,1}, // 2
  {1,1,1,1,0,0,1}, // 3
  {0,1,1,0,0,1,1}, // 4
  {1,0,1,1,0,1,1}, // 5
  {1,0,1,1,1,1,1}, // 6
  {1,1,1,0,0,0,0}, // 7
  {1,1,1,1,1,1,1}, // 8
  {1,1,1,1,0,1,1}, // 9
  {0,0,0,0,0,0,0}, // blank
  {0,0,0,0,0,0,1}  // -
};

uint8_t disp_char_sel = 0;
uint8_t disp_1_values[DISP_CHAR_COUNT] = {0};
uint8_t disp_2_values[DISP_CHAR_COUNT] = {0};
bool update_disp_vals = false;

Ads1220_t adc;
bool adc_drdy_flag = false;
uint8_t last_adc_mux = ADC_INIT_MUX_SET;
float fwd_hi_det_offset = 0.05;
float fwd_lo_det_offset = 0.05;
float rfl_hi_det_offset = 0.05;
float rfl_lo_det_offset = 0.05;

uint8_t cal_state = CAL_DEFAULT;
int cal_step = 0;
CalPoint_t fwd_hi_rng_cal[CAL_POINTS];
CalPoint_t fwd_lo_rng_cal[CAL_POINTS];
CalPoint_t rfl_hi_rng_cal[CAL_POINTS];
CalPoint_t rfl_lo_rng_cal[CAL_POINTS];
uint8_t uart_data[UART_RX_WORD_SIZE];

float fwd_hi_rng_v = 0.0;
float fwd_lo_rng_v = 0.0;
float rfl_hi_rng_v = 0.0;
float rfl_lo_rng_v = 0.0;
float fwd_pwr = 0.0;
float rfl_pwr = 0.0;
float vswr = 0.0;
float dir_coupler_factor_db = 20.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_adc_cs(int state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // init pins to stable state
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, 1); 

  HAL_GPIO_WritePin(CHAR_1_GPIO_Port, CHAR_1_Pin, 0);
  HAL_GPIO_WritePin(CHAR_2_GPIO_Port, CHAR_2_Pin, 0);
  HAL_GPIO_WritePin(CHAR_3_GPIO_Port, CHAR_3_Pin, 0);
  HAL_GPIO_WritePin(CHAR_4_GPIO_Port, CHAR_4_Pin, 0);
  

  /* See if flash has data in it. */
  ee_init();
  uint32_t flag = 0;
  uint32_t addr = 0;
  ee_read(addr, 4, (uint8_t*)&flag);
  if (flag == FLASH_FLAG) {
    /* Retrieve calibration from flash */

    cal_state = CAL_GOOD;
  } else {
    /* write some default calibration values to ram */
    float voltage = 0.005;
    float power = -26.0;
    for (int i = 0; i < CAL_POINTS; i++) {
      fwd_lo_rng_cal[i].reading = voltage;
      fwd_lo_rng_cal[i].value = power;
      rfl_lo_rng_cal[i].reading = voltage;
      rfl_lo_rng_cal[i].value = power;
      voltage += 0.125;
      power += 1.0;
    }
    voltage = 0.01;
    for (int i = 0; i < CAL_POINTS; i++) {
      fwd_hi_rng_cal[i].reading = voltage;
      fwd_hi_rng_cal[i].value = power;
      rfl_hi_rng_cal[i].reading = voltage;
      rfl_hi_rng_cal[i].value = power;
      voltage += 0.125;
      power += 1.0;
    }
  }

  // ADC setup
  adc.vref = VREF_IDEAL;
  adc.hspi = &hspi1;
  adc.set_cs = set_adc_cs;
  ads_reset(&adc);
  HAL_Delay(10);
  ads_begin(&adc, ADC_INIT_CONFIG);
  HAL_Delay(2);
  ads_start_sync(&adc);

  HAL_TIM_Base_Start_IT(&htim1); // timer for flipping through the display characters

  HAL_UART_Receive_DMA(&huart1, uart_data, UART_RX_WORD_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (adc_drdy_flag) {
      adc_drdy_flag = false;
      float adc_rdg = ads_get_voltage(&adc);

      uint8_t next_adc_mux = 0;
      switch(adc.mux_setting) {
        default: {
          next_adc_mux = ADC_INIT_MUX_SET;
          break;
        }
        case FWD_PWR_LO_RNG_MUX: {
          fwd_lo_rng_v = adc_rdg - fwd_lo_det_offset;
          next_adc_mux = FWD_PWR_HI_RNG_MUX;
          break;
        }
        case FWD_PWR_HI_RNG_MUX: {
          fwd_hi_rng_v = adc_rdg - fwd_hi_det_offset;
          next_adc_mux = RFL_PWR_LO_RNG_MUX;
          break;
        }
        case RFL_PWR_LO_RNG_MUX: {
          rfl_lo_rng_v = adc_rdg - rfl_lo_det_offset;
          next_adc_mux = RFL_PWR_HI_RNG_MUX;
          break;
        }
        case RFL_PWR_HI_RNG_MUX: {
          rfl_hi_rng_v = adc_rdg - rfl_hi_det_offset;
          update_disp_vals = true;
          next_adc_mux = FWD_PWR_LO_RNG_MUX;
          break;
        }
      }
      ads_set_input_mux(&adc, next_adc_mux);
      ads_start_sync(&adc);

      if (update_disp_vals) {
        // TODO: compute power and VSWR
        static char scratch[16];
        snprintf(scratch, 16, "%05.2f", fwd_pwr);
        update_disp_vals = false;
      }
    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void set_adc_cs(int state) {
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, state & 1);
}

/**
 * My implementation of timer callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  HAL_UART_Receive_DMA(&huart1, uart_data, UART_RX_WORD_SIZE);
}

/**
 * My implementation of gpio interrupt callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ADC_DRDY_Pin) {
    adc_drdy_flag = true;
  }
}

/**
 * what to do when we've received a word of data
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uint32_t rx_word = 0;
  memcpy((uint8_t*)&rx_word, uart_data, UART_RX_WORD_SIZE);

  if (cal_state != CAL_IN_PROGRESS && rx_word == UART_BEGIN_CAL) {

    cal_step = 0;
    cal_state = CAL_IN_PROGRESS;

  } else if (cal_state == CAL_IN_PROGRESS) {
    if(rx_word == UART_SAVE_CAL) {
      /* TODO save calibration to flash */

      cal_state = CAL_GOOD;
    } 
    else {
      if (cal_step < CAL_POINTS) {
        /* calibrating FWD_LO_RNG */
      }
      else if (cal_step < 2*CAL_POINTS) {
        /* calibrating FWD_HI_RNG */
      }
      else if (cal_step < 3*CAL_POINTS) {
        /* calibrating RFL_LO_RNG */
      }
      else if (cal_step < 4*CAL_POINTS) {
        /* calibrating RFL_HI_RNG */
      } else {
        cal_step = 0; /* at this point, calibration should end anyway */
      }


      cal_step++;
    }
  }

  HAL_UART_Receive_DMA(&huart1, uart_data, UART_RX_WORD_SIZE);
}

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
