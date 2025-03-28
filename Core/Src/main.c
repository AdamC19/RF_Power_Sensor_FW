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
#define DISP_STR_LEN        5

#define RNG_SEL_LOW         0
#define RNG_SEL_HIGH        1

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

#define ADC_RDG_MAX         3.85
#define RNG_SEL_HYST        0.05
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
uint8_t disp1_str_ind = 0;
uint8_t disp2_str_ind = 0;
uint8_t disp_1_str[DISP_STR_LEN + 1] = {};
uint8_t disp_2_str[DISP_STR_LEN + 1] = {};
uint8_t disp_1_values[DISP_CHAR_COUNT] = {0};
uint8_t disp_2_values[DISP_CHAR_COUNT] = {0};
bool update_disp_vals = false;

Ads1220_t adc;
bool adc_drdy_flag = false;
uint8_t last_adc_mux = ADC_INIT_MUX_SET;
uint8_t fwd_rng_sel = RNG_SEL_LOW;
uint8_t rfl_rng_sel = RNG_SEL_LOW;
float fwd_hi_det_offset = 0.02;
float fwd_lo_det_offset = 0.02;
float rfl_hi_det_offset = 0.02;
float rfl_lo_det_offset = 0.02;

uint8_t cal_state = CAL_DEFAULT;
int cal_step = 0;
CalPoint_t cal_data_tmp[CAL_POINTS];
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
float dir_coupler_factor = 900.0; // 30 turns, 29.54...dB

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_adc_cs(int state);
void set_disp_1_pins(int value, bool dp_on);
void set_disp_2_pins(int value, bool dp_on);
void select_disp_char(int sel);
float get_cal_power(float value, CalPoint_t* cal_data);

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
    float voltage = 0.0;
    float v_step = ADC_RDG_MAX / (CAL_POINTS - 1);
    for (int i = 0; i < CAL_POINTS; i++) {
      float power_tmp = 0.2579*voltage + 2.064e-4;
      fwd_lo_rng_cal[i].reading = voltage;
      fwd_lo_rng_cal[i].value = power_tmp;
      rfl_lo_rng_cal[i].reading = voltage;
      rfl_lo_rng_cal[i].value = power_tmp;
      voltage += 0.15;
    }
    voltage = 0.0;
    for (int i = 0; i < CAL_POINTS; i++) {
      float power_tmp = 29.71*voltage + 6.962e-2;
      fwd_hi_rng_cal[i].reading = voltage;
      fwd_hi_rng_cal[i].value = power_tmp;
      rfl_hi_rng_cal[i].reading = voltage;
      rfl_hi_rng_cal[i].value = power_tmp;
      voltage += 0.15;
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
          fwd_lo_rng_v = adc_rdg;
          next_adc_mux = FWD_PWR_HI_RNG_MUX;
          break;
        }
        case FWD_PWR_HI_RNG_MUX: {
          fwd_hi_rng_v = adc_rdg;
          next_adc_mux = RFL_PWR_LO_RNG_MUX;
          break;
        }
        case RFL_PWR_LO_RNG_MUX: {
          rfl_lo_rng_v = adc_rdg;
          next_adc_mux = RFL_PWR_HI_RNG_MUX;
          break;
        }
        case RFL_PWR_HI_RNG_MUX: {
          rfl_hi_rng_v = adc_rdg;
          update_disp_vals = true;
          next_adc_mux = FWD_PWR_LO_RNG_MUX;
          break;
        }
      }
      ads_set_input_mux(&adc, next_adc_mux);
      ads_start_sync(&adc);

      if (update_disp_vals) {
        // make decisions on which range to use for power computations
        if (fwd_rng_sel == RNG_SEL_LOW && fwd_lo_rng_v > (ADC_RDG_MAX + RNG_SEL_HYST)) {
          fwd_rng_sel = RNG_SEL_HIGH;
        }
        else if (fwd_rng_sel == RNG_SEL_HIGH && fwd_lo_rng_v < (ADC_RDG_MAX - RNG_SEL_HYST)) {
          fwd_rng_sel = RNG_SEL_LOW;
        }

        if (fwd_rng_sel == RNG_SEL_LOW) {
          fwd_pwr = dir_coupler_factor * get_cal_power(fwd_lo_rng_v - fwd_lo_det_offset, fwd_lo_rng_cal);
        }
        else {
          fwd_pwr = dir_coupler_factor * get_cal_power(fwd_hi_rng_v - fwd_hi_det_offset, fwd_hi_rng_cal);
        }

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


void set_disp_1_pins(int value, bool dp_on) {
  if (value == ('-' - 48)) {
    value = 11; // LUT index of the hyphen
  }  else if (value >= 12 || value < 0) {
    value = 10; // blank
  } 

  uint8_t* segment = seven_seg_lut[value];
  HAL_GPIO_WritePin(DISP1_A_GPIO_Port, DISP1_A_Pin, segment[0]);
  HAL_GPIO_WritePin(DISP1_B_GPIO_Port, DISP1_B_Pin, segment[1]);
  HAL_GPIO_WritePin(DISP1_C_GPIO_Port, DISP1_C_Pin, segment[2]);
  HAL_GPIO_WritePin(DISP1_D_GPIO_Port, DISP1_D_Pin, segment[3]);
  HAL_GPIO_WritePin(DISP1_E_GPIO_Port, DISP1_E_Pin, segment[4]);
  HAL_GPIO_WritePin(DISP1_F_GPIO_Port, DISP1_F_Pin, segment[5]);
  HAL_GPIO_WritePin(DISP1_G_GPIO_Port, DISP1_G_Pin, segment[6]);
  HAL_GPIO_WritePin(DISP1_DP_GPIO_Port, DISP1_DP_Pin, dp_on);
}


void set_disp_2_pins(int value, bool dp_on) {
  if (value == ('-' - 48)) {
    value = 11; // LUT index of the hyphen
  }  else if (value >= 12 || value < 0) {
    value = 10; // blank
  } 

  uint8_t* segment = seven_seg_lut[value];
  HAL_GPIO_WritePin(DISP2_A_GPIO_Port, DISP2_A_Pin, segment[0]);
  HAL_GPIO_WritePin(DISP2_B_GPIO_Port, DISP2_B_Pin, segment[1]);
  HAL_GPIO_WritePin(DISP2_C_GPIO_Port, DISP2_C_Pin, segment[2]);
  HAL_GPIO_WritePin(DISP2_D_GPIO_Port, DISP2_D_Pin, segment[3]);
  HAL_GPIO_WritePin(DISP2_E_GPIO_Port, DISP2_E_Pin, segment[4]);
  HAL_GPIO_WritePin(DISP2_F_GPIO_Port, DISP2_F_Pin, segment[5]);
  HAL_GPIO_WritePin(DISP2_G_GPIO_Port, DISP2_G_Pin, segment[6]);
  HAL_GPIO_WritePin(DISP2_DP_GPIO_Port, DISP2_DP_Pin, dp_on);
}


/**
 * Select which character of the displays to turn on. A value outside 0
 * through 3 will disable all displays
 */
void select_disp_char(int sel) {
  uint8_t bits = 0;
  if (sel >= 0 && sel < DISP_CHAR_COUNT) {
    bits = (1 << sel);
  }
  HAL_GPIO_WritePin(CHAR_1_GPIO_Port, CHAR_1_Pin, (bits >> 0) & 1);
  HAL_GPIO_WritePin(CHAR_2_GPIO_Port, CHAR_2_Pin, (bits >> 1) & 1);
  HAL_GPIO_WritePin(CHAR_3_GPIO_Port, CHAR_3_Pin, (bits >> 2) & 1);
  HAL_GPIO_WritePin(CHAR_4_GPIO_Port, CHAR_4_Pin, (bits >> 3) & 1);
}


/**
 * Calculate power based on the provided voltage value and calibration data.
 */
float get_cal_power(float rdg, CalPoint_t* cal_data) {
  int upper = CAL_POINTS;
  int lower = 0;
  int i;
  CalPoint_t* a;
  CalPoint_t* b;
  bool found = false;
  while (!found) {
    i = (int)((upper - lower) / 2) + lower;

    if (i <= 1) {
      break;
    }
    a = cal_data + (i - 1);
    b = cal_data + i;
    found = ((rdg > a->reading) && (rdg <= b->reading));
    if (!found) {
      if (rdg <= a->reading) {
        // shift search range down
        upper = i;
      }
      else if (rdg > b->reading) {
        // shift search range up
        lower = i;
      }
    }
  }
  float slope = (b->value - a->value)/(b->reading - a->reading);
  return a->value + (slope * (rdg - a->reading));
}


/**
 * My implementation of timer callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  char disp1_char = disp_1_str[disp1_str_ind];
  disp1_str_ind++;
  bool disp1_dp_on = false;
  if (disp_1_str[disp1_str_ind] == '.') {
    disp1_dp_on = true;
    disp1_str_ind++; // ensure next time, we're pointing at a character rather than a decimal point
  }

  char disp2_char = disp_2_str[disp2_str_ind];
  disp2_str_ind++;
  bool disp2_dp_on = false;
  if (disp_2_str[disp2_str_ind] == '.') {
    disp2_dp_on = true;
    disp2_str_ind++; // ensure next time, we're pointing at a character rather than a decimal point
  }

  select_disp_char(-1);
  set_disp_1_pins(disp1_char, disp1_dp_on);
  set_disp_2_pins(disp2_char, disp2_dp_on);
  select_disp_char(disp_char_sel);

  disp_char_sel++;
  if (disp_char_sel >= DISP_CHAR_COUNT) {
    disp_char_sel = 0;
  }
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
      /* no other match, so this must be calibration data */
      CalPoint_t* cal = NULL;
      int cal_ind = 0;
      float reading = 0.0;
      if (cal_step < CAL_POINTS) {
        /* calibrating FWD_LO_RNG */
        cal = fwd_lo_rng_cal;
        cal_ind = cal_step;
        reading = fwd_lo_rng_v; // whatever the last value was
      }
      else if (cal_step < 2*CAL_POINTS) {
        /* calibrating FWD_HI_RNG */
        cal = fwd_hi_rng_cal;
        cal_ind = cal_step - CAL_POINTS;
        reading = fwd_hi_rng_v; // whatever the last value was
      }
      else if (cal_step < 3*CAL_POINTS) {
        /* calibrating RFL_LO_RNG */
        cal = rfl_lo_rng_cal;
        cal_ind = cal_step - 2*CAL_POINTS;
        reading = rfl_lo_rng_v; // whatever the last value was
      }
      else if (cal_step < 4*CAL_POINTS) {
        /* calibrating RFL_HI_RNG */
        cal = rfl_hi_rng_cal;
        cal_ind = cal_step - 3*CAL_POINTS;
        reading = rfl_hi_rng_v; // whatever the last value was
      }
      cal_step++;

      if (cal_step >= 4*CAL_POINTS) {
        cal_step = 0;
      }
      
      if (cal != NULL) {
        cal[cal_ind].reading = reading;
        float value = 0.0;
        memcpy(&value, uart_data, sizeof(value));
        cal[cal_ind].value = value;
      }
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
