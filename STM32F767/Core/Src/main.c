/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ArduCAM.h"
#include "capture.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum CameraWorkMode
{
    JPEGStartSingleShot     = 1,
    JPEGStartVideoStreaming = 2,
    BMPStartSingleShot      = 3
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t vid, pid;
uint8_t Camera_WorkMode = 0;
uint8_t start_shoot     = 0;
uint8_t stop            = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/// Detection of the camera model which tests communication between that
/// peripheral and STM. Prints camera's model on UART. Sets correct camera model
/// and address value for ArduCAM library.
void CameraModelDetection()
{
    while (1)
    {
        sensor_addr = 0x60;
        wrSensorReg8_8(0xff, 0x01); // Table13 register bank
        rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        if (vid == 0x26 && (pid == 0x41 || pid == 0x42))
        {
            sensor_model = OV2640;
            uprintf("ACK CMD OV2640 detected.\r\n");
            break;
        }
        else
        {
            uprintf("ACK CMD Can't find OV2640 module!\r\n");
        }
        sensor_addr = 0x78;
        rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
        rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
        if ((vid == 0x56) && (pid == 0x40))
        {
            sensor_model = OV5640;
            uprintf("ACK CMD OV5640 detected.\r\n");
            break;
        }
        else
        {
            uprintf("ACK CMD Can't find OV5640 module!\r\n");
        }

        sensor_addr = 0x78;
        rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
        rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
        if ((vid == 0x56) && (pid == 0x42))
        {
            sensor_model = OV5642;
            uprintf("ACK CMD OV5642 detected.\r\n");
            break;
        }
        else
        {
            uprintf("ACK CMD Can't find OV5642 module!\r\n");
        }
        HAL_Delay(1000);
    }
}

/// Process input from USART
void ProcessInput()
{
    uint8_t received = 0;
    if (NewCMD == 1)
    {
        NewCMD = 0;
        while (HAL_UART_Receive_IT(&huart3, &received, 0x01) != HAL_BUSY)
            ;
        switch (received)
        {
            case 0:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_160x120);
                    uprintf("ACK CMD switch to OV2640_160x120\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_320x240);
                    uprintf("ACK CMD switch to OV5640_320x240\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_320x240);
                    uprintf("ACK CMD switch to OV5642_320x240\r\n");
                }
                break;
            case 1:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_176x144);
                    uprintf("ACK CMD switch to OV2640_176x144\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_352x288);
                    uprintf("ACK CMD switch to OV5640_352x288\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_640x480);
                    uprintf("ACK CMD switch to OV5642_640x480\r\n");
                }
                break;
            case 2:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_320x240);
                    uprintf("ACK CMD switch to OV2640_320x240\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_640x480);
                    uprintf("ACK CMD switch to OV5640_640x480\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_1024x768);
                    uprintf("ACK CMD switch to OV5642_1024x768\r\n");
                }
                break;
            case 3:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_352x288);
                    uprintf("ACK CMD switch to OV2640_352x288\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_800x480);
                    uprintf("ACK CMD switch to OV5640_800x480\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_1280x960);
                    uprintf("ACK CMD switch to OV5642_1280x960\r\n");
                }
                break;
            case 4:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_640x480);
                    uprintf("ACK CMD switch to OV2640_640x480\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_1024x768);
                    uprintf("ACK CMD switch to OV5640_1024x768\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_1600x1200);
                    uprintf("ACK CMD switch to OV5642_1600x1200\r\n");
                }
                break;
            case 5:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_800x600);
                    uprintf("ACK CMD switch to OV2640_800x600\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_1280x960);
                    uprintf("ACK CMD switch to OV5640_1280x960\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_2048x1536);
                    uprintf("ACK CMD switch to OV5642_2048x1536\r\n");
                }
                break;
            case 6:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_1024x768);
                    uprintf("ACK CMD switch to OV2640_1024x768\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_1600x1200);
                    uprintf("ACK CMD switch to OV5640_1600x1200\r\n");
                }
                else if (sensor_model == OV5642)
                {
                    OV5642_set_JPEG_size(OV5642_2592x1944);
                    uprintf("ACK CMD switch to OV5642_2592x1944\r\n");
                }
                break;
            case 7:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_1280x1024);
                    uprintf("ACK CMD switch to OV2640_1280x1024\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_2048x1536);
                    uprintf("ACK CMD switch to OV5640_2048x1536\r\n");
                }
                break;
            case 8:
                if (sensor_model == OV2640)
                {
                    OV2640_set_JPEG_size(OV2640_1600x1200);
                    uprintf("ACK CMD switch to OV2640_1600x1200\r\n");
                }
                else if (sensor_model == OV5640)
                {
                    OV5640_set_JPEG_size(OV5640_2592x1944);
                    uprintf("ACK CMD switch to OV5640_2592x1944\r\n");
                }
                break;
            case 0x10:
                Camera_WorkMode = 1;
                start_shoot     = 1;
                uprintf("ACK CMD CAM start single shoot.\r\n");
                break;
            case 0x11:
                set_format(JPEG_FORMAT);
                ArduCAM_Init();
#if !defined(OV2640)
                set_bit(ARDUCHIP_TIM, VSYNC_MASK);
#endif
                break;
            case 0x20:
                Camera_WorkMode = 2;
                start_shoot     = 2;
                uprintf("ACK CMD CAM start video streaming.\r\n");
                break;
            case 0x21:
                stop = 1;
                uprintf("ACK CMD CAM stop video streaming.\r\n");
                break;
            case 0x30:
                Camera_WorkMode = 3;
                start_shoot     = 3;
                uprintf("ACK CMD CAM start single BMP shoot.\r\n");
                break;
            case 0x31:
                set_format(BMP_FORMAT);
                ArduCAM_Init();
                if (sensor_model != OV2640)
                {
                    clear_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
                }
                wrSensorReg16_8(0x3818, 0x81);
                wrSensorReg16_8(0x3621, 0xa7);
                uprintf("ACK CMD SetToBMP \r\n");
                break;
            default:
                break;
        }
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
  MX_DCMI_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    delay_init();
    sccb_bus_init();

    // Test of I2C communication with camera and printing its model on UART
    CameraModelDetection();

    ArduCAM_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        ProcessInput();
        if (Camera_WorkMode == JPEGStartSingleShot)
        {
            // TODO: Handle JPEG capture
            if (start_shoot == JPEGStartSingleShot)
            {
                start_shoot = 0;
                SingleCapTransfer();
            }
            if (receive_OK)
            {
                receive_OK = 0;
                SendbyUSART1();
            }
        }
        else if (Camera_WorkMode == JPEGStartVideoStreaming)
        {
            // TODO: Handle JPEG capture
            if (start_shoot == JPEGStartVideoStreaming)
            {
                if (send_OK)
                {
                    if (stop)
                    {
                        uprintf("ACK CMD CAM stop video streaming.\r\n");
                        stop            = 0;
                        Camera_WorkMode = 0;
                        start_shoot     = 0;
                    }
                    send_OK = false;
                    SingleCapTransfer();
                }
                if (receive_OK)
                {
                    receive_OK = 0;
                    SendbyUSART1();
                }
            }
        }
        else if (Camera_WorkMode == BMPStartSingleShot)
        {
            if (start_shoot == BMPStartSingleShot)
            {
                start_shoot     = 0;
                Camera_WorkMode = 0;
                StartBMPcapture();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);
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
    /* User can add his own implementation to report the HAL error return state
     */
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
    /* User can add his own implementation to report the file name and line
       number, ex: uprintf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
