#include "stm32h7xx_hal.h"
#include "MemoryMap.h"

#include "bootloaderFunctions.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Increment_Bootloop_Count(volatile uint32_t*);
void Error_Handler(void);

typedef struct{
  uint32_t flags; //| boot_loop_count (Last 8bits) |   RESERVED / unused   |  flags (first 16bits)  |
  uint32_t magic;
} shared_memory_t;

//Shared Mem contains the flags and magic number shared between the apps and bootloader
volatile shared_memory_t sharedmemory __attribute__((section(".shared_memory")));

// #define SHARED_MAGIC 0x69696

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  
  // SwitchToApp(APP2);

  #if ENABLE_DEBUG
    SwitchToApp(APP2);
  #endif

  // VerifyApp(APP1);
  // GetAppPartition();
  // SetAppActive(APP1);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // for(int i = 0; i < 5; i++){
    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_12);
    // HAL_Delay(5000);
    // }
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12);
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(50);

    if(sharedmemory.magic == SHARED_MAGIC){
      //Get Flags from the shared memory
      Bootloader_Flag_Handler(&sharedmemory.flags);
    }
    else{
      sharedmemory.flags = 0; // क्या ये जरूरी है?
    }

    sharedmemory.magic = SHARED_MAGIC; //Why set the magic num here?

    Increment_Bootloop_Count(&sharedmemory.flags);

    uint8_t Current_App_Partition = GetAppPartition();

    if(Current_App_Partition == APP1){
      // SetAppActive(APP1); Unecessary flash writing removed
      SwitchToApp(APP1);
    }
    else if (Current_App_Partition == APP2)
    {
      // SetAppActive(APP2);
      SwitchToApp(APP2);
    }
    else{ //When none of condition is satisifed /  no valid app partition is found. We set manually
      //TODO: Add switching app failure here
      if(VerifyApp(APP1) == 1){
        SetAppActive(APP1);
        SwitchToApp(APP1);
      }
      else if(VerifyApp(APP2) == 1){
        SetAppActive(APP2);
        SwitchToApp(APP2);
      }
      else{
        //No logic here :(
        // SwitchToApp(APP2);
      }
    }    

  }
}

void Increment_Bootloop_Count(volatile uint32_t* flags_ptr){
  uint32_t Bootloop_Count = ((*flags_ptr >> 24) & 0xFF);
  *flags_ptr &= 0x00FFFFFF; //Clear the flags by removing top 8 bits
  Bootloop_Count++; //no clamp for bootloop count btw here
  *flags_ptr |= (Bootloop_Count << 24); //Write back newer bootloop coun back to shared mem
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD10 PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

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
