
#define LED0 GPIO_FD_REVERSE(PORTA, 15)
#define LED1 GPIO_FD_REVERSE(PORTB, 3)
#define LED2 GPIO_FD_REVERSE(PORTB, 4)
#define LED3 GPIO_FD_REVERSE(PORTA, 12)
#define LED4 GPIO_FD_REVERSE(PORTA, 11)

#define KEY1 GPIO_FD(PORTB, 4)

#define SERIAL1 UART_FD(1)
#define SERIAL2 UART_FD(2)


#define TEST_SPI1_NSS GPIO_FD(PORTA, 8)
#define TEST_SPI1  SPI_FD(2)

#define MOUNT_FD(filename, fd) if(strcmp(name, filename)==0)return fd


int System_Open(const char* name, int flags){
    MOUNT_FD("/leds/led0", LED0);
    MOUNT_FD("/leds/led1", LED1);
    MOUNT_FD("/leds/led2", LED2);
    MOUNT_FD("/leds/led3", LED3);
    MOUNT_FD("/leds/led4", LED4);
    return -1;
}


void System_Config(){
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio_init(LED0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    gpio_init(LED1, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    gpio_init(LED2, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    gpio_init(LED3, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    gpio_init(LED4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

    //gpio_init(KEY1, GPIO_MODE_IT_RISING, 0);

    //uart_init(SERIAL1, UART1_DEFAULT_PINS, B115200|CS8);
    //uart_init(SERIAL2, UART3_DEFAULT_PINS, B115200|CS8);


    //gpio_init(TEST_SPI1_NSS, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    //spi_init(TEST_SPI1, SPI2_DEFAULT_PINS);

    //stdio_set_fd(SERIAL2, SERIAL2, SERIAL2);
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

