
#define LED1 GPIO_FD(PORTA, 15)
#define LED2 GPIO_FD(PORTB, 3)

#define KEY1 GPIO_FD(PORTB, 4)

#define SERIAL1 UART_FD(1)
#define SERIAL2 UART_FD(3)


#define TEST_SPI1_NSS GPIO_FD(PORTA, 8)
#define TEST_SPI1  SPI_FD(2)

#define MOUNT_FD(filename, fd) if(strcmp(name, filename)==0){fcntl(fd, F_SETFL, flags); return fd;}


int System_Open(const char* name, int flags){
    MOUNT_FD("/leds/led0", LED2);
    MOUNT_FD("/leds/led1", LED1);
    MOUNT_FD("/dev/ttyS0", SERIAL2);
    return -1;
}


void System_Config(){
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio_init(LED1, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    gpio_init(LED2, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

    //gpio_init(KEY1, GPIO_MODE_IT_RISING, 0);

    uart_init(SERIAL1, UART1_DEFAULT_PINS, B115200|CS8);
    uart_init(SERIAL2, UART3_DEFAULT_PINS, B115200|CS8);


    gpio_init(TEST_SPI1_NSS, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    spi_init(TEST_SPI1, SPI2_DEFAULT_PINS);

    //stdio_set_fd(SERIAL2, SERIAL2, SERIAL2);
}


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

