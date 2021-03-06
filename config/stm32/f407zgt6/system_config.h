#pragma once

#include "lcd_console.h"

#define LED0 GPIO_FD_REVERSE(PORTE, 3)
#define LED1 GPIO_FD_REVERSE(PORTE, 4)
#define LED2 GPIO_FD_REVERSE(PORTG, 9)

#define KEY1 GPIO_FD_REVERSE(PORTF, 9)

#define SERIAL1 UART_FD(3)
#define SERIAL2 UART_FD(5)

#define MOUNT_FD(filename, fd) if(strcmp(name, filename)==0){fcntl(fd, F_SETFL, flags); return fd;}

int System_Open(const char* name, int flags){
    MOUNT_FD("/leds/led0", LED0);
    MOUNT_FD("/leds/led1", LED1);
    MOUNT_FD("/leds/led2", LED2);
    MOUNT_FD("/keys/key0", KEY1);
    MOUNT_FD("/dev/ttyS0", SERIAL1);
    return -1;
}

extern LCD_DrvTypeDef* lcd_drv;

void System_Config(){
    int fsmcpins[] = {
        GPIO_FD(PORTF, 10),
        GPIO_MULTI_FD(PORTD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15),
        GPIO_MULTI_FD(PORTE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15),
        GPIO_MULTI_FD(PORTG, GPIO_PIN_2|GPIO_PIN_12),        
    };

    gpio_init(LED0, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
    gpio_init(LED1, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
    gpio_init(LED2, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    gpio_init(KEY1, GPIO_MODE_IT_RISING_FALLING, 0);

    uart_init(SERIAL1, UART3_DEFAULT_PINS, B9600|CS8);
    uart_init(SERIAL2, UART5_DEFAULT_PINS, B9600|CS8);
  

    fsmc_init(fsmcpins, 4, FSMC_NORSRAM_BANK4);
    
    lcd_console_init();

    //stdio_set_fd(SERIAL2,SERIAL2,SERIAL2);
    stdio_set_func(NULL,lcd_console_write,lcd_console_write);
}


void SystemClock_Config()
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    //Error_Handler();
  }
}

