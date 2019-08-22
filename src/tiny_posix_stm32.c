
#include "tiny_posix_stm32.h"
#include "tiny_posix_driver.h"

#ifndef GPIO_SPEED_FREQ_HIGH
#define GPIO_SPEED_FREQ_HIGH GPIO_SPEED_HIGH
#endif

GPIO_TypeDef* gpio_ports_[] = {
    GPIOA,
    GPIOB,
    GPIOC
};

static void gpio_clock_init(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();   
    __HAL_RCC_GPIOC_CLK_ENABLE(); 
}

static int SystemClock_Config(){
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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
    return -1;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    return -1;
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    return -1;
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  return 0;
}


int tiny_posix_init(){
    int ret = 0;
    HAL_Init();
    ret += SystemClock_Config();
    gpio_clock_init();
    return ret;
}


int gpio_init(int fd, int flags){
    int pull, mod;
    int flagm, flagp;
    flagm = ((flags >> 8) & 0xff);
    flagp = ((flags ) & 0xff);

    mod = GPIO_MODE_OUTPUT_PP;
    if(flagm == GPIO_FLAGS_INPUT){
        mod = GPIO_MODE_INPUT;
    }    
    pull = GPIO_NOPULL;
    if(flagp == GPIO_FLAGS_PULL_UP){
        pull = GPIO_PULLUP;
    }else if(flagp == GPIO_FLAGS_PULL_DOWN){
        pull = GPIO_PULLDOWN;
    } 

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_FD_GET_PIN(fd);
    GPIO_InitStruct.Mode = mod;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_FD_GET_PORT(fd), &GPIO_InitStruct);   

    return 0; 
}
/*
int  gpio_status(int fd){
    return HAL_GPIO_ReadPin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd));
}
void gpio_set(int fd){
    HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), !GPIO_FD_IS_REVERSE(fd));
}
void gpio_reset(int fd){
    HAL_GPIO_WritePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd), GPIO_FD_IS_REVERSE(fd));
}
void gpio_toggle(int fd){
    HAL_GPIO_TogglePin(GPIO_FD_GET_PORT(fd), GPIO_FD_GET_PIN(fd));
}
*/

unsigned int _tp_sleep(unsigned int seconds){
    HAL_Delay(seconds * 1000);
    return 0;
}
unsigned int _tp_usleep(unsigned int micro_seconds){
    HAL_Delay(micro_seconds / 1000);
    return 0;
}




void SysTick_Handler(void){ 
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler(); 
}

