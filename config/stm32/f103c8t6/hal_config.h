



void hal_config(){
    gpio_init(GPIO_FD(PORTA, PIN15), GPIO_FLAGS_OUTPUT);

    //uart_init(UART_FD(2), B115200|CS8);
}



