#pragma once

void lcd_console_init();

int lcd_console_write(int fd, const void* data, int len);
