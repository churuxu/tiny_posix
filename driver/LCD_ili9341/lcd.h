#ifndef __LCD_H
#define __LCD_H		
 
#include <stdlib.h>
#include <stdint.h>
#include "tiny_posix.h"
//////////////////////////////////////////////////////////////////////////////////	
 
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//LCD驱动参数
extern  uint16_t  lcd_id;         //LCD ID
extern  uint8_t   dir_flag;       //横屏还是竖屏控制：0，竖屏；1，横屏。	
extern  uint16_t  lcd_width;      //LCD 宽度
extern  uint16_t  lcd_height;     //LCD 高度
extern  uint16_t	 write_gramcmd;	 //写gram指令
extern	uint16_t  setxcmd;		     //设置x坐标指令
extern	uint16_t  setycmd;		     //设置y坐标指令	 

//LCD的画笔颜色和背景色	   
extern uint16_t  BRUSH_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色


//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD背光端口定义---------------- 
#define	LCD_BACK GPIO_FD(PORTF,10)  //LCD背光    	PF10 	    

//A12作为数据命令区分线  设置时STM32内部会右移一位对齐	    
#define  CMD_BASE     ((uint32_t)(0x6C000000 | 0x00001FFE))
#define  DATA_BASE    ((uint32_t)(0x6C000000 | 0x00002000))

#define LCD_CMD       ( * (uint16_t *) CMD_BASE )
#define LCD_DATA      ( * (uint16_t *) DATA_BASE)
	 
//扫描方向定义
#define  L2R_U2D  0 //从左到右,从上到下
#define  L2R_D2U  1 //从左到右,从下到上
#define  R2L_U2D  2 //从右到左,从上到下
#define  R2L_D2U  3 //从右到左,从下到上

#define  U2D_L2R  4 //从上到下,从左到右
#define  U2D_R2L  5 //从上到下,从右到左
#define  D2U_L2R  6 //从下到上,从左到右
#define  D2U_R2L  7 //从下到上,从右到左	 

//颜色值定义
#define  WHITE        0xFFFF
#define  BLACK        0x0000	
#define  RED          0xF800
#define  GREEN        0x07E0
#define  BLUE         0x001F 
#define  BRED         0XF81F
#define  GRED 			  0XFFE0
#define  GBLUE			  0X07FF
#define  BROWN 			  0XBC40  
#define  BRRED 			  0XFC07  
#define  GRAY  			  0X8430  
#define  MAGENTA      0xF81F
#define  CYAN         0x7FFF
#define  YELLOW       0xFFE0

	    			
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_Value);
uint16_t LCD_ReadReg(uint16_t LCD_Reg);
void LCD_WriteGRAM(void);

void LCD_Init(void);													   	//初始化
void LCD_DisplayOn(void);													//开显示
void LCD_DisplayOff(void);												//关显示
void LCD_Clear(uint16_t Color);	 											//清屏
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);						//设置光标
void LCD_DrawPoint(uint16_t x,uint16_t y);									//画点
void LCD_Color_DrawPoint(uint16_t x,uint16_t y,uint16_t color);	//颜色画点
uint16_t  LCD_GetPoint(uint16_t x,uint16_t y); 								  //读点 

void LCD_Open_Window(uint16_t X0,uint16_t Y0,uint16_t width,uint16_t height); 	
void Set_Scan_Direction(uint8_t direction);					 
void Set_Display_Mode(uint8_t mode);						 

void LCD_Fill_onecolor(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);		//填充单个颜色
void LCD_Draw_Picture(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);		//填充指定颜色
void LCD_DisplayChar(uint16_t x,uint16_t y,uint8_t word,uint8_t size);						      //显示一个字符
void LCD_DisplayString(uint16_t x,uint16_t y,uint8_t size,const char *p);		           //显示一个12/16/24字体字符串
void LCD_DisplayString_color(uint16_t x,uint16_t y,uint8_t size,const char *p,uint16_t brushcolor,uint16_t backcolor); //显示一个12/16/24字体自定义颜色的字符串
void LCD_DisplayNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode);				//显示 数字
void LCD_DisplayNum_color(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode,uint16_t brushcolor,uint16_t backcolor); //显示自定义颜色数字	  	   																			 
							  		 
#endif  
	 
	 



