#include "lcd_console.h"

#ifdef LCD_DRIVER_ID

#include "lcd.h"

#define RECT_LEFT 4
#define RECT_TOP  2
#define RECT_RIGTH 236
#define RECT_BOTTOM 316


#define CHAR_WIDTH  8
#define LINE_HEIGHT 16

#define MAX_LINE 19
#define MAX_WIDTH 28

#define BACK_COLOR WHITE
#define FONT_COLOR GREEN

#define SCROLL_STEP 15 //超过屏幕滚动多少行

//行buffer
typedef struct line_buffer{
	uint8_t count;
	char data[31];
}line_buffer;


static line_buffer lines_[MAX_LINE]; //当前屏幕buffer
static int first_line_; //当前第一行索引
static int line_count_; //当前多少行

static int x_; //当前x
static int y_; //当前y

void lcd_console_init(){
	
 	LCD_Init();
	BRUSH_COLOR = FONT_COLOR;
	x_ = RECT_LEFT;
	y_ = RECT_TOP;
}

//
static void update(int clean, int fromline, int linec){
	int i;
	line_buffer* line;	
	if(clean){		
		LCD_Clear(BACK_COLOR);
		y_ = RECT_TOP;
		fromline = first_line_;
		linec = line_count_;
	}
	i = fromline;
		
	while(linec){
		linec -- ;
		line = &lines_[i];
		if(line->count){
			LCD_DisplayString(x_,y_,16,(line->data));
		}
		
		i++;
		y_ += LINE_HEIGHT;
		if(i>=MAX_LINE)i=0;		
		if(y_>RECT_BOTTOM){
			y_ = RECT_TOP;
			break;
		}
	}	
}


int inc_line(){
	int scroll = 0;
	line_count_ ++;
	if(line_count_ > MAX_LINE){ //超过屏幕，移除n行
		line_count_ -= SCROLL_STEP;	
		first_line_ += SCROLL_STEP;	
		if(first_line_ >= MAX_LINE){
			first_line_ -= MAX_LINE;			
		}
		scroll = 1;				
	}	
	return scroll;	
}

int lcd_console_write(int fd, const void* data, int len){
	char* ptr = (char*)data;
	int i;
	char ch;
	int scroll = 0;	
	int line_index;
	int fromline;
	int linec = 0;

	line_index = first_line_ + line_count_;
	if(line_index >= MAX_LINE)line_index-= MAX_LINE;
	fromline = line_index;

	for(i=0;i<len;i++){
		ch = *ptr;
		ptr++;
		if(ch == '\r'){
			continue;
		}
		if(ch == '\n' || lines_[line_index].count >= MAX_WIDTH){
			lines_[line_index].data[ lines_[line_index].count ] = 0;
			scroll += inc_line();
			linec ++;
			line_index ++;
			if(line_index >= MAX_LINE)line_index = 0;
			lines_[line_index].count = 0;			
		}
		if(ch != '\n'){
			lines_[line_index].data[ lines_[line_index].count ] = ch;
			lines_[line_index].count ++;
		}
	}
	update(scroll, fromline, linec);
	
	return len;
}

#endif
