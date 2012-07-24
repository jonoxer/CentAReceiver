#ifndef DEFINES_H_
#define DEFINES_H_

typedef unsigned int       uint;   //16bit
/*--------------------------------------------------------------------------------------
 Main project defines
--------------------------------------------------------------------------------------*/
#define  ASCIINUMBASE      0x30  //base value for ascii number '0'
#define  CHAR_CR           0x0D  //carriage return character
#define  CHAR_LF           0x0A  //line feed character

//keypad debounce parameter
#define DEBOUNCE_MAX        15
#define DEBOUNCE_ON         10
#define DEBOUNCE_OFF        3 
#define NUM_KEYS            5
//joystick number
#define UP_KEY              3
#define LEFT_KEY            0
#define CENTER_KEY          1
#define DOWN_KEY            2
#define RIGHT_KEY           4
//menu stuff
#define MENU_X              25
#define MENU_Y              50
#define ITEM_H              10
#define NUM_MENU_ITEM       4
//#define BACKGROUND_COLOR    BLUE   //I don't think this one is being used...
#define NORMAL_FG_COLOR     GREEN//BLUE
#define NORMAL_BK_COLOR     WHITE
#define HIGHLIGHT_FG_COLOR  CYAN
#define HIGHLIGHT_BK_COLOR  MAGENTA

//test LED flashings
#define LED_TEST_FLASH_DELAY   40

//PrintNumberToLCD bConversionType
#define CV_DIRECT5               0
#define CV_DIRECT5_NOBLANK       1
#define CV_DIRECTDP2_5           2
#define CV_DIRECT3               3
#define CV_DIRECT3_NOBLANK       4
#define CV_DIRECTDP1_3           5
#define CV_DIRECTDP1_3_BYTE      6
#define CV_ADVOLTS_8BIT          7
#define CV_DIRECT5NOBLANK        8
#define CV_TEMP                  9
#define CV_DIRECTDP1_5           10
#define CV_ZEROTO100             11
#define CV_PLUSMINUS50           12
#define CV_PLUSMINUS100          13


#endif DEFINES_H_
