#ifndef RFDECODE_H_
#define RFDECODE_H_
/*--------------------------------------------------------------------------------------
 Function Declarations
--------------------------------------------------------------------------------------*/
//extern void Init_RF_Decode(void);

/*--------------------------------------------------------------------------------------
 Defines
--------------------------------------------------------------------------------------*/
//Reset of bICP_RF_Decode_State and any other state machine values are currently done by 
//these macros instead of normal function calls, since there is a report of a bug with calling
//functions from inside ISR's with the OS X gcc compiler with Arduino 0013(and possibly 0012)
#define RF_PULSE_PUSH_HISTORY()  { bICP_RF_PulseHistory[3] = bICP_RF_PulseHistory[2]; bICP_RF_PulseHistory[2] = bICP_RF_PulseHistory[1]; bICP_RF_PulseHistory[1] = bICP_RF_PulseHistory[0]; }
#define RF_PULSE_CLEAR_HISTORY() { bICP_RF_PulseHistory[3] = 0; bICP_RF_PulseHistory[2] = 0; bICP_RF_PulseHistory[1] = 0; bICP_RF_PulseHistory[0] = 0; }
#define RF_DECODE_RESET()       { bICP_RF_Decode_State = RF_DECODE_STATE_IDLE; bICP_RF_Decode_Counter = 0; RF_PULSE_CLEAR_HISTORY() }

//RF periods and filters
//The transmitter sends ~975uS LONG pulse, (high or low)
//                  and ~489uS SHORT pulse (high or low)
//Depending on the RC filter and receiver, the short pulse received can be shrunk down as far as
//about 270uS, and the long pulse can be stretched as far as 1220uS by the RC filter present
//on the receiver's output.
//The Jaycar ZW3102 433MHz receiver (aka Keymark RXB1, now appears to be made under "Wenshing" brand)
// used has no squelch and only a weak RC filter onboard,
// so the proto board has a discrete RC filter inserted.
// This currently gives ~1062uS LONG LOW pulse,  ~878uS LONG HIGH pulse,
//                       ~570uS SHORT LOW pulse, ~402uS SHORT HIGH pulse,
#define RF_TIMER_PERIOD_US          4                           //Timer1 resolution is 4uS
#define RF_PERIOD_FILTER_MIN        ( 340/RF_TIMER_PERIOD_US)   //min 340uS  allowed through filter
#define RF_PERIOD_FILTER_MAX        (1250/RF_TIMER_PERIOD_US)   //max 1250uS allowed through filter
#define RF_LONG_PERIOD_MIN          ( 720/RF_TIMER_PERIOD_US)   //min 720uS for a "LONG"
#define RF_SHORT_PERIOD_MAX         ( 710/RF_TIMER_PERIOD_US)   //max 710uS for a "SHORT"
//#define RF_LONG_PERIOD_MIN          ( 800/RF_TIMER_PERIOD_US)   //min 800uS for a "LONG"
//#define RF_SHORT_PERIOD_MAX         ( 650/RF_TIMER_PERIOD_US)   //max 650uS for a "SHORT"

#define RF_LONG_PREAMBLE_MINCOUNT   10  //

//bICP_RF_PulseHistory[4]   bitmasks are used so that 0x00 may be seen as no pulse in history yet
#define RF_PULSE_LONG          B00000001   //
#define RF_PULSE_SHORT         B00000010   //
#define RF_PULSE_LOW           B00000100   //
#define RF_PULSE_HIGH          B00001000   //
#define RF_PULSE_VALIDBIT      B00010000   //
#define RF_PULSE_VALIDBIT_ONE  B00110000   //
#define RF_PULSE_VALIDBIT_ZERO B01010000   //
#define RF_PULSE_CURRENT       0   //
#define RF_PULSE_LAST          1   //
#define RF_PULSE_2NDLAST       2   //
#define RF_PULSE_3RDLAST       3   //

#define RF_PACKET_BITCOUNT     66   //

//bRF_Decode_ICP_State
#define RF_DECODE_STATE_IDLE                0
#define RF_DECODE_STATE_WAITLONGPREAMBLE    1
#define RF_DECODE_STATE_WAITSTARTBITS       2
#define RF_DECODE_STATE_LOADING_BITSTREAM   3

/*
	 Delta?          |LCurrent||CCurrent||RCurrent|  |    CRC16?    |
         Ddddd   some    LLLLLLLLLLCCCCCCCCCCRRRRRRRRRR  0000000000111111
	 00123  stamp?   012345678901234567890123456789  0123456789012345
         |---|???????????|--------||--------||--------|  |--------------|
0.3  R 001000000000010100000000000000000000001010000000000010100001011010
0.4  R 000100000010100010100000000000000000000011000000000011110001011000 (PREVIOUS LOGIC ANALYSER/EXCEL DECODE, different header/stamp)
0.4  R 001000000000010100000000000000000000000011000000001110100000001111                                   and note lsb glitch on L channel
0.5  R 000001000000010100000000000000000000001001000000001001110000111011
0.6  R 001000000000010100000000000000000000000101000000001000100001010101
0.8  R 000001000000010100000000000000000000000001100000000001010011101110
0.9  R 001000000000010100000000000000000000001101100000000110110000010100
1.0  R 001000000000010100000000000000000000000111100000001010110011111110
1.2  R 001101000000010100000000000000000000000010100000001011010011110010
1.2  C 001011000000010100000000000000101000000000000000000000110011100011
1.2  L 001110000000010100001010000000000000000000000000001001110010111101
1.2  R 000101100000010100000000000000000000000010100000000100010010001010 1.2 (corrupted? extra delta bit)
                         ^         ^         ^
                        are these bits truly lsb or something else?

*/

#endif /*RFDECODE_H_*/
