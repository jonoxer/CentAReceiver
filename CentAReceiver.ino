/***************************************************************************************
 
  Name:           RF_Meter_Receiver_LCD Test project
  Module:
  Revision Info:  0.01
  Module Info:
  Written By:     Marc Alexander
  Date:           Dec 2008 - Feb 2009

     Receives and decodes a pulse-width and transition encoded RF bitstream,
     received through a 433MHz receiver module into the PB0 ICP Input Capture pin.
     
     The transmitter is from the Clipsal "cent-a-meter"
     433.855MHz, Model CMR113A, Whole package part number is CM113A
     
  Update and Change Log:
  
  -------------------------------------------------------------------------------------
  0.01
  ----------------------------------------------------------------------------------------  
   1. A work in progress at the moment, 
   
      - Need to fix the timestamp, it's not incrementing properly there
      
      - Some unexpected packet loss, have to put back in existing and new interrupt decoder
        bitstream codes out of serial port to discover why and where it is happening.
        
      - First debug tests didn't show any error exit points tripped in the decoder,
        the data stream out of the serial port just drops out or stops early and the next line is usually ok.
      - Is it just comms dropouts out of the serial port > ftdi > usb > pc chain?
      - Is it the Arduino architecture causing occasional interrupt dropouts? Need to
        check what the architecture interrupt entry and exit handlers do, to make sure
        that it's not clearing flags on entry and allowing interrupts to next unexpectedly.
   
 
****************************************************************************************/
/*--------------------------------------------------------------------------------------
  Includes
--------------------------------------------------------------------------------------*/
#include "defines.h"
#include "macro.h"
#include "rf_decode.h"
#include <avr/pgmspace.h>

/*--------------------------------------------------------------------------------------
  Defines
--------------------------------------------------------------------------------------*/
//in defines.h
#define   REMOVE_UPDATE_ADC_KEY_INTERRUPT
//#define   BITSTREAM_SERIAL_BLIT_FROM_INT

/*--------------------------------------------------------------------------------------
  Variables
--------------------------------------------------------------------------------------*/
//RF_Decode module variables
uint uiICP_EventTime;
uint uiICP_PreviousEventTime;
uint uiICP_EventPeriod;
uint uiICP_PreviousEventPeriod;
byte bICP_EventPolarity;
byte bICP_PreviousEventPolarity;
byte bICP_RF_Decode_State;
byte bICP_RF_Decode_Counter;
byte bICP_RF_Decode_FilterCounter;
byte bICP_RF_PulseHistory[4];
byte bICP_LastPacketBit;
unsigned long ulICP_RF_Timestamp_262_144mS;   //
byte bICP_RF_PacketData[4][4+8];              //incoming RF packet data with 4 byte timestamp at start, already bit reversed to suit.
                                              //main array size must be ^2, some other count dependencies in decoder.
byte bICP_RF_PacketInPointer;                 //
byte bICP_RF_PacketOutPointer;                //
byte bICP_RF_PacketInBitMask;                 //
byte bICP_RF_PacketInBytePointer;             //
uint uiICP_RF_PacketInCount;                  //count of how many good packets received
//converted values that come out of RF_Decode_ConvertPacketToValues
unsigned long ulRF_Meter_Timestamp_262_144mS;   //
byte bRF_Meter_Delta;
byte bRF_Meter_AddressBroadcastIsOn;
byte bRF_Meter_Address;
uint uiRF_Meter_LeftCurrent;                  //current of left channel,   fixed single decimal place, eg. 123 = 12.3 amps
uint uiRF_Meter_CentreCurrent;                //current of centre channel, fixed single decimal place, eg. 123 = 12.3 amps
uint uiRF_Meter_RightCurrent;                 //current of right channel,  fixed single decimal place, eg. 123 = 12.3 amps
uint uiRF_Meter_Checksum;
uint uiRF_Meter_TotalCurrent;                 //total current summed from all channels,  fixed single decimal place, eg. 345 = 34.5 amps
uint uiRF_Meter_LeftPower;
uint uiRF_Meter_CentrePower;
uint uiRF_Meter_RightPower;

#define LocalVoltage 240

/*--------------------------------------------------------------------------------------
  Function Declarations
--------------------------------------------------------------------------------------*/
void Init_RF_Decode(void);

/*--------------------------------------------------------------------------------------
  InitPort()
  Initialise Ports initial state and data direction registers
--------------------------------------------------------------------------------------*/
void InitPort()
{
   DDRB = 0x2F;
}


/*--------------------------------------------------------------------------------------
  setup
  Called by arduino architecture before main loop begins
--------------------------------------------------------------------------------------*/
void setup(void)
{
  Serial.begin(38400);
  Serial.println( "Cent-a-Meter receiver starting up" );
  
  //Init the rf_decode module
  //Note: This uses PORTB0 for the Input capture input, sets up and uses Timer/Counter1 registers, 
  //      and also uses PORTD6 and PORTD7 for test LEDs in testing
  Init_RF_Decode();
   
  //Interrupts enabled
  interrupts();   //using this to enable interrupts instead of the above, same result but it follows the Arduino guidelines

}

/*--------------------------------------------------------------------------------------
  loop
  Arduino architecture main loop
--------------------------------------------------------------------------------------*/
void loop(void)
{
  RF_Decode_TestDisplay();
}



/*--------------------------------------------------------------------------------------
   RF_Decode_TestDisplayToLCD   
--------------------------------------------------------------------------------------*/
void RF_Decode_TestDisplay(void)
{   
   //debug display of input capture period and polarity
   char strString[5+1];

   //----------------------------
   //arrow pointing to input pointer, previous line will be the valid data
   switch( bICP_RF_PacketInPointer )
   {
      case 0:
      {
         RF_Decode_PrintDataLine( 10, 60 );
         break;
      }
      case 1:
      {
         RF_Decode_PrintDataLine( 10, 30 );
         break;
      }
      case 2:
      {
         RF_Decode_PrintDataLine( 10, 40 );
         break;
      }
      case 3:
      {
         RF_Decode_PrintDataLine( 10, 50 );
         break;
      }
   }
   
   //----------------------------
   //incoming packet count
   ValueTo5DigitString( (char*)&strString, uiICP_RF_PacketInCount );
//   lcd.cLCD_String( (char*)&strString, 10, 120,  WHITE, BLACK );
   
   //----------------------------
   //current period
   ValueTo5DigitString( (char*)&strString, uiICP_EventPeriod );

   //----------------------------
   //previous period
   ValueTo5DigitString( (char*)&strString, uiICP_PreviousEventPeriod );

}



/*--------------------------------------------------------------------------------------
 ValueTo5DigitString
--------------------------------------------------------------------------------------*/
void ValueTo5DigitString( char* pString, uint uiValue )
{
   volatile byte bResult;

   bResult = (byte)( uiValue / 10000 );            //10,000's
   pString[0] = ((byte)(bResult + ASCIINUMBASE));
   bResult = (byte)( (uiValue % 10000) / 1000 );   //1,000's
   pString[1] = ((byte)(bResult + ASCIINUMBASE));

   bResult = (byte)( (uiValue % 1000) / 100 );     //100's
   pString[2] = ((byte)(bResult + ASCIINUMBASE));
   bResult = (byte)( (uiValue % 100) / 10 );       //10's
   pString[3] = ((byte)(bResult + ASCIINUMBASE));
   bResult = (byte)( (uiValue % 10 ) );            //1's
   pString[4] = ((byte)(bResult + ASCIINUMBASE));
   
   pString[5] = 0x00;                               //ensure null on the end of the string
}



/*--------------------------------------------------------------------------------------
  rf_decode

  Notes:
 
  ICP1 (Input Capture 1) on PB0 (PORTB0) is used to capture any incoming RF
    bitstreams and decode them appropriately.
  The ATmega168 has only one dedicated Input Capture pin, used here, and it belongs
    to the "16-bit Timer/Counter1 with PWM" module.
  TCNT1 is the main 16 bit free running counter, and the register ICR1 holds the latest
    captured event time.
    
  Measurement of an external signal duty cycle (high and low period durations) requires
    that the trigger edge be changed after each capture (usually at the start of the IC ISR)
 
    Freeduino board crystal is 16.000MHz   (62.5nS system clock period):
  //Clock Select Bit Description (for TCCR1B)
  //CS12   CS11  CS10  Description
  //0      0     0     No clock source (Timer/Counter stopped)
  //0      0     1     clkI/O /1    (No prescaling)     62.5nS period   4.096 mS TOF
  //0      1     0     clkI/O /8    (From prescaler)    500nS period   32.768 mS TOF
  //0      1     1     clkI/O /64   (From prescaler)      4uS period  262.144 mS TOF <== selected
  //1      0     0     clkI/O /256  (From prescaler)     16uS period  1.048576 S TOF
  //1      0     1     clkI/O /1024 (From prescaler)     64uS period  4.194304 S TOF
  //1      1     0     External clock source on T0 pin. Clock on falling edge.
  //1      1     1     External clock source on T0 pin. Clock on rising edge.   TCCR0B = (CS02);
 
  TCNT1    
  TCCR1B   ICNC1 ICES1    ???       WGM13   WGM12 CS12 CS11 CS10
           1     0(falling edge)  0       0     0    1    1 
                 1(rising edge)
                 
  http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234619741/4
  "It's reasonably well documented there's a bug with the OS X gcc compiler with Arduino 0013 
   (and 0012?) in regard to ISR routine code generation."
--------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------
   Init_RF_Decode   
--------------------------------------------------------------------------------------*/
void Init_RF_Decode(void)
{
   //Call macro that resets any RF_Decode state machine and housekeeping values
   RF_DECODE_RESET();
   //---------------------------------------------------------------------------------------------
   //RF decode ports setup
   //Marc making PB0 (ICP1 Input Capture) a floating input for RX ASK bitstream receiving
   //PB0 was used by the Color LCD/Joystick Shield for the backlight_on signal,
   DDRB  &= ~(1<<DDB0);    //PBO(ICP1) input
   PORTB &= ~(1<<PORTB0);  //ensure pullup resistor is also disabled
   
   //PORTD6 and PORTD7, GREEN and RED test LED setup
   DDRD  |=  B11000000;      //(1<<PORTD6);   //DDRD  |=  (1<<PORTD7); (example of B prefix)
   GREEN_TESTLED_OFF();      //GREEN test led off
   RED_TESTLED_ON();         //RED test led on
   //PORTD |=  _BV(PORTD6);    //GREEN led off  (example of _BV macro)
   //PORTD &= ~_BV(PORTD7);    //RED led on     (example of _BV macro)
   //PORTD |=  (1<<PORTD6);    //GREEN led off  (example of AVR studio style)
   //PORTD &= ~(1<<PORTD7);    //RED led on     (example of AVR studio style)
   
   //---------------------------------------------------------------------------------------------
   //ICNC1: Input Capture Noise Canceler         On, 4 successive equal ICP1 samples required for trigger (4*4uS = 16uS delayed)
   //ICES1: Input Capture Edge Select            1 = rising edge to begin with, input capture will change as required
   //CS12,CS11,CS10   TCNT1 Prescaler set to 0,1,1 see table and notes above
   TCCR1A = B00000000;   //Normal mode of operation, TOP = 0xFFFF, TOV1 Flag Set on MAX
                         //This is supposed to come out of reset as 0x00, but something changed it, I had to zero it again here to make the TOP truly 0xFFFF
   TCCR1B = ( _BV(ICNC1) | _BV(CS11) | _BV(CS10) );
   SET_INPUT_CAPTURE_RISING_EDGE();
   //Timer1 Input Capture Interrupt Enable, Overflow Interrupt Enable  
   TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );

}



/*--------------------------------------------------------------------------------------
   RF_Decode_PrintDataLine   
--------------------------------------------------------------------------------------*/
void RF_Decode_PrintDataLine( byte bX, byte bY )
{
   byte b = 0;
   
   //write out the test data line to the serial port
   if( bICP_RF_PacketInPointer != bICP_RF_PacketOutPointer )
   {
      //---------Serial output --------------------------------------------------------
      Serial.write( CHAR_CR );
      Serial.write( CHAR_LF );
      //Serial.print( "time:" );
      //we have valid data at the out pointer, send out the 4 byte timestamp and 8 byte packet

      //send out real decoded decimal data now
      RF_Decode_ConvertPacketToValues( bICP_RF_PacketOutPointer );
      Serial.print( "PWR1=" );
      uiRF_Meter_LeftPower = uiRF_Meter_LeftCurrent * LocalVoltage / 10;
      Serial.println( uiRF_Meter_LeftPower, DEC );
      //Serial.print( " " );
      Serial.print( "PWR2=" );
      uiRF_Meter_CentrePower = uiRF_Meter_CentreCurrent * LocalVoltage / 10;
      Serial.println( uiRF_Meter_CentrePower, DEC );
      //Serial.print( " " );
      Serial.print( "PWR3=" );
      uiRF_Meter_RightPower = uiRF_Meter_RightCurrent * LocalVoltage / 10;
      Serial.println( uiRF_Meter_RightPower, DEC );

      //----------------------------------------------------------------------------
      //data output done, move the out pointer along
      bICP_RF_PacketOutPointer = ((bICP_RF_PacketOutPointer+1)&0x03);   ///only the lower two bits are used for the 4 entry buffer array      
   }
}

/*--------------------------------------------------------------------------------------
   RF_Decode_ConvertPacketToValues   
   
0.3A left, centre, then right
	                               Delta?   Address |LCurrent|CCurrent|RCurrent|        | CRC14/checksum? |
        c = CHECK(address broadcast)   DDDDDDc? aaaaaaaa LLLLLLLL LLCCCCCC CCCCRRRR RRRRRR?? 00000000 00111111
	                               012345c? 01234567 01234567 8901234567890123456789     01234567 89012345
                                       ---4----|---5----|---6----|---7----|---8----|---9----|---10---|---11--- 
                                       01234567 01234567 01234567 01234567 01234567 01234567 01234567 01234567 
                                       -------- -------- -------- -------- -------- -------- -------- -------- 
time:000000D5 hex:00B80300000016AE bin:00000000 00011101 11000000 00000000 00000000 00000000 01101000 01110101 
time:000000B9 hex:00B80300000016AE bin:00000000 00011101 11000000 00000000 00000000 00000000 01101000 01110101 
time:0000002C hex:05B8000C0000242D bin:10100000 00011101 00000000 00110000 00000000 00000000 00100100 10110100 
time:0000002D hex:05B8000C0000242D bin:10100000 00011101 00000000 00110000 00000000 00000000 00100100 10110100 
time:00000043 hex:04B8000030001A4E bin:00100000 00011101 00000000 00000000 00001100 00000000 01011000 01110010 
time:00000043 hex:04B8000030001A4E bin:00100000 00011101 00000000 00000000 00001100 00000000 01011000 01110010 
no current, part of CHECK burst (address broadcast), see packet 4 bit 6 is set during broadcast
time:00000065 hex:4FB8000000002639 bin:11110010 00011101 00000000 00000000 00000000 00000000 01100100 10011100 
time:00000075 hex:4DB8000000002478 bin:10110010 00011101 00000000 00000000 00000000 00000000 00100100 00011110

--------------------------------------------------------------------------------------*/
void RF_Decode_ConvertPacketToValues( byte bPacketPointer )
{
   volatile uint ui;      //used for temporary manipulations here to ensure any async/interrupt reads of these values are not corrupted by partial calcs
   
   //arduino created 32 bit timestamp is bytes [0] to [3]
   ulRF_Meter_Timestamp_262_144mS = (((unsigned long)(bICP_RF_PacketData[bPacketPointer][0]) << 24))
                                   +(((unsigned long)(bICP_RF_PacketData[bPacketPointer][1]) << 16))
                                   +(((unsigned long)(bICP_RF_PacketData[bPacketPointer][2]) <<  8))
                                   + ((unsigned long)(bICP_RF_PacketData[bPacketPointer][3]));                                  
   //"delta"/check sequence number is bottom 6 bits of [4]
   bRF_Meter_Delta                 = (bICP_RF_PacketData[bPacketPointer][4] & B00111111);
   //CHECK flag(address broadcast) is bit 6 of [4]
   bRF_Meter_AddressBroadcastIsOn  = ((bICP_RF_PacketData[bPacketPointer][4] >> 6) & B00000001 );
   //address is all 8 bits of [5]
   bRF_Meter_Address              = bICP_RF_PacketData[bPacketPointer][5];
   //left channel current is all of [6] plus bottom 2 bits of [7]
   ui  =  (uint)(bICP_RF_PacketData[bPacketPointer][6]);
   ui += ((uint)(bICP_RF_PacketData[bPacketPointer][7]) << 10) & 0x0300;
   uiRF_Meter_LeftCurrent          = ui;
   //centre channel current is top 6 bits of [7] plus bottom 4 bits of [8]
   ui  = ((uint)(bICP_RF_PacketData[bPacketPointer][7]) >> 2);
   ui += ((uint)(bICP_RF_PacketData[bPacketPointer][8]) << 6) & 0x03C0;  
   uiRF_Meter_CentreCurrent        = ui;   
   //right channel current is top 4 bits of [8] plus bottom 6 bits of [9]
   ui  = ((uint)(bICP_RF_PacketData[bPacketPointer][8]) >> 4);
   ui += ((uint)(bICP_RF_PacketData[bPacketPointer][9]) << 4) & 0x03F0;     
   uiRF_Meter_RightCurrent         = ui;
   //checksum is bytes [10] and [11]
   ui  = ((uint)(bICP_RF_PacketData[bPacketPointer][10]) << 8 );
   ui +=  (uint)(bICP_RF_PacketData[bPacketPointer][11]);
   uiRF_Meter_Checksum             = ui;

   //sum all the currents into one
   ui = uiRF_Meter_LeftCurrent;
   if( (ui + uiRF_Meter_CentreCurrent) >= ui )
      ui += uiRF_Meter_CentreCurrent;
   else
      ui = 65535;
   if( (ui + uiRF_Meter_RightCurrent) >= ui )
      ui += uiRF_Meter_RightCurrent;
   else
      ui = 65535;     
   uiRF_Meter_TotalCurrent = ui;

}


/*--------------------------------------------------------------------------------------
   TIMER1_OVF_vect
   Timer1 overflow interrupt routine
   262.144 mS TOF period
   If used to feed a 32 bit timestamp counter, (0xFFFFFFFF = 4294967295 count before overlow)
   = 1125899906 seconds = 18764998 minutes = 312749 = 13031 days = 35 years.
--------------------------------------------------------------------------------------*/
ISR( TIMER1_OVF_vect ) 
{  
   //nothing needs to happen here, can be disabled later unless it's used for delayed filtering or state machine clearing
   //actually, should use this for period overflow filtering, periods forced clear/invalid if less that 3 come in per 2 TOFs?
   
   //increment the 32 bit timestamp counter (see overflow notes above)
   //overflow is allowed as this timestamp is most likely to be used as a delta from the previous timestamp,
   //so if it's used externally in the same 32 bit unsigned type it will come out ok.
   ulICP_RF_Timestamp_262_144mS++;
   
   //toggle RED(PD7) and GREEN(PD6) test leds for testing
   //GREEN_TESTLED_TOGGLE();
   //RED_TESTLED_TOGGLE();
}
/*--------------------------------------------------------------------------------------
   TIMER1_CAPT_vect
   Timer1 input capture interrupt routine
--------------------------------------------------------------------------------------*/
ISR( TIMER1_CAPT_vect ) 
{  
   byte bProcessValidBitToBuffer = false;
   uint uiCapturedEventPeriod;

//GREEN test led on (flicker for debug)
GREEN_TESTLED_ON();      

   //immediately grab the current capture time in case it triggers again and overwrites ICR1 with an unexpected new value
   uiICP_EventTime = ICR1;
   //immediately grab the current capture polarity and reverse it to catch all the subsequent high and low periods coming in
   //If the initial period filter passes below, this will be inspected to become bICP_EventPolarity
   if( INPUT_CAPTURE_IS_RISING_EDGE() )
      SET_INPUT_CAPTURE_FALLING_EDGE();   //previous period was low and transitioned high
   else
      SET_INPUT_CAPTURE_RISING_EDGE();    //previous period was high and transitioned low
   
   //calculate the current period just measured, to accompany the polarity now stored in bInputCaptureEventPolarity
   uiCapturedEventPeriod = (uiICP_EventTime - uiICP_PreviousEventTime);
   //discard the captured period if it is out of the expected range, it is noise...
   if( (uiCapturedEventPeriod >= RF_PERIOD_FILTER_MIN) && (uiCapturedEventPeriod <= RF_PERIOD_FILTER_MAX) )
   {
      //----------------------------------------------------------------------------
      //PERIOD INITIAL DURATION FILTER OK, CONTINUE
      //----------------------------------------------------------------------------
//NOTE there is currently no quiet time reset being performed here, at the moment the constant
//     RF junk coming in from the unsquelched receiver fails validity testing and resets the state machine constantly.
//     Will need a timer/TOF based reset here later if a nice quiet squelched receiver is used...
      //passed initial period duration filter, allow the filtered period and polarity to come through to the state machine
      uiICP_EventPeriod = uiCapturedEventPeriod;
      if( INPUT_CAPTURE_IS_RISING_EDGE() )
         bICP_EventPolarity = true;    //measured period in uiICP_EventPeriod was high
      else
         bICP_EventPolarity = false;   //previous period in uiICP_EventPeriod was low      
      //----------------------------------------------------------------------------
      //Load the bICP_RF_PulseHistory[4] up with this pulse information, except for RF_PULSE_VALIDBIT which will come
      //later in the bitstream decoding as it is required
      RF_PULSE_PUSH_HISTORY();   //push the period history data back before writing the latest one to bICP_RF_PulseHistory[RF_PULSE_CURRENT]
      if( uiICP_EventPeriod > RF_LONG_PERIOD_MIN )
      {
         if( bICP_EventPolarity )
            bICP_RF_PulseHistory[RF_PULSE_CURRENT] = ( RF_PULSE_LONG | RF_PULSE_HIGH );
         else
            bICP_RF_PulseHistory[RF_PULSE_CURRENT] = ( RF_PULSE_LONG | RF_PULSE_LOW );
      }
      else if( uiICP_EventPeriod < RF_SHORT_PERIOD_MAX )
      {
         if( bICP_EventPolarity )
            bICP_RF_PulseHistory[RF_PULSE_CURRENT] = ( RF_PULSE_SHORT | RF_PULSE_HIGH );
         else
            bICP_RF_PulseHistory[RF_PULSE_CURRENT] = ( RF_PULSE_SHORT | RF_PULSE_LOW );
      }
      else
      {
         //inside the hysteresis band between high and low, discard
#ifdef BITSTREAM_SERIAL_BLIT_FROM_INT
//debug - show packet decode exit from invalid period coming in midstream
if( bICP_RF_Decode_State == RF_DECODE_STATE_LOADING_BITSTREAM )
{
Serial.print( "x" );
Serial.print( uiICP_EventPeriod, DEC );
}
#endif
         RF_DECODE_RESET();
      }
      //----------------------------------------------------------------------------
      //RF_Decode state machine to detect, analyse and decode the incoming RF bitstream
      //load the bICP_RF_PulseHistory[4] up with this pulse information, except for RF_PULSE_VALIDBIT which will come
      //later in the bitstream decoding as required
      switch( bICP_RF_Decode_State )
      {
         //--------------------------------------------
         case RF_DECODE_STATE_IDLE:
         {
            //the preamble starts with 11 pairs of LONG-ON/LONG-OFF combinations
            if( (bICP_RF_PulseHistory[RF_PULSE_CURRENT] & RF_PULSE_LONG) )
            {
               //got the first filtered LONG period, start counting them
               bICP_RF_Decode_Counter++;
               bICP_RF_Decode_State = RF_DECODE_STATE_WAITLONGPREAMBLE;
            }
            break;
         }
         //--------------------------------------------
         case RF_DECODE_STATE_WAITLONGPREAMBLE:
         {
            //a minimum of RF_LONG_PERIOD_MIN LONGs (high or low, /2 pairs of them) will allow movement past this state
            if( (bICP_RF_PulseHistory[RF_PULSE_CURRENT] & RF_PULSE_LONG) )
            {
               //got another valid long, count it
               if( bICP_RF_Decode_Counter != 255 )
                  bICP_RF_Decode_Counter++;
               if( bICP_RF_Decode_Counter > RF_LONG_PREAMBLE_MINCOUNT )
               {
                  //passed the count of long preambles, move along to wait for the 6 LSS "1" start bits
                  bICP_RF_Decode_Counter       = 0;
                  bICP_RF_Decode_FilterCounter = 0;
                  bICP_RF_Decode_State         = RF_DECODE_STATE_WAITSTARTBITS;
               }
            }
            else
            {
               //got something other than a long in the preamble test, reset state machine 
               RF_DECODE_RESET();
            }
            break;
         }
         //--------------------------------------------
         case RF_DECODE_STATE_WAITSTARTBITS:
         {
            //LONG,SHORT,SHORT is a '1' bit, LONG,LONG is a '0' bit
            //line up with the six consecutive combinations of LONG,SHORT,SHORT after passing the preamble test above
            if(  ( bICP_RF_PulseHistory[RF_PULSE_2NDLAST] & RF_PULSE_LONG  )
               &&( bICP_RF_PulseHistory[RF_PULSE_LAST]    & RF_PULSE_SHORT )
               &&( bICP_RF_PulseHistory[RF_PULSE_CURRENT] & RF_PULSE_SHORT )
              )
            {
               //there should be six LONG,SHORT,SHORT in a row or this is not a valid packet start
               bICP_RF_PulseHistory[RF_PULSE_CURRENT] |= RF_PULSE_VALIDBIT_ONE;
               bICP_RF_Decode_Counter++;
               bICP_RF_Decode_FilterCounter = 0;
               if( bICP_RF_Decode_Counter >= 6 )
               {
                  //we are now lined up at the end of the last start bit, move on to bistream loading,
                  //there will be 66 bits in the packet after this point,
                  //two fixed '0' bits after this (LONG_HIGH,LONG_LOW then another LONG_HIGH,LONG_LOW),
                  //then 64 more bits to follow.
                  bICP_RF_Decode_Counter       = 0;   //used in the next state to count incoming bits
                  bICP_RF_Decode_FilterCounter = 0;   //used in the next state to filter corrupted packet bits and reset
                  bICP_LastPacketBit           = 1;   //first bit is always a '1', toggles are done from this original state
                  //load the timestamp to the start of the incoming packet buffer. Interrupts are not setup to nest so this value will not change here
                  bICP_RF_PacketData[bICP_RF_PacketInPointer][0] = byte(ulICP_RF_Timestamp_262_144mS >> 24);
                  bICP_RF_PacketData[bICP_RF_PacketInPointer][1] = byte(ulICP_RF_Timestamp_262_144mS >> 16);
                  bICP_RF_PacketData[bICP_RF_PacketInPointer][2] = byte(ulICP_RF_Timestamp_262_144mS >>  8);
                  bICP_RF_PacketData[bICP_RF_PacketInPointer][3] = byte(ulICP_RF_Timestamp_262_144mS);
                  bICP_RF_PacketInBytePointer               = 4;   //start past the prewritten 32 bit timestamp
                  bICP_RF_PacketInBitMask      = 0x01;
                  bICP_RF_Decode_State         = RF_DECODE_STATE_LOADING_BITSTREAM;
//change this setup later and the decoder below to deal with only 64 bits, throw the first two away
#ifdef BITSTREAM_SERIAL_BLIT_FROM_INT
Serial.print( " ." );
Serial.print( CHAR_CR, BYTE );
Serial.print( CHAR_LF, BYTE );
#endif
RED_TESTLED_TOGGLE();
               }
            }
            else
            {
               //not a '1' bit (yet), wait up to 16 periods to get the first one and only 3 periods to get subsequent ones
               if( bICP_RF_Decode_Counter == 0 )
               {
                  //waiting for first '1' bit
                  bICP_RF_Decode_FilterCounter++;
                  if( bICP_RF_Decode_FilterCounter > 16 )
                     RF_DECODE_RESET();
               }
               else
               {
                  //subsequent '1' bits
                  bICP_RF_Decode_FilterCounter++;
                  if( bICP_RF_Decode_FilterCounter > 3 )
                     RF_DECODE_RESET();
               }
            }
            break;
         }
         //--------------------------------------------
         case RF_DECODE_STATE_LOADING_BITSTREAM:
         {
            //start of the 66 bit packet reception
            //using bICP_RF_Decode_Counter to count the incoming bits,
            //and bICP_RF_Decode_FilterCounter to filter out any unexpected pattern corruption
            if(  ( bICP_RF_PulseHistory[RF_PULSE_2NDLAST] & RF_PULSE_LONG  )
               &&( bICP_RF_PulseHistory[RF_PULSE_LAST]    & RF_PULSE_SHORT )
               &&( bICP_RF_PulseHistory[RF_PULSE_CURRENT] & RF_PULSE_SHORT )
              )
            {
               //-------------------
               //got a valid '1' bit, a LONG,SHORT,SHORT
               bICP_RF_PulseHistory[RF_PULSE_CURRENT] |= RF_PULSE_VALIDBIT_ONE;
               //set for bitstream processor below to load in this new bit
               bICP_RF_Decode_FilterCounter = 0;
               bProcessValidBitToBuffer = true;
            }
            else if(  (   bICP_RF_PulseHistory[RF_PULSE_LAST]    & RF_PULSE_LONG  )
                    &&(   bICP_RF_PulseHistory[RF_PULSE_CURRENT] & RF_PULSE_LONG )
                    &&( ( bICP_RF_PulseHistory[RF_PULSE_LAST] & RF_PULSE_VALIDBIT ) == 0)
                   )
            {
               //-------------------
               //got a valid '0' bit, A LONG,LONG where the last period did not amount to a valid bit
               bICP_RF_PulseHistory[RF_PULSE_CURRENT] |= RF_PULSE_VALIDBIT_ZERO;
               //set for bitstream processor below to load in this new bit
               bICP_RF_Decode_FilterCounter = 0;
               bProcessValidBitToBuffer = true;
            }
            else
            {
               //waiting in between periods, cancel reception and the state machine here if a valid 0 or 1 bit combination
               //doesn't come in within n periods
               bICP_RF_Decode_FilterCounter++;
               if( bICP_RF_Decode_FilterCounter >= 4 )
               {
                  //got too many transitions without a valid bit, reset the state machine, cancel this reception and exit
                  RF_DECODE_RESET();
#ifdef BITSTREAM_SERIAL_BLIT_FROM_INT
Serial.print( "n" );
#endif
               }
            }
            //if a valid bit is loaded from the bitstream above, we end up here to load it into the incoming packet buffer
            if( bProcessValidBitToBuffer )
            {
               //count this bit, bICP_RF_Decode_Counter started at zero
               bICP_RF_Decode_Counter++;
               //note special throw away of the first 2 fixed '1' bits, to keep this down to an 8 byte data packet
               if( (bICP_RF_Decode_Counter != 1) && (bICP_RF_Decode_Counter != 2) )
               {
                  if( (bICP_RF_PulseHistory[RF_PULSE_CURRENT] & RF_PULSE_VALIDBIT_ONE) == RF_PULSE_VALIDBIT_ONE ) 
                  {
                     //got a 1, toggle the true packet bit
                     if( bICP_LastPacketBit )
                        bICP_LastPacketBit = 0;
                     else
                        bICP_LastPacketBit = 1;
                  }
                  else
                  {
                     //got a zero, no toggle of the bit
                  }
                  //write this true final bit data into the incoming packet buffer
                  if( bICP_LastPacketBit )
                     bICP_RF_PacketData[bICP_RF_PacketInPointer][bICP_RF_PacketInBytePointer] |=  bICP_RF_PacketInBitMask;
                  else
                     bICP_RF_PacketData[bICP_RF_PacketInPointer][bICP_RF_PacketInBytePointer] &= ~bICP_RF_PacketInBitMask;
                  if( bICP_RF_PacketInBitMask == 0x80 )
                  {
                     //reached the top of this byte, move on to the next one
                     bICP_RF_PacketInBitMask = 0x01;
                     bICP_RF_PacketInBytePointer++;   //no range checks on this, bICP_RF_Decode_Counter counting bits will exit
                  }
                  else
                  {
                     bICP_RF_PacketInBitMask = (bICP_RF_PacketInBitMask << 1);
                  }
#ifdef BITSTREAM_SERIAL_BLIT_FROM_INT
//-----debug binary blit out of serial port-----
//note potential conflict here between this and any serial data also being sent out of the main loop!
if( bICP_LastPacketBit )
   Serial.print( "1" );
else     
   Serial.print( "0" );
//---------------------------------------------
#endif
                  if( bICP_RF_Decode_Counter == RF_PACKET_BITCOUNT )
                  {
                     //all the bits are loaded from this packet, reset the state machine, inform the main loop and exit
                     bICP_RF_PacketInPointer = ((bICP_RF_PacketInPointer+1)&0x03);   ///only the lower two bits are used for the 4 entry array
                     uiICP_RF_PacketInCount++;
                     RF_DECODE_RESET();
                  }
               }
            }
            break;
         }
         //--------------------------------------------
         default:
         {
            //unexpected/corrupted state, return to RF_DECODE_STATE_IDLE
            RF_DECODE_RESET();
            break;
         }
      }
      //----------------------------------------------------------------------------
   }
   else
   {
      //----------------------------------------------------------------------------
      //PERIOD OUT OF BOUNDS, DISCARD
      //----------------------------------------------------------------------------
      //Call macro that resets any RF_Decode state machine and housekeeping values,
      //any invalid period measurement will throw the incoming packet away and wait for the next good preamble
      RF_DECODE_RESET();
   }
   
   //save the current capture data as previous so it can be used for period calculation again next time around
   uiICP_PreviousEventTime    = uiICP_EventTime;
   uiICP_PreviousEventPeriod  = uiICP_EventPeriod;
   bICP_PreviousEventPolarity = bICP_EventPolarity;
   
//GREEN test led off (flicker for debug)
GREEN_TESTLED_OFF();
}



// Function that prints data from the server
void printData(char* data, int len)
{
  // Print the data returned by the server
  // Note that the data is not null-terminated, may be broken up into smaller packets, and 
  // includes the HTTP header.
  while (len-- > 0) {
    Serial.print(*(data++));
  }
}
