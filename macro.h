#ifndef MACRO_H_
#define MACRO_H_
/*--------------------------------------------------------------------------------------
 General macros
--------------------------------------------------------------------------------------*/
#define INPUT_CAPTURE_IS_RISING_EDGE()    ((TCCR1B & _BV(ICES1)) != 0)
#define INPUT_CAPTURE_IS_FALLING_EDGE()   ((TCCR1B & _BV(ICES1)) == 0)
#define SET_INPUT_CAPTURE_RISING_EDGE()   (TCCR1B |=  _BV(ICES1))
#define SET_INPUT_CAPTURE_FALLING_EDGE()  (TCCR1B &= ~_BV(ICES1))


#define GREEN_TESTLED_IS_ON()       ((PORTD & (1<<PORTD6)) == 0)
#define GREEN_TESTLED_IS_OFF()      ((PORTD & (1<<PORTD6)) != 0)
#define GREEN_TESTLED_ON()          ((PORTD &= ~(1<<PORTD6)))
#define GREEN_TESTLED_OFF()         ((PORTD |=  (1<<PORTD6)))
#define GREEN_TESTLED_TOGGLE()      if(GREEN_TESTLED_IS_ON()){GREEN_TESTLED_OFF();}else{GREEN_TESTLED_ON();}

#define RED_TESTLED_IS_ON()         ((PORTD & (1<<PORTD7)) == 0)
#define RED_TESTLED_IS_OFF()        ((PORTD & (1<<PORTD7)) != 0)
#define RED_TESTLED_ON()            ((PORTD &= ~(1<<PORTD7)))
#define RED_TESTLED_OFF()           ((PORTD |=  (1<<PORTD7)))
#define RED_TESTLED_TOGGLE()        if(RED_TESTLED_IS_ON()){RED_TESTLED_OFF();}else{RED_TESTLED_ON();}


#endif /*MACRO_H_*/
