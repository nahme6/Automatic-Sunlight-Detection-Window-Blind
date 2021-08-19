#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h> 
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "bh1750.h"

#define SET_BIT(reg, pin)           reg|= 1 <<pin
#define SET_BITS(reg, mask)         reg |= mask

#define CLEAR_BIT(reg, pin)         reg &= ~(1<<pin)
#define CLEAR_BITS(reg, mask)       reg &= ~(mask)

#define TOGGLE_BIT(reg, pin)        reg ^= (1 << pin)
#define TOGGLE_BITS(reg, mask)      reg ^= mask

#define BIT_IS_SET(reg, pin)        (reg>>pin) & 1
#define BITS_ARE_SET(REG, mask)     (reg & mask) == mask

#define WRITE_BIT(reg, pin, value)  reg = (reg & ~(1 << pin)) | (value << pin)

#define SET_INPUT(ddr, bit)         CLEAR_BIT(ddr, bit)
#define SET_OUTPUT(ddr, bit)        SET_BIT(ddr, bit) 

#define HIGH 1
#define LOW  0

// On Stepper Motor Controller IN1/A
void PD3_PWM(int val){
  if (val == 1)
  {
    DDRD |= (1 << PD3);
    // PD6 is now an output

    OCR2B = 255;
    // set PWM for 50% duty cycle

    TCCR2A |= (1 << COM2B1);
    // set none-inverting mode
 
    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 8 and starts PWM  
    TCCR2B = (1 << CS01);
    
    TCCR2A |= (1 << WGM01 | 1 << WGM00);
    // set fast PWM Mode
  } else if (val == 0) 
  {
    CLEAR_BIT(DDRD, PD3);

    OCR2B = 0;

    CLEAR_BIT(TCCR2A, COM2B1);

    CLEAR_BIT(TCCR2B, CS01);

    CLEAR_BIT(TCCR2A, WGM00);
    CLEAR_BIT(TCCR0A, WGM01);
  }
}

// On Stepper Motor Controller IN2/B
void PD6_PWM(int val){
  if (val == 1)
  {
    DDRD |= (1 << PD5);
    // PD6 is now an output

    OCR0B = 255;
    // set PWM for 50% duty cycle

    TCCR0A |= (1 << COM0B1);
    // set none-inverting mode

    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 8 and starts PWM  
    TCCR0B = (1 << CS01);

    TCCR0A |= (1 << WGM01 | 1 << WGM00);
    // set fast PWM Mode
  } else if (val == 0) 
  {
    CLEAR_BIT(DDRD, PD5);

    OCR0B = 0;

    CLEAR_BIT(TCCR0A, COM0B1);

    CLEAR_BIT(TCCR0B, CS01);

    CLEAR_BIT(TCCR0A, WGM00);
    CLEAR_BIT(TCCR0A, WGM01);
  }
}

// On Stepper Motor Controller IN3/C
void PD5_PWM(int val){
  if (val == 1)
  {
    DDRD |= (1 << PD6);
    // PD5 is now an output

    OCR0A = 255;
    // set PWM for 50% duty cycle

    TCCR0A |= (1 << COM0A1);
    // set none-inverting mode

    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 8 and starts PWM  
    TCCR0B = (1 << CS01);

    TCCR0A |= (1 << WGM01 | 1 << WGM00);
    // set fast PWM Mode
  } else if (val == 0)
  {
    CLEAR_BIT(DDRD, PD6);

    OCR0A = 0;

    CLEAR_BIT(TCCR0A, COM0A1);

    CLEAR_BIT(TCCR0B, CS01);

    CLEAR_BIT(TCCR0A, WGM00);
    CLEAR_BIT(TCCR0A, WGM01);
  }
}

// On Stepper Motor Controller IN4/D
void PB3_PWM(int val){
  if (val == 1)
  {
    DDRB |= (1 << PB3);
    // PD6 is now an output

    OCR2A = 255;
    // set PWM for 50% duty cycle

    TCCR2A |= (1 << COM2A1);
    // set none-inverting mode

    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 8 and starts PWM  
    TCCR2B = (1 << CS21);

    TCCR2A |= (1 << WGM21 | 1 << WGM20);
    // set fast PWM Mode
  } else if (val == 0)
  {
    CLEAR_BIT(DDRB, PB3);

    OCR2A = 0;
    // set PWM for 50% duty cycle

    CLEAR_BIT(TCCR2A, COM2A1);
    // set none-inverting mode

    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 8 and starts PWM  
    CLEAR_BIT(TCCR2B, CS21);

    CLEAR_BIT(TCCR2A, WGM20);
    CLEAR_BIT(TCCR2A, WGM21);
    // set fast PWM Mode
  }
}

int Steps = 4096;
int cstep = 0;
int astep = 0;

void cStep() {
    switch(cstep) 
    {
    case 0:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(HIGH);
      break; 
    case 1:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(HIGH);
      PB3_PWM(HIGH);
      break; 
    case 2:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(HIGH);
      PB3_PWM(LOW);
      break; 
    case 3:
      PD3_PWM(LOW);
      PD6_PWM(HIGH);
      PD5_PWM(HIGH);
      PB3_PWM(LOW);
      break; 
    case 4:
      PD3_PWM(LOW);
      PD6_PWM(HIGH);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    case 5:
      PD3_PWM(HIGH);
      PD6_PWM(HIGH);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    case 6:
      PD3_PWM(HIGH);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    case 7:
      PD3_PWM(HIGH);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(HIGH);
      break; 
    default:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    }

    cstep++;
    if(cstep==8)
    {
      cstep=0;
    }
}

void aStep() {
    switch(astep) 
    {
    case 0:
      PD3_PWM(HIGH);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    case 1:
      PD3_PWM(HIGH);
      PD6_PWM(HIGH);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    case 2:
      PD3_PWM(LOW);
      PD6_PWM(HIGH);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    case 3:
      PD3_PWM(LOW);
      PD6_PWM(HIGH);
      PD5_PWM(HIGH);
      PB3_PWM(LOW);
      break; 
    case 4:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(HIGH);
      PB3_PWM(LOW);
      break; 
    case 5:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(HIGH);
      PB3_PWM(HIGH);
        break; 
    case 6:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(HIGH);
      break; 
    case 7:
      PD3_PWM(HIGH);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(HIGH);
      break; 
    default:
      PD3_PWM(LOW);
      PD6_PWM(LOW);
      PD5_PWM(LOW);
      PB3_PWM(LOW);
      break; 
    }

    astep++;
    if(astep==8)
    {
      astep=0;
    }
}

void clockwiseRotation(int state){
  if (state == 1)
  {
    for(int x=0;x<Steps;x++)
    {
      cStep(state);
      _delay_ms(1);
    }
    return;
  } 
}

void antiClockwiseRotation(int state){
  if (state == 1)
  {
    for(int x=0;x<Steps;x++)
    {
      aStep(state);
      _delay_ms(1);
    }
    return;
  }
}

void resetMotor(){
  PD3_PWM(LOW);
  PD6_PWM(LOW);
  PD5_PWM(LOW);
  PB3_PWM(LOW);
}

void setup(){
  clockwiseRotation(0);
  antiClockwiseRotation(0);

  SET_BIT(TCCR0B,CS02);
  SET_BIT(TIMSK0, TOIE0);

  sei();

  CLEAR_BIT(DDRB, 0); // right (switch 2)
  CLEAR_BIT(DDRB, 5); // left (switch 1)

  SET_BIT(DDRC, 1);
  SET_BIT(DDRC, 0); 
}

uint8_t history = 0;
uint8_t mask = 0b00011111;
uint8_t debounced_state = 0;
volatile int state_count = 0;
volatile int state_count2 = 0;
volatile int is_pressed = 0;

ISR(TIMER0_OVF_vect){
    state_count = ((state_count << 1) & mask) | (BIT_IS_SET(PINB, 0));
    state_count2 = ((state_count << 1) & mask) | (BIT_IS_SET(PINB, 5));

    if (state_count == mask) {
        is_pressed = 1;
    }

    if (state_count == 0)
    {
        is_pressed = 0;
    }

    if (state_count2 == mask) {
        is_pressed = 1;
    }

    if (state_count2 == 0)
    {
        is_pressed = 0;
    }
}

int main(void) {
  setup();
  bh1750_setup();

  while (1)
  {
    if ((BIT_IS_SET(PINB, 0)))
    {
      while (!(BIT_IS_SET(PINB, 5)))
      {
        uint16_t light_level;
        bh1750_read(&light_level);

        while (light_level < 50)
        {
          resetMotor();
          clockwiseRotation(0);
          antiClockwiseRotation(0);
          SET_BIT(PORTC, 1);
          CLEAR_BIT(PORTC,0);
          clockwiseRotation(1);
          break;
        } 
  
        while (light_level > 50)
        {
          resetMotor();
          clockwiseRotation(0);
          antiClockwiseRotation(0);
          CLEAR_BIT(PORTC,1);
          SET_BIT(PORTC, 0);
          antiClockwiseRotation(1);
          break;
        }
      }
    }
    
    if ((BIT_IS_SET(PINB, 5)))
    {
      CLEAR_BIT(PORTC, 1);
      CLEAR_BIT(PORTC,0);
      clockwiseRotation(0);
      antiClockwiseRotation(0);
    }
  }
  return 0;
}