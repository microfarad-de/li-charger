/*
 * Abstraction layer for the ATmega328p ADC 
 * Non-blocking read the ADC output as an alternative
 * to the blocking analogRead() method.
 *
 * Karim Hraibi - 2018
 */

#ifndef __ADC_H
#define __ADC_H

#include <Arduino.h>


#define ADC_NUM_PINS 8  /* Total number of ADC pins */

/*
 * ADC prescaler values
 */
enum AdcPrescaler_t {
  ADC_PRESCALER_2  = 1,
  ADC_PRESCALER_4  = 2,
  ADC_PRESCALER_8  = 3,
  ADC_PRESCALER_16 = 4,
  ADC_PRESCALER_32 = 5,
  ADC_PRESCALER_64 = 6,
  ADC_PRESCALER_128 = 7
};

/*
 * Analog pin numbers
 * Note: 
 * Arduino macro A0 = ADC_PIN0 + 14
 * the same applies for A1, A2...
 */
enum AdcPin_t {
  ADC_PIN0 = 0,
  ADC_PIN1 = 1,
  ADC_PIN2 = 2,
  ADC_PIN3 = 3,
  ADC_PIN4 = 4,
  ADC_PIN5 = 5,
  ADC_PIN6 = 6,
  ADC_PIN7 = 7
};

/*
 * Analog reference sources
 */
enum AdcReference_t {
  ADC_EXTERNAL = 0,                       /* external input */
  ADC_DEFAULT  = _BV(REFS0),              /* AVcc */
  ADC_INTERNAL = _BV(REFS0) | _BV(REFS1)  /* internal 1.1V */
};

/*
 * ADC class definition
 */
class AdcClass {

  public:
  
    /* 
     *  Initialize the ADC
     *  Parameters:
     *    prescaler : ADC prescaler value
     *    reference : ADC reference voltage
     *    numPins   : number of analog pins to sample
     */
    void initialize (AdcPrescaler_t prescaler = ADC_PRESCALER_128, AdcReference_t reference = ADC_DEFAULT, uint8_t numPins = 0);

    /*
     * Start ADC conversion
     * Parameters:
     *   adcPin : ADC pin number (0..7 for ATmega328p)
     */
    void start (AdcPin_t adcPin);

    /*
     * Read ADC result
     * Return value:
     *   -1      : no result available yet
     *   0..1023 : ADC result
     */
    int16_t readVal (void);

    /*
     * Read all preset ADC inputs
     * When called it will store the result of the next ADC conversion 
     * and store the result into the result array.
     * Return value:
     *   -1     : no result available yet
     *   0..8   : index of the current result array (ADC pin index)
     */
    int8_t readAll (void);

    /*
     * ADC result can be read from this array
     */
    int16_t result[ADC_NUM_PINS];

  private:  

    bool working = false;
    AdcReference_t reference;
    uint8_t numPins = 0;
    uint8_t currentPin = 0;
  
};



/*
 * ADC class instantiated as a singleton
 */
extern AdcClass ADConv;

#endif // __ADC_H
