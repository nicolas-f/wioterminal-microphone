#include"seeed_line_chart.h" //include the library
#include <math.h>
#include "wiring_private.h"  // pinPeripheral() function

// https://github.com/Dennis-van-Gils/SAMD51_InterruptTimer
#include "SAMD51_InterruptTimer.h"
TFT_eSPI tft;
 
#define max_size 50 //maximum size of data

TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite 
 
int mic_rate = 32000;
unsigned long mic_tick_us_delay = max(1, 1000000/ mic_rate);

#define MAX_AUDIO_WINDOW_SIZE 512
static int64_t scaled_input_buffer_feed_cursor = 0;
static int64_t scaled_input_buffer_consume_cursor = 0;
static float scaled_input_buffer[MAX_AUDIO_WINDOW_SIZE];

Adc *adc;
static bool dacEnabled[2];
static int _ADCResolution = 10;
static int _readResolution = 10;
void initMic() {
  
  uint32_t valueRead = 0;
  uint32_t pin = WIO_MIC;
  pinPeripheral(pin, PIO_ANALOG);
  if (pin == PIN_DAC0 || pin == PIN_DAC1) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
    uint8_t channel = (pin == PIN_DAC0 ? 0 : 1);

    if (dacEnabled[channel]) {
      dacEnabled[channel] = false;

      while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
      DAC->CTRLA.bit.ENABLE = 0;     // disable DAC

      while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
      DAC->DACCTRL[channel].bit.ENABLE = 0;

      while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
      DAC->CTRLA.bit.ENABLE = 1;     // enable DAC
    }

    while (DAC->SYNCBUSY.bit.ENABLE);
  }

  if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG) adc = ADC0;
  else if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG_ALT) adc = ADC1;
  else return;

  while ( adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
  adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input

  // Control A
  /*
     Bit 1 ENABLE: Enable
       0: The ADC is disabled.
       1: The ADC is enabled.
     Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
     value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
     (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.

     Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
     configured. The first conversion after the reference is changed must not be used.
  */
  while ( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while ( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

  adc->SWTRIG.bit.START = 1;
}



void releaseMic() {
  
  while ( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x00;             // Disable ADC

  while ( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

}

void threadmic() {

    // Clear the Data Ready flag
    adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

    // Start conversion again, since The first conversion after the reference is changed must not be used.
    adc->SWTRIG.bit.START = 1;

    // Store the value
    while (adc->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
    int valueRead = adc->RESULT.reg;

    mapResolution(valueRead, _ADCResolution, _readResolution);

    scaled_input_buffer[scaled_input_buffer_feed_cursor % MAX_AUDIO_WINDOW_SIZE] = (float)valueRead;
    scaled_input_buffer_feed_cursor += 1;
}

void setup() {
    Serial.begin(9600);
    delay(2000);
    pinMode(WIO_MIC, INPUT);
 
    tft.begin();
    tft.setRotation(3);
    spr.createSprite(TFT_HEIGHT,TFT_WIDTH);

    Serial.println("run ");
    Serial.println(mic_tick_us_delay);
    
    initMic();

    TC.startTimer(mic_tick_us_delay, threadmic); // mic_tick_us_delay usec
}

//High pass butterworth filter order=1 alpha1=0.0125 
class  FilterBuHp1
{
  public:
    FilterBuHp1()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (9.621952458291035404e-1f * x)
         + (0.92439049165820696974f * v[0]);
      return 
         (v[1] - v[0]);
    }
};
FilterBuHp1 filter;
void loop() {
    doubles data; //Initilising a doubles type to store data
    int samples_ar[max_size];
    float sample = 0;
    // take only last feed samples
    scaled_input_buffer_consume_cursor = scaled_input_buffer_feed_cursor - max_size;
    for(int samples = 0;  samples < max_size; samples++) {
      sample = filter.step(scaled_input_buffer[scaled_input_buffer_consume_cursor % MAX_AUDIO_WINDOW_SIZE]);
      scaled_input_buffer_consume_cursor+=1;
      samples_ar[samples] = sample;
    }
    float sum = 0;
    for(int i=0; i < max_size; i++) {
      sum += samples_ar[i];
    }
    float avg = sum / max_size;
    float rms = 0;
    for(int i=0; i < max_size; i++) {
      data.push(samples_ar[i] - avg);
    }
    spr.fillSprite(TFT_DARKGREY);
 
    //Settings for the line graph title
    auto header =  text(0, 0)
                .value("Microphone Reading")
                .align(center)
                .color(TFT_WHITE)
                .valign(vcenter)
                .width(tft.width())
                .thickness(2);
 
    header.height(header.font_height() * 2);
    header.draw(); //Header height is the twice the height of the font
 
  //Settings for the line graph
    auto content = line_chart(20, header.height()); //(x,y) where the line graph begins
         content
                .height(tft.height() - header.height() * 1.5) //actual height of the line chart
                .width(tft.width() - content.x() * 2) //actual width of the line chart
                .based_on(-90) //Starting point of y-axis, must be a float
                .show_circle(true) //drawing a cirle at each point, default is on.
                .y_role_color(TFT_WHITE)
                .x_role_color(TFT_WHITE)
                .value(data) //passing through the data to line graph
                .color(TFT_RED) //Setting the color for the line
                .draw();
 
    spr.pushSprite(0, 0);    
    delay(500);  
}
