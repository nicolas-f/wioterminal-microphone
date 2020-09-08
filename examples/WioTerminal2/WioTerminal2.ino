#include"seeed_line_chart.h" //include the library
#include <math.h>

// https://github.com/Dennis-van-Gils/SAMD51_InterruptTimer
#include "SAMD51_InterruptTimer.h"
TFT_eSPI tft;
 
#define max_size 50 //maximum size of data

TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite 
 
int mic_rate = 16000;
unsigned long mic_tick_us_delay = max(1, 1000000/ mic_rate);

#define MAX_AUDIO_WINDOW_SIZE 512
static int64_t scaled_input_buffer_feed_cursor = 0;
static int64_t scaled_input_buffer_consume_cursor = 0;
static float scaled_input_buffer[MAX_AUDIO_WINDOW_SIZE];

void threadmic() {
    scaled_input_buffer[scaled_input_buffer_feed_cursor % MAX_AUDIO_WINDOW_SIZE] = (float)analogRead(WIO_MIC);
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
                .based_on(-20) //Starting point of y-axis, must be a float
                .show_circle(true) //drawing a cirle at each point, default is on.
                .y_role_color(TFT_WHITE)
                .x_role_color(TFT_WHITE)
                .value(data) //passing through the data to line graph
                .color(TFT_RED) //Setting the color for the line
                .draw();
 
    spr.pushSprite(0, 0);    
    delay(500);  
}
