#include"seeed_line_chart.h" //include the library
#include <math.h>
 
TFT_eSPI tft;
 
#define max_size 50 //maximum size of data

TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite 
 
void setup() {
    pinMode(WIO_MIC, INPUT);
 
    tft.begin();
    tft.setRotation(3);
    spr.createSprite(TFT_HEIGHT,TFT_WIDTH);
}

float rate = 16000.0f; // sample rate in HZ
int micro_delay = 1 / rate * 1e6;
unsigned long last_fetch = 0;

void loop() {
    doubles data; //Initilising a doubles type to store data
    int samples_ar[max_size];
    for(int samples = 0;  samples < max_size; samples++) {
      samples_ar[samples] = analogRead(WIO_MIC);
      delayMicroseconds(micro_delay);
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
                .based_on(-200) //Starting point of y-axis, must be a float
                .show_circle(true) //drawing a cirle at each point, default is on.
                .y_role_color(TFT_WHITE)
                .x_role_color(TFT_WHITE)
                .value(data) //passing through the data to line graph
                .color(TFT_RED) //Setting the color for the line
                .draw();
 
    spr.pushSprite(0, 0);    
    delay(500);  
}
