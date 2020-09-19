#include"seeed_line_chart.h" //include the library
#include <math.h>

#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h" // pinPeripheral() function
#define AUDIO_TC TC2
#define AUDIO_GCLK_ID TC2_GCLK_ID
#define AUDIO_TC_DMAC_ID_OVF TC2_DMAC_ID_OVF
#define WAIT_TC8_REGS_SYNC(x) while(x->COUNT8.SYNCBUSY.bit.ENABLE || x->COUNT8.SYNCBUSY.bit.SWRST);
#define AUDIO_PRESCALER TC_CTRLA_PRESCALER_DIV16
#define AUDIO_CLKRATE (VARIANT_GCLK2_FREQ >> 4)
#define AUDIO_PRESCALER TC_CTRLA_PRESCALER_DIV16
#define AUDIO_SAMPLE_RATE_EXACT 44014.085 // 100 MHz / 16 / 142
#define AUDIO_TC_FREQ 44100

TFT_eSPI tft;

#define max_size 50 //maximum size of data

TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite

//------------------------------------
// DMA AUDIO CODE
#define AUDIO_BLOCK_SAMPLES 128
static uint16_t audio_buffer[AUDIO_BLOCK_SAMPLES];
int16_t  audiodata[AUDIO_BLOCK_SAMPLES];
uint32_t audiodata_offset;
Adafruit_ZeroDMA* dma0;
ZeroDMAstatus stat;
DmacDescriptor *desc;
uint32_t *daddr;

#define AUDIODATA_STATE_EMPTY 0
#define AUDIODATA_STATE_FILLED 1
int audiodata_state = AUDIODATA_STATE_EMPTY;

void initMic(uint8_t pin0) {
  dma0 = new Adafruit_ZeroDMA;
  stat = dma0->allocate();
  pinPeripheral(pin0, PIO_ANALOG);

  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)


  ADC1->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT | ADC_CTRLB_FREERUN;
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB ); //wait for sync

  ADC1->SAMPCTRL.reg = 5;                        // sampling Time Length
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL ); //wait for sync

  ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync

  ADC1->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin0].ulADCChannelNumber; // Selection for the positive ADC input
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync

  // Averaging (see datasheet table in AVGCTRL register description)
  //TODO: this is weirdly set for a 13 bit result for now... we may want to change later
  ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x0);
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL ); //wait for sync

  ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

  ADC1->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

  // Start conversion
  ADC1->SWTRIG.bit.START = 1;

  //TODO: on SAMD51 lets find an unused timer and use that
  GCLK->PCHCTRL[AUDIO_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

  AUDIO_TC->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NFRQ;

  AUDIO_TC->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  WAIT_TC8_REGS_SYNC(AUDIO_TC)

  AUDIO_TC->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
  WAIT_TC8_REGS_SYNC(AUDIO_TC)
  while (AUDIO_TC->COUNT8.CTRLA.bit.SWRST);

  AUDIO_TC->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | AUDIO_PRESCALER;
  WAIT_TC8_REGS_SYNC(AUDIO_TC)

  AUDIO_TC->COUNT8.PER.reg = (uint8_t)( AUDIO_CLKRATE / AUDIO_TC_FREQ);
  WAIT_TC8_REGS_SYNC(AUDIO_TC)

  AUDIO_TC->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
  WAIT_TC8_REGS_SYNC(AUDIO_TC)

  dma0->setTrigger(AUDIO_TC_DMAC_ID_OVF);
  dma0->setAction(DMA_TRIGGER_ACTON_BEAT);

  desc = dma0->addDescriptor(
           (void *)(&ADC1->RESULT.reg),    // move data from here
           audio_buffer,      // to here
           AUDIO_BLOCK_SAMPLES / 2,               // this many...
           DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
           false,                             // increment source addr?
           true);
  desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

  desc = dma0->addDescriptor(
           (void *)(&ADC1->RESULT.reg),    // move data from here
           audio_buffer + AUDIO_BLOCK_SAMPLES / 2,      // to here
           AUDIO_BLOCK_SAMPLES / 2,               // this many...
           DMA_BEAT_SIZE_HWORD,               // bytes/hword/words
           false,                             // increment source addr?
           true);
  desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
  dma0->loop(true);
  dma0->setCallback(isr0);
  dma0->startJob();
}

void isr0(Adafruit_ZeroDMA *dma)
{
  if (audiodata_state == AUDIODATA_STATE_EMPTY) {
    const uint16_t *src, *end;
    uint16_t *dest;
    if (daddr != (uint32_t *)(audio_buffer + AUDIO_BLOCK_SAMPLES / 2)) {
      // DMA is receiving to the first half of the buffer
      // need to remove data from the second half
      src = (uint16_t *)&audio_buffer[AUDIO_BLOCK_SAMPLES / 2];
      end = (uint16_t *)&audio_buffer[AUDIO_BLOCK_SAMPLES];
      daddr = (uint32_t *)(audio_buffer + AUDIO_BLOCK_SAMPLES / 2);
    } else {
      // DMA is receiving to the second half of the buffer
      // need to remove data from the first half
      src = (uint16_t *)&audio_buffer[0];
      end = (uint16_t *)&audio_buffer[AUDIO_BLOCK_SAMPLES / 2];
      daddr = (uint32_t *)audio_buffer;
    }
    audiodata_offset += AUDIO_BLOCK_SAMPLES / 2;
    if (audiodata_offset == AUDIO_BLOCK_SAMPLES) {
      audiodata_offset = 0;
    }
    dest = (uint16_t *) & (audiodata[audiodata_offset]);

    do {
      *dest++ = *src++;
    } while (src < end);
    if (audiodata_offset == AUDIO_BLOCK_SAMPLES / 2) {
      audiodata_state = AUDIODATA_STATE_FILLED;
      Serial.println("audiodata_state = AUDIODATA_STATE_FILLED;");
    }
  }
}

//------------------------------------
// END OF AUDIO CODE

void setup() {
  Serial.begin(9600);
  // while(!Serial);
  initMic(WIO_MIC);

  tft.begin();
  tft.setRotation(3);
  spr.createSprite(TFT_HEIGHT, TFT_WIDTH);


}

float rate = 16000.0f; // sample rate in HZ
int micro_delay = 1 / rate * 1e6;
unsigned long last_fetch = 0;

void loop() {
  if (audiodata_state != AUDIODATA_STATE_FILLED) {
    return;
  }
  doubles data; //Initilising a doubles type to store data
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    data.push(audiodata[i]);
  }
  audiodata_state = AUDIODATA_STATE_EMPTY;
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
