// Sample from the ADC continuously at a particular sample rate
// and then compute an FFT over the data
//
// much of this code is from pico-examples/adc/dma_capture/dma_capture.c
// the rest is written by Alex Wulff (www.AlexWulff.com)

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "kiss_fftr.h"

// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 3840
#define FSAMP 12500

#define UART_ID uart0
#define BAUD_RATE 31250

#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25
#define LED_100HZ 16
#define LED_1000HZ 17
#define LED_2000HZ 18

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 1000

// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[3 * NSAMP];
float note_freqs[88];
const uint8_t note_on = 0x90;
const uint8_t note_off = 0xB0;
const uint8_t note1 = 60;
const uint8_t note2 = 61;
const uint8_t vel = 127;
uint8_t buf1_on[3] = {note_on, note1, vel};
uint8_t buf1_off[3] = {note_off, 123, 0};
uint8_t buf2_on[3] = {note_on, note2, vel};
uint8_t buf2_off[3] = {note_off, note2, 0};
int note_diff_min = 89;

void setup();
void sample(uint8_t *capture_buf);

int main() {
  uint8_t cap_buf[NSAMP];
  kiss_fft_scalar fft_in[NSAMP]; // kiss_fft_scalar is a float
  kiss_fft_cpx fft_out[NSAMP];
  kiss_fftr_cfg cfg = kiss_fftr_alloc(NSAMP,false,0,0);
  
  // setup ports and outputs
  setup();

  while (1) {
    // get NSAMP samples at FSAMP
    sample(cap_buf);
    // fill fourier transform input while subtracting DC component
    uint64_t sum = 0;
    for (int i=0;i<NSAMP;i++) {sum+=cap_buf[i];}
    float avg = (float)sum/NSAMP;
    for (int i=0;i<NSAMP;i++) {fft_in[i]=(float)cap_buf[i]-avg;}

    // compute fast fourier transform
    kiss_fftr(cfg , fft_in, fft_out);
    
    // compute power and calculate max freq component
    float max_power = 0;
    int max_idx = 0;
    // any frequency bin over NSAMP/2 is aliased (nyquist sampling theorum)
    for (int i = 0; i < NSAMP/2; i++) {
      float power = fft_out[i].r*fft_out[i].r+fft_out[i].i*fft_out[i].i;
      if (power>max_power) {
	      max_power=power;
	      max_idx = i;
      }
    }

    float left_power = fft_out[max_idx-1].r*fft_out[max_idx-1].r+fft_out[max_idx-1].i*fft_out[max_idx-1].i;
    float right_power = fft_out[max_idx+1].r*fft_out[max_idx+1].r+fft_out[max_idx+1].i*fft_out[max_idx+1].i;
    float proportionR = max_power / right_power;
    float proportionL = max_power / left_power;

    float rightP = proportionR - (10 - (2.0 / (proportionR + 2.0)));
    float rightMult = (-1.0 / (rightP + 1.0)) + 8.5;
    float rightAdd = (4.0 / (rightP + 1.0)) + 24.0;

    if (rightP < 1)
    {
      rightP = 1;
    }

    max_idx *= 3;

    float max_freq = freqs[max_idx];
    float left_freq = freqs[max_idx-1];
    float right_freq = freqs[max_idx+1];
    

    float leftLean = rightAdd - (rightMult * log10(rightP));

    if (leftLean < 0)
    {
        leftLean = 0;
    }
    else if (leftLean > 25)
    {
        leftLean = 25;
    }

    int swap = 0;

    float numR = 3330 * log10(proportionL + 520) - 8750;
    float rightLean = (numR/((proportionL+4)));
    float actual_freq = max_freq;
    float note_val = 0;

    if (leftLean < 0)
    {
      leftLean = 0;
    }

    if (left_power < right_power && left_power > 2 && left_power < 50 && max_idx > 0)
    {
      actual_freq = freqs[max_idx - 1];
    }
    else if (left_power > right_power && right_power < 50 && max_idx < (3 * NSAMP) / 2)
    {
      actual_freq = freqs[max_idx + 1];
    }
    else
    {
        actual_freq = freqs[max_idx];
    }

    for (int i = 0; i < 88; i++)
    {
        if (fabs(note_freqs[i] - actual_freq) < fabs(note_freqs[note_diff_min] - actual_freq))
        {
            note_diff_min = i;
        }
    }

    //printf("L: %0.1f Hz G: %0.1f Hz R: %0.1f Hz N: %0.1f\n",left_freq, max_freq, right_freq, note_freqs[note_diff_min]);
    //printf("L: %0.9f V G: %0.9f V R: %0.9f V\n",left_power, max_power, right_power);

    if (max_power > 500000)
    {
        buf1_on[1] = note_diff_min + 21;
        uart_write_blocking(UART_ID, buf1_on, 3);
        //sleep_ms(500);
    }
    else
    {
        /*for(int i = 0; i < 128; i++)
        {
            buf1_off[1] = i;
            uart_write_blocking(UART_ID, buf1_off, 3);
        }*/

        uart_write_blocking(UART_ID, buf1_off, 3);
        //sleep_ms(500);
        //uart_write_blocking(UART_ID, buf2_off, 3);
        //sleep_ms(500);
    }
    
    
    if (max_freq > 2500.0 && max_freq <= 5000.0)
    {
    	gpio_put(LED_100HZ, 1);
    	gpio_put(LED_1000HZ, 0);
    	gpio_put(LED_2000HZ, 0);
    }
    else if (max_freq > 5000.0 && max_freq <= 10000.0)
    {
    	gpio_put(LED_100HZ, 1);
    	gpio_put(LED_1000HZ, 1);
    	gpio_put(LED_2000HZ, 0);
    }
    else if (max_freq > 10000.0)
    {
    	gpio_put(LED_100HZ, 1);
    	gpio_put(LED_1000HZ, 1);
    	gpio_put(LED_2000HZ, 1);
    }
    else
    {
    	gpio_put(LED_100HZ, 0);
    	gpio_put(LED_1000HZ, 0);
    	gpio_put(LED_2000HZ, 0);
    }
    
  }

  // should never get here
  kiss_fft_free(cfg);
}

void sample(uint8_t *capture_buf) {
  adc_fifo_drain();
  adc_run(false);
      
  dma_channel_configure(dma_chan, &cfg,
			capture_buf,    // dst
			&adc_hw->fifo,  // src
			NSAMP,          // transfer count
			true            // start immediately
			);

  gpio_put(LED_PIN, 1);
  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
  gpio_put(LED_PIN, 0);
}

void setup() {
  stdio_init_all();
  uart_init(UART_ID, BAUD_RATE);
  uart_set_translate_crlf(UART_ID, false);

  sleep_ms(1000);

  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);


  gpio_init(LED_PIN);
  gpio_init(LED_100HZ);
  gpio_init(LED_1000HZ);
  gpio_init(LED_2000HZ);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_set_dir(LED_100HZ, GPIO_OUT);
  gpio_set_dir(LED_1000HZ, GPIO_OUT);
  gpio_set_dir(LED_2000HZ, GPIO_OUT);

  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
		 true,    // Write each completed conversion to the sample FIFO
		 true,    // Enable DMA data request (DREQ)
		 1,       // DREQ (and IRQ) asserted when at least 1 sample present
		 false,   // We won't see the ERR bit because of 8 bit reads; disable.
		 true     // Shift each sample to 8 bits when pushing to FIFO
		 );

  // set sample rate
  adc_set_clkdiv(CLOCK_DIV);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  // calculate frequencies of each bin
  float f_max = FSAMP;
  float f_res = f_max / (3 * NSAMP);
  for (int i = 0; i < 3 * NSAMP; i++) {freqs[i] = f_res*i;}

  float note_power_offset = -48;
  for (int i = 0; i < 88; i++)
  {
      note_freqs[i] = 440 * pow(2, (note_power_offset / 12));
      note_power_offset++;
  }
}
