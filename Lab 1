#include <math.h>

#define VREF 5.0
#define ADC_BITS 1023.0

#define LUT_N 100
#define ADC_BUF_LEN 400

#define SETTLE_MS 200

#define PWM_PIN 11
#define VOUT_PIN A0
#define VIN_PIN  A1

#define SINE_OFFSET 0.5
#define SINE_AMP 0.4

uint8_t dac_lut[LUT_N];

volatile uint16_t lut_index = 0;

uint16_t vin_buf[ADC_BUF_LEN];
uint16_t vout_buf[ADC_BUF_LEN];

void build_sine_lut()
{
  for(int i=0;i<LUT_N;i++)
  {
    float phase = 2.0 * PI * ((float)i / LUT_N);
    float v = SINE_OFFSET + SINE_AMP * sin(phase);

    if(v<0) v=0;
    if(v>1) v=1;

    dac_lut[i] = (uint8_t)(255 * v);
  }
}

ISR(TIMER1_COMPA_vect)
{
  analogWrite(PWM_PIN, dac_lut[lut_index]);

  lut_index++;
  if(lut_index >= LUT_N)
    lut_index = 0;
}

void setSampleRate(uint32_t fs)
{
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1B |= (1 << WGM12);

  uint32_t compare = (16000000 / fs) - 1;

  OCR1A = compare;

  TCCR1B |= (1 << CS10);

  TIMSK1 |= (1 << OCIE1A);

  sei();
}

void compute_pp(uint16_t *buf, int len, float *vpp)
{
  uint16_t minv = 65535;
  uint16_t maxv = 0;

  for(int i=0;i<len;i++)
  {
    if(buf[i] < minv) minv = buf[i];
    if(buf[i] > maxv) maxv = buf[i];
  }

  *vpp = ((maxv - minv)/ADC_BITS)*VREF;
}

void setup()
{
  Serial.begin(115200);

  pinMode(PWM_PIN, OUTPUT);

  build_sine_lut();

  Serial.println("f_hz, Vin_pp_V, Vout_pp_V, gain, gain_dB");
}

void loop()
{
  int f_list[] = {100,300,500,800,1000,1500,2000,5000};
  int nf = sizeof(f_list)/sizeof(int);

  for(int k=0;k<nf;k++)
  {
    int f = f_list[k];

    int fs = f * LUT_N;

    setSampleRate(fs);

    delay(SETTLE_MS);

    for(int i=0;i<ADC_BUF_LEN;i++)
    {
      vout_buf[i] = analogRead(VOUT_PIN);
      vin_buf[i]  = analogRead(VIN_PIN);
    }

    float vout_pp;
    float vin_pp;

    compute_pp(vout_buf, ADC_BUF_LEN, &vout_pp);
    compute_pp(vin_buf, ADC_BUF_LEN, &vin_pp);

    float gain = vout_pp/vin_pp;
    float gain_db = 20*log10(gain);

    Serial.print(f);
    Serial.print(", ");
    Serial.print(vin_pp,4);
    Serial.print(", ");
    Serial.print(vout_pp,4);
    Serial.print(", ");
    Serial.print(gain,4);
    Serial.print(", ");
    Serial.println(gain_db,2);

    delay(200);
  }

  Serial.println("Sweep done");

  while(1);
}
