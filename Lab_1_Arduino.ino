#include <math.h>
#define VREF 5.0
#define ADC_BITS 1023.0
#define ADC_BUF_LEN 400
#define SETTLE_MS 200
#define VOUT_PIN A0
#define VIN_PIN  A1

uint16_t vin_buf[ADC_BUF_LEN];
uint16_t vout_buf[ADC_BUF_LEN];

void compute_pp(uint16_t *buf, int len, float *vpp)
{
  uint16_t minv = 65535, maxv = 0;
  for(int i = 0; i < len; i++) {
    if(buf[i] < minv) minv = buf[i];
    if(buf[i] > maxv) maxv = buf[i];
  }
  *vpp = ((maxv - minv) / ADC_BITS) * VREF;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("f_hz, time_s, Vin_pp_V, Vout_pp_V, gain, gain_dB");
}

void loop()
{
  static unsigned long lastPrint = 0;
  static int manual_freq = 100; // CHANGE THIS VALUE AS YOU ADJUST THE SINE INPUT FREQUENCY
  delay(SETTLE_MS); // Wait for settling

  // Acquire
  for(int i = 0; i < ADC_BUF_LEN; i++)
  {
    vout_buf[i] = analogRead(VOUT_PIN);
    vin_buf[i]  = analogRead(VIN_PIN);
  }

  // Process
  float vout_pp, vin_pp;
  compute_pp(vout_buf, ADC_BUF_LEN, &vout_pp);
  compute_pp(vin_buf, ADC_BUF_LEN, &vin_pp);
  float gain = (vin_pp > 1e-6) ? (vout_pp / vin_pp) : 0.0;
  float gain_db = (gain > 1e-9) ? (20 * log10(gain)) : -999.0;
  float elapsed_time = millis() / 1000.0;

  // CSV Output
  Serial.print(manual_freq); Serial.print(", ");
  Serial.print(elapsed_time, 2); Serial.print(", ");
  Serial.print(vin_pp, 4); Serial.print(", ");
  Serial.print(vout_pp, 4); Serial.print(", ");
  Serial.print(gain, 4); Serial.print(", ");
  Serial.println(gain_db, 2);

  delay(1000); // Print every second (adjust as needed)
}
