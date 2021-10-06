// Read LIS3DH, find FFT and spectral peak
// send data out serial port
// optionally transmit data words via LoRa
// 06-Oct-2021 JPB

#include <SPI.h>       // for RFM95 board
#include <RH_RF95.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "arduinoFFT.h"    // general FFT library 
// ------------------------------------------------------------
#define VBATPIN A7           // for measuring battery voltage
#define RFM95_CS 8          // for RFM95 on Feather M0
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0  // MHz center freq. for Tx/Rx
#define PKTLEN 20        // bytes in RF packet to send

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// ------------------------------------------------------------

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

boolean doRF = false;  // true to enable RF transmitter

char buf[PKTLEN];  // buffer to send out via LoRa

Adafruit_LIS3DH lis = Adafruit_LIS3DH();


arduinoFFT FFT = arduinoFFT(); // Create FFT object

const double samplingFrequency = 25;
const uint16_t samples = 256; //for FFT, this MUST ALWAYS be a power of 2
float rawData[samples]; // raw data from sensor, before ADC
double vReal[samples];  // data for FFT (gets windowed, then transformed)
double vImag[samples];

long int xsum,ysum,zsum;  // accumulated readings of X,Y,Z axes
const int acount = 2;  // how many readings to average
float norm = 80000;    // summed raw values that equals 1G
float xsm,ysm,zsm;     // smoothed accel values
float dsm=0;           // smoothed delta-accel value

const float f=0.35;         // low-pass filter value
const float f2 = 0.05;      // delta-mag lowpass filter value
//float rscale = 16000;  // roughly 1 G
const float rscale = 1;  // 1 ADC unit = roughly 0.0625 mG

int packetnum = 0;  // packet counter, we increment per xmit
uint16_t ptr=0;    // pointer into rawData[] buffer

// =====================================================
void setup(void) {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);  
  pinMode(LED_BUILTIN, OUTPUT);    // enable onboard LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);  // USB going to PC for debug
  // Serial1.begin(9600);  // hardware UART for OpenLog data storage
  Serial1.begin(115200);  // hardware UART for OpenLog data storage
  delay(1000);  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);  digitalWrite(LED_BUILTIN, LOW);  
  // Serial.println("log-dmag,mIndex");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }

  lis.setDataRate(LIS3DH_DATARATE_25_HZ); // [1,10,25,50,100,200,400]
  /*
    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
  */
  lis.read();      // get X,Y,Z data at once
  xsm = lis.x*rscale;
  ysm = lis.y*rscale;
  zsm = lis.z*rscale;
  
  // --------------------------- RFM95 setup ---------------
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);


  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");    
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  Serial1.println("LoRa radio init OK!");
  rf95.setFrequency(RF95_FREQ);
  RH_RF95::ModemConfig myconfig = { 0x72, 0x84, 0x00 }; // 125 kHz BW, SF=8
  rf95.setModemRegisters(&myconfig);  // setup LoRa to optimal values
  //rf95.setTxPower(18, false);
  rf95.setTxPower(23, false);

} // end setup()

// ---------------------------------------------------------------------------
const int sampleGroup = samples/4;  // data batch size

void loop() {

  for (int i=0; i<sampleGroup; i++) {
      int dcount = 0;
      while (! (lis.haveNewData()) ) {  // poll for ready; delay when not
        delay(1);
        dcount++;
      } 
      lis.read();      // get X,Y,Z data at once
      // Serial.println(dcount);
      
      float ax = lis.x; // previously divided by rscale;
      float ay = lis.y;
      float az = lis.z;
      xsm = ax*f + (xsm*(1-f));  // LP-filtered (smoothed) value
      ysm = ay*f + (ysm*(1-f));
      zsm = az*f + (zsm*(1-f));
      float dx = ax-xsm;  // highpass-filtered X,Y,Z values
      float dy = ay-ysm;
      float dz = az-zsm;
      // dmag is magnitude of change in direction from recent average
      // which is an estimate of overall motion
      float dmag = sqrt(dx*dx + dy*dy + dz*dz); // + 0.0005 if you need to avoid 0
      dsm = dmag*f2 + (dsm * (1-f2));  // low-pass filtered dmag value
      rawData[ptr] = (dmag - dsm);  // (new - average) is highpass filter; no DC offset
      ptr = (ptr+1) % samples;  // wraps around at end of buffer
      if (dcount > 0) {
        delay(34);  // msec delay  25 Hz => 40 msec
      }
  } // for (i...)
  // we just added sampleGroup readings into rawData[] buffer, now run FFT on all of it

  int j=ptr;  // start reading from oldest sample in buffer
  for (int i=0;i<samples;i++) {
    vReal[i] = rawData[j]; // transfer full buffer into FFT buffer
    j = (j+1) % samples;
    vImag[i] = 0.0; // set imag part to zero    
  }
  
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HANN, FFT_FORWARD);  /* Weight data */
  //FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weight data */
  //Serial.println("Weighed data:");
  //PrintVector(vReal, samples, SCL_TIME);

  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  //Serial.println();

  double k;
  double v;
  findPeak(vReal, samples, &k, &v);
  double fMax = k * (samplingFrequency/samples); // convert array index to units of frequency

  /*
  Serial.print(int(millis()/1000));
  Serial.print(", ");  
  Serial.print(fMax, 2);
  Serial.print(", ");
  Serial.println(int(v));
  */

  int fMi = fMax * 1000;
  fMi=min(fMi,9999);  // clamp to 4 digits
  int vi = v;
  sprintf(buf,"%04d,%05d,%04d",fMi,vi,packetnum);
  buf[PKTLEN-1] = 0;  // make sure it's null-terminated

  digitalWrite(LED_BUILTIN, HIGH);   // blink onboard LED with each packet

  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY); // print all Freq,FFT_Value

   
  float measuredvbat = analogRead(VBATPIN);  // sample battery voltage  
  measuredvbat *= (2 * 3.261);  // 2x divider x reference voltage
  measuredvbat /= 1024; // convert to voltage

  
  Serial.print("# ");
  Serial.print(millis()/1000.0,3);
  Serial.print(",");  
  Serial.print(measuredvbat,3);
  Serial.print(",");  
  Serial.println(buf);  // send data string out USB port for debug info
  // Serial.println();  // extra newline
  

  Serial1.print("# ");
  Serial1.print(millis()/1000.0,3);
  Serial1.print(",");  
  Serial1.print(measuredvbat,3);
  Serial1.print(",");  
  Serial1.println(buf);  // send data string out USB port for debug info
  Serial1.println();  // extra newline

  if (doRF) {
    rf95.send((uint8_t *)buf, PKTLEN);
  }
  
  // delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  packetnum++;     // move on to a new packet

} // end loop()


// find pinterpolated position and value of maximum
void findPeak(double *vData, uint16_t bufferSize, double *iPk, double *yPk)
{
  double maxY = 0;
  uint16_t ix = 0;
  for (int i=0;i< bufferSize/2; i++) {  // only bottom half of array has meaning
    if (vData[i] > maxY) {
      maxY = vData[i];
      ix = i;
    }    
  }
  
  if ((ix > 0) && (ix < (bufferSize/2)-1)) { // interpolate, if we aren't at one extreme
    double a,b,c;          // Assume the three points to be on a parabola
    Parabola(ix-1, vReal[ix-1], ix, vReal[ix], ix+1, vReal[ix+1], &a, &b, &c);
    double x = -b/(2*a);   // Peak is at the middle of the parabola
    double y = a*x*x+b*x+c;  // value of y=ax^2 + bx + c at that point
    //Serial.print(ix);        // DEBUG jpb
    //Serial.print(",");
    //Serial.println(x,3);
    *iPk = x;  // return values
    *yPk = y;
  } else {
    *iPk = ix;
    *yPk = maxY;
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    /*
    Serial.print(abscissa, 3);
    if(scaleType==SCL_FREQUENCY)
      Serial.print(","); // was "Hz "
    Serial.print(" ");
    Serial.println(vData[i], 1);
    */
    
    //Serial1.print(abscissa, 2);
    //Serial1.print(","); // was "Hz "    
    Serial1.println(int(vData[i]));
  }
  //Serial.println();  // extra return at end
}

void Parabola(double x1, double y1, double x2, double y2, double x3, double y3, double *a, double *b, double *c)
{
  double reversed_denom = 1/((x1 - x2) * (x1 - x3) * (x2 - x3));

  *a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) * reversed_denom;
  *b = (x3*x3 * (y1 - y2) + x2*x2 * (y3 - y1) + x1*x1 * (y2 - y3)) * reversed_denom;
  *c = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) *reversed_denom;
}
