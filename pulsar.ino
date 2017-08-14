
/* PULSAR

  by DFLD & Laszlo Lebrun
  Open Source License GNU GPL

  Pulsar is an Arduino-Based Radio Transmission System for Measurements and Digital Values 
  It aims to be a powerful and versatile RF analog/digital medium-fast data transmission system. 
  The first throw runs with NRF24L01 modules, the next one will be LoRa. It providees a 1 Sec data transmission pace 
  -optimised for long range: only 16 Bytes payLoad data and reduced transmission speed 
  -safe transmission: echo verification, retry 125ms later and again 500mS later if NACK 
  -real time scheduler: data acquisition occurs with constant, reliable 125mS slices 
  -transmits 4 analog measures 0-1V with parametrizable range begin, range end, value for voltage dividers... 
  -Preprocessing of analog data: 
    a) averages 8 measures @ 125mS into one second value 
    b) provides attack and decay filters to filter out irrelevant spikes
  -option to transmit [A0, A0 min, A0 max, A2] instead of [A1,A2,A3,A4] 
  -reports digital data with 8 bits per input: one bit for every 125mS slice, 
   so you can get a 125mS resolution on digital values even at a slower one second pace. 
   (at the cost of an average latency of ~500mSec)
  -reports measured own Vin/Raw (battery load status) voltage on one byte (200mV resolution, with 5V offset e.g. from 5V to 10V) WIP 
  -reports evaluation of calculated own Vcc voltage on one byte (200mV resolution) WIP 
  -reports evaluation of calculated internal temperature on one byte 
  -reports number of 1st, 2nd transmission retries and failed transmission per running hour WIP 
  -debug mode report on serial monitor 
  -plot mode 
  -can easily be tweaked to send every minute and sample every second.
  -LCD Display on the receiving side
*/

//============ (Fetch libraries) ============
#include "SPI.h" // (SPi Interface)
#include "RF24.h" // (RF24 Library)
#include "printf.h" // (Printf Library)
#include "nRF24L01.h" // (Definitions for symbolics)
#include "Wire.h"
#include "LiquidCrystal_I2C.h"
// #include "LcdBarGraphX.h"

//============ (Options)============
//Operation mode
const byte operationMode     = 2;  // Mode 1 = Sender; Mode 2 = Reciever; Mode 3 = Repeater
// Radio pipe addresses for the 2 nodes to communicate.
byte addresses[][6] = {"0427a", "0427b"}; //define Region#(04),Station#(27),Pipe(a or b)
const boolean debugPlot    = true;   // Output A0 to Serial Plotter for debugging
const boolean debugSerial  = false;  // Output to Serial Monitor for debugging
const boolean modeMaxMin   = false; // modeMaxMin = Transmission of A1, A2, A1min, A1max instead of A1,A2,A3,A4

//============ (Parameters) ============
//Parameters for Radio Transmitter
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 --> 9,10
RF24 radio(9, 10);

//Parameter for Arduino Hardware
const boolean lcdDisplay   = false; // I2C 1602 LCD Display
byte lcdNumCols = 16; // -- number of columns in the LCD

//A0 remains unused since some LCD boards need it.
const byte value1 = 1;            //pin first Analog input
const byte value2 = 2;            //pin second Analog input
const byte value3 = 3;            //pin third Analog input
const byte value4 = 4;            //pin fourth Analog input
const byte vin = 7;               //pin for Vin/10
const byte v33 = 6;               //pin for Vin/10

const byte bin1 = 2;              //pin first Digital input
const byte bin2 = 3;              //pin second Digital input
const byte bin3 = 4;              //pin third Digital input
const byte bin4 = 5;              //pin fourth Digital input

//Voltages
const int internalReferenceVoltage = 1050;  // you may enter the reference voltage measured at pin Aref for increased accuracy.
// Remark: A0 input remains unused, since it my be used by some LCD shields
const int sensorMin1 =  0;        //ADC Begin of range
const int sensorMax1 = 1023;      //ADC End of Range
const int voltRatio1 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)
const int sensorMin2 =  0;        //ADC Begin of range
const int sensorMax2 = 1023;      //ADC End of Range
const int voltRatio2 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)
const int sensorMin3 =  0;        //ADC Begin of range
const int sensorMax3 = 1023;      //ADC End of Range
const int voltRatio3 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)
const int sensorMin4 =  0;        //ADC Begin of range
const int sensorMax4 = 1023;      //ADC End of Range
const int voltRatio4 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)

const int voltRatio5 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)
const int voltRatio6 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)
const int voltRatio7 = 1000;      //ADC Voltage divider/correction factor (1000=100,0% no correction)

//Filter slope tolerances for Analog Inputs
const int sensorRise = 10;       // ADC 10mV/125mS Attack limit
const int sensorFall = 10;       // ADC 10mV/125mS Decay limit

// Thresholds for digital reads
const unsigned int dig00 =  0 ;           // Always 0
const unsigned int dig01 = 1 ;           // Majoritary 0
const unsigned int dig10 = 4 ;           // Majoritary 1
const unsigned int dig11 = 7 ;           // Always 1


// ============ (Variables) ==========
int A1ADC;    // Saves 1. Raw Measurement
int A2ADC;    // Saves 2. Raw Measurement
int A3ADC;    // Saves 3. Raw Measurement
int A4ADC;    // Saves 4. Raw Measurement

int A1Sum;    //Integrator Channel 1
int A1Max;    //Maximum Channel 1
int A1Min;    //Minimum Channel 1
int A2Sum;    //Integrator Channel
int A3Sum;    //Integrator Channel 3
int A4Sum;    //Integrator Channel 3

int A1Average; //Average Channel 1
int A2Average; //Average Channel 2
int A3Average; //Average Channel 3
int A4Average; //Average Channel 4

int A1memory = 0; // Memory for Attack/Decay
int A2memory = 0; // Memory for Attack/Decay
int A3memory = 0; // Memory for Attack/Decay
int A4memory = 0; // Memory for Attack/Decay

byte Byte1;
byte Byte2;
byte Byte3;
byte Byte4;

int Vin;                 // Vin Measurement
int V33;                 // V3,3V Measurement
int Vcc;                 // Internal Vcc evaluation
int Temp;                // Internal Temperature evaluation

unsigned long millisNow;  // track of the millisecond time stamp
unsigned long milliSlice; // Current value of millis()
unsigned long millisMemory = 0;   // Memory for millis()
unsigned long microsNow; // track of the microsecond time stamp
unsigned long start_time; // Start transmission time
unsigned long end_time;   // Stop transmission time

byte slice;               // 10 slices every 100mS to schedule one second

boolean ok;               // Transmission succeeded
byte i;                   // Increment variable
int garbage;              // Dummy variable
byte PALevel = 0;         // 0-3 representing -18dBm, -12dBm, -6dBm and 0dBm

struct dataStruct1 {
  int Int1;   //analog Value 1
  int Int2;   //analog Value 2
  int Int3;   //analog Value 3
  int Int4;   //analog Value 4
  byte Byte1; //digital Values 1
  byte Byte2; //digital Values 2
  byte Byte3; //digital Values 3
  byte Byte4; //digital Values 4
  byte Byte5; //Vcc 5V in steps  of 200mV (5v=250)
  byte Byte6; //V3,3V  in steps  of 200mV (3v=150)
  byte Byte7; //Vin-5V  in steps of 200mV (7V=200)
  byte Byte8; //Temp in °c
} payLoad;

struct dataStruct2 {
  int Int1;   //analog Value 1
  int Int2;   //analog Value 2
  int Int3;   //analog Value 3
  int Int4;   //analog Value 4
  byte Byte1; //digital Values 1
  byte Byte2; //digital Values 2
  byte Byte3; //digital Values 3
  byte Byte4; //digital Values 4
  byte Byte5; //Vcc 5V in steps  of 200mV (5v=250)
  byte Byte6; //V3,3V  in steps  of 200mV (3v=150)
  byte Byte7; //Vin-5V  in steps of 200mV (7V=200)
  byte Byte8; //Temp in °c
} Echo;

// ============ (Initialisation) ============
void setup()
{
  Serial.begin(115200);
  if (debugSerial)
  {
  Serial.println(F("Arduino RF24L01 Transmitter (c)DFLD"));
  }
  // Initialisation Display
  if (lcdDisplay)
  {
    LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
    //    LcdBarGraphX lbg(&lcd, 16);         // Initializing Bargraphs

    lcd.init();                      // initialize the lcd
    lcd.backlight();
    //see https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
    lcd.begin(2, 16); // ***Print a message to the LCD.
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DFLD-dB(A)rduino");
    lcd.setCursor(0, 1);
    lcd.print("by Laszlo Lebrun");
    delay (2000);
    lcd.clear();
  }

  // Initialisation Radio
  radio.begin();
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(7, 7);

  //radio.setAutoAck(false);

  // Reduce the payLoad size -> longer range, better reliability
  // radio.setPayloadSize(8); // 4 words for Analog values * digital on MSB
  radio.setPayloadSize(16); // 4 words+8 Bytes

  //Power Saving instructions
  //radio.powerDown ();
  radio.powerUp ();

  //Set Channel 0-125
  radio.setChannel(107); // Channel Freq = 2,4Ghz + 1Mhz * channel

  //Set Power level
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices.
  radio.setPALevel(RF24_PA_HIGH);
  // radio.setPALevel(RF24_PA_HIGH); //RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX

  //Set Data Rate
  radio.setDataRate(RF24_250KBPS); // RF24_1MBPS , or RF24_2MBPS, or RFA_24_250KBPS

  //Get Channel
  //byte radioChannel = radio.getChannel()

  //Get PA Level
  //byte PALevel = radio.getPALevel () // 0-3 representing -18dBm, -12dBm, -6dBm and 0dBm

  //Get isPVariant (nRF24L01+ or not)
  boolean isPVariant = radio.isPVariant();

  // Pre-load an ack-paylod into the FIFO buffer for pipe 1
  //radio.writeAckPayload(1,&counter,1);

  //see more config @ http://tmrh20.github.io/RF24/classRF24.html

  // This opens two pipes for the two nodes to communicate back and forth.
  // Open 'our' pipe for writing Open the 'other' pipe for reading, in position #1
  // (we can have up to 5 pipes open for reading)

  // Open a writing and reading pipe on each radio, with opposite addresses
  if (operationMode == 1) {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  } else {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }
  // Start the radio listening for data
  radio.startListening();

  // Dump the configuration of the rf unit for debugging
  // radio.printDetails();

  //ADC preparations
  analogReference (INTERNAL); //Analoge inputs i the range 0..1023 mv: replaces analogReference(DEFAULT) 0..5V
  garbage = analogRead (A6); //Read once for peanuts: executes immediately analogReference.
  delay(50);

} // Ende Void Setup Loop

//============ Running program ===============
void loop(void)
{
  switch (operationMode)
  {
    //************* Sender Mode ********************
    case 1:

      //**** Scheduler, generates 8 slices@125mS (0..7) and within each slice, 125 millislices (0..124) ****

      // Forming millislice number
      milliSlice = millis() % 125UL; //We make a modulo division with the milliseconds.
      //The result will be directly the expected milliSlice value, that behaves like a chainsaw between 0..124

      // Forming slice number
      if (millisMemory > milliSlice) //We trigger on the falling edge of the chainsaw.
      {
        ++slice;                      // Next 125mS increase the slice number
        if (slice > 7) {
          slice = 0;
        };

        //============== Running every 125mS ================
        //========= This part must be finished within ~20mS ===========
        //   start_time = millis();
        // Gathering Analog values
        garbage = analogRead(value1); // read once for peanuts, that prepares the ADC
        A1ADC = map(analogRead(value1), sensorMin1, sensorMax1, 0, internalReferenceVoltage);
        A1ADC = constrain (A1ADC, (A1memory - sensorFall), (A1memory + sensorRise)); // Attack/Decay limits
        A1Sum = A1Sum + A1ADC;
        A1memory = A1ADC;
        A1Max = max(A1ADC, A1Max);
        A1Min = min(A1ADC, A1Min);
        delayMicroseconds(100);

        garbage = analogRead(value2); // read once for peanuts, that prepares the ADC
        A2ADC = map(analogRead(value1), sensorMin2, sensorMax2, 0, internalReferenceVoltage);
        A2Sum = A2Sum + A2ADC;
        A2memory = A2ADC;
        delayMicroseconds(100);

        garbage = analogRead(value3); // read once for peanuts, that prepares the ADC
        A3ADC = map(analogRead(value3), sensorMin3, sensorMax3, 0, internalReferenceVoltage);
        A3ADC = constrain (A3ADC, (A3memory - sensorFall), (A3memory + sensorRise));  // Attack/Decay limits
        A3Sum = A3Sum + A3ADC;
        A3memory = A3ADC;
        delayMicroseconds(100);

        garbage = analogRead(value4); // read once for peanuts, that prepares the ADC
        A4ADC = map(analogRead(value4), sensorMin4, sensorMax4, 0, internalReferenceVoltage);
        A4ADC = constrain (A4ADC, (A4memory - sensorFall), (A4memory + sensorRise));  // Attack/Decay limits
        A4memory = A4ADC;
        A4Sum = A4Sum + A4ADC;

        // Gathering digital values
        // 1 bit for every slice, MSB first
        if (digitalRead(bin1)) {
          Byte1++;
        }
        if (digitalRead(bin2)) {
          Byte2++;
        }
        if (digitalRead(bin3)) {
          Byte3++;
        }
        if (digitalRead(bin4)) {
          Byte4++;
        }
        //all values will be shifted one bit left to make room for the next slice
        Byte1 = Byte1 << 1;
        Byte2 = Byte2 << 1;
        Byte3 = Byte3 << 1;
        Byte4 = Byte4 << 1;

        //    end_time = millis();

        //=========== 8 slices running every second ===========

        switch (slice)
        {
          case 0:
            // Do the math and compute payLoad
            //
            A1Average = A1Sum / 8;
            A2Average = A2Sum / 8;
            A3Average = A3Sum / 8;
            A4Average = A4Sum / 8;
            if (modeMaxMin)
            {
              payLoad.Int1 = A1Average;
              payLoad.Int2 = A1Min;
              payLoad.Int3 = A1Max;
              payLoad.Int4 = A4Average;
            }
            else
            {
              payLoad.Int1 = A1Average;
              payLoad.Int2 = A2Average;
              payLoad.Int3 = A3Average;
              payLoad.Int4 = A4Average;
            }
            //reinitialize values for the next averaging
            A1Sum = 0;
            A2Sum = 0;
            A3Sum = 0;
            A4Sum = 0;
            A1Max = 0;
            A1Min = 1023;
            Byte1 = 0;
            Byte2 = 0;
            Byte3 = 0;
            Byte4 = 0;

            // Calculating own 5Volt power supply value (to compensate errors in ADC in case of  analogReference (DEFAULT)
            for (int i = 0; i <= 3; i++) Vcc = getBandgap(); //4 readings seem required for stable value?
            // delay(5);
            Vcc = Vcc / 10;

            // Getting Temp
            Temp = getIntTemp();

            //Getting V33
            V33 = analogRead(vin);
            V33 = map(V33, 0, 1023, 0, internalReferenceVoltage) * voltRatio6 / 1000;

            // Getting Vin
            Vin = analogRead(vin);
            Vin = map(Vin, 0, 1023, 0, internalReferenceVoltage) * voltRatio7 / 1000;

            payLoad.Byte1 = Byte1;
            payLoad.Byte2 = Byte2;
            payLoad.Byte3 = Byte3;
            payLoad.Byte4 = Byte4;

            payLoad.Byte5 = Vin - 50; //Vin -5v in steps of 200mV
            payLoad.Byte6 = V33; //V3,3v in steps of 200mV
            payLoad.Byte7 = Vcc ; //Vcc in steps of 200mV
            payLoad.Byte8 = Temp; //Temp in 0,1°C
            break;

          case 1:
            // Send payLoad
            // Reactivate Radio
            radio.powerUp();
            // Wait 5mS to get radio ready
            delay (5);
            // Send current time.  This will block until complete
            radio.stopListening(); // First, stop listening so we can talk.
            start_time = millis();
            ok = radio.write( &payLoad, sizeof(payLoad));
            end_time = millis();
            radio.startListening();
            radio.powerDown();
            break;

          case 2:
            // If send failed, resend payLoad
            if (!ok)
            {
              if (debugSerial)
              {
                Serial.print(" 1st Resend ");
              }
              // Reactivate Radio
              radio.powerUp();
              // Wait 5mS to get radio ready
              delay (5);
              // Send current time.  This will block until complete
              radio.stopListening(); // First, stop listening so we can talk.
              start_time = millis();
              ok = radio.write( &payLoad, sizeof(payLoad));
              end_time = millis();
              radio.startListening();
              radio.powerDown();
            } //end OK
            break;

          case 3:
            break;

          case 4:
            // If last send failed, resend a last time payLoad
            if (!ok)
            {
              if (debugSerial)
              {
                Serial.print(" 2nd Resend ");
              }
              // Reactivate Radio
              radio.powerUp();
              // Wait 5mS to get radio ready
              delay (5);
              // Sending a last time the payload
              radio.stopListening(); // First, stop listening so we can talk.
              start_time = millis();
              ok = radio.write( &payLoad, sizeof(payLoad));
              end_time = millis();
              radio.startListening();
              radio.powerDown();
            } //end OK
            break;

          case 7:
            // debugging information
            if (debugPlot)
            {
              Serial.println(A1Average);
            } //end Debug Plot
            if (debugSerial)
            {
              Serial.print("Start time = ");   Serial.print(start_time);
              Serial.print(" | End time = ");  Serial.print(end_time);
              Serial.print(" | Runtime = ");   Serial.print(end_time - start_time);
              Serial.print("  |  Vcc (10mV) =  "); Serial.print(Vcc);
              Serial.print("  |  Vin(10mV) =  ");  Serial.print(Vin);
              Serial.print("  |  Temp(0,1°C) =  "); Serial.println(Temp);

              Serial.print("A1Min(10mV) =  "); Serial.print(A1Min);
              Serial.print(" | A1(10mV) =  "); Serial.print(A1Average);
              Serial.print(" | A1Max(10mV) =  "); Serial.print(A1Max);
              Serial.print(" | A2(10mV) =  ");  Serial.print(A2Average);
              Serial.print(" | A3(10mV) =  ");  Serial.print(A3Average);
              Serial.print(" | A4(10mV) =  ");  Serial.println(A4Average);

              Serial.print("Bin1 =  ");
              for (i = 0; i < 8; i++) {
                Serial.print (bitRead(Byte1, i));
              };
              Serial.print(" | Bin2 =  ");
              for (i = 0; i < 8; i++) {
                Serial.print (bitRead(Byte2, i));
              };
              Serial.print(" | Bin3 =  ");
              for (i = 0; i < 8; i++) {
                Serial.print (bitRead(Byte3, i));
              };
              Serial.print(" | Bin4 =  ");
              for (i = 0; i < 8; i++) {
                Serial.print (bitRead(Byte4, i));
              };
              Serial.println();
              Serial.println();
            } //end Debug Serial
            break;
        } // end Slice Switch
      } // end 125mS
      millisMemory = milliSlice; // Comparison value to detect the falling edge.
      break;

    //************* Receiver Mode ********************
    case 2:  // Receiver mode
      radio.startListening();


      if ( radio.available()) {
        // Variable for the received timestamp
        start_time = millis();
        while (radio.available())
        { // While there is data ready
          radio.read( &payLoad, sizeof(payLoad));             // Get the payload
        } // End While
        A1Average =  payLoad.Int1;
        A2Average =  payLoad.Int2;
        A3Average =  payLoad.Int3;
        A4Average =  payLoad.Int4;
        Byte1 = payLoad.Byte1;
        Byte2 =  payLoad.Byte2;
        Byte3 =  payLoad.Byte3;
        Byte4 =  payLoad.Byte4 ;
        Vin =  payLoad.Byte5 + 50; //Vin -5v in steps of 200mV
        V33 = payLoad.Byte6 ;      //V3,3v in steps of 200mV
        Vcc = payLoad.Byte7;       //Vcc in steps of 200mV
        Temp =  payLoad.Byte8;     //Temp in 0,1°C
        end_time = millis();

        // debugging information
        if (debugPlot)
        {
          Serial.println(A1Average);
        } //end Debug Plot
        if (debugSerial)
        {
          Serial.print("Start time = "); Serial.print(start_time);
          Serial.print(" | End time = ");  Serial.print(end_time);
          Serial.print(" | Runtime = ");   Serial.print(end_time - start_time);
          Serial.print("  |  Vcc (10mV) =  "); Serial.print(Vcc);
          Serial.print("  |  Vin(10mV) =  ");  Serial.print(Vin);
          Serial.print("  |  Temp(0,1°C) =  "); Serial.println(Temp);

          Serial.print("A1Min(10mV) =  "); Serial.print(A1Min);
          Serial.print(" | A1(10mV) =  "); Serial.print(A1Average);
          Serial.print(" | A1Max(10mV) =  "); Serial.print(A1Max);
          Serial.print(" | A2(10mV) =  ");  Serial.print(A2Average);
          Serial.print(" | A3(10mV) =  ");  Serial.print(A3Average);
          Serial.print(" | A4(10mV) =  ");  Serial.println(A4Average);

          Serial.print("Bin1 =  ");
          for (i = 0; i < 8; i++) {
            Serial.print (bitRead(Byte1, i));
          };
          Serial.print(" | Bin2 =  ");
          for (i = 0; i < 8; i++) {
            Serial.print (bitRead(Byte2, i));
          };
          Serial.print(" | Bin3 =  ");
          for (i = 0; i < 8; i++) {
            Serial.print (bitRead(Byte3, i));
          };
          Serial.print(" | Bin4 =  ");
          for (i = 0; i < 8; i++) {
            Serial.print (bitRead(Byte4, i));
          };
          Serial.println();
          Serial.println();
        } // End If Debug Serial
        if (lcdDisplay)
        {
          /*        lcd.begin(2,16); // ***Print a message to the LCD.
                    lcd.setCursor(0, 0);
                    lcd.print("Print 1st line");
                    lcd.setCursor(0, 1);
                    lcd.print("Print 2nd line");
                    lcd.setCursor(0, 0);
                    A1 = float(A1 / 10.0);
                    lcd.print(A1, 1);
                    lbg.drawValue(map(max(A1, 200), 200, 1000, 0, 1024), 1024); //Balkendiagramm von 20 bis 100 (1 Pixel=1dB)
          */
        } //End LCD Display
      }
      else
      {
        // Trigger message after 1100ms (failed), 1250mS (1st retry failed as well), 1500mS (2nd retry failed) , 1700mS (Measure lost)
        // Count failed events.
      } // End If Radio Available
      break;

    //************* Repeater Mode ********************
    case 3: //Repeater Mode
      radio.startListening();
      if ( radio.available()) {
        // Reading payload
        while (radio.available()) {                           // While there is data ready
          radio.read( &payLoad, sizeof(payLoad));             // Get the payload
        } // End While
        radio.stopListening(); // First, stop listening so we can talk.
        delay(5);
        // Resending payload
        ok = radio.write( &payLoad, sizeof(payLoad));
        radio.startListening();
      } //End If Radio Available
      break;
  } //end operationMode Switch
} //end Loop.

// ============ Function modules ============

// *** Function to obtain chip's actual Vcc voltage value, using internal bandgap reference ***
// Function to obtain chip's actual Vcc voltage value, using internal bandgap reference
// This provides ability to maintain A/D calibration with changing Vcc in case of analogReference (DEFAULT)
// For 328 chip only, mod needed for 1280/2560 chip

int getBandgap(void)
{
  // REFS1 REFS0          --> 0 1, AVcc internal ref.
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
  // Start a conversion
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
  // Scale the value
  unsigned int results = (((internalReferenceVoltage * 1024L) / ADC) + 5L) ;
  return results;
}

// *** Function to obtain chip's actual internal temperatur, using internal channel 8 ***
// with the internal reference of 1.1V. Channel 8 can not be selected with the analogRead function yet.
// Set the internal reference and mux.

int getIntTemp(void)
{
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  unsigned int wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  // The returned temperature is in degrees Celsius.

  unsigned int ChipTemp = (wADC - 324.31 ) / 0.122;
  return (ChipTemp);
}
