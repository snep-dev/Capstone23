//libraries
#include <Keypad.h>
#include <SoftwareSerial.h>
#include "XBee.h"
#include "queue.h"
#include "XBee2.h"
//Defines
#define TONE_PERIOD 10        //period in ms
#define COMMAND_PREFIX1 'x01'
#define COMMAND_PREFIX2 'x02'
//Xbee declarations
XBee xbee;
XBee2 xbee2 = XBee2();
Queue RxQ;
SoftwareSerial sserial(10,11);
//Declaration of pins
const byte input_pin = A8;     //input pin
const int MAX_RESULTS = 250;   //results resolution
const int switch_pin1 = 49;    //analog switch
const int switch_pin2 = 50;    //analog switch
const int switch_pin3 = 51;
const int switch_pin4 = 52;
const byte led_1 = 22;
const byte led_2 = 24;
const byte led_3 = 26;
const byte led_4 = 28;
const byte led_5 = 30;
unsigned long lastPulseStart;

//variables
//volatile int results [MAX_RESULTS]; //storage array
volatile int resultNumber;          //results counter
volatile bool adc_flag;             //flag for raising ADC check
volatile bool wireless_flag;        //flag for raising wireless check
const int timer1_counter = 65223;   //preload timer 65536-8MHz/256/100Hz
//volatile short monitor[MAX_RESULTS]; //before conversion
volatile float voltage[MAX_RESULTS]; //voltage array to be simulated

float my_array[50] = {-0.00820449,
0.001286776,
0.030581948,
0.094492946,
0.193476307,
0.278691748,
0.294300358,
0.221521503,
0.121859141,
-0.176957823,
-0.497920277,
-0.721377064,
-1.319613497,
0.444314649,
2.5,
0.178016886,
-0.34979431,
-0.148648266,
-0.06322777,
-0.045944297,
-0.015419382,
-0.185267419,
-0.504496799,
0.3622965158,
0.6537506890,
0.6550967642,
1.0742312267,
0.5959289681,
0.28013328,
0.179204593,
0.080144752,
0.0000967168,
-0.051678511,
-0.080736975,
-0.093971881,
-0.097467344,
-0.096479137,
-0.094590575,
};

//const int ARRAY_SIZE = sizeof(my_array) / sizeof(my_array[0]);
char str_array[50][20]; // Maximum length of each string is 19 characters (including null terminator)

//pin controls
int output_pin = 12;    //stim/sim output pin
int pin_button1 = 1;    //keypad button to enable EMG
int pin_button2 = 2;    //keypad button to enable ECG

//keypad setup
const byte ROWS = 4;    //four rows
const byte COLS = 3;    //three columns

//sample frequencies
float sampleFreq = 1000;

//defining the symbols on the buttons on the keypads
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'#', '0', '#'}
};

byte rowPins[ROWS] = {9,8,7,6};
byte colPins[COLS] = {5,4,3};
//creating a keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
//initialize User input keypad

char bPressedKey; //last pressed key
char bLastKnownPressedKey; //last known pressed key
int counter = 0; //counter variable

void setup() {
    Serial.begin(9600);  //XBee/UART1/pins 0 and 1
    sserial.begin(9600); //XBEE serial
    xbee2.setSerial(sserial); //xbee 2
    adc_flag = false;
    wireless_flag = false;
    //digital rising edge interrupts for keypad pins
    attachInterrupt(digitalPinToInterrupt(pin_button1), EMGMode, RISING); 
    attachInterrupt(digitalPinToInterrupt(pin_button2), ECGMode, RISING);    
  
    pinMode(12, OUTPUT);        //output pin for stimulator signals
    pinMode(output_pin, OUTPUT); //output pin for simulator signals
    pinMode(led_1, OUTPUT);
    pinMode(led_2, OUTPUT);
    pinMode(led_3, OUTPUT);
    pinMode(led_4, OUTPUT);
    pinMode(led_5, OUTPUT);
    //analog switches 
    pinMode(switch_pin1, OUTPUT);   //pin1
    pinMode(switch_pin2, OUTPUT);   //pin2
  
    //initialize timer1
    noInterrupts();               //disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
  
    TCNT1 = timer1_counter;       //preload timer
    TCCR1B |= (1 << CS12);        // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);       // enable timer overflow interrupt
    interrupts();                 // enables all interrupts

    bPressedKey = NULL;
    bLastKnownPressedKey = 't';
}

//TIMER INTERRUPT ROUTINE
ISR(TIMER1_OVF_vect)  {          // interrupt service routine, runs every 1/100Hz seconds
  TCNT1 = timer1_counter;        //preload timer

  adc_flag = true;               //sets ADC to TRUE 
  wireless_flag = true;          //sets Wireless to TRUE
}


//ECG
void ECGMode()  {
  sampleFreq = 200;                   //vary the sampling frequencies to try to remove noise
  //Serial.println ("ECG Mode enabled");
  digitalWrite(switch_pin1, HIGH);      //TURN SWITCH 1 ON FOR ECG
  digitalWrite(switch_pin2, LOW);       //TURN SWITCH 2 OFF FOR ECG

  digitalWrite(led_1, LOW);
  digitalWrite(led_2, HIGH);
  digitalWrite(led_3, LOW);
  digitalWrite(led_4, LOW);
  digitalWrite(led_5, LOW);
  //taking ECG input
}

//EMG
void EMGMode()  {
  sampleFreq = 1000;                  //vary the sampling frequencies to try to remove noise
  //Serial.println ("EMG Mode enabled");
  digitalWrite(switch_pin1, LOW);      //TURN SWITCH 1 OFF FOR EMG
  digitalWrite(switch_pin2, HIGH);     //TURN SWITCH 2 ON FOR EMG

  digitalWrite(led_1, HIGH);
  digitalWrite(led_2, LOW);
  digitalWrite(led_3, LOW);
  digitalWrite(led_4, LOW);
  digitalWrite(led_5, LOW);
  //generation of EMGs
  //taking EMG input
}

//STIMULUS
void Stim() {
  Serial.println ("Stimulus Mode Enabled");

  digitalWrite(switch_pin3, HIGH);      //TURN SWITCH 3 ON FOR STIM
  digitalWrite(switch_pin4, LOW);       //TURN SWITCH 4 OFF FOR STIM

  digitalWrite(led_1, LOW);
  digitalWrite(led_2, LOW);
  digitalWrite(led_3, HIGH);
  digitalWrite(led_4, LOW);
  digitalWrite(led_5, LOW);

  /*pulseStart = millis(); //record the start time of the pulse
  while (millis() - pulseStart < pulseDuration) {
    //generate 20 V p-p signal that gets outputted continuosly
    tone(12, 50); //generates square
  }
  noTone(12); //turn off the signal after the pulse duration has elapsed*/
}

void Output() { //sends data to a digital output pin
  Serial.println("Outputting Stored Data");
  //analog switch logic here
  digitalWrite(switch_pin3, LOW);      //TURN SWITCH 3 OFF FOR SIM
  digitalWrite(switch_pin4, HIGH);     //TURN SWITCH 4 ON FOR SIM

  digitalWrite(led_1, LOW);
  digitalWrite(led_2, LOW);
  digitalWrite(led_3, LOW);
  digitalWrite(led_4, HIGH);
  digitalWrite(led_5, LOW);

}


void ADC_func()  { //need to change the duration of this so that the arduino can change functions on the fly
    voltage[counter] = (analogRead(input_pin)*(5.0/1023));    //reading analog pin

    Serial.print("\t");
    Serial.println(voltage[counter]);                //plots voltage
    counter++;
   //turns ADC back on
}

void send_from_PC() //wireless transmission protocol
{
  delay(5);
  int queueLen = 0;
  int delPos = 0;
  int anArray[50];
  int arraylen = 0;
  
  while (sserial.available() > 0)
  {
      unsigned char in = (unsigned char)sserial.read();
      if (!RxQ.Enqueue(in))
      {
          break;
      }
  }

  queueLen = RxQ.Size();
  for (int i=0;i<queueLen;i++)
  {
      if (RxQ.Peek(i) == 0x7E)
      {
          unsigned char checkBuff[Q_SIZE];
          unsigned char msgBuff[Q_SIZE];
          int checkLen = 0;
          int msgLen = 0;
          float myArray [100];
          int index = 0;

          checkLen = RxQ.Copy(checkBuff, i);
          msgLen = xbee.Receive(checkBuff, checkLen, msgBuff);
          if (msgLen > 0)
          {
              unsigned char outMsg[Q_SIZE];
              unsigned char outFrame[Q_SIZE];
              int frameLen = 0;
              int addr = ((int)msgBuff[4] << 8) + (int)msgBuff[5];
 
              // 10 is length of "you sent: "
              memcpy(outMsg, "          ", 10);
              // len - (9 bytes of frame not in message content)
              memcpy(&outMsg[10], &msgBuff[8], msgLen-9);    

              float x = atof(outMsg);
              String msg = outMsg;

              if (msg == COMMAND_PREFIX1)
              {
                // 10 + (-9) = 1 more byte in new content than in previous message
                frameLen = xbee.Send(outMsg, msgLen+1, outFrame, addr);
                sserial.write(outFrame, frameLen);
                arraylen += 1;
                i += msgLen;
                delPos = i;

                myArray[index] = x;
                index += 1;
                
                for (int i = 0; i < index; i++) 
                {  // Print the new array
                  Serial.println(myArray[i],9);
                }
              }
          }
          else
          {
              if (i>0)
              {
                  delPos = i-1;
              }
          }
      }
  }
  RxQ.Clear(delPos);
}

void record_from_mcu() //wireless transmission protocol
{
  delay(5);
  int queueLen = 50;
  int delPos = 0;
  int anArray[50];
  int arraylen = 0;
  
  while (sserial.available() > 0)
  {
      unsigned char in = (unsigned char)sserial.read();
      if (!RxQ.Enqueue(in))
      {
          break;
      }
  }

  queueLen = RxQ.Size();
  for (int i=0;i<queueLen;i++)
  {
      if (RxQ.Peek(i) == 0x7E)
      {
          unsigned char checkBuff[Q_SIZE];
          unsigned char msgBuff[Q_SIZE];
          int checkLen = 0;
          int msgLen = 0;
          float myArray [100];
          int index = 0;

          checkLen = RxQ.Copy(checkBuff, i);
          msgLen = xbee.Receive(checkBuff, checkLen, msgBuff);
          if (msgLen > 0)
          {
              unsigned char outMsg[Q_SIZE];
              unsigned char outFrame[Q_SIZE];
              int frameLen = 0;
              int addr = ((int)msgBuff[4] << 8) + (int)msgBuff[5];
 
              // 10 is length of "you sent: "
              memcpy(outMsg, "          ", 10);
              // len - (9 bytes of frame not in message content)
              memcpy(&outMsg[10], &msgBuff[8], msgLen-9);    

              float x = atof(outMsg);
              String msg = outMsg;

              if (msg == COMMAND_PREFIX2)
              {
               
                for (int i = 0; i < 50; i++) 
                {
                  dtostrf(my_array[i], 16, 15, str_array[i]); // Convert float to string with 10 total characters, including 9 decimal places
                }

                for (int i = 0; i < 50; i++) 
                {
                  int messageLength = strlen(str_array[i]);
                  byte messageBytes[messageLength];
                  strncpy((char*)messageBytes, str_array[i], messageLength);

                  Tx16Request tx = Tx16Request(0xFFFF, 0x01, (uint8_t*) messageBytes, messageLength, 0x00);
                  xbee2.send(tx);

                  delay(125);
                  Serial.println("transmitting");
                }
                
              }
          }
          else
          {
              if (i>0)
              {
                  delPos = i-1;
              }
          }
      }
  }
  RxQ.Clear(delPos);
}

void loop() {
  //keypad
  char bPressedKey = keypad.getKey();
  if ((bPressedKey != NULL) && (bLastKnownPressedKey != bPressedKey))
  {
    if (bLastKnownPressedKey == ('1' ) || bLastKnownPressedKey == ('2'))  {
    counter = 0;
    }
    bLastKnownPressedKey = bPressedKey;
  }
//   // Serial.println("hi");
//  l1: if (keys)  { //check for valid key
//     Serial.print("Key pressed: ");
//     Serial.println(keys);
      switch (bLastKnownPressedKey) {

        case '1': //EMG mode
          EMGMode();
          noTone(12);
          if (adc_flag && counter <= MAX_RESULTS) {
            adc_flag = false;
            ADC_func();
            //Serial.println(voltage[counter]);
            Serial.println(counter);
            if (wireless_flag && counter == MAX_RESULTS)  {//write an if function here to check IF whatever = max_results, if true, export data with function, then reset to 0 
             wireless_flag = false;
             Serial.println("Array filled. Data Exported");
             record_from_mcu();
            }
          }
         break;

        case '2': //ECG mode
          ECGMode();
          noTone(12);
          if (adc_flag && counter <= MAX_RESULTS) {
            adc_flag = false;
            ADC_func();
            Serial.println(counter);
            if (wireless_flag && counter == MAX_RESULTS)  {//write an if function here to check IF whatever = max_results, if true, export data with function, then reset to 0
             Serial.println("Array filled. Data Exported");
             record_from_mcu();
            }
          }
        break;

        case '4': //Stim mode
          Stim();
          tone(12,50);
        break;

        case '5': //SIM mode
          //Output();
          digitalWrite(switch_pin3, LOW);      //TURN SWITCH 3 OFF FOR SIM
          digitalWrite(switch_pin4, HIGH);     //TURN SWITCH 4 ON FOR SIM
          digitalWrite(led_1, LOW);
          digitalWrite(led_2, LOW);
          digitalWrite(led_3, LOW);
          digitalWrite(led_4, HIGH);
          digitalWrite(led_5, LOW);
          send_from_PC();
          tone(12, 50);
         // analogWrite(12, 100);
          //analogWrite(12, voltage[counter]);
          /*Output();
          noTone(12);
          if (counter <= MAX_RESULTS) {
            analogWrite(simout_pin, voltage[counter]);
            Serial.println(voltage[counter]);
             counter++;
          }*/

        break;

        case '0': //EXIT/STOP RECORDING
          noTone(12);
          digitalWrite(led_1, LOW);
          digitalWrite(led_2, LOW);
          digitalWrite(led_3, LOW);
          digitalWrite(led_4, LOW);
          digitalWrite(led_5, HIGH);
        break;

      //anything else
      default:
        break;
 }
} 