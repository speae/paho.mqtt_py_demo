// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// ? This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// ? The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// ? The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// CONFIGURATION on the hoverboard side in config.h:
// ? Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// ? Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
//#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define HOVER_SERIAL_BAUD   38400      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(2,3);        // RX, TX
SoftwareSerial HoverSerial_front(2,3);        // RX, TX
SoftwareSerial HoverSerial_back(4,5);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  steer2;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand2;
SerialCommand2 Command2;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial_front.begin(HOVER_SERIAL_BAUD);
  HoverSerial_back.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

char temp;

// 
int g_speed = 0;
int g_speed2 = 0;

// 
int g_steer = 0;
int g_steer2 = 0;

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSteer2, int16_t uSpeed, int16_t uSpeed2)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  Command2.start    = (uint16_t)START_FRAME;
  Command2.steer2    = (int16_t)uSteer2;
  Command2.speed    = (int16_t)uSpeed2;
  Command2.checksum = (uint16_t)(Command2.start ^ Command2.steer2 ^ Command2.speed);

  // Write to Serial
  HoverSerial_front.write((uint8_t *) &Command, sizeof(Command)); 
  HoverSerial_back.write((uint8_t *) &Command2, sizeof(Command2)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    ////if (HoverSerial_front.available()) {
    ////    incomingByte    = HoverSerial_front.read();                                   // Read the incoming byte
    ////    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    ////}
    ////else 
    {
/////// start     
      while(Serial.available() <=0) // wait for incoming serial data
      {
          return;
      }
      temp = (char)Serial.read();
      switch(temp)
      {
        //  
        case '1': //'1' 
            Serial.println("1");
            g_speed = 30;
            break;
        
        case '2': //'2'  
            Serial.println("2");
            g_speed = 60;
            break;
        
        case '3': //'3'
            Serial.println("3");
            g_speed = 90;        
            break;
        
        case '4': //'4'
            Serial.println("4");
            g_speed = 120;
            break;
        
        case '5': //'5'
            Serial.println("5");
            g_speed = 150;
            break;

        case '6': //'6'
            Serial.println("6");
            g_speed = 180;
            break;
                
        case '7': //'7'        
            Serial.println("7");
            g_speed = 210;
            break;
        
        case '8': //'8'
            Serial.println("8");
            g_speed = 240;
            break;
        
        case '9': //'9'
            Serial.println("9");
            g_speed = 270;
            break;
        
        case '0': //'0'
            Serial.println("0");
            g_speed = 0;
            g_steer = 0;
            break;

        // 
        case 'a': //'a'
            Serial.println("a");
        
            g_speed += 10;
            g_speed2 += 10;
            if(g_speed > SPEED_MAX_TEST)
               g_speed = SPEED_MAX_TEST;
            break;

        // 
        case 'b': //'b'
            Serial.println("b");
            g_speed -= 10;
            g_speed2 -= 10;
            if(g_speed < -SPEED_MAX_TEST)
                g_speed = -SPEED_MAX_TEST;
            break;

        // 
        case 'c': //'c'
            g_steer += 10;
            Serial.println("c");
            break;

        // 
        case 'd': //'d'
            g_steer -= 10;
            Serial.println("d");
            break;

        // 
        case 'e': //'e'
            g_steer = 0;
            Serial.println("e");
            break;
//////////////////////////////////////////        
        case 'f': //'f'
            Serial.println("f"); 
            g_steer = 400;
            g_steer2 = 400;
            g_speed = 10;
            g_speed2 = -10;
            
            break;
        //
        case 'g': //'g'
            Serial.println("g");
            g_steer = 80;
            g_steer2 = 80;
            g_speed = 50;
            g_speed2 = 50;
            break;

        //
        case 'h': //'h'
            Serial.println("h");
            g_steer = 0;
            g_steer2 = 0;
            g_speed = 80;
            g_speed2 = 80;
            
            break;

        //
        case 'i': //'i'
            Serial.println("i");
            g_steer = 0;
            g_speed = -180;
            break;

        //stop
        case 'j': //'j'
            Serial.println("j");
            g_steer = 0;
            g_steer2 = 0;
            g_speed = 0;
            g_speed2 = 0;
            break;
        
        case 'k': //'k'
            Serial.println("k");
            g_steer = 0;
            g_steer2 = 0;
            g_speed = -80;
            g_speed2 = -80;
            break;
        
        case 'l': //'l'
            Serial.println("l");
            break;
        
        case 'm': //'m'
            Serial.println("m");
            break;
        
        case 'n': //'n'
            Serial.println("n");
            break;
        
        case 'o': //'o'
            Serial.println("o");
            break;
        
        case 'p': //'p'
            Serial.println("p");
            break;
        
        case 'r': //'r'
            Serial.println("r");
            break;
        
        case 's': //'s'
            Serial.println("s");
            break;
        
        case 't': //'t'
            Serial.println("t");
            break;
        
        case 'u': //'u'
            Serial.println("u");
            break;
        
        case 'v': //'v'
            Serial.println("v");
            break;
        
        case 'w': //'w'
            Serial.println("w");
            break;
        
        // A(0.9), B(0.8), C(0.7), D(0.6) 

        case 'A': //'A'
            Serial.println("A");
            g_steer = 300;
            g_steer2 = 300;
            //g_speed = 10;
            //g_speed2 = -10;
            break;
            
        case 'B': //'B'
            Serial.println("B");
            g_steer = 270;
            g_steer2 = 270;
            break;

        case 'C': //'C'
            Serial.println("C");
            g_steer = 240;
            g_steer2 = 240;
            break;

        case 'D': //'D'
            Serial.println("D");
            g_steer = 210;
            g_steer2 = 210;
            break;

        // E(0.4), F(0.3), G(0.2), H(0.1) 
        case 'E': //'E'
            Serial.println("E");
            g_steer = -210;
            g_steer2 = -210;
            break;

        case 'F': //'F'
            Serial.println("F");
            g_steer = -240;
            g_steer2 = -240;
            break;

        case 'G': //'G'
            Serial.println("G");
            g_steer = -270;
            g_steer2 = -270;
            break;

        case 'H': //'H'
            Serial.println("H");
            g_steer = -300;
            g_steer2 = -300;
            break;

        
//        case 'A': //'A'
//            Serial.println("A");
//            g_steer = 200;
//            g_steer2 = 200;
//            //g_speed = 10;
//            //g_speed2 = -10;
//            break;
//            
//        case 'B': //'B'
//            Serial.println("B");
//            g_steer = 50;
//            g_speed = -70;
//            break;
//
//        case 'C': //'C'
//            Serial.println("C");
//            g_steer = 40;
//            g_speed = -100;
//            break;
//
//        case 'D': //'D'
//            Serial.println("D");
//            g_steer = 30;
//            g_speed = -150;
//            break;
//
//        // E(0.4), F(0.3), G(0.2), H(0.1) 
//        case 'E': //'E'
//            Serial.println("E");
//            g_steer = -30;
//            g_speed = -150;
//            break;
//
//        case 'F': //'F'
//            Serial.println("F");
//            g_steer = -40;
//            g_speed = -100;
//            break;
//
//        case 'G': //'G'
//            Serial.println("G");
//            g_steer = -50;
//            g_speed = -70;
//            break;
//
//        case 'H': //'H'
//            Serial.println("H");
//            g_steer = -60;
//            g_speed = -30;
//            break;
       
        default : //default :
            break;
        
        /////// end
      //return;
      }
    }

      // If DEBUG_RX is defined print all incoming bytes
      #ifdef DEBUG_RX
          Serial.print(incomingByte);
          return;
      #endif

      // Copy received data
      if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
          p       = (byte *)&NewFeedback;
          *p++    = incomingBytePrev;
          *p++    = incomingByte;
          idx     = 2;  
      } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
          *p++    = incomingByte; 
          idx++;
      } 
    
      // Check if we reached the end of the package
      if (idx == sizeof(SerialFeedback)) {
          uint16_t checksum;
          checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                              ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
  
          // Check validity of the new data
          ////if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
              // Copy the new data
              ////memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
  
              // Print data to built-in Serial
              ////Serial.print("1: ");   Serial.print(Feedback.cmd1);
              ////Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
              ////Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
              ////Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
              ////Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
              ////Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
              ////Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
          ////} else {
          ////  Serial.println("Non-valid data skipped");
          ////}
      idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
      }

      // Update previous states
      incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int iTest = 0;


void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  //Send(0, SPEED_MAX_TEST - 2*abs(iTest));
  //Send(20, SPEED_MAX_TEST - 2*abs(iTest));
  Send(g_steer, g_steer2, g_speed, g_speed2);

  // Calculate test command signal
  //iTest += 10;
  iTest += 1;
  if (iTest > iTestMax) iTest = -iTestMax;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
