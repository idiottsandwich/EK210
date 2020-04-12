/************************************************************************
 * EK210 Electrical: Quadrature motor encoder position control
 * Written by Calvin Lin
 * v.0.2 -- April 12, 2020
 * 
 * Copyright (C) 2020 Boston University
 * This code is intended to be a open source demo 
 * and is to be used for educational purposes only. 
 * Suggestions/constructive criticisms are welcome. 
 ************************************************************************
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * **********************************************************************/
/*
 * A journey of a thousand compiler errors begins with a single step
 */
 
// AltSoftSerial works much better than the SoftwareSerial library but only works for pins 8 and 9 on the UNO
#include <AltSoftSerial.h> 
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>

const uint8_t MAX_INPUT = 50;
AltSoftSerial smc_serial_1;
static int16_t pwr_req;
static uint8_t buffer_idx = 0;
bool on = false;
char input_buffer[MAX_INPUT];
char current_byte;

// these pins should never change so they are declared const
const uint8_t a_channel_pin = 2;
const uint8_t b_channel_pin = 3;

// global variables that are part of ISRs should be declared volatile
volatile int32_t enc_count = 0;

// this is the pulse per revolution count of the motor and will never change
static const double ppr = 1000.0; 
static const double enc_scale_deg = 360.0 / (ppr * 4);

// keeping time
uint32_t print_time_ms_prev = 0;
uint32_t print_time_ms;

// motor position variables
double motor_RPM;
double delta_pos = 0.0;
double pos = 0.0;
const static double zero_pos = 0.0;

void setup() 
{
  // initializing encoder input pins
  pinMode(a_channel_pin, INPUT);
  pinMode(b_channel_pin, INPUT);

  // setting interrupts
  attachInterrupt(digitalPinToInterrupt(a_channel_pin), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b_channel_pin), encoder_isr, CHANGE);
  
  Serial.begin(115200);
  Serial.println("----------Starting Sequence-----------");
  Serial.println("-------Motor Control Test v0.2--------");
  Serial.println("---------------4/12/2020---------------");
  delay(2000);
  smc_serial_1.begin(19200);
  
  // write 0xAA so G2 can auto detect baud rate
  smc_serial_1.write(0xAA);
  
  // Next, send the Exit Safe Start command, which clears the safe-start violation and lets the motor run.
  exitSafeStart();
} // end of setup

void loop() 
{

  // non-volatile version of the encoder count
  static int32_t current_enc_count;
  static uint32_t loop_count = 0;
  static double calc_rpm;
  static double pos_req = 0.0;
  static double* p_pos_request = &pos_req;
  static int16_t speed_cmd = 0;
  static double error = 0;
  
  // stop all interrupts while reading enc_count - this prevents interrupts from firing while we read/write enc_count
  cli();

  // write the enc_count value to the local variable and reset enc_count while interrupts are disabled
  current_enc_count = enc_count;
  enc_count = 0;
  pos = calc_pos(current_enc_count, enc_scale_deg, pos, zero_pos);
  
  // resume interrupts
  sei();
  
  if (Serial.available() > 0) {
    processIncomingByte(Serial.read(), p_pos_request);
    Serial.println("----------------processing byte----------------------");
  } // end if Serial.available()
 
  // pos_req = readPwrReq(pos_req);
  error = pos_req - pos;

  // power request is now a position request
  if (fabs(error) > 20.0) {
    speed_cmd = calc_kp_speed(pos, pos_req);
    pwr_req_send(speed_cmd);
  }
  else {
    speed_cmd = 0;
    pwr_req_send(speed_cmd);
  } // end if fabs(error) > 2.0

  print_time_ms = millis();
  if (print_time_ms - print_time_ms_prev > 200.0) {
    Serial.print(F("-----------------loop no.            "));
    Serial.println(loop_count);
    Serial.print(F("pos req:            "));
    Serial.println(pos_req, 4);
    Serial.print(F("current position:   "));
    Serial.println(pos, 4);
    Serial.print(F("current speed cmd:  "));
    Serial.println(speed_cmd);
    Serial.print(F("current error:      "));
    Serial.println(error);
    loop_count++;
    print_time_ms_prev = print_time_ms;
  } // end if print_time_ms 
} // end of main loop


/****** FUNCTIONS *******/
/******************mapping position to speed********************/
int16_t calc_kp_speed(double current_point, double set_point) {
  //map speed based on magnitude of difference (linear mapping)
  double kp = 2.5;
  double current_diff = set_point - current_point;
  double mapped_diff;

  // going the shortest way to the commanded position
  if (current_diff >= 180.0) {
    mapped_diff = (360 - current_diff) * -1 * kp;
  }
  else if (current_diff < -180.0) {
    mapped_diff = 360 + current_diff * kp;
  }
  else {
    mapped_diff = current_diff * kp;
  }// end if current_diff > 180
  
  int16_t mapped_speed = 0;

  if (mapped_diff >= 0.0) {
    mapped_speed = int16_t(mapped_diff + 0.5); //apparently casting doubles requires this?
  }
  else {
    mapped_speed = int16_t(mapped_diff - 0.5);
  }
  int16_t abs_speed = abs(mapped_speed);
  //set lower limit independent of direction
  if (abs_speed < 95) {
    mapped_speed = abs_speed / mapped_speed * 95; // this keeps the sign correct
  }
  else if (abs_speed > 600) {
    mapped_speed = abs_speed / mapped_speed * 600;
  } // end if mapped_dif >= 0.0

  return mapped_speed;
} // end of calc_kp_speed


// this function takes the old position of the motor and calculates the new position based on encoder ticks
double calc_pos(int32_t encoder_sum, double scale, double last_pos, double home_pos) {
  
  double new_pos;
  double angle_change = double(encoder_sum) * scale;
  
  new_pos = last_pos + angle_change - home_pos;

  // handle positive and negative position changes, plus position changes past 360 degrees
  if (new_pos < 0.0) {
    new_pos = (fmod(new_pos, 360.0) * -1.0) + 360;
  }
  else if (new_pos >= 359.9999) {
    new_pos = fmod(new_pos, 360.0);
  }
  else {
    ;;
  } // end if new_pos < 0.0

  return new_pos;
} // end of calc_pos

// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart() { 
  smc_serial_1.write(0x83);
} // end of exitSafeStart

/*readPwrReq -- this sometimes drops 3rd digit if more than 2 are entered (100 -> 10) 0 -- using Nick Gammon's version instead*/
double readPwrReq(double last_req_pos){

  int32_t req;
  double out;  
  if (!Serial.available() && !on ) {
    Serial.println("NO CMD RECEIVED");
    on = true;
    return 0;
  } // end of if !Serial.available()

  else if (Serial.available()){
      while (Serial.available() > 0) {
        //read one byte
        current_byte = Serial.read();

        //check for ASCII newline, add a \0 to terminate as a string
        if ( current_byte == '\n' ) {
          //end of message, reset buffer index
          req = strtol(input_buffer, NULL, 10);
          input_buffer[buffer_idx] = '\0';
          buffer_idx = 0;  //reset index to 'flush' the buffer
          return double(req);
        }
        else {
          //put in buffer
          input_buffer[buffer_idx] = current_byte;
          buffer_idx++;
        }
      }
  } // end of else if Serial.available()
  else{
    return last_req_pos;
  } // end of if !Serial.available()
} // end of readPwrReq

///////////////////NICK GAMMON http://www.gammon.com.au/serial
void process_data (const char * data, double * p_pos_req)
  {
  *p_pos_req = double(strtol(data, NULL, 10));
  }  // end of process_data
  
void processIncomingByte (const byte inByte,  double * p_pos_req)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
    {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte
      
      // terminator reached! process input_line here ...
      process_data(input_line, p_pos_req);
      
      // reset buffer for next time
      input_pos = 0;  
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch
   
  } // end of processIncomingByte  

///////////////////NICK GAMMON http://www.gammon.com.au/serial



/**************************************
 * pwr_req_send 
 * 
 * Sends power command to the motor controller 
 */
void pwr_req_send(int16_t req_pwr) {

  byte MOTOR_CMD;
  
  //implement a range check

  if (req_pwr < 0 ) {
    MOTOR_CMD = 0x05;  //motor set forward (current encoder lookup table flips things)
  }
  else {
    MOTOR_CMD = 0x06;  //motor set reverse
  } // end of if req_pwr < 0

  int16_t SPEED_REQ = abs(req_pwr);

  // cap speed for now this should be 
  if (SPEED_REQ > 1000){
    SPEED_REQ = 1000;
  }

  //next two lines are pololu magic
  //calculate data byte 1 (low 5 bits of speed)
  byte SPEED_LOW_BYTE = SPEED_REQ % 32;
  
  //calculate data byte 2 (high 7 bits of speed)
  byte SPEED_HIGH_BYTE = SPEED_REQ >> 5; 

  //command packet via pololu protocol = (Protocol Byte) (Device Number) (Command Byte) (Data Byte 1) (Data Byte 2)

  //send command packet over serial
  smc_serial_1.write(0xAA);
  smc_serial_1.write(0x0D);
  
  smc_serial_1.write(MOTOR_CMD);
  //Serial.print("MOTOR_CMD: ");
  //Serial.println(MOTOR_CMD);
  
  smc_serial_1.write(SPEED_LOW_BYTE);
//  Serial.print("SPEED_LOW_BYTE: ");
//  Serial.println(SPEED_LOW_BYTE);
//  
  smc_serial_1.write(SPEED_HIGH_BYTE);
//  Serial.print("SPEED_HIGH_BYTE: ");
//  Serial.println(SPEED_HIGH_BYTE);
} // end of pwr_req_send


/**************************************
 *ISR routine to 
 * 
 * Sends power command to the motor controller 
 */
 /*Interrupt service routine--an ISR is a special function that runs when the conditions for the interrupt that are set above are met
These generally need to be short and efficient pieces of code, especially if we're trying to measure motor speed. The Arduino will 
stop everything it's doing and then run the ISR if it detects an edge (a pulse) on the encoder channel.*/
// this was a really elegant solution I found at http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
void encoder_isr() {
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((PIND & 0b1100) >> 2);
 
    enc_count = enc_count + lookup_table[enc_val & 0b1111];
}
