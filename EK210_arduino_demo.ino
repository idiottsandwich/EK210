/************************************************************************
 * EK210 Electrical: Arduino demo project
 * Written by Calvin Lin
 * v.0.1 -- May 2, 2020
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

/* Step 1: Set up global variables
 These variables outside both loops are considered global variables. We're going to initialize them with the pin numbers that correspond 
 to the pins to which we've connected our hardware. Global variables can be accessed anywhere in the program, so we're going to declare
 them with the C++ keyword 'static'. This makes the variable name limited to the current file and functions within the file. It's just a
 precaution in case any other variables from other code we include happens to have the same variable name and perform unintended
 modifications to it.  In addition, we're going to use the keyword 'const' to ensure that we will get an error on compile if our code tries 
 to change the variable's value. */
static const int led_pin = 3;
static const int fan_ctrl_pin = 4;
static const int door_ctrl_pin = 9;
static const int temp_read_pin = A0;

/* the output from the sensor will be integer-converted ADC reading every iteration. The temperature will be converted to a double type
which allows for decimal readings.*/  
static int temp_adc_read;
static double temperature;
static double set_point = -99.0;

/* set up user input variables*/
static const int max_input = 50;
static double user_input;
double* p_user_input = &user_input;
static bool full_message_processed = false;

/* Step 2: Set up our states
 Defining our states in one place and early on in the code helps readability. An enumeration allows us to assign numbers to the aliases 
 that we define. We could use numbers as states, but the enumeration helps make the code a bit more clear.  
*/

typedef enum {
  IDLE_NO_INPUT = 1,
  IDLE_AT_TARGET = 2,
  COOLING = 3
} SYS_STATE;

// this line puts the system in the first state, which is waiting for user input.
SYS_STATE current_state = IDLE_NO_INPUT;

// Step 2: Initialize pins and serial communication in the setup loop 
void setup() {

  // pinMode sets up the input and output pins electronically for their corresponding function as an input or output 
  pinMode(led_pin, OUTPUT);           
  pinMode(fan_ctrl_pin, OUTPUT);      
  pinMode(door_ctrl_pin, OUTPUT);     
  pinMode(temp_read_pin, INPUT);      

  /* This line sets up a Serial object and starts communication at the specified baud rate (9600 bits per second). The actual value won't 
  matter for most projects; just make sure both devices' baud rates match, otherwise the communication will look garbled or not show up as 
  text at all. The Arduino Uno has one serial interface and it is connected by default to the USB. */
  Serial.begin(9600); 
} // end void setup

void loop() {

  /* This reads serial input one byte at a time over the Serial bus. This method allows for reading incoming data with minimal blocking 
  to the execution of the program.*/
  if (Serial.available() > 0) {
    full_message_processed = false;
    process_incoming_byte(Serial.read(), p_user_input, &full_message_processed);
    Serial.println("-----processing byte------");
  } // end if Serial.available()

  // read the ADC value from the temperature measurement pin
  temp_adc_read = analogRead(temp_read_pin);

  /* The code in the inner two parentheses converts the ADC output (integer) to a voltage of type double. From the datasheet, the TMP36 
     output equation is: Temperature (Celsius) = (100 * Voltage) + 0.750. We can use this to perform the engineering unit conversion */
  temperature = (((double)temp_adc_read * 5.0 / 1023.0) / 0.010) + 0.750; 

  // switch statement for the main loop creates a branch in the program execution
  switch (current_state) {
    case IDLE_NO_INPUT:

      /* remember we have a boolean flag that tells us if a full message has been received via serial*/
      if (full_message_processed) {
        Serial.println("INPUT RECEIVED");

        /* on the first user input, we want to be able to go directly into either IDLE or COOLING, so check the temperature value. */
        if (temperature <= set_point) {
          current_state = IDLE_AT_TARGET;
        }
        else {
          current_state = COOLING;
          digitalWrite(led_pin, HIGH);
          digitalWrite(fan_ctrl_pin, HIGH);
          digitalWrite(door_ctrl_pin, HIGH);
        } // end if (temperature < = set_point
      }
      
      else {
        Serial.println("WAITING FOR USER INPUT");
      } // end if (full_message_processed)
      break;

    case COOLING:
      // If the temperature is too hot, then turn on fan and open door
      if (temperature <= set_point) {
        digitalWrite(led_pin, LOW);
        digitalWrite(fan_ctrl_pin, LOW);
        digitalWrite(door_ctrl_pin, LOW);
        current_state = IDLE_AT_TARGET;
      } 
      else {
        Serial.print("COOLING! Current temp: ");
        Serial.println(temperature);
        Serial.print(" Target temp: ");
        Serial.println(set_point);
      } // end if/else temperature <= set_point
      break;
      
    case IDLE_AT_TARGET:
      if (temperature >= set_point) {
        current_state = COOLING;
        digitalWrite(led_pin, HIGH);
        digitalWrite(fan_ctrl_pin, HIGH);
        digitalWrite(door_ctrl_pin, HIGH);
      }
      else {
        Serial.print("At target: ");
        Serial.println(set_point);
      } // end if (temperature >= set_point)
      break;
      
    default: 
      Serial.print("----Current state: ");
      Serial.println(current_state);
      Serial.println("----");
      delay(1000);
  } // end switch(current_state)
} // end void loop


/* FUNCTIONS*/ 

/* process_data() 
   This function is not directly called by the user. When a terminating character is received over serial, the buffer value is 
   converted from a string to an integer and then cast to a double. It also sets the flag that signals that a full message has 
   been received. */
void process_data (const char* data, double* out_value, bool* rec_status){
  *out_value = double(strtol(data, NULL, 10));
  *rec_status = true;
}  // end of process_data

/* process_incoming_byte() (credit goes to Nick Gammon http://www.gammon.com.au/serial) 
   This function takes the read serial character and adds it to the buffer.  By default, each serial message between the computer 
   port and the Arduino ends with a newline character '\n' that signals the end of the message. When the serial port receives 
   this terminating character OR when the serial port has received more than our max allowable number of characters, 
   process_incoming_byte() calls this function starts overwriting the buffer.*/
void process_incoming_byte (const byte inByte,  double* input_buffer, bool* full_message_flag) {
  static char input_line [max_input];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte
      
      // terminator reached! process input_line here ...
      process_data(input_line, p_user_input, full_message_flag);
      
      // reset buffer for next time
      input_pos = 0;  
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (max_input - 1))
        input_line [input_pos++] = inByte;
      break;
      
   }  // end of switch 
 } // end of processIncomingByte  
