/************************************************************************
 * EK210 Electrical: Expanded state machine demo
 * Written by Calvin Lin
 * v.0.2 -- May 3, 2020
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
 /* CHANGES/TODO LIST/NOTES May 3, 2020 v0.2:
    CHANGES:
    -Tested state transitions artificially without hardware
    NOTES: 
    -For debugging, annunciate state transitions!
    TODO:
    -SIMPLIFY -- create a version of this without user input as it complicates things -- demo just basic hardware and loop
        keep it simple because this version is a bit overwhelming for complete beginners
    -Hardware test with thermistor (Currently don't have a TMP36 on hand);
    -Scope limiting (all functions should be passed their values, all variables that can be in main loop should stay there)
    -Range check on user inputs/function passed values
    -Store strings in memory
    -Store system state
 */

// We're going to use the standard Arduino servo library https://www.arduino.cc/en/reference/servo
#include <Servo.h>

/* Step 1: Set up global variables
 These variables outside both loops are considered global variables. We're going to initialize them with the pin numbers that correspond 
 to the pins to which we've connected our hardware. Global variables can be accessed anywhere in the program, so we're going to declare
 them with the C++ keyword 'static'. This makes the variable name limited to the current file and functions within this file. It's just a
 precaution in case any other variables from other code we include happens to have the same variable name and perform unintended
 modifications to it.  In addition, we're going to use the keyword 'const' to ensure that we will get an error on compile if our code tries 
 to change the variable's value. This is because we typically don't expect the hardware pins to change for our prototype */
static const int led_pin = 3;
static const int fan_ctrl_pin = 4;
static const int door_ctrl_pin = 9;
static const int temp_read_pin = A0;

/* the output from the sensor will be integer-converted ADC reading every iteration. The temperature will be converted to a double type
which allows for decimal readings.*/  
static int temp_adc_read;
static double temperature;

/* set up user input variables*/
static const int max_input = 50;                // this is used to limit the max size of the user input message
static double set_point = -99.0;               // this stores the actual user input
static bool full_message_processed = false;     // this is used identify if when a user's input is finished processing

/* Step 2: Set up our states
 Defining our states in one place and early on in the code helps readability. An enumeration allows us to assign numbers to the states 
 that we define. We could use numbers as states, but the enumeration helps make the code a bit more clear.  Typedef enum allows us to 
 create a new 'type' (like int or double) but in this case it is enum. We assign it SYS_STATE to make it slightly cleaner when 
 calling a new instance. */
typedef enum {
  IDLE_NO_INPUT = 1,
  IDLE_AT_TARGET = 2,
  COOLING = 3
} SYS_STATE;

// this line puts the system in the first state, which is waiting for user input.
static SYS_STATE current_state = IDLE_NO_INPUT;

static unsigned long last_time = 0;

// Step 2: Initialize pins and serial communication in the setup loop 
void setup() {

  // pinMode tells the microcontroller to set up the input and output pins appropriately for their corresponding functions
  pinMode(led_pin, OUTPUT);           
  pinMode(fan_ctrl_pin, OUTPUT);      
  pinMode(door_ctrl_pin, OUTPUT);     
  pinMode(temp_read_pin, INPUT);      

  /* This line sets up a Serial object and starts communication at the specified baud rate (9600 bits per second). The actual value won't 
  matter for most projects; just make sure both devices' baud rates match, otherwise the communication will look garbled or not show up as 
  text at all. The Arduino Uno has one serial interface and it is connected by default to the USB. */
  Serial.begin(9600); 
  Serial.println("----------------------EK 210 Arduino State Machine Demo---------------------");
  delay(2000);
} // end void setup

void loop() {

  /* This reads serial input one byte at a time over the Serial bus. This method allows for reading incoming data with minimal blocking 
  to the execution of the program.*/
  if (Serial.available() > 0) {
    full_message_processed = false;
    process_incoming_byte(Serial.read(), &set_point, &full_message_processed);
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

      // check the boolean flag to see if a full message has been received via serial
      if (full_message_processed) {
        
        // on the first user input, we want to be able to go directly into either IDLE or COOLING, so check the temperature value
        if (temperature <= set_point) {
          Serial.println("INPUT RECEIVED");
          current_state = IDLE_AT_TARGET;
        }
        else {
          Serial.println("Temperature exceeded, cooling");
          current_state = COOLING;
          digitalWrite(led_pin, HIGH);
          digitalWrite(fan_ctrl_pin, HIGH);
          digitalWrite(door_ctrl_pin, HIGH);
        } // end if (temperature < = set_point
      }
      
      else {
        ;;
      } // end if (full_message_processed)
      break;

    case COOLING:

      // if target temperature is reached, turn off actuators and transition states -> IDLE
      if (temperature <= set_point) {
        digitalWrite(led_pin, LOW);
        digitalWrite(fan_ctrl_pin, LOW);
        digitalWrite(door_ctrl_pin, LOW);
        current_state = IDLE_AT_TARGET;
        Serial.println("Reached target temperature");
      }

      // otherwise, keep fan on and door open
      else {
        ;;
      } // end if/else temperature <= set_point
      break;
      
    case IDLE_AT_TARGET:

      // if temperature exceeds target value, turn on actuators and transition states -> COOLING
      if (temperature >= set_point) {
        current_state = COOLING;
        Serial.println("Temperature exceeded, cooling");  
        digitalWrite(led_pin, HIGH);
        digitalWrite(fan_ctrl_pin, HIGH);
        digitalWrite(door_ctrl_pin, HIGH);
      }
      
      else {
        ;;
      } // end if (temperature >= set_point)
      break;
      
    default: 
      ;;
  } // end switch(current_state)
  
  last_time = print_status(500, last_time);
  
} // end void loop


/* FUNCTIONS*/ 
/* print status 
   This function prints the status of the system over serial at a user specified rate (milliseconds) */
unsigned long print_status(int period, unsigned long previous_time) {

  unsigned long current_time = millis();
  static unsigned long loop_counter = 0;

  if ((current_time - previous_time) > period) {
    Serial.print("--------------------print number ");
    Serial.print(loop_counter);
    Serial.println("-----------------------");
    Serial.print("Current state: ");
    
    if (current_state == IDLE_NO_INPUT) {
      Serial.println("IDLE, NO USER INPUT RECEIVED");
    }
    else if (current_state == COOLING) {
      Serial.println("COOLING");
    }
    else if (current_state == IDLE_AT_TARGET) {
      Serial.println("IDLE");
    }
    else {
      Serial.println("UNDEFINED STATE, PRESS RESET");
    }
    Serial.print("Current temperature: ");
    Serial.println(temperature);
    Serial.print("Target temperature: ");
    Serial.println(set_point);
    Serial.println("------------------------------------------------------");
    loop_counter++;
    
    return current_time;
  }
  
  else {
    return previous_time;
    
  }  // end if (current_time - previous_time);
} // end print_status

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
void process_incoming_byte (const byte inByte,  double* out_val, bool* full_message_flag) {
  static char input_line [max_input];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte
      
      // terminator reached! process input_line here ...
      process_data(input_line, out_val, full_message_flag);
      
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
