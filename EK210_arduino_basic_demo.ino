/************************************************************************
 * EK210 Electrical: Arduino demo project
 * Written by Calvin Lin
 * v.0.1 -- May 3, 2020
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
 them with the C++ keyword 'static'. This makes the variable name limited to the current file and functions within this file. It's just a
 precaution in case any other variables from other code we include happens to have the same variable name and perform unintended
 modifications to it.  In addition, we're going to use the keyword 'const' to ensure that we will get an error on compile if our code tries 
 to change the variable's value. This is because we typically don't expect the hardware pins to change for our prototype */
static const int led_pin = 3;
static const int temp_read_pin = A0;

/* the output from the sensor will be integer-converted ADC reading every iteration. The temperature will be converted to a double type
which allows for decimal readings.*/  
static int temp_adc_read;
static double temperature;
static double set_point = 20.0; // Change this value to change the system's target temperature

// Step 2: Initialize pins and serial communication in the setup loop 
void setup() {
  
  // pinMode tells the microcontroller to set up the input and output pins appropriately for their corresponding functions
  pinMode(led_pin, OUTPUT);           
  pinMode(temp_read_pin, INPUT);      

  /* This line sets up a Serial object and starts communication at the specified baud rate (9600 bits per second). The actual value won't 
  matter for most projects; just make sure both devices' baud rates match, otherwise the communication will look garbled or not show up as 
  text at all. The Arduino Uno has one serial interface and it is connected by default to the USB. */
  Serial.begin(9600); 
  Serial.println("----------------------EK 210 Arduino Basic Demo---------------------");
  delay(2000);
} // end void setup

void loop() {
  
  // read the ADC value from the temperature measurement pin
  temp_adc_read = analogRead(temp_read_pin);

  /* The code in the inner two parentheses converts the ADC output (integer) to a voltage of type double. From the datasheet, the TMP36 
     output equation is: Temperature (Celsius) = (100 * Voltage) + 0.750. We can use this to perform the engineering unit conversion */
  temperature = (((double)temp_adc_read * 5.0 / 1023.0) / 0.010) + 0.750; 

  // main comparison 
  if (temperature > set_point) {

    // turn on the LED pin with the digitalWrite function -- this can either output 5V or 0V; in digital logic this is 1 or 0
    digitalWrite(led_pin, HIGH);

    // print to serial for debugging. println prints with a newline and print does not
    Serial.println("----OVERHEATING----");
    Serial.print("Current temperature: ");
    Serial.println(temperature);
    Serial.print("Target temperature: ");
    Serial.println(set_point);
  }
  
  else {

    // turn off the actuator pins
    digitalWrite(led_pin, LOW);

    // print for debugging
    Serial.println("----Reached target temperature----");
    Serial.print("Current temperature: ");
    Serial.println(temperature);
    Serial.print("Target temperature: ");
    Serial.println(set_point);
    
  } // end if (temperature > set_point)

  // wait 1000 milliseconds between loops
  delay(1000); 
  
} // end void loop
