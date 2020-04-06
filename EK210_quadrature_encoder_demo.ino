/************************************************************************
 * EK210 Electrical: Quadrature Motor Encoder demo code
 * Written by Calvin Lin
 * v.0.1 -- April 6, 2020
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

#include <avr/interrupt.h>
#include <math.h>

// these pins should never change so they are declared const
const uint8_t a_channel_pin = 2;
const uint8_t b_channel_pin = 3;

// global variables that are part of ISRs should be declared volatile
volatile int32_t enc_count = 0;

// this is the pulse per revolution count of the motor and will never change
static const double ppr = 1000.0; 
static const double enc_scale_deg = 360.0 / (ppr * 4);

// keeping time
uint32_t current_ms;
uint32_t previous_ms = 0;
int interval_ms = 100;

double motor_RPM;
double delta_pos = 0.0;
double pos = 0.0;
double zero_pos = 0.0;

void setup() 
{
  //initializing encoder input pins--the Arduino needs to know what the pin is doing at the beginning of the code
  pinMode(a_channel_pin, INPUT);
  pinMode(b_channel_pin, INPUT);

  //setting interrupts--these look for changes in the signal, look below @ additional functions for info as to what an interrupt is
  attachInterrupt(digitalPinToInterrupt(a_channel_pin), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b_channel_pin), encoder_isr, CHANGE);

  previous_ms = 0;
  Serial.begin(115200);
  pos = 0;
}

void loop() 
{
  // non-volatile version of the encoder count
  static int32_t current_enc_count;
  static uint32_t loop_count = 0;
  static double calc_rpm;

  // stop all interrupts while reading enc_count - this prevents interrupts from firing while we read/write enc_count
  cli();

  // write the enc_count value to the local variable and reset enc_count while interrupts are disabled
  current_enc_count = enc_count;
  enc_count = 0;

  // resume interrupts
  sei();

  // calculate position change
  delta_pos = current_enc_count * enc_scale_deg;
  
  // this handles if multiple rotations occur within the sampling period
  if (abs(delta_pos) >= 360.0) {
    delta_pos = fmod(delta_pos, 360.0);
  }

  pos = pos + delta_pos - zero_pos;

  // this handles positive and negative position changes, plus position changes past 360 degrees
  if (pos < 0) {
    pos = 360 + pos;
  }
  else if (pos >= 359.9999) {
    pos = fmod(pos, 360.0);
  }
  else {
    ;;
  }
  
  // calculate speed
  current_ms = millis();
  calc_rpm = (current_enc_count * enc_scale_deg / 360.0) / ((double)(current_ms - previous_ms))  * 60000.0;
  previous_ms = current_ms;

  Serial.print(F("loop no. "));
  Serial.println(loop_count);
  Serial.print(F("pos change: "));
  Serial.println(delta_pos, 4);
  Serial.print(F("current position: "));
  Serial.println(pos, 4);
  Serial.print(F("current speed: "));
  Serial.println(calc_rpm, 2);
  loop_count++;
  delay(1000);
}

// functions and things 
//-----ISRs-----
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
