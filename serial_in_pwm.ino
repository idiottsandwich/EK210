/************************************************************************
 * EK210 Electrical: MOSFET LED demo code
 * Written by Calvin Lin
 * 
 * Copyright (C) 2019 Boston University
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

/************************************************************************
 * To use the serial monitor, click 'Tools' -> Serial Monitor. 
 * -Ensure that Tools -> Board is set to the correct Arduino board
 * 
 * -Ensure that in 'Tools' -> Port, the proper COM port is selected. The 
 * port with the proper board should show something like:
 * "COM3 (Arduino/Genuino UNO)"
 * 
 * Once the Serial Monitor is open, 
 * -Ensure that the end of line condition is set to 'Newline'--this tells 
 * the user interface to automatically send a terminating character at the
 * end of the user's input. 
 * 
 * -Ensure that the baud rate is set to the same rate
 * as the Serial.begin() passed value
 * 
 * Feel free to type in different values between 0 and 255!
 **********************************************************************
*/

// initialization code 
int ledPin = 3;                     // change this to whatever PWM capable pin you'd like
const int bufferSize = 10;          // constant buffer size of 10; analogWrite only takes 0-255
char readBuffer[bufferSize];        // array for storing the command from the serial port
int idx = 0;                        // running index for buffer
char c;                             // running character for Serial read
int cmd;                            // int converted and range checked command from Serial port

// this loop runs once at every startup
void setup() {
  // set pin to be used as output
  pinMode(ledPin,OUTPUT); 
   
  // begin Serial communication        
  Serial.begin(9600);

  Serial.println("--EK210 MOSFETs demo code--");
  delay(2000);
}

// this loop runs continuously after the setup loop
void loop() {
  
  // read while characters are available via serial but stop when buffer size is maxed 
  while (Serial.available() > 0) {
    
    // read a single character from serial buffer
    c = Serial.read();

    // if we've reached the limit for our buffer, then ignore remaining characters
    if (idx > bufferSize - 1) {
      Serial.println("buffer overflow, ignoring character\n");
      idx = 0;
      break;
    }
    // a newline character signals the end of the serial stream by default 
    if (c == '\n') {
      readBuffer[idx] = '\0';
      idx = 0;
    }
    // continue reading
    else {
      readBuffer[idx] = c;
      idx++;
    }
  } // end while(Serial.available()

  // atoi converts a string to an integer
  cmd = atoi(readBuffer);  

  // range check--only output command if it is between 0 and 255
  if (cmd >= 0 && cmd <= 255) {
    Serial.print("command: " );
    Serial.println(cmd);

    // analogWrite controls PWM output
    analogWrite(ledPin, cmd);
  }
  else {
    Serial.println("out of range");
  }

  delay(500);
}
