/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/
//pins:

#define ENCODER_1_PIN 14
#define ENCODER_2_PIN 13
#define ENCODER_SELECT_PIN 4




// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder2, FALLING); 
  Serial.begin(115200);

  pinMode(ENCODER_SELECT_PIN, OUTPUT);
  pinMode(ENCODER_1_PIN, INPUT);
  pinMode(ENCODER_2_PIN, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
digitalWrite(ENCODER_SELECT_PIN,HIGH);
  delay(2100);  // delay in between reads for stability
}

int distance = 0;
bool moving_forward = 0;

void encoder1(){
 int encoder_1 = digitalRead(ENCODER_1_PIN);

     if(encoder_1){
      moving_forward = 0;
      }
      else{
      moving_forward = 1;
      }

}


void encoder2(){
 int encoder_1 = digitalRead(ENCODER_1_PIN);


       if(encoder_1){
      
      if(moving_forward)
      { 
        Serial.printf("Moving Backwards\n");
        distance++;
        Serial.printf("Distance: %d\n",distance);
      }
      }
      else
      {
      
      if(!moving_forward){
        Serial.printf("Moving Forwards\n");
        distance--;
        Serial.printf("Distance: %d\n",distance);
      }
      }

}

