// HW-201 IR Sensor based Tachometer with 5 sec refresh
//Counts pulses (5s window) + I2C LCD + Serial
//Using EMA smoothing to filter noise 

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define IR_PIN 2               // out pin for IR sensor
#define INTERVAL_MS 5000UL     // counting window (5 seconds)
#define MARKERS 1              //represents no. of reflective marks on rotating shaft/disc
#define MIN_PULSE_INTERVAL_US 1000UL   //minimum time gap between two valid signals
#define PULSE_TIMEOUT 6000UL     // 6sec threshold to determine motor has stopped
#define MAX_POSSIBLE_RPM 20000UL       //To discard very high results due to errors
                                      //can change depending on device rpm  being measured

/*EMA(Exponential Moving Average),a techniue used to filter noise
  in sensor data by adjusting the weight of current and previous 
  reading to calculate the result*/

const float EMA_ALPHA = 0.25; 

//volatile keyword to avoid caching the values in registers
volatile unsigned long pulseCount = 0;
volatile unsigned long lastpulse = 0; //for finding time difference
volatile unsigned long lastpulseISR = 0;//for timeout

unsigned long lastWindow = 0;//track time to calculate last RPM value
float corrected_RPM = 0.0;// final value of RPM

LiquidCrystal_I2C lcd(0x27, 16, 2); //Initialising the LCD

// ISR function using rising edge trigerring
void  on_pulse() {
  unsigned long current_time = micros(); //micros to calculate time in milliseconds
  if (current_time - lastpulse >= MIN_PULSE_INTERVAL_US)//to filter noise in pulse
   {
    pulseCount++;
    lastpulse = current_time;
    lastpulseISR = millis();
  }
}

void setup() {
  pinMode(IR_PIN, INPUT); 

  //executing the ISR and picking the pin for execution
  attachInterrupt(digitalPinToInterrupt(IR_PIN), on_pulse, RISING);

  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tachometer Ready");
  delay(700);
  lastWindow = millis();
  corrected_RPM = 0.0;
}

void loop() {
  unsigned long now = millis();//capture start time

  //check for timing threshold for calculation
  if (now - lastWindow >= INTERVAL_MS) {
    noInterrupts();//diasable interrupt to export values into local variables
    unsigned long count = pulseCount;
    pulseCount = 0;
    unsigned long lastpulse_time = lastpulseISR;
    interrupts();//enable after successful export

    //convert from miiliseconds to seconds
    float intervalinsec = INTERVAL_MS / 1000.0;
    float rpm = 0.0;

    //formula for calculating RAW_RPM
    if (count > 0) {
      rpm = (count * 60.0) / (intervalinsec * (float)MARKERS); 
    
    //drop results with errors
    if (rpm > (float)MAX_POSSIBLE_RPM) {
        Serial.print("Dropped false RPM: ");
        Serial.println(rpm);
        rpm = 0.0;
      }
    }
     
    //timeout check 
    if ((now - lastpulse_time) >=PULSE_TIMEOUT) {
      rpm = 0.0;
      corrected_RPM = 0.0;
    } else {
      //to remove noise due to change in sensor position or environment
      corrected_RPM = (EMA_ALPHA * rpm) + ((1.0 - EMA_ALPHA) * corrected_RPM);
    }

    // Serial output
    Serial.print("Count:");
    Serial.print(count);
    Serial.print("  RawRPM:");
    Serial.print(rpm, 1);
    Serial.print("  corrected_RPM:");
    Serial.println(corrected_RPM, 1);

    // LCD output
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RPM:");
    lcd.setCursor(5, 0);
    lcd.print((int)round(corrected_RPM));
    lcd.setCursor(0, 1);
    lcd.print("P:");//pulse count in the interval
    lcd.print(count);
    lcd.print(" T:");//time since last pulse
    lcd.print(((now - lastpulse_time) < 9999) ? (now - lastpulse_time) : 9999);

    //store last iteration value
    lastWindow = now;
  }

  //show 0 if no pulses were received beyond timeout
  if ((now - lastpulseISR) >= PULSE_TIMEOUT) {
    if (corrected_RPM != 0.0) {
      corrected_RPM = 0.0;
      Serial.println("No pulses -> forced RPM=0");
      
    }
  }
//delay to avoid overload of buffer on screen and serial monitor
  delay(10);
}
