/*
   La Cigüita firmware

   @Author: Christopher Mendez | MCMCHRIS
   @Date: 03/10/2023 (mm/dd/yy)
   @Brief:
   This firmware runs on a nRF52840 (RAK4631) and using a BME680 sensor, analyze CO2 concentration,
   if the air quality is bad, controls a servo motor to let you know, dropping down La Cigüita

   Average power consumption: 12.87ma 
   With a 3000mah battery should last for 10 days.

*/

#include "Arduino.h"
#include <Adafruit_TinyUSB.h>
#include "bsec.h"  // Click to download library: https://github.com/BoschSensortec/BSEC-Arduino-library then install it as .ZIP
#include <Servo.h>

// Comment the next line if you want DEBUG output. But the power savings are not as good then!!!!!!!
//#define MAX_SAVE


float co2;  // The magic variable that will store the CO2 level.

unsigned long previousMillis = 0;  // Will store last time the Servo was turned.

#define BAD_CO2 1000  //Constant of CO2 ppm that will considered to trigger the Servo.
#define GOOD_CO2 1000  //Constant of CO2 ppm that will considered to trigger the Servo.

#define INTERVAL_GOOD 10 * 60 * 1000 //Sampling interval when the air is clean in (ms)
#define INTERVAL_BAD 5 * 60 * 1000 //Sampling interval when the air is polluted in (ms)

long INTERVAL = INTERVAL_GOOD;

#define ALIVE_DG 0   // Servo position when the air is clean (Deg)
#define DIED_DG 180  // Servo position when the air is polluted (Deg)


int pos = 0;  // Variable to store the servo position

// Create an object of the class Bsec
Bsec iaqSensor;  // create Bsec object to read the sensor data
Servo myservo;   // create servo object to control a servo


/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void) {
#ifndef MAX_SAVE
  Serial.begin(115200);
#endif
  Wire.begin();  // Initialize I2C communication.
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  myservo.attach(PIN_SERIAL1_RX);  // Attaches the servo on pin RX to the servo object

  //Initialize Sensor Communication
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);

  //Check sensor status
  checkIaqSensorStatus();

  //Define the sensor variables you need from the BME680
  bsec_virtual_sensor_t sensorList[1] = {
    BSEC_OUTPUT_CO2_EQUIVALENT,
  };

  //Subscribe to the selected variable
  iaqSensor.updateSubscription(sensorList, 1, BSEC_SAMPLE_RATE_LP);

  //Check sensor status
  checkIaqSensorStatus();

  // Tell servo to go to initial position
  myservo.write(pos);
  digitalWrite(LED_GREEN, HIGH);
}



/**
   @brief Function that will be looped forever
*/
void loop(void) {

  if (iaqSensor.run()) {  // If new data is available

    co2 = iaqSensor.co2Equivalent;  //Store the CO2 level on variable "co2"
    
  } else {
    checkIaqSensorStatus();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    // Save the last time the servo was running
    previousMillis = currentMillis;
#ifndef MAX_SAVE
    Serial.print("CO2: ");
    Serial.println(co2);

#endif

    if (co2 > BAD_CO2 ) {  // If the CO2 level exceeded the BAD_CO2 param, turn down the servo to DIED_DG
      INTERVAL = INTERVAL_BAD;
      digitalWrite(LED_GREEN, LOW);
      for (pos; pos <= DIED_DG; pos += 1) {  // goes to DIED position
        // in steps of 1 degree
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(15);           // waits 15ms for the servo to reach the position
      }
    } else if (co2 <= GOOD_CO2 ) {  // If the CO2 level returns to normal, turn up the servo to ALIVE_DG
    digitalWrite(LED_GREEN, HIGH);
      INTERVAL = INTERVAL_GOOD;
      for (pos; pos >= ALIVE_DG; pos -= 1) {  // goes to ALIVE position
        // in steps of 1 degree
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(15);           // waits 15ms for the servo to reach the position
      }
    }
    
  }
}


// Helper function definitions
void checkIaqSensorStatus(void) {
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {

      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {

      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
    }
  }
}

void errLeds(void) {
  delay(100);
  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
}
