//Variável que controla qual o tipo de sensor está sendo lido
//PH = 0
//ORP = 1
//DO = 2
//TEMP = 3
//TURBIDITY = 4

int SENSOR_TYPE = 1;

//TEMPERATURA
unsigned long thermistor;
const int ThermistorPIN = A0; // A0 for Analog1 and A2 for Analog 2
float temp;
int rawAnalogReading;

//PH
const int Vernier_PH_Pin = A0;
float ph;

#include "VernierLib.h" //include Vernier functions in this sketch
VernierLib Vernier; //create an instance of the VernierLib library
float sensorReading; //create global variable to store sensor reading

void setup() {
  Serial.begin(9600);
  Vernier.autoID(); //identify the sensor being used
  
  switch (SENSOR_TYPE) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    default:
      break;
  }
}

float readORP_Vernier() {
   sensorReading = Vernier.readSensor();
  return sensorReading;
}

float readPH_Vernier2() {
  int aRead = 0;
  aRead = analogRead(Vernier_PH_Pin);

  float voltage = aRead * (5.0 / 1023.0);  // assuming 5V reference 
  ph = 14 - (voltage/0.25);
  
  return ph;
}
float readPH_Vernier() {
  sensorReading = Vernier.readSensor();
  return sensorReading;
}

float readDO_Vernier() {
  sensorReading = Vernier.readSensor();
  return sensorReading;
}
float readTURBIDITY_Vernier() {
  sensorReading = Vernier.readSensor();
  return sensorReading;
}

float readTEMP_Vernier2() {
  rawAnalogReading = analogRead(ThermistorPIN);  // reads raw analog value from Arduino
  thermistor = resistance(rawAnalogReading);     // converts raw analog value to a resistance
  temp = steinharthart(thermistor);              // Applies the Steinhart-hart equation
  return temp;
}

float readTEMP_Vernier() {
  sensorReading = Vernier.readSensor();
  return sensorReading;
}

void loop() {
  switch (SENSOR_TYPE) {
    case 0:
      Serial.print("PH: ");
      Serial.println(readPH_Vernier());
      break;
    case 1:
      Serial.print("ORP: ");
      Serial.print(readORP_Vernier());
      Serial.println(" mV");
      break;
    case 2:
      Serial.print("OD: ");
      Serial.print(readDO_Vernier());
      Serial.println(" mg/L");
      break;
    case 3:
      Serial.print("TEMP: ");
      Serial.println(readTEMP_Vernier2());
      break;
    case 4:
      Serial.print("TURBIDITY: ");
      Serial.println(readTURBIDITY_Vernier());
      break;
    default:
      break;
  }

  delay(1000);

}

unsigned long resistance(unsigned long rawAnalogInput)
/* function to convert the raw Analog Input reading to a resistance value    
 * Schematic:
 *   [Ground] -- [thermistor] -------- | -- [15,000 ohm bridge resistor] --[Vcc (5v)]
 *                                     |
 *                                Analog Pin 0
 *
 * For the circuit above:
 * Resistance = ((rawAnalogInput*15000) /(1023 - rawAnalogInput))
 */
{
  unsigned long temp;  // temporary variable to store calculations in
  temp = (rawAnalogInput * 15000) / (1023 - rawAnalogInput);
  return temp; // returns the value calculated to the calling function.
}

float steinharthart(unsigned long resistance)
// function users steinhart-hart equation to return a temperature in degrees celsius. 
/* Inputs ADC count from Thermistor and outputs Temperature in Celsius
 * There is a huge amount of information on the web about using thermistors with the Arduino.
 * Here we are concerned about using the Vernier Stainless Steel Temperature Probe TMP-BTA and the 
 * Vernier Surface Temperature Probe STS-BTA, but the general principles are easy to extend to other
 * thermistors.
 * This version utilizes the Steinhart-Hart Thermistor Equation:
 *    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
 *   for the themistor in the Vernier TMP-BTA probe:
 *    A =0.00102119 , B = 0.000222468 and C = 1.33342E-7
 *    Using these values should get agreement within 1 degree C to the same probe used with one
 *    of the Vernier interfaces
 * 
 */
{
  float temp; // temporary variable to store calculations in
  float logRes = log(resistance); 
  // calculating logirithms is time consuming for a microcontroller - so we just
  // do this once and store it to a variable.
  float k0 = 0.00102119;
  float k1 = 0.000222468;
  float k2 = 0.000000133342; 

  temp = 1 / (k0 + k1 * logRes + k2 * logRes*logRes*logRes);
  temp = 273.15 - temp;  // convert from Kelvin to Celsius 
  return temp;
}
