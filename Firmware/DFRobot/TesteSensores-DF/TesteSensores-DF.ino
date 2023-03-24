#include <OneWire.h>
#include "DFRobot_ORP_PRO.h"

//Variável que controla qual o tipo de sensor está sendo lido
//PH = 0
//ORP = 1
//DO = 2
//TEMP = 3
//TURBIDITY = 4

int SENSOR_TYPE = 1;

//TEMPERATURA
int DS18S20_Pin = A14; //DS18S20 Signal pin on digital 2
OneWire ds(DS18S20_Pin);  // on digital pin 2

//PH
#define SEN0161_Pin A14            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;

//ORP
#define DF_ORP_Pin A14
#define ADC_RES 1024
#define V_REF 5000
float ADC_voltage;
DFRobot_ORP_PRO DF_ORP(0);

//Turbidity
#define DF_TURBIDITY_Pin A14
#define ADC_RES 1024
#define V_REF 5000

//OD
#define DF_DO_Pin A14
const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (20) //Current water temperature ℃, Or temperature sensor function
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1845) //mv
#define CAL1_T (20)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (22)   //℃

void setup() {
  Serial.begin(9600);

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

float readTEMP_DF_DS18B20() {
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
float readORP_DF() {
  ADC_voltage = ((unsigned long)analogRead(DF_ORP_Pin) * V_REF + ADC_RES / 2) / ADC_RES;
  return DF_ORP.getORP(ADC_voltage);
}

float readPH_DF() {
  static float pHValue, voltage;

  while (pHArrayIndex != ArrayLenth) {
    pHArray[pHArrayIndex++] = analogRead(SEN0161_Pin);
  }
  if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
  voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
  pHValue = 3.5 * voltage + Offset;

  return pHValue;
}
float readDO_DF() {
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DF_DO_Pin);
  ADC_Voltage = uint32_t(V_REF) * ADC_Raw / ADC_RES;
  //Dividir por 1000 para ter mg/L
  float od = readDO(ADC_Voltage, Temperaturet) / 1000.0;
  return od;
}
float readTURBIDITY_DF() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1024.0);
  return voltage;
}

void loop() {
  switch (SENSOR_TYPE) {
    case 0:
      Serial.print("PH: ");
      Serial.println(readPH_DF());
      break;
    case 1:
      Serial.print("ORP: ");
      Serial.print(readORP_DF());
      Serial.println(" mV");
      break;
    case 2:
      Serial.print("OD: ");
      Serial.print(readDO_DF());
      Serial.println(" mg/L");
      break;
    case 3:
      Serial.print("TEMP: ");
      Serial.println(readTEMP_DF_DS18B20());
      break;
    case 4:
      Serial.print("TURBIDITY: ");
      Serial.println(readTURBIDITY_DF());
      break;
    default:
      break;
  }

  ADC_Raw = analogRead(DF_DO_Pin);
  ADC_Voltage = uint32_t(V_REF) * ADC_Raw / ADC_RES;
  Serial.print("Voltage:");
  Serial.println(ADC_Voltage);

  delay(1000);

}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}
