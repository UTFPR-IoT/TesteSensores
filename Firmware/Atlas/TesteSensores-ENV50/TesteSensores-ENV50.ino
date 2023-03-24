#include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>    //include arduinos i2c library
#include <Ezo_i2c_util.h> //brings in common print statements

//Variável que controla qual o tipo de sensor está sendo lido
//PH = 0
//ORP = 1
//DO = 2
//TEMPERATURE = 3
int SENSOR_TYPE = 1;

Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
Ezo_board ORP = Ezo_board(62, "ORP");      //create an EC circuit object who's address is 62 and name is "ORP"
Ezo_board TEMP = Ezo_board(98, "TEMP");      //create an EC circuit object who's address is 98 and name is "EC"
Ezo_board DO = Ezo_board(97, "DO");      //create an EC circuit object who's address is 97 and name is "EC"

void setup() {
  Wire.begin();                           //start the I2C
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
    default:
      break;
  }
}

void loop() {
  switch (SENSOR_TYPE) {
    case 0:
      readPH_ENV50_i2c();
      break;
    case 1:
      readORP_ENV50_i2c();
      break;
    case 2:
      readDO_ENV50_i2c();
      break;
    case 3:
      readTEMP_ENV50_i2c();
      break;
    default:
      break;
  }

  delay(1000);
}

void readPH_ENV50_i2c() {
  PH.send_read_cmd();
  delay(1000);
  PH.receive_read_cmd();
  Serial.print("received: ");
  Serial.println(PH.get_last_received_reading());
  Serial.println();
}
void readORP_ENV50_i2c() {
  ORP.send_read_cmd();
  delay(1000);
  receive_and_print_reading(ORP);
  Serial.println();
}
void readDO_ENV50_i2c() {
  DO.send_read_cmd();
  delay(1000);
  receive_and_print_reading(DO);
  Serial.println();
}
void readTEMP_ENV50_i2c() {
  TEMP.send_read_cmd();
  delay(1000);
  receive_and_print_reading(TEMP);
  Serial.println();
}
