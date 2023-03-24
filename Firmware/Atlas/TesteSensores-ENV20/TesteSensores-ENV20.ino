#include "ph_grav.h"
#include "orp_grav.h"
#include "do_grav.h"


//Variável que controla qual o tipo de sensor está sendo lido
//PH = 0
//ORP = 1
//DO = 2

int SENSOR_TYPE = 1;

//Define a porta do sensor de PH, usando a placa Gravity
Gravity_pH pH = Gravity_pH(A13);
//Define a porta do sensor de ORP, usando a placa Gravity
Gravity_ORP ORP = Gravity_ORP(A13);
//Define a porta do sensor de ORP, usando a placa Gravity
Gravity_DO DO = Gravity_DO(A13);

void setup() {
  Serial.begin(9600);

  switch (SENSOR_TYPE) {
    case 0:
      //Inicializa o ENV-20 da Atlas com a placa Gravity
      if (pH.begin()) {
        Serial.println("Erro ao inicializar sensor de PH");
      }
      break;
    case 1:
      if (ORP.begin()) {
        Serial.println("Erro ao inicializar sensor de ORP");
      }
      break;
    case 2:
      if (DO.begin()) {
        Serial.println("Erro ao inicializar sensor de DO");
      }
      break;
    default:
      break;
  }
}

void readPH_ENV20_Gravity() {
  Serial.println(pH.read_ph());
}
void readORP_ENV20_Gravity() {
  Serial.println((int)ORP.read_orp());
}
void readDO_ENV20_Gravity() {
  Serial.print(DO.read_do_percentage());
  Serial.println("%");
}

void loop() {
  switch (SENSOR_TYPE) {
    case 0:
      readPH_ENV20_Gravity();
      break;
    case 1:
      readORP_ENV20_Gravity();
      break;
    case 2:
      readDO_ENV20_Gravity();
      break;
    default:
      break;
  }

  delay(1000);

}
