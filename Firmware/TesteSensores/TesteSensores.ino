//----------- Includes funcionamento geral -----------
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include <Time.h>
#include <TimeAlarms.h>

//----------- Variáveis funcionamento geral -----------
//RTC
RTC_DS1307 RTC;
DateTime dtPrincipal;
//SD
#define SDCARD 53
Sd2Card card;
SdVolume volume;
SdFile root;
boolean failSD = false;
//Gerais
boolean alarmed = false;
boolean hasError = false;
volatile int mainLoopCounter = 0;

//----------- Bloco Atlas ENV20-----------
//#include "ph_grav.h"
#include "orp_grav.h"
//#include "do_grav.h"

//Define a porta do sensor de PH, usando a placa Gravity
//Gravity_pH pH = Gravity_pH(A13);
const int ENV20_PH = A13;
float ENV20_PH_4 = 2.0186; //Valor em tensao referente a amostra de pH 4,01 (encontrado pela execução do programa de calibração)
float ENV20_PH_7 = 1.5248; //Valor em tensao referente a amostra de pH 7,01
float ENV20_PH_coeficiente_angular = (7.01 - 4.01) / (ENV20_PH_7 - ENV20_PH_4); // m = (y2-y1)/(x2-x1)
float ENV20_PH_coeficiente_linear = 4.01 - (ENV20_PH_coeficiente_angular * ENV20_PH_4); // n = y - m*x

//Define a porta do sensor de ORP, usando a placa Gravity
Gravity_ORP ORP = Gravity_ORP(A13);

//Define a porta do sensor de ORP, usando a placa Gravity
//Gravity_DO DO = Gravity_DO(A13);

#define ENV_DO_PIN A13
float ENV20_DO_calibration = 0.2688;

//----------- Bloco Atlas ENV50-----------
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>

Ezo_board ENV50_PH = Ezo_board(99, "PH");
Ezo_board ENV50_ORP = Ezo_board(62, "ORP");
Ezo_board ENV50_TEMP = Ezo_board(98, "TEMP");
Ezo_board ENV50_DO = Ezo_board(97, "DO");

//----------- Bloco Vernier -----------
#include "VernierLib.h"
VernierLib Vernier;
unsigned long thermistor;
const int ThermistorPIN = A0; // A0 for Analog1 and A2 for Analog 2
float temp;
int rawAnalogReading;

//----------- Bloco Delfino -----------
// Define a porta onde estão conectados os pinos DE e RE do módulo RS485
#define MAX485_DE 31

// Define as constantes do protocolo Modbus
const byte READ_COIL_STATUS = 0x01;
const byte READ_INPUT_STATUS = 0x02;
const byte READ_HOLDING_REGISTERS = 0x03;
const byte READ_INTERNAL_REGISTERS = 0x04;
const byte FORCE_SINGLE_COIL = 0x05;
const byte PRESET_SINGLE_REGISTER = 0x06;
const byte FORCE_MULTIPLE_COILS = 0x0F;
const byte PRESET_MULTIPLE_REGISTERS = 0x10;
const byte MASKED_WRITE_REGISTERS = 0x16;

// Define o ID do sensor (variável conforme o sensor)
const byte PH_ID = 0x09;
const byte DO_ID = 0x01;
const byte TURBIDITY_ID = 0x10;

// Define os tamanhos de pacotes a serem enviados e recebidos
int default_Request_Size = 8;
int calibration_Request_Size = 13;
int default_Response_Size = 0x04;

//----------- Bloco DFRobot-----------
#include <OneWire.h>
#include "DFRobot_ORP_PRO.h"

//TEMPERATURA
int DS18S20_Pin = A14;
OneWire ds(DS18S20_Pin);

//PH
//#define SEN0161_Pin A14 //pH meter Analog output to Arduino Analog Input 0
//#define Offset 2.87       //deviation compensate
//#define ArrayLenth  40    //times of collection
//int pHArray[ArrayLenth];  //Store the average value of the sensor feedback
//int pHArrayIndex = 0;
const int DF_PH = A14;
float DF_PH_4 = 0.0635; //Valor em tensao referente a amostra de pH 4,01 (encontrado pela execução do programa de calibração)
float DF_PH_7 = 1.74; //Valor em tensao referente a amostra de pH 7,01
float DF_PH_coeficiente_angular = (7.01 - 4.01) / (DF_PH_7 - DF_PH_4); // m = (y2-y1)/(x2-x1)
float DF_PH_coeficiente_linear = 4.01 - (DF_PH_coeficiente_angular * DF_PH_4); // n = y - m*x

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
uint16_t DF_DO;
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
#define CAL2_T (15)   //℃

//----------- Bloco de Controle -----------
//Variável que controla qual o tipo de sensor está sendo lido
//PH = 0
//ORP = 1
//DO = 2
//TEMP = 3
//TURBIDITY = 4
#define SENSOR_TYPE 1
#define TEST_MODE true

//Variavel que controla em quantos minutos deve ser registrado o dado
int INTERVALO_COLETA = 1;
//Variavel que controla a cada quantos registros a informação deve ser enviada pela serial
int INTERVALO_TRANSMISSAO = 1;
String sensorName = "";

void setup() {
  Serial.begin(9600);

  //Inicializa o I2C
  Wire.begin();
  //Inicializa o RTC
  RTC.begin();

  //Atualiza a hora da compilacao como horario inicial
  Serial.print("Data/Hora: ");
  Serial.println(__DATE__);
  Serial.println(__TIME__);

  if (!RTC.isrunning()) {
    Serial.print("(RTC nao estava inicializado, setando data da compilacao) ");
    DateTime dt = DateTime(__DATE__, __TIME__);
    printDate(dt);
    RTC.adjust(dt);
  }

  //Inicializa o módulo SDCARD
  if (!SD.begin(SDCARD)) {
    Serial.print("Modulo SD falhou! ");
    failSD = true;
    hasError = true;
  }

  if (!card.init(SPI_HALF_SPEED, SDCARD)) {
    Serial.print("Cartao SD falhou! ");
    failSD = true;
    hasError = true;
  }

  if ((!volume.init(card)) && (!failSD)) {
    Serial.println("Nao foi possivel encontrar uma particao FAT16 ou FAT32.\nVerifique se voce formatou o cartao corretamente ");
    failSD = true;
  }

  if (hasError) {
    Serial.print("Inicializacao falhou! ");
  } else {
    Serial.println("Inicializacao completa. ");
  }
  Serial.println();

  //Se o SD não falhou, inicializa as rotinas
  if (!failSD) {
    //Sincroniza o metodo se alarme
    setSyncProvider(syncProvider);
    setSyncInterval(600);
    printDate(now());

    switch (SENSOR_TYPE) {
      case 0:
        sensorName = "pH";

        //Inicializa o ENV-20 da Atlas com a placa Gravity
        //        if (pH.begin()) {
        //          Serial.println("Erro ao inicializar sensor de PH da Atlas ENV-20");
        //        }
        break;
      case 1:
        sensorName = "ORP";
        if (ORP.begin()) {
          Serial.println("Erro ao inicializar sensor de ORP da Atlas ENV-20");
        }
        break;
      case 2:
        sensorName = "OD";
        //        if (DO.begin()) {
        //          Serial.println("Erro ao inicializar sensor de DO da Atlas ENV-20");
        //        }
        break;
      case 3:
        sensorName = "Temperatura";
        break;
      case 4:
        sensorName = "Turbidez";
        break;
      default:
        sensorName = "Desconhecido";
        break;
    }

    //Chama a identificação do sensor Vernier conectado a porta Analog1 da Shield
    Vernier.autoID();

    //Inicia a Serial 1 para comunicação com os sensores da Delfino
    pinMode(MAX485_DE, OUTPUT);
    digitalWrite(MAX485_DE, 0);
    Serial1.begin(9600);
  }
}

void loop() {
  //Busca data e hora atuais
  dtPrincipal = now();

  //Se tiver sido alarmado, incrementa 1 segundo. Caso contrario, incrementa o loop.
  if (alarmed) {
    //Necessario para incrementar o contador de segundos do alarme.
    printDate(dtPrincipal);
    Alarm.delay(1000);
  } else {
    printDate(dtPrincipal);
    delay(1000);
  }

  if (!alarmed) {
    //Quanto o resto da divisão do minuto pelo intervalo configurado for 0, alarma
    //---------------------------------------------------------------------- alterar para debug -------------------------------------------------------------
    //    if ((dtPrincipal.second() % INTERVALO_COLETA) == 0) {
    if ((dtPrincipal.minute() % INTERVALO_COLETA) == 0) {
      alarm();
    } else
      //Se intervalo = 60, testar minuto 0
      //---------------------------------------------------------------------- alterar para debug -------------------------------------------------------------
      //      if ((dtPrincipal.second() == 0) && (INTERVALO_COLETA == 60)) {
      if ((dtPrincipal.minute() == 0) && (INTERVALO_COLETA == 60)) {
        alarm();
      }
  }
}

void alarm() {
  alarmed = true;

  //Programa alarme (HORA, MINUTO, SEGUNDO, METODO)
  //Alarm.timerRepeat(horaAlarm, minAlarm, secAlarm, mainLoop);
  //------------------------------------------------------------------------------------------ alterar para debug -----------------------------------
  //Alarm.timerRepeat((INTERVALO_COLETA), mainLoop);
  Alarm.timerRepeat((INTERVALO_COLETA * 60), mainLoop);

  Serial.println("Sistema alarmado com sucesso");
  Serial.println("");

  //Chama loop principal
  mainLoop();
}

void mainLoop() {
  mainLoopCounter++;

  //Obtem a data e hora atuais
  DateTime dt = now();

  //Chama a escrita em arquivo e envio, de acordo com a quantidade configurada para ser acumulada antes de ennviar.
  if (mainLoopCounter == INTERVALO_TRANSMISSAO) {
    fileLoop(dtPrincipal);
    mainLoopCounter = 0;
  }

  //Exibe memoria disponivel depois do processo
  Serial.println("Processo de escrita executado com sucesso!");
}

void fileLoop(DateTime dt) {
  //Gerando arquivo (um por dia)
  String fileName = printFileName(dt);
  fileName += ".txt";

  //Instancia String de dados
  String dataString = "";

  /* FORMATO PREVISTO:
      SENSOR;DATA;ENV20;ENV50;DF;VERNIER
  */

  //Imprime qual o sensor
  dataString += sensorName;
  dataString += ";";
  //Imprime a data
  dataString += printFormatedDate(dt);
  dataString += ";";

  
  //Feitos vários switchs pq simplesmente não testava corretamente quando em um único.
  switch (SENSOR_TYPE) {
    case 2:
      float do_env20 = readDO_ENV20_Manual();
      if (TEST_MODE) {
        dataString += "ENV20: ";
      }
      dataString += do_env20;
      dataString += ";";

      float do_env50 = readDO_ENV50_i2c();
      if (TEST_MODE) {
        dataString += "ENV50: ";
      }
      dataString += do_env50;
      dataString += ";";

      float do_df = readDO_DF();
      if (TEST_MODE) {
        dataString += "DF: ";
      }
      dataString += do_df;
      dataString += ";";

      float do_vernier = readDO_Vernier();
      if (TEST_MODE) {
        dataString += "VERNIER: ";
      }
      dataString += do_vernier;
      dataString += ";";

      float do_delfino = readDO_Delfino();
      if (TEST_MODE) {
        dataString += "DELFINO: ";
      }
      dataString += do_delfino;
      dataString += ";";

      break;
    default:
      Serial.println(" Default ");
      break;
  }

  switch (SENSOR_TYPE) {
    case 3:
      float temp_env50 = readTEMP_ENV50_i2c();
      dataString += temp_env50;
      dataString += ";";

      float temp_df = readTEMP_DF_DS18B20();
      dataString += temp_df;
      dataString += ";";

      float temp_vernier = readTEMP_Vernier();
      dataString += temp_vernier;
      dataString += ";";

      break;
    default:
      Serial.println(" Default ");
      break;
  }

  switch (SENSOR_TYPE) {
    case 4:
      float turbidity_df = readTURBIDITY_DF();
      dataString += turbidity_df;
      dataString += ";";

      float turbidity_delfino = readTURBIDITY_Delfino();
      dataString += turbidity_delfino;
      dataString += ";";

      break;
    default:
      Serial.println(" Default ");
      break;
  }

  switch (SENSOR_TYPE) {
    case 1:
      float orp_env20 = readORP_ENV20_Gravity();
      dataString += orp_env20;
      dataString += ";";

      float orp_env50 = readORP_ENV50_i2c();
      dataString += orp_env50;
      dataString += ";";

      float orp_df = readORP_DF();;
      dataString += orp_df;
      dataString += ";";

      float orp_vernier = readORP_Vernier();
      dataString += orp_vernier;
      dataString += ";";

      break;
    default:
      Serial.println(" Default ");
      break;
  }

  switch (SENSOR_TYPE) {
    case 0:
      float ph_env20 = readPH_ENV20_Manual();
      if (TEST_MODE) {
        dataString += "ENV20: ";
      }
      dataString += ph_env20;
      dataString += ";";

      float ph_env50 = readPH_ENV50_i2c();
      if (TEST_MODE) {
        dataString += "ENV50: ";
      }
      dataString += ph_env50;
      dataString += ";";

      float ph_df = readPH_DF_Manual();
      if (TEST_MODE) {
        dataString += "DFROBOT: ";
      }
      dataString += ph_df;
      dataString += ";";

      float ph_vernier = readPH_Vernier();
      if (TEST_MODE) {
        dataString += "VERNIER: ";
      }
      dataString += ph_vernier;
      dataString += ";";

      float ph_delfino = readPH_Delfino();
      if (TEST_MODE) {
        dataString += "DELFINO: ";
      }
      dataString += ph_delfino;
      dataString += ";";

      break;
    default:
      Serial.println("Não encontrou tipo de sensor");
      break;
  }

  //Gerando arquivo
  File file = SD.open(fileName.c_str() , FILE_WRITE);

  Serial.print("Abrindo o arquivo: ");
  Serial.print(fileName);
  Serial.print(" ... ");

  //Se o arquivo abrir corretamente, escrever.
  if (file) {
    file.println(dataString);
    file.close();
    Serial.print("Escreveu a leitura:");
    Serial.println(dataString);
  }
  else {
    //Se o arquivo nao abrir, exibir erro.
    Serial.println("Erro ao abrir o arquivo");
  }
}

String printFileName(DateTime dt) {
  String fileName = "";
  String strDay = "";
  if (dt.day() < 10) {
    strDay += "0";
    strDay += dt.day();
  } else {
    strDay += dt.day();
  }
  fileName += strDay;

  String strMonth = "";
  if (dt.month()  < 10) {
    strMonth += "0";
    strMonth += dt.month();
  } else {
    strMonth += dt.month();
  }

  fileName += strMonth;
  fileName += dt.year();
  return fileName;
}

String printFormatedDate(DateTime dt) {
  String date = "";

  String strDay = "";
  if (dt.day() < 10) {
    strDay += "0";
    strDay += dt.day();
  } else {
    strDay += dt.day();
  }
  date += strDay;
  date += '/';

  String strMonth = "";
  if (dt.month()  < 10) {
    strMonth += "0";
    strMonth += dt.month();
  } else {
    strMonth += dt.month();
  }
  date += strMonth;
  date += '/';

  date += dt.year();
  date += ' ';
  date += dt.hour();
  date += ':';
  date += dt.minute();
  date += ':';
  date += dt.second();
  return date;
}

void printDate(DateTime dt) {
  Serial.print(dt.day(), DEC);
  Serial.print('/');
  Serial.print(dt.month(), DEC);
  Serial.print('/');
  Serial.print(dt.year(), DEC);
  Serial.print(' ');
  Serial.print(dt.hour(), DEC);
  Serial.print(':');
  Serial.print(dt.minute(), DEC);
  Serial.print(':');
  Serial.print(dt.second(), DEC);
  Serial.println();
}

time_t syncProvider() {
  return RTC.now().unixtime();
}

//----------- Bloco Atlas ENV20-----------
float readPH_ENV20_Gravity() {
  //Serial.println(pH.read_ph());
  //return pH.read_ph();
  return 0;
}
float readPH_ENV20_Manual() {
  int PH_raw = analogRead(ENV20_PH); // Lê a voltagem do sensor
  float voltage = PH_raw * (5.0 / 1023.0);
  float PH_value = (ENV20_PH_coeficiente_angular * voltage) + ENV20_PH_coeficiente_linear; //y = m*x + n

  return PH_value;
}


int readORP_ENV20_Gravity() {
  //Serial.println((int)ORP.read_orp());
  return (int)ORP.read_orp();
}
float readDO_ENV20_Manual() {
  int DO_raw = analogRead(ENV_DO_PIN); // Lê a voltagem do sensor
  float voltage = DO_raw * (5.0 / 1023.0);

  float DO_value = (voltage / ENV20_DO_calibration) * 100; //voltagem/calibração * 100
  return DO_value;
}

//----------- Bloco Atlas ENV50 -----------
float readPH_ENV50_i2c() {
  ENV50_PH.send_read_cmd();
  delay(1000);
  ENV50_PH.receive_read_cmd();
  return ENV50_PH.get_last_received_reading();
  //  receive_and_print_reading(PH);
  //  Serial.println();
}
float readORP_ENV50_i2c() {
  ENV50_ORP.send_read_cmd();
  delay(1000);
  ENV50_ORP.receive_read_cmd();
  return ENV50_ORP.get_last_received_reading();
  //  receive_and_print_reading(ORP);
  //  Serial.println();
}
float readDO_ENV50_i2c() {
  ENV50_DO.send_read_cmd();
  delay(1000);
  ENV50_DO.receive_read_cmd();
  return ENV50_DO.get_last_received_reading();
  //  receive_and_print_reading(DO);
  //  Serial.println();
}
float readTEMP_ENV50_i2c() {
  ENV50_TEMP.send_read_cmd();
  delay(1000);
  ENV50_TEMP.receive_read_cmd();
  return ENV50_TEMP.get_last_received_reading();
  //  receive_and_print_reading(TEMP);
  //  Serial.println();
}

//----------- Bloco DFRobot -----------
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
  //  static float pHValue, voltage;
  //
  //  while (pHArrayIndex != ArrayLenth) {
  //    pHArray[pHArrayIndex++] = analogRead(SEN0161_Pin);
  //  }
  //  if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
  //  voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
  //  pHValue = 3.5 * voltage + Offset;

  return 0.00;
}
float readPH_DF_Manual() {
  int PH_raw = analogRead(DF_PH); // Lê a voltagem do sensor
  float voltage = PH_raw * (5.0 / 1023.0);
  float PH_value = (DF_PH_coeficiente_angular * voltage) + DF_PH_coeficiente_linear; //y = m*x + n

  return PH_value;
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


//----------- Bloco Vernier -----------
float readORP_Vernier() {
  return Vernier.readSensor();
}

float readPH_Vernier() {
  return Vernier.readSensor();
}

float readDO_Vernier() {
  return Vernier.readSensor();
}

float readTURBIDITY_Vernier() {
  return Vernier.readSensor();
}

float readTEMP_Vernier() {
  //  return Vernier.readSensor();
  rawAnalogReading = analogRead(ThermistorPIN);  // reads raw analog value from Arduino
  thermistor = resistance(rawAnalogReading);     // converts raw analog value to a resistance
  temp = steinharthart(thermistor);              // Applies the Steinhart-hart equation
  return temp;
}

unsigned long resistance(unsigned long rawAnalogInput)
/* function to convert the raw Analog Input reading to a resistance value
   Schematic:
     [Ground] -- [thermistor] -------- | -- [15,000 ohm bridge resistor] --[Vcc (5v)]
                                       |
                                  Analog Pin 0

   For the circuit above:
   Resistance = ((rawAnalogInput*15000) /(1023 - rawAnalogInput))
*/
{
  unsigned long temp;  // temporary variable to store calculations in
  temp = (rawAnalogInput * 15000) / (1023 - rawAnalogInput);
  return temp; // returns the value calculated to the calling function.
}

float steinharthart(unsigned long resistance)
// function users steinhart-hart equation to return a temperature in degrees celsius.
/* Inputs ADC count from Thermistor and outputs Temperature in Celsius
   There is a huge amount of information on the web about using thermistors with the Arduino.
   Here we are concerned about using the Vernier Stainless Steel Temperature Probe TMP-BTA and the
   Vernier Surface Temperature Probe STS-BTA, but the general principles are easy to extend to other
   thermistors.
   This version utilizes the Steinhart-Hart Thermistor Equation:
      Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
     for the themistor in the Vernier TMP-BTA probe:
      A =0.00102119 , B = 0.000222468 and C = 1.33342E-7
      Using these values should get agreement within 1 degree C to the same probe used with one
      of the Vernier interfaces

*/
{
  float temp; // temporary variable to store calculations in
  float logRes = log(resistance);
  // calculating logirithms is time consuming for a microcontroller - so we just
  // do this once and store it to a variable.
  float k0 = 0.00102119;
  float k1 = 0.000222468;
  float k2 = 0.000000133342;

  temp = 1 / (k0 + k1 * logRes + k2 * logRes * logRes * logRes);
  temp = 273.15 - temp;  // convert from Kelvin to Celsius
  return temp;
}

//----------- Bloco Delfino-----------
//MÉTODOS DE LEITURA
float readPH_Delfino() {
  //Monta os bytes a serem enviados, conforme necessidade. OBS: 2 últimos valores zerados serão substituidos pelo CRC
  byte requestPH[] = {PH_ID, READ_INTERNAL_REGISTERS, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestPH, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  return getResponseMessage();
}

float readTURBIDITY_Delfino() {
  //Monta os bytes a serem enviados, conforme necessidade. OBS: 2 últimos valores zerados serão substituidos pelo CRC
  byte requestTurbidity[] = {TURBIDITY_ID, READ_INTERNAL_REGISTERS, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestTurbidity, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  return getResponseMessage();
}

float readDO_Delfino() {
  startReadingProcess();
  float value = readDO_Delfino_Value();
  endReadingProcess();

  return value;
}

float readDO_Delfino_Value() {
  //Monta os bytes a serem enviados, conforme necessidade. OBS: 2 últimos valores zerados serão substituidos pelo CRC
  byte requestDO[] = {DO_ID, READ_HOLDING_REGISTERS, 0x26, 0x02, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestDO, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  return getResponseMessageDO();
}

void startReadingProcess() {
  //Monta os bytes a serem enviados, conforme necessidade. OBS: 2 últimos valores zerados serão substituidos pelo CRC
  byte requestDO[] = {DO_ID, READ_HOLDING_REGISTERS, 0x25, 0x00, 0x00, 0x01, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestDO, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage(false);
}

void endReadingProcess() {
  //Monta os bytes a serem enviados, conforme necessidade. OBS: 2 últimos valores zerados serão substituidos pelo CRC
  byte requestDO[] = {DO_ID, READ_HOLDING_REGISTERS, 0x2E, 0x00, 0x00, 0x01, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestDO, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage(false);
}
//Auxiliares
float readValue_Delfino(byte response[]) {
  float value = -100;

  uint16_t lowerByte = ((uint16_t)response[3] << 8) | response[4];
  uint16_t higherByte = ((uint16_t)response[5] << 8) | response[6];

  value = modbus_16bit_register_pair_to_float(lowerByte, higherByte);

  return value;
}

float readValue_Delfino_DO(byte response[]) {
  float value = -100;

  uint16_t lowerByte = ((uint16_t)response[4] << 8) | response[3];
  uint16_t higherByte = ((uint16_t)response[6] << 8) | response[5];

  value = modbus_16bit_register_pair_to_float(lowerByte, higherByte);

  return value;
}

float getResponseMessage() {
  byte responseSize = default_Response_Size;

  while (Serial1.available() < 5); // espera o mínimo de bytes para a resposta
  //Serial.print("Resposta: ");

  // Lê a resposta do dispositivo Modbus
  byte response[9];
  Serial1.readBytes(response, sizeof(response));

  float value = readValue_Delfino(response);
  return value;
}

float getResponseMessageDO() {
  byte responseSize = default_Response_Size;

  while (Serial1.available() < 5); // espera o mínimo de bytes para a resposta
  //Serial.print("Resposta: ");

  // Lê a resposta do dispositivo Modbus
  byte response[9];
  Serial1.readBytes(response, sizeof(response));

  float value = readValue_Delfino_DO(response);
  return value;
}

void printResponseMessage(boolean printValue) {
  byte responseSize = default_Response_Size;

  while (Serial1.available() < 5); // espera o mínimo de bytes para a resposta
  Serial.print("Resposta: ");

  // Lê a resposta do dispositivo Modbus
  byte response[9];
  Serial1.readBytes(response, sizeof(response));
  printArray(response, sizeof(response));

  if (printValue) {
    float value = readValue_Delfino(response);
    Serial.print("value: ");
    Serial.println(value);
  }
  Serial.println("");
}

void sendMessage(uint8_t msg[], uint8_t lenght, unsigned int crc) {
  preTransmission();

  //Adiciona o CRC ao Array de Bytes a ser enviado
  msg[lenght - 2] = crc & 0b11111111;
  msg[lenght - 1] = crc >> 8;

  //Debuga quais dados estão sendo enviados
  Serial.print("Enviando pacote de dados: ");
  printArray(msg, lenght);

  //Escreve os dados na serial
  Serial1.write(msg, lenght);
  //Aguarda os dados serem enviados
  Serial1.flush();

  postTransmission();
}

void printArray(uint8_t msg[], uint8_t len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    printHex(msg[i]);
  }
  Serial.println("");
}

void printHex(uint8_t num) {
  Serial.print(F(" 0x"));
  if (num < 16) {
    Serial.print('0');
  }
  Serial.print(num, HEX);
  Serial.print(" ");
}

// Calcula o CRC: Recebe a mensagem em formato de vetor e o tamanho da mensagem (quantos bytes)
unsigned int CRC16(uint8_t msg[], uint8_t len) {
  len -= 2;
  unsigned int crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++)  {
    crc ^= msg[pos];    // Faz uma XOR entre o LSByte do CRC com o byte de dados atual

    for (uint8_t i = 8; i != 0; i--) {    // Itera sobre cada bit
      if ((crc & 0b1) != 0) {      // Se o LSB for 1:
        crc >>= 1;                  // Desloca para a direita
        crc ^= 0xA001;              // E faz XOR com o polinômio 0xA001 (1010 0000 0000 0001 ): x16 + x15 + x2 + 1
      } else {                     // Senão:
        crc >>= 1;                  // Desloca para a direita
      }
    }
  }

  // O formato retornado já sai invertido (LSByte primeiro que o MSByte)
  return crc;
}


void preTransmission() {
  digitalWrite(MAX485_DE, 1);
}

float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b) {
  unsigned long aux = (unsigned long)a + (unsigned long)b * 65536;
  return *((float*)&aux);
}

void postTransmission() {
  digitalWrite(MAX485_DE, 0);
}

void copy(byte* src, byte* dst, int len) {
  memcpy(dst, src, sizeof(src[0])*len);
}
