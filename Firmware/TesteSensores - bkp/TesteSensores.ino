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
#include "do_grav.h"

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
Gravity_DO DO = Gravity_DO(A13);

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

//----------- Bloco Delfino -----------
#include <ModbusMaster.h>
ModbusMaster node;
#define MAX485_DE 31

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
#define SENSOR_TYPE 2
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
        if (DO.begin()) {
          Serial.println("Erro ao inicializar sensor de DO da Atlas ENV-20");
        }
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

    node.begin(9, Serial1);

    //Iniciando a comunicação ModBus
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
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
    //if ((dtPrincipal.second() % INTERVALO_COLETA) == 0) {
    if ((dtPrincipal.minute() % INTERVALO_COLETA) == 0) {
      alarm();
    } else
      //Se intervalo = 60, testar minuto 0
      //---------------------------------------------------------------------- alterar para debug -------------------------------------------------------------
      //if ((dtPrincipal.second() == 0) && (INTERVALO_COLETA == 60)) {
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
    case 2:
      float do_env20 = readDO_ENV20_Gravity();
      dataString += do_env20;
      dataString += ";";

      float do_env50 = readDO_ENV50_i2c();
      dataString += do_env50;
      dataString += ";";

      float do_df = readDO_DF();
      dataString += do_df;
      dataString += ";";

      float do_vernier = readDO_Vernier();
      dataString += do_vernier;
      dataString += ";";

      break;
    case 3:
      dataString += "-";
      dataString += ";";

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
    case 4:
      dataString += "-";
      dataString += ";";

      dataString += "-";
      dataString += ";";

      float turbidity_df = readTURBIDITY_DF();
      dataString += turbidity_df;
      dataString += ";";

      float turbidity_vernier = readTURBIDITY_Vernier();
      dataString += turbidity_vernier;
      dataString += ";";

      float turbidity_delfino = readTURBIDITY_Delfino();
      dataString += turbidity_delfino;
      dataString += ";";

      break;
    default:
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
float readDO_ENV20_Gravity() {
  //Serial.print(DO.read_do_percentage());
  //Serial.println("%");
  return DO.read_do_percentage();
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
  return Vernier.readSensor();
}

//----------- Bloco Delfino-----------
float readPH_Delfino() {
  uint8_t j;
  uint16_t data[4];
  float ph;
  float temp;

  //Declara uma variavel de 8bits e começa a leitura dos conjuntos de bytes de retorno
  uint8_t resultMain;
  //Manda função (0x02 e quanots bytes deve retornar, no caso do PH e Temperatura, são 4)
  resultMain = node.readInputRegisters(0x02, 4);
  if (resultMain == node.ku8MBSuccess) {
    for (j = 0; j < 4; j++) {
      data[j] = node.getResponseBuffer(j);
    }
    ph = modbus_16bit_register_pair_to_float(data[0], data[1]);
    temp = modbus_16bit_register_pair_to_float(data[2], data[3]);
  } else {
    ph = -100;
  }
  return ph;
}

float readTURBIDITY_Delfino() {
  uint8_t j;
  uint16_t data[2];
  float tu;
  float umidade;

  //Declara uma variavel de 8bits e começa a leitura dos conjuntos de bytes de retorno
  uint8_t resultMain;
  //Manda função (0x02 e quantos bytes deve retornar, no caso da Turbidez, sao 2)
  resultMain = node.readInputRegisters(0x02, 2);
  if (resultMain == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++) {
      data[j] = node.getResponseBuffer(j);
    }
    tu = modbus_16bit_register_pair_to_float(data[0], data[1]);
  } else {
    tu = -100;
  }

  return tu;
}

void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b) {
  unsigned long aux = (unsigned long)a + (unsigned long)b * 65536;
  return *((float*)&aux);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}
