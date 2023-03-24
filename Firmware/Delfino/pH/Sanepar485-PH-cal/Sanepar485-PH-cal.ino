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
const byte ID = 0x09;

// Define os tamanhos de pacotes a serem enviados e recebidos
int default_Request_Size = 8;
int calibration_Request_Size = 13;
int default_Response_Size = 0x04;

void setup() {
  //Seta o estado inicial dos pinos utilizados na comunicação 485
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);

  // Inicializa a comunicação serial com a taxa de transmissão de 9600 bps
  Serial.begin(9600);
  //Inicia a Serial 1 para comunicação com os sensores da Delfino
  Serial1.begin(9600);

  //Na primeira execução, solicitar a alteração dos pontos de calibração para 4,7,10
  //changePHCalPointsToDefault();

  //Iniciar a calibração sempre pela solução 7
  //calibratePH7();
  //Se necessário, calibrar os demais pontos
  //calibratePH4();  
  //calibratePH10();
}

void loop() {
  Serial.println("Valor atual de PH:");
  readPH();  
  delay(10000);
}

//Função para trocar os CalPoints para os padrões de soluções que usamos 4, 7 e 10
void changePHCalPointsToDefault() {
  writeCalPoint1To4();
  writeCalPoint2To7();
  writeCalPoint3To10();

  readCalPoint1();
  readCalPoint2();
  readCalPoint3();    
}

//Função para ler os valores de calibração calculados para cada ponto 
void readPHCalValues() {
  readZeroPoint();
  readSlope1();
  readSlope2();  
}

//MÉTODOS DE LEITURA
void readPH() {
  //Monta os bytes a serem enviados, conforme necessidade. OBS: 2 últimos valores zerados serão substituidos pelo CRC
  byte requestPH[] = {ID, READ_INTERNAL_REGISTERS, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};
  
  byte request[default_Request_Size];
  copy(requestPH, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}


//MÉTODOS DE CALIBRAÇÃO
void calibratePH4() {
  byte changeCalPoint1[] = {ID, PRESET_MULTIPLE_REGISTERS, 0x00, 0xC8, 0x00, 0x02, 0x04, 0x00, 0x00, 0x40, 0x80, 0x00, 0x00};

  //Altera calibração do ponto 1 (4)
  byte request[calibration_Request_Size];
  copy(changeCalPoint1, request, calibration_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void calibratePH7() {
  byte changeZeroPoint[] = {ID, PRESET_MULTIPLE_REGISTERS, 0x00, 0xCE, 0x00, 0x02, 0x04, 0x00, 0x00, 0x40, 0xE0, 0x00, 0x00};

  //Altera calibração do ponto zero (7)
  byte request[calibration_Request_Size];
  copy(changeZeroPoint, request, calibration_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void calibratePH10() {
  byte changeCalPoint2[] = {ID, PRESET_MULTIPLE_REGISTERS, 0x00, 0xD6, 0x00, 0x02, 0x04, 0x00, 0x00, 0x41, 0x10, 0x00, 0x00};

  //Altera calibração do ponto 1 (4)
  byte request[calibration_Request_Size];
  copy(changeCalPoint2, request, calibration_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}


//CONSULTA VALORES CALCULADOS DE CALIBRAÇÃO
void readZeroPoint() {
  byte requestZeroPoint[] = {ID, READ_HOLDING_REGISTERS, 0x00, 0x6A, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestZeroPoint, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void readSlope1() {
  byte requestSlope1[] = {ID, READ_HOLDING_REGISTERS, 0x00, 0x6C, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestSlope1, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void readSlope2() {
  byte requestSlope2[] = {ID, READ_HOLDING_REGISTERS, 0x00, 0x6E, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestSlope2, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

//CONSULTAS PONTOS DE CALIBRAÇÃO
void readCalPoint1() {
  byte requestCalPoint1[] = {ID, READ_HOLDING_REGISTERS, 0x00, 0x64, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestCalPoint1, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void readCalPoint2() {
  byte requestCalPoint2[] = {ID, READ_HOLDING_REGISTERS, 0x00, 0x66, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestCalPoint2, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void readCalPoint3() {
  byte requestCalPoint3[] = {ID, READ_HOLDING_REGISTERS, 0x00, 0x68, 0x00, 0x02, 0x00, 0x00};

  byte request[default_Request_Size];
  copy(requestCalPoint3, request, default_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void writeCalPoint1To4() {
  byte requestCalPoint1[] = {ID, PRESET_MULTIPLE_REGISTERS, 0x00, 0x64, 0x00, 0x02, 0x04, 0x00, 0x00, 0x40, 0x80, 0x00, 0x00};

  byte request[calibration_Request_Size];
  copy(requestCalPoint1, request, calibration_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}

void writeCalPoint2To7() {
    byte requestCalPoint2[] = {ID, PRESET_MULTIPLE_REGISTERS, 0x00, 0x66, 0x00, 0x02, 0x04, 0x00, 0x00, 0x40, 0xE0, 0x00, 0x00};

  byte request[calibration_Request_Size];
  copy(requestCalPoint2, request, calibration_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);
  
  printResponseMessage();  
}

void writeCalPoint3To10() {
    byte requestCalPoint3[] = {ID, PRESET_MULTIPLE_REGISTERS, 0x00, 0x68, 0x00, 0x02, 0x04, 0x00, 0x00, 0x41, 0x20, 0x00, 0x00};

  byte request[calibration_Request_Size];
  copy(requestCalPoint3, request, calibration_Request_Size);

  //Calcula o CRC
  unsigned int crc = CRC16(request, sizeof(request));
  sendMessage(request, sizeof(request), crc);

  printResponseMessage();
}


//FUNÇÕES AUXILIARES
float readValue_Delfino(byte response[]) {
  float value = -100;

  uint16_t lowerByte = ((uint16_t)response[3] << 8) | response[4];
  uint16_t higherByte = ((uint16_t)response[5] << 8) | response[6];

  value = modbus_16bit_register_pair_to_float(lowerByte, higherByte);

  return value;
}

void printResponseMessage() {
  byte responseSize = default_Response_Size;

  while (Serial1.available() < 5); // espera o mínimo de bytes para a resposta
  Serial.print("Resposta: ");

  // Lê a resposta do dispositivo Modbus
  byte response[9];
  Serial1.readBytes(response, sizeof(response));
  printArray(response, sizeof(response));

  float value = readValue_Delfino(response);
  Serial.print("value: ");
  Serial.println(value);
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
