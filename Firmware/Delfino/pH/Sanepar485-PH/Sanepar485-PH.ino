#include <ModbusMaster.h>

#define MAX485_DE 31

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  Serial.begin(9600);

  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0);
  Serial1.begin(9600);
  
  node.begin(9, Serial1);
  
  //Iniciando a comunicação ModBus
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}


void loop() {
  uint8_t j;
  uint16_t data[4];
  float ph, ph2;
  float temp, temp2;

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

    Serial.print("PH = ");
    Serial.println(ph, 2);
    Serial.print("Temperatura = ");
    Serial.println(temp, 2);

    Serial.println("Dados Brutos:");
    Serial.print(node.getResponseBuffer(0x00), HEX);
    Serial.print(" ");
    Serial.print(node.getResponseBuffer(0x01), HEX);
    Serial.print(" ");
    Serial.print(node.getResponseBuffer(0x02), HEX);
    Serial.print(" ");
    Serial.print(node.getResponseBuffer(0x03), HEX);
    Serial.println(" ");
    Serial.println(" ");
  } else {
    Serial.println("Erro ao receber dados do sensor");
  }
  delay(10000);
}

float modbus_16bit_register_pair_to_float(uint16_t a, uint16_t b) {
  unsigned long aux = (unsigned long)a + (unsigned long)b * 65536;
  return *((float*)&aux);
}
