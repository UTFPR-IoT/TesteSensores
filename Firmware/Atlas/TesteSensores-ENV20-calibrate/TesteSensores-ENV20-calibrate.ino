const int pH_pin = A13; // Define o pino analógico a ser utilizado
const int OD_pin = A13; // Define o pino analógico a ser utilizado

float ph_amostra_4 = 2.0186; //Valor em tensao referente a amostra de pH 4,01
float ph_amostra_7 = 1.5249;//Valor em tensao referente a amostra de pH 7,01

float coeficiente_angular = (7.01 - 4.01) / (ph_amostra_7 - ph_amostra_4); // m = (y2-y1)/(x2-x1)
float coeficiente_linear = 4.01 - (coeficiente_angular * ph_amostra_4); // n = y - m*x

float calibracaoOD = 0.2688;

//Variável que controla qual o tipo de sensor está sendo lido
//PH = 0
//ORP = 1
//DO = 2
//TEMPERATURE = 3
int SENSOR_TYPE = 2;

void setup() {
  Serial.begin(9600); // Inicia a comunicação serial
}

void loop() {
  switch (SENSOR_TYPE) {
    case 0: 
      read_pH();
      break;
    case 1:
      break;  
    case 2:
      read_OD();
      break;
    default: 
      break;
  }
  
  delay(1000); // Espera 1 segundo antes de realizar a próxima leitura
}


void read_pH() {
  int pH_raw = analogRead(pH_pin); // Lê a voltagem do sensor
  float voltage = pH_raw * (5.0 / 1023.0);
  
  float pH_value = (coeficiente_angular * voltage) + coeficiente_linear; //y = m*x + n

  Serial.print("pH = ");
  Serial.print(pH_value, 4); // Imprime o valor de pH com duas casas decimais
  Serial.print(", voltage: ");
  Serial.println(voltage, 4);
}

void read_OD() {
  int OD_raw = analogRead(OD_pin); // Lê a voltagem do sensor
  float voltage = OD_raw * (5.0 / 1023.0);
  
  float OD_value = (voltage/calibracaoOD) * 100; //voltagem/calibração * 100

  Serial.print("OD = ");
  Serial.print(OD_value, 4); // Imprime o valor de OD com duas casas decimais
  Serial.print("%, voltage: ");
  Serial.println(voltage, 4);
}
