const int pH_pin = A14; // Define o pino analógico a ser utilizado

float ph_amostra_4 = 0.0635; //Valor em tensao referente a amostra de pH 4,01
float ph_amostra_7 = 1.74;//Valor em tensao referente a amostra de pH 7,01

float coeficiente_angular = (7.01 - 4.01) / (ph_amostra_7 - ph_amostra_4); // m = (y2-y1)/(x2-x1)
float coeficiente_linear = 4.01 - (coeficiente_angular * ph_amostra_4); // n = y - m*x

void setup() {
  Serial.begin(9600); // Inicia a comunicação serial
}

void loop() {
  read_pH(); // Lê o valor do sensor de pH
  
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
