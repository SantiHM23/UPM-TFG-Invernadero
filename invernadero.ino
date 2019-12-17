
#include <SPI.h>
#include <Ethernet.h>
#include <math.h>
#include <Time.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <DHT.h>
#include <PID_v1.h>

#define DHTTYPE DHT22
 
/** Ethernet client instance */
EthernetClient client;
 
/** Default local IP address, in case of DHCP IP assignement fails, la IP obtenida con el getIP para la ethernet shield de arduino */
const IPAddress eth_default_ip( 192,  168,  1,  106);
 
/** MAC Address of the ethernet shield */ 
byte eth_mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
 
/** Server address wich runs InfluxDB, la IP de la propia raspberry pi */
const byte eth_server[] = {192, 168, 1, 107};
 
/** InfluxDB HTTP port */
const int eth_port = 8086;
 
 
/** Size of the buffer for HTTP content */
const int bufferSize = 2048;
 
/** An character array filled with null terminate chars */
char buf[bufferSize] = {'\0'};
 
 //Pines de los actuadores
const int pinv1 = 33; //Ventilador DC grande
const int pinv2 = 34; //Ventilador DC pequeño
const int pinv3 = 37; //Ventilador AC grande
const int pinluz = 41; //Proyector LED
const int pinbom = 45; //Bomba de agua
const int pinhum = 49; //Humidificador

//Pines de los sensores
const int sh1 = 0; //Primer sensor de humedad del suelo
const int sh2 = 2; //Segundo sensor de humedad del suelo
const int ldr = 10; //Sensor de cantidad de luz
const int ntc1 = 12; //Primer sensor de temperatura
const int ntc2 = 14; //Segundo sensor de temperatura
const int DHTpin = 48; //Sensor de humedad del aire (digital)

DHT dht(DHTpin, DHTTYPE);

//Estado de los actuadores
boolean vent1 = 0;
boolean vent2 = 0;
boolean vent3 = 0;
boolean bomba = 0;
boolean humid = 0;
boolean proy = 0;

//Valores limite de las variables
const float tempMin = 22.0;
const float tempMax = 24.0;
const float htMax = 60.0;
const float htMin = 55.0;
const float haMax= 70.0;
const float haMin = 60.0;

//Valores del PID
double Inputtemp, Setpointtemp, Outputtemp;
double Inputhum, Setpointhum, Outputhum;
double kptemp = 0.9368;
double kitemp = 0.0095;
double kdtemp = 6.745;
double kphum = 0.02181;
double kihum = 0.0001;
double kdhum = 0.00;
double kpmulti = 0.041;
double kimulti = 0.0022;
double kdmulti = 0.4;

//Creacion del PID
PID tempPID(&Inputtemp, &Outputtemp, &Setpointtemp, kpmulti, kimulti, kdmulti, DIRECT);
PID humPID(&Inputhum, &Outputhum, &Setpointhum, kpmulti, kimulti, kdmulti, DIRECT);

 //Calculo de la temperatura
 //Valores fijos del circuito
float rAux = 10000.0;
//float rAux2 = 10000.0;
float vcc = 5.0;
float beta = 3977.0;
float temp0 = 298.0;
float r0 = 10000.0;

//Variables usadas en el calculo de la temperatura
float vm = 0.0;
float rntc = 0.0;
float temperaturaK = 0.0;
//float vm2 = 0.0;
//float rntc2 = 0.0;
//float temperaturaK2 = 0.0;
 
//Funcion para obtener el valor de la temperatura 
float getTemp(int pin) {
  vm=(vcc / 1024)*( analogRead(pin) );
  rntc = rAux / ((vcc/vm)-1);
  temperaturaK = beta/(log(rntc/r0)+(beta/temp0));
  float temp=temperaturaK - 273.15;
  return temp;
}

//Calculo de la humedad del suelo
float voltmax = 1024.0;
//Variables usadas en el calculo de la humedad del suelo
float vhum = 0.0;

float getHum(int pin) {
  vhum = 1 -( analogRead(pin) / voltmax);
  float valorhum = (vhum)*100.0;
  return valorhum;
}

//Medicion de la cantidad de luz recibida por el LDR
int getLight() {
  int luz = map(analogRead(ldr), 0, 1023, 0, 100);
  return luz;
}
 
//Encendido y apagado de los ventiladores 
void activarVentilador(){
 if ((dht.readTemperature()) < tempMin)// && ((getTemp(ntc2)) < tempMin))
  {
    vent1 = vent2 = vent3 = 0;  
  }
 if ((dht.readTemperature()) >= tempMax)// || ((getTemp(ntc2)) >= tempMax))
  {
    vent1 = vent2 = vent3 = 1;
  }
}   

//Encendido y apagado de la bomba de agua
void encenderBomba(){
 if (((getHum(sh1)) >= htMax) && ((getHum(sh2)) >= htMax))
  {
    bomba = 0;  
  }
 if (((getHum(sh1)) < htMin) || ((getHum(sh2)) < htMin))
  {
    bomba = 1;
  }
} 

//Encendido y apagado del humidificador
void encenderHumidificador(){
  if (dht.readHumidity() >= haMax)
  {
    humid = 0;
  }
  if (dht.readHumidity() < haMin)
  {
    humid = 1;
  }
}

/*Encendido y apagado del proyector
void encenderProyector(){
  if((hour(t) > 7) && (hour(t) < 22))
  {
    proy = 1;
  }
  else 
  {
    proy = 0;
  }
}*/

//Estado de cada ventilador
int vent1State(){
  if(vent1==1)
  {
    digitalWrite(pinv1,LOW);
   return 1; //encender ventilador
  }
  else
  {
    digitalWrite(pinv1,HIGH);
    return 0;  //apagar ventilador
  }
}

int vent2State(){
   if(vent2==1)
  {
    digitalWrite(pinv2,LOW);
   return 1; //encender ventilador
  }
  else
  {
    digitalWrite(pinv2,HIGH);
  return 0;  //apagar ventilador
  }
}

int vent3State(){
   if(vent3==1)
  {
    digitalWrite(pinv3,LOW);
   return 1; //encender ventilador
  }
  else
  {
    digitalWrite(pinv3,HIGH);
  return 0;  //apagar ventilador
  }
}

/*Estado del proyector
int proyState(){
  if(proy==1)
  {
    digitalWrite(pinluz,LOW);
    return 1;
  }
  else
  {
    digitalWrite(pinluz,HIGH);
  return 0; 
  }
}*/

//Estado del la bomba de agua
int bombaState(){
  if(bomba==1)
  {
    digitalWrite(pinbom,LOW);
    return 1;
  }
  else
  {
    digitalWrite(pinbom,HIGH);
  return 0; 
  }
}

//Estado del humidificador
int humidState(){
  if(humid==1)
  {
    digitalWrite(pinhum,LOW);
    return 1;
  }
  else
  {
    digitalWrite(pinhum,HIGH);
    return 0;
  }
}


/**
 * @brief Starts the ethernet shield as client
 */
boolean eth_start(){
   Ethernet.begin(eth_mac, eth_default_ip);
   delay(2000); //delay to allow connection to be done
 
   //do a fast test if we can connect to server
   int conState = client.connect(eth_server, eth_port);
 
   if(conState > 0) {
    Serial.println("Connected to InfluxDB server");
    client.stop();
    return true;
   }
 
  //print the error number and return false
  Serial.print("Could not connect to InfluxDB Server, Error #");
  Serial.println(conState);
  return false;
}
 
/**
 * @brief Send  HTTP data to InfluxDB
 * @param data  Pointer to the beginning of the buffer
 * @param dataSize  Number of valid characters to send
 */
void eth_send_data(char* data, int dataSize) {
  //first we need to connect to InfluxDB server   
  int conState = client.connect(eth_server, eth_port);
 
  if(conState <= 0) { //check if connection to server is stablished
    Serial.print("Could not connect to InfluxDB Server, Error #");
    Serial.println(conState);
    return;
  }
  
  //Send HTTP header and buffer
  client.println("POST /write?db=inv HTTP/1.1");
  client.println("Host: 192.168.1.107:8083");
  client.println("User-Agent: Arduino/1.0");
  client.println("Connection: close");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(dataSize);
  client.println();
  client.println(data);
 
  delay(50); //wait for server to process data
 
  //Now we read what server has replied and then we close the connection
  Serial.println("Reply from InfluxDB");
  while(client.available()) { //receive char
    Serial.print((char)client.read());
  }  
  Serial.println(); //empty line
 
  client.stop();  
}
 
/**
 * @brief Setups the peripherals
 */
void setup() {
  //Serial interface for debugging purposes
  Serial.begin(115200);
  setTime(12,20,00,25,06,2018); //h,min,seg,dia,mes,año
  pinMode(pinv1,OUTPUT);
  pinMode(pinv2,OUTPUT);
  pinMode(pinv3,OUTPUT);
  pinMode(pinluz,OUTPUT);
  pinMode(pinbom,OUTPUT);
  pinMode(pinhum,OUTPUT);
  dht.begin();
  Setpointtemp=23.0;
  Setpointhum=70.0;
  tempPID.SetMode(AUTOMATIC);
  humPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(tempMin, tempMax);
  humPID.SetOutputLimits(haMin, haMax);
  delay(5000);
  eth_start(); 
}
 
 
 
/**
 * @brief Main loop
 */
void loop() {
  
 //Declaracion de variables
  float temperature;
  float temp2;
  float hum1;
  float hum2;
  int light;
  float h;
  int numChars = 0;
  int v1;
  int v2;
  int v3;
  int proyector;
  int bomba;
  int humidificador;
  time_t t = now();
  
  //Variables del PID
  Inputtemp = temp2;
  Inputhum = h;
  
  tempPID.Compute();
  humPID.Compute();
 
 //Obtencion de valores de los sensores
  temperature = getTemp(ntc1);
  temp2 = dht.readTemperature();
  hum1 = getHum(sh1);
  hum2 = getHum(sh2);
  light = getLight();
  h = dht.readHumidity();
//  t3= dht.ReadTemperature();
  
 //Actuacion sobre actuadores  
  activarVentilador();
  encenderBomba();
  encenderHumidificador();
//  encenderProyector();
 
 //Estado de los actuadores
  v1 = vent1State();
  v2 = vent2State(); 
  v3 = vent3State();
  bomba = bombaState();
  humidificador = humidState();
//  proyector = proyState();
  
  if((hour(t) > 7) && (hour(t) < 22))
  {
    proyector = 1;
  }
  else 
  {
    proyector = 0;
  }
  
  if(proyector==1)
  {
    digitalWrite(pinluz,LOW);
  }
  else
  {
    digitalWrite(pinluz,HIGH);
  }
  
    //First of all we need to add the name of measurement to beginning of the buffer
    numChars = sprintf(buf, "my_greenhouse_data,");
    numChars += sprintf(&buf[numChars], "SOURCE=arduinoMega2560 ");
    
    //after tags, comes the values!
 //   numChars += sprintf(&buf[numChars], "TEMP1=%d.%02d,", (int)temperature, (int)(temperature*100)%100);
    numChars += sprintf(&buf[numChars], "TEMP2=%d.%02d,", (int)temp2, (int)((temp2)*100)%100);
    numChars += sprintf(&buf[numChars], "HUM1=%d.%02d,", (int)hum1, (int)((hum1)*100)%100);
    numChars += sprintf(&buf[numChars], "HUM2=%d.%02d,", (int)hum2, (int)((hum2)*100)%100);
    numChars += sprintf(&buf[numChars], "HUMAMB=%d.%02d,", (int)h, (int)((h)*100)%100);
    numChars += sprintf(&buf[numChars], "LIGHT=%d,", light);
    numChars += sprintf(&buf[numChars], "VENT1=%d,", v1);
    numChars += sprintf(&buf[numChars], "VENT2=%d,", v2);
    numChars += sprintf(&buf[numChars], "VENT3=%d,", v3);
    numChars += sprintf(&buf[numChars], "PROYECTOR=%d,", proyector);
    numChars += sprintf(&buf[numChars], "BOMBA=%d,", bomba);
    numChars += sprintf(&buf[numChars], "HUMID=%d", humidificador);
 
    //Print the buffer on the serial line to see how it looks
    Serial.print("Sending following dataset to InfluxDB: ");
    Serial.println(buf);
    
    Serial.print(day(t));
   Serial.print("/");
   Serial.print(month(t));
   Serial.print("/");
   Serial.print(year(t));
   Serial.print(" ");
   Serial.print(hour(t));
   Serial.print(":");
   Serial.print(minute(t));
   Serial.print(":");
   Serial.println(second(t));
       
    //send to InfluxDB
    eth_send_data(buf, numChars);
 
    //we have to reset the buffer at the end of loop
    memset(buf, '\0', bufferSize);
    delay(5000);
  
}
