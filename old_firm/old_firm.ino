#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#define RXD2 16
#define TXD2 17
#include <Wire.h>
#define I2C_SDA 21                    
#define I2C_SCL 22
#include <TinyGPSPlus.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESP32Time.h>

TinyGPSPlus gps;
Servo servo1;
Adafruit_MPU6050 mpu;
ESP32Time rtc;

#define I1 36  
#define I2 39
#define I3 34
#define I4 35
#define I5 32
#define I_maq_onoff 33    /////si esta apagada manda 0, encendida 1
#define cont_premios 25  //CONTADOR PREMIOS  en la tarjeta el derecho
#define cont_monedas 26  //CONTADOR MONEDERO  lado izquiero

int servo1Pin = 18;
#define LED 2
#define P_ir 5  //Salida ir
#define relay_prog 15  //Salida ir
#define relay_onoff 4  //Salida ir


unsigned int guarda=0,flag_moneda=0,flag_premio=0;
const int eepromAddress = 0x50;
char buffer_sprinf[256];
unsigned int flag_sensores_puertas=1; //flag_maquina_encendida=0,
int salida_onoff=0,flag_servo_activo=0,flag_timerservooff=0;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -6*3600;
const int   daylightOffset_sec = 0;

int hora_inicio=0,hora_fin=0;
int hora=0,cordenada=0;
double latitud=0;
double longitud=0;
char str_gps[64];
int var_2horas=0,var_monh=0;

const char* ssid = "Microtech1";
const char* password = "Microtech-tl01";
const char* mqttServer = "node02.myqtthub.com";
const int mqttPort = 1883;
const char* mqttUser = "yumiso_hub";
const char* mqttPassword = "maquina_garra";
char buffer_union_publish[30]; 
char buffer_union_subcribe[30];
const char* publish_topic="/mensajes"; 
const char* subcribe_topic="/puertas"; 

const char* client_id = "maquina00018";////////////////////////////////////////////
#define Ubicacion 18  /////////////////////////////////////////////////////////////
#define Nombre 18    //////////////////////////////////////////////////////////////

WiFiClient espClient;
PubSubClient client(espClient);
WiFiClient client_http;

#define DEBOUNCE_TIME 50   // Filtre anti-rebond (debouncer)
volatile uint32_t DebounceTimer = 0;

#define ESPERA_30_SEGUNDOS 30000   // Filtre anti-rebond (debouncer)
volatile uint32_t timer_30s = 0; //  acelerometro
volatile uint32_t timer_30sp = 0;//  puertas
volatile uint32_t timer_30sa = 0;//  apagada maquina

void IRAM_ATTR botonpress_premios(){  /////////////////////////////probar un pulso de 20ms
  if ( millis() - DEBOUNCE_TIME  >= DebounceTimer ){
    DebounceTimer = millis();
    if (digitalRead(cont_premios)==0){
       flag_premio++;
    }
  }
}

void IRAM_ATTR botonpress_monedas(){
if ( millis() - DEBOUNCE_TIME  >= DebounceTimer ) {
    DebounceTimer = millis();
    if (digitalRead(cont_monedas)==0){
       flag_moneda++;
       var_monh=1;
    }
  }
}

void setup() {
  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);      // Standard 50hz servo
  servo1.attach(servo1Pin, 500, 2400);
  servo1.write(180);               ///tira objetos
  delay(2000);
  servo1.write(30);                 ///abierta
  //servo1.detach();
  
  pinMode(LED,OUTPUT);
  pinMode(P_ir, OUTPUT); //
  pinMode(relay_onoff, OUTPUT); //
  pinMode(relay_prog, OUTPUT); //
  
  digitalWrite(LED, LOW);
  digitalWrite(P_ir, LOW);
  digitalWrite(relay_onoff, LOW);
  digitalWrite(relay_prog, LOW);

  pinMode(I1, INPUT_PULLUP);
  pinMode(I2, INPUT_PULLUP);
  pinMode(I3, INPUT_PULLUP);
  pinMode(I4, INPUT_PULLUP);
  pinMode(I5, INPUT_PULLUP);
  pinMode(I_maq_onoff, INPUT_PULLUP);
  pinMode(cont_premios, INPUT_PULLUP);
  pinMode(cont_monedas, INPUT_PULLUP);

  attachInterrupt(cont_premios, botonpress_premios,FALLING);
  attachInterrupt(cont_monedas, botonpress_monedas,FALLING);
  
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  WiFi.begin(ssid, password);  //hasta aqui aun no se conecta
  //Serial.println(WiFi.localIP());
  
  digitalWrite(LED, HIGH); 
  delay(200); 
  digitalWrite(LED, LOW); 
  delay(500);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  hora_inicio=readI2CByte(14);
  hora_fin=readI2CByte(16);

  if(mpu.begin()){
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    digitalWrite(LED, HIGH);
    delay(200); 
    digitalWrite(LED, LOW); 
  }

}

void loop(){
strcat(strcpy(buffer_union_publish,client_id),publish_topic);

  digitalWrite(LED, LOW);
  delay(500);
  mems_o();
  delay(500);
  mems_o();
  delay(500);
  mems_o();
/*  
  if (flag_sensores_puertas){
      sensores_sms(); 
    }  
 */   
  delay(500);
  mems_o();    
  delay(500);
  mems_o();
  delay(500);
  mems_o();
 /* 
  if (flag_sensores_puertas){
      sensores_sms(); 
    }  
 */   
  digitalWrite(LED, HIGH);


  if (WiFi.status() != WL_CONNECTED){
   Serial.println("try to conect to wifi");
   WiFi.begin();
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }
  
  if(!client.connected()){
          reconnect();
  } 
  client.loop();// comprueba que no haya mensajes pendientes de entrada y mantiene la conexión con el server
  
  mon_y_sal();

  /*
  if (flag_servo_activo){
    flag_servo_activo++;
    if (flag_servo_activo>20){   //cada minuto aprox
      flag_servo_activo=1;
      client.publish(buffer_union_publish,"volteada"); 
    } 
  }
 */

  no_ventas();

  if (digitalRead(I_maq_onoff)==0){ //alarma  //&& flag_maquina_encendida==0
    if ( millis() - ESPERA_30_SEGUNDOS  >= timer_30sa ){
        timer_30sa = millis();
        client.publish(buffer_union_publish,"Apagada");
    }
  }
}


void no_ventas(void){
strcat(strcpy(buffer_union_publish,client_id),publish_topic);
  var_2horas++;
      if (var_2horas>100){  /////aqui ajustar las horas      //30 min   
        var_2horas=0;
           if (var_monh){
           var_monh=0;
           }
           else{
            hora=get_wifi_hora();
            if (hora!=-1){
                  Serial.println(hora); 
                  if (hora>=hora_inicio && hora<hora_fin){    ////////////poner el rango de horas
                    client.publish(buffer_union_publish,"No_ventas");  
                    }
            }
            }
        }
  }

void callback(char* topic, byte* payload, unsigned int length) {
strcat(strcpy(buffer_union_publish,client_id),publish_topic);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  switch (payload[0]-48){  //checar en ascci que sigue despues del 9
  case 0:  if (salida_onoff){
                 salida_onoff=0;
                 digitalWrite(relay_onoff, LOW);
              }
              else{
                salida_onoff=1;
                 digitalWrite(relay_onoff, HIGH);
                }
    break;
    
    case 1:  hora_inicio=payload[1]-48; 
             hora_inicio*=10;
             hora_inicio+=payload[2]-48; 
             writeI2CByte(14,hora_inicio);
    break;
    
    case 2:  hora_fin=payload[1]-48; 
             hora_fin*=10;
             hora_fin+=payload[2]-48;
             writeI2CByte(16,hora_fin);
    break;

    case 3:  flag_sensores_puertas=0;  
    break;
    case 4:  flag_sensores_puertas=1;  
    break;
    
  case 7:    //limpia los sensores
    break;
  case 8:    digitalWrite(relay_prog, HIGH);
             delay(2000);
             digitalWrite(relay_prog, LOW);
    break;
   case 9:    ESP32PWM::allocateTimer(0);
              servo1.setPeriodHertz(50);
              servo1.attach(servo1Pin, 500, 2400);  //en el programa otro tenia 2 instrcciones antes
              servo1.write(30);    //////////abierta
              //delay(3000);
              //servo1.detach();
              flag_servo_activo=0;
    break;
   case 10:   cordenada=get_gps(latitud,longitud);
              if (cordenada!= -1){
              sprintf(str_gps,"=%.6f,%.6f",latitud,longitud);
              client.publish(buffer_union_publish,str_gps);   
              Serial.print(str_gps);
              Serial.println();
             /*
              Serial.print(latitud, 6);
              Serial.print(",");
              Serial.print(longitud, 6);
              Serial.println();
              */
              }
              else{
                  Serial.print("no gps");
                  client.publish(buffer_union_publish,"0.000000,0.000000");           
              }
    break;
  default:
  break;
  }
}


void mon_y_sal(void){
//////////////////////////////////////////////////////////////////////////////entrada de mondero
      if (flag_moneda){
        delay(500);  //////////////////////////////////////por si llego otra moneda en el trasncurso
        guarda=readI2CByte(10);   ///entrada de monedas
        guarda+=flag_moneda;
        flag_moneda=0;
        sprintf(buffer_sprinf,"/peluches/w_mon.php?Ubicacion=%i&Nombre=%i&Monto=%i",Ubicacion,Nombre,guarda);
        if (insrt_db_data(buffer_sprinf)!=-1){
          guarda=0; 
        }
        writeI2CByte(10,guarda);
      }
/////////////////////////////////////////////////////////////////////////////salida de peluches
       if (flag_premio){ 
        guarda=readI2CByte(12);   ///salida de muñecos
        guarda+=flag_premio;
        flag_premio=0;
        sprintf(buffer_sprinf,"/peluches/w_sal.php?Ubicacion=%i&Nombre=%i&Monto=%i",Ubicacion,Nombre,guarda);
        if (insrt_db_data(buffer_sprinf)!=-1){
          guarda=0; 
        }
        writeI2CByte(12,guarda);
       }     
//////////////////////////////////////////////////////////////////////////////////
}

void mems_o(void){
strcat(strcpy(buffer_union_publish,client_id),publish_topic);
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);
if (a.acceleration.x>3 || a.acceleration.y>3 || a.acceleration.x<-3 || a.acceleration.y<-3 ){   //tenia 1.5
  if ( millis() - ESPERA_30_SEGUNDOS  >= timer_30s ){
    timer_30s = millis();
    Serial.println("movimiento");
    ESP32PWM::allocateTimer(0);
    servo1.setPeriodHertz(50);      // Standard 50hz servo
    servo1.attach(servo1Pin, 500, 2400);
    servo1.write(120);   /// cierra compuerta
    //client.publish(buffer_union_publish,"volteada");  
    flag_servo_activo=1;
    }
}
        else{
      if((flag_servo_activo==1)){
      flag_timerservooff++;
      }
      if((flag_timerservooff>=100)){
       flag_servo_activo=0;
       flag_timerservooff=0;
         ESP32PWM::allocateTimer(0);
         servo1.setPeriodHertz(50);      // Standard 50hz servo
          servo1.attach(servo1Pin, 500, 2400);
          servo1.write(180);               ///tira objetos
          delay(2000);
          servo1.write(30); 
        }
      }
}


void reconnect(){  
strcat(strcpy(buffer_union_subcribe,client_id),subcribe_topic);
  if(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    if (client.connect(client_id, mqttUser, mqttPassword)){
      Serial.println("connected");
      client.subscribe(buffer_union_subcribe);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try in the next");
    }
  }
}

void sensores_sms(void){
strcat(strcpy(buffer_union_publish,client_id),publish_topic);
int var_sen=0;   
    var_sen = sensores();
    if (var_sen){

      if ( millis() - ESPERA_30_SEGUNDOS  >= timer_30sp ){
        timer_30sp = millis();
        switch (var_sen) {
          case 1: client.publish(buffer_union_publish,"p_garrra"); 
            Serial.println("sensor 1");
          break;
          case 2: client.publish(buffer_union_publish,"p_tarjeta"); 
            Serial.println("sensor 2");
          break;
          case 3: client.publish(buffer_union_publish,"p_monedero"); 
            Serial.println("sensor 3");
          break;
          case 4: client.publish(buffer_union_publish,"p_dinero"); 
            Serial.println("sensor 4");
          break;
          default:
          break;
        }
      }
   
    }
}



unsigned int sensores(void){
  digitalWrite(P_ir,HIGH); 
  delay(10);                        //1ms no alcanza, minimo 10
  if (digitalRead(I1)){
    digitalWrite(P_ir,LOW);
    return 1;
    }
  digitalWrite(P_ir,LOW); 
  delay(10);
  if (digitalRead(I1)==0){
    return 1;
    }

  digitalWrite(P_ir,HIGH); 
  delay(10);                        //1ms no alcanza, minimo 10
  if (digitalRead(I2)){
    digitalWrite(P_ir,LOW);
    return 2;
    }
  digitalWrite(P_ir,LOW); 
  delay(10);
  if (digitalRead(I2)==0){
    return 2;
    }

  digitalWrite(P_ir,HIGH); 
  delay(10);
  if (digitalRead(I3)){
    digitalWrite(P_ir,LOW);
    return 3;
    }
  digitalWrite(P_ir,LOW); 
  delay(10);
  if (digitalRead(I3)==0){
    return 3;
    }

  digitalWrite(P_ir,HIGH); 
  delay(10);
  if (digitalRead(I4)){
    digitalWrite(P_ir,LOW);
    return 4;
    }
  digitalWrite(P_ir,LOW); 
  delay(10);
  if (digitalRead(I4)==0){
    return 4;
    }

 return(0);
}


int insrt_db_data(char *buf){   
  if (!client_http.connect("www.microtech-tl.com", 80)) {
    return -1;
  }
  client_http.print(String("GET ") + buf + " HTTP/1.1\r\n" + "Host: " + "www.microtech-tl.com" + "\r\n" + "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client_http.available() == 0){
    if (millis() - timeout > 3000) {
      Serial.println(">>> Client Timeout !");
      client_http.stop();
      return -1;
     }
  }
    // Read all the lines of the reply from server and print them to Serial
  while(client_http.available()) {
    String line = client_http.readStringUntil('\r');
    Serial.print(line);
  }
  return 0;
}

int get_gps(double &latitud, double &longitud){
  while (Serial2.available() > 0){   //todo el buffer aprox 4 daos completos
    if (gps.encode(Serial2.read())){
      if (gps.location.isValid()){
         latitud=gps.location.lat();
         longitud=gps.location.lng();
         return (1);
        }
      }
    }
  return -1;
}

int get_wifi_hora(void){
int hora=0;
struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return (-1);
  }
  return timeinfo.tm_hour;
}

void writeI2CByte(byte data_addr, unsigned int data){
  byte a,b;
  a=data>>8;
  b=data;
  Wire.beginTransmission(eepromAddress);
  Wire.write(data_addr);
  Wire.write(a);
  Wire.write(b);
  Wire.endTransmission();
}

unsigned int readI2CByte(byte data_addr){
  byte a,b;
  unsigned int data = NULL;
  Wire.beginTransmission(eepromAddress);
  Wire.write(data_addr);
  Wire.endTransmission();
  Wire.requestFrom(eepromAddress, 2); //retrieve 2 returned byte
  a = Wire.read();    // receive a byte as character
  b = Wire.read(); 
  data=a;
  data<<=8;
  data|=b;
  return data;
}
