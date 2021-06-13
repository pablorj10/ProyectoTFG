//LIBRERIAS
#include<Wire.h>                  
#include<ADXL345_WE.h>           
#include <Arduino.h>             
#include <bluefruit.h>            
#include <Adafruit_LittleFS.h>    
#include <InternalFileSystem.h>   
#include <lmic.h>                
#include <hal/hal.h>              
#include <SPI.h>                 
#include "NRF52TimerInterrupt.h"
#include "NRF52_ISR_Timer.h"
#include <stdlib.h>

/////VARIABLES E INSTANCIAS ///////
#define ADXL345_I2CADDR 0x53                                         
#define TIMER_INTERRUPT_DEBUG                0
#define _TIMERINTERRUPT_LOG_LEVEL_           0
#define PININT1 14
#define PINLED 17
NRF52Timer ITimer0 (NRF_TIMER_1);
NRF52_ISR_Timer  ISR_Timer0;

NRF52Timer ITimer1 (NRF_TIMER_3);
NRF52_ISR_Timer  ISR_Timer1;

ADXL345_WE myAcc(ADXL345_I2CADDR);

unsigned long temporizador_predefinido = 10;
volatile uint32_t temporizador; //SEGUNDOS
volatile uint32_t temporizador_variable;
unsigned long tiempo_reinicio = 30;
volatile uint32_t contador_reinicio = 0;
unsigned long numero_reinicio;
volatile uint8_t tiempo_espera = 20; //Tiempo en el que estara el dispositivo buscando nodos para configurarlos. 
unsigned long tiempo;

volatile uint8_t estado = 1;
typedef struct
{
  char name[16+1];
  uint16_t conn_handle;
  BLEClientUart clientUart;
}prph_info_t;

prph_info_t prphs[3];

volatile bool power = true;
int cont = 0;
volatile int posicion = 0;
uint8_t datos = 0;
unsigned long tiempoinicio;
unsigned long tiempofin;

void registro_dato();


SoftwareTimer blinkTimer;

volatile bool parar_lorawan = false;
volatile bool parar_ble = false;
volatile bool salir_estado5 = false;
volatile bool salir_ble1 = false;
volatile bool salir_lorawan1 = false;
volatile bool reinicio = false;





String datoscentral = "Central";
char datos1[100];
char datos2[100];
char datos3[100];
char datos4[100];
volatile uint8_t contador = 0;

volatile uint8_t tiempo_inactividad = 4;
static const u1_t PROGMEM APPEUI[8]={0x3F,0x19,0x04,0xD0,0x7E,0xD5,0xB3,0x70};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={0x3E,0x4A,0x75,0xFE,0xFF,0xA8,0xC5,0x60};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xDC,0x20,0x25,0xE8,0xCA,0xD9,0x0E,0x4A,0xF7,0x96,0xB0,0x9E,0xC1,0x82,0x38,0x0E };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

volatile uint32_t lora_count = 0;
static osjob_t sendjob;
char Datos_conf[50] = "Configuracion";

uint8_t N = 10;
int Q;

volatile uint8_t cont1 = 0;
volatile uint8_t variable = 0;
typedef enum {
  LORA_UNCONFIRMED,
  LORA_CONFIRMED  
};

const unsigned TX_INTERVAL = 10;

const lmic_pinmap lmic_pins = {
    .nss = 4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {27, 28, 29},
};









////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////VARIABLE DINAMICOS////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int* datostiempo;
size_t counta;
size_t capacity;


void reinicio_datos()
{
  Serial.println("Valores reiniciados");
  datoscentral = "";
  datoscentral = "Central";
  datos = 0;
  cont = 0;
  posicion = 0;
  memset(datos2, 0, sizeof(datos2));
}

void convertir_datos()
{
  Serial.println("Datos convertidos");
  int datos_len = datoscentral.length() + 1;
  datoscentral.toCharArray(datos1, 100);
}


void CreateList(size_t _capacity)
{
  datostiempo = new int[_capacity];
  capacity = _capacity;
  counta = 0;
}

void AddItem(int item)
{
  ++counta;
  if(counta> capacity)
  {
    size_t newSize = capacity*2;
    resize(newSize);
  }
  datostiempo[counta -1] = item;
}
void RemoveTail()
{
  --counta;
}

void Trim()
{
  resize(counta);
  
}

void resize(size_t newCapacity)
{
  int* newList = new int[newCapacity];
  memmove(newList, datostiempo, counta * sizeof(int));
  delete[] datostiempo;
  capacity = newCapacity;
  datostiempo = newList;
}

void PrintList()
{
  Serial.print("Datos Tiempos: ");
  for (size_t index = 0; index < counta; index++)
  {
    Serial.print(datostiempo[index]);
    Serial.print(' ');
  }
  Serial.println();

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////TEMPORIZADOR///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void funcion_tempo(TimerHandle_t xTimerID)
{
    (void) xTimerID;
    Serial.println("TIEMPO----------------------------");
    salir_estado5 = true;
    estado = 6;
    Serial.println("Parar temporizador");
  
}
void tiempo_de_espera()
{
  parar_ble = true;
  estado = 3;
}
void temporizador_handler()
{
  if(estado == 2)
  {
    ISR_Timer1.run();
    ISR_Timer1.setTimeout(tiempo_espera*1000, tiempo_de_espera);
  }
  if(estado == 4)
  {
    
    //ISR_Timer0.run();
    //ISR_Timer0.setTimeout(tiempo_reinicio*1000, funcion_tempo);

  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////LORAWAN/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void apagar_lorawan()
{
  os_clearCallback(&sendjob);
  LMIC_shutdown();
  Serial.println("LoRaWAN apagado");
  parar_lorawan = true;
  if(estado == 1){estado = 2;}
  if(estado == 7){estado = 8; salir_lorawan1 = true;}
}




void envio_lorawan_conf(osjob_t* j)
{
  if(LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    LMIC_setTxData2(1, (unsigned char *) Datos_conf, strlen(Datos_conf), LORA_CONFIRMED);
    Serial.println("Dato enviado a LoRaWAN");
    contador++;
    
    if(contador == 5 && temporizador == 0)
    {
      Serial.println("No se ha establecido conexión con LoRaWAN");
      Serial.println("Se establece el temporizador predeterminado de 400 segundos");
      temporizador = temporizador_predefinido;
      apagar_lorawan();
    }
  }
}

void envio_lorawan_datos(osjob_t* j)
{
  switch(variable)
  {
    case 0:
    Serial.println("CASO 0");
    convertir_datos();
    if(strlen(datos1) > 1)
    {
      Serial.println("Datos1 hay datos");
      LMIC_setTxData2(1, (unsigned char *)datos1, strlen(datos1), LORA_CONFIRMED);
      Serial.println(F("Paquete enviado 1"));
      variable++;
    }
    else
    {
      Serial.println("No hay datos1");
      variable = 0;
    }
    break;

    case 1:
    Serial.println("CASO 1");
    if(strlen(datos2) > 1 && *datos2 != '0')
    {
      LMIC_setTxData2(1, (unsigned char *)datos2, strlen(datos2), LORA_CONFIRMED);
      Serial.println(F("Paquete enviado 2"));
      variable++;
    }
    else
    {
      Serial.println("No hay más datos para enviar por LoRaWAN");
      variable = 0;
      apagar_lorawan();
      break;
    }
    break;

    case 2:

    Serial.println("CASO 2");
    if(strlen(datos3) > 1 && *datos3 != '0')
    {
      LMIC_setTxData2(1, (unsigned char *)datos3, strlen(datos3), LORA_CONFIRMED);
      Serial.println(F("Paquete enviado 3"));
      variable++;
    }
    else
    {
      Serial.println("No hay más datos para enviar por LoRaWAN");
      variable = 0;
      apagar_lorawan();
      break;
    }
    break;

    case 3:

    Serial.println("CASO 3");
    if(strlen(datos4) > 1 && *datos4 != '0')
    {
      LMIC_setTxData2(1, (unsigned char *)datos4, strlen(datos4), LORA_CONFIRMED);
      Serial.println(F("Paquete enviado 4"));
      variable = 0;
    }
    else
    {
      Serial.println("No hay más datos para enviar por LoRaWAN");
      variable = 0;
      apagar_lorawan();
      break;
    }
    break;
  }
  
}






void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            //Serial.println(F("EV_JOINING"));
            Serial.println(F("Comenzando a Unirse..."));
            break;
        case EV_JOINED:
            Serial.println(F("Enlazado con LoraWAN"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.println("Unido a LoraWAN");
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");

              lora_count = 1;
            }

            LMIC_setLinkCheckMode(0);
            break;

        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:

            if(estado == 1)
            {
              if (LMIC.dataLen == 1) 
              {
                temporizador = LMIC.frame[LMIC.dataBeg + 0];
                Serial.println("Dato de temporizador recibido");
                Serial.print("Tiempo de temporizador: "); 
                Serial.println(temporizador);
                
                apagar_lorawan();
                break;
              } 
            }  
            else
            { 
              
              if (LMIC.txrxFlags & TXRX_ACK)
              {
              Serial.println(F("Recibida Confirmacion \r\n"));
              }
    
              if (LMIC.dataLen) 
              {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
              
              }                                         
            }
            
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;

        case EV_TXSTART:
        
            if(estado == 1)
            {
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), envio_lorawan_conf);
            }
            if(estado == 7)
            {
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), envio_lorawan_datos);
            }
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}




void lorawan_conf()
{
  os_init();
  LMIC_reset();
  if(estado == 1){envio_lorawan_conf(&sendjob);}
  if(estado == 7){envio_lorawan_datos(&sendjob);}
  while(1)
  {
    if(parar_lorawan == true) {break;}
    os_runloop_once();
  }
  
  
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////BLUETOOTH///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void scan_callback(ble_gap_evt_adv_report_t* report)
 {
  Serial.println("Detectado dispositivo");
  Serial.println("Conectando...");
  Bluefruit.Central.connect(report);
 }

int findConnHandle(uint16_t conn_handle)
{
   for(int id=0; id<3; id++)
  {
    if (conn_handle == prphs[id].conn_handle)
    {
      return id;
    }
  }

  return -1;  
}

void conexion_callback(uint16_t conne_handle)
{
  int id = findConnHandle(BLE_CONN_HANDLE_INVALID);
  if(id < 0)return;
  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conne_handle;
  Bluefruit.Connection(conne_handle)->getPeerName(peer->name, sizeof(peer->name)-1);
  Serial.print("Conectado a ");
  Serial.println(peer->name);
  if(prphs[id].clientUart.discover(conne_handle))
  {
    prphs[id].clientUart.enableTXD();
    Bluefruit.Scanner.resume();
  }
  else
  {
    Bluefruit.disconnect(conne_handle);
  }
  
}

void desconexion_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  int id = findConnHandle(conn_handle);
  if(id<0) return;
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;
  Serial.print(prphs[id].name);
  Serial.println(" desconectado");
}

void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  if(estado == 6)
  {
    Serial.println("Recibiendo dato por BLE");
    while(uart_svc.available())
    {
      Serial.println("");
      Serial.print("Dato recibido: ");
      if(uart_svc.read(datos2, sizeof(datos2)-1))
      {
        Serial.println(datos2);
      
      }
      if(strlen(datos2) > 1)
      {
        estado = 7;
        salir_ble1 = true;
        break;
      }
    }
  }
}

 void config_ble()
 {
  Bluefruit.begin(0,3);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Dispositivo Central");
  for (uint8_t idx=0; idx<3;idx++)
  {
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    prphs[idx].clientUart.begin();
    prphs[idx].clientUart.setRxCallback(bleuart_rx_callback);
    
  }
  Bluefruit.Central.setConnectCallback(conexion_callback);
  Bluefruit.Central.setDisconnectCallback(desconexion_callback);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(false);
  Serial.println("BLE Configurado");
  if (estado == 2)
  {
    Serial.println("Comienzo de conexión con nodos");
    ITimer1.attachInterruptInterval(tiempo_espera*1000, temporizador_handler);
    
  }
  Bluefruit.Scanner.start(20000);
  Serial.println("Escaneando....");
  
 }
 
 void sendAll(uint32_t buf)
 {
  Serial.print("Enviando a todos los nodos: ");
  Serial.println(buf);
  for(uint8_t id=0; id<3 ;id++)
  {
    prph_info_t* peer = &prphs[id];
    if (peer->clientUart.discovered())
    {
      delay(1000);
      peer->clientUart.write(buf);
    }
  }
 }


void desconexion_ble()
{
  for(uint8_t id = 0; id<3; id++)
  {
    uint16_t conn_handle = prphs[id].clientUart.connHandle();
    Bluefruit.disconnect(conn_handle);
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////SETUP/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  //pinMode(PININT1, INPUT);
  pinMode(PINLED, OUTPUT);
  digitalWrite(PINLED, HIGH);
  delay(3000);
  digitalWrite(PINLED, LOW);
  Serial.println("Inicio Dispositivo Central");
  Serial.println();
  CreateList(1);

}





void conf_acelerometro()
{
  if (!myAcc.init()) {
    Serial.println("ADXL345 no conectado!");
  }
  myAcc.setCorrFactors(-266.0, 285.0, -268.0, 278.0, -291.0, 214.0);
  myAcc.setDataRate(ADXL345_DATA_RATE_25); //Se establece el DATA RATE del acelerometro en 25 Hz
  myAcc.setRange(ADXL345_RANGE_4G); //Se establece el rango de medida del acelerometro en 4g.
  attachInterrupt(digitalPinToInterrupt(PININT1), registro_dato, ISR_DEFERRED | RISING); //Se declara la interrupcion por actividad del acelerometro. 
  myAcc.setActivityParameters(ADXL345_AC_MODE, ADXL345_XYZ, 0.8); //Se establecen los parametros necesarios para la deteccion de actividad (Modo AC, tres ejes, limite de deteccion)
  myAcc.setInactivityParameters(ADXL345_DC_MODE, ADXL345_XYZ, 1, tiempo_inactividad); //Se establecen los parametros necesarios para la deteccion de la inactividad (Modo AC, tres ejes, limite de deteccion, tiempo para deteccion)
  myAcc.setLinkBit(true); //Se configura el Link Bit para que una vez detecte actividad no vuelva a detectarla hasta que detecte inactividad. 
  myAcc.setInterrupt(ADXL345_ACTIVITY, INT_PIN_1); //Se configura el Pin INT1 como la interrupcion de salida del acelerometro para la actividad.
  myAcc.setInterrupt(ADXL345_INACTIVITY, INT_PIN_1); //Se configura el Pin INT1 como la interrupcion de salida del acelerometro para la inactividad.
  Serial.println("Acelerometro configurado");
}








///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////REGISTRO DEL DATO///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void registro_dato()
{
  byte intSource = myAcc.readAndClearInterrupts();
  if(myAcc.checkInterrupt(intSource, ADXL345_ACTIVITY))
  {
    tiempo = 0;
    datos++;
    tiempoinicio = millis();
    digitalWrite(PINLED, HIGH);
    Serial.print("ACTIVIDAD ");
    Serial.println(datos);
  }
  if(myAcc.checkInterrupt(intSource, ADXL345_INACTIVITY))
  {
    digitalWrite(PINLED, LOW);
    tiempofin = millis();
    if(datos == 0){Serial.println("INACTIVIDAD");}
    else
    {
      Serial.println("INACTIVIDAD");
      tiempo = tiempofin - tiempoinicio;
      if(tiempo < tiempo_inactividad*1000)
      {
         Serial.println(tiempo/1000);
         datoscentral.concat(";" + String(tiempo/1000));
         Serial.print("Datos: ");
         Serial.println(datoscentral);
      }
      else
      {
        tiempo = tiempo - tiempo_inactividad*1000;
        Serial.println(tiempo/1000);
        datoscentral.concat(";" + String(tiempo/1000));
        Serial.print("Datos: ");
        Serial.println(datoscentral);
      }
     
    }
  }
  myAcc.readAndClearInterrupts();
}











void loop()
{
  switch(estado)
  {


    //---------------------------------------------------------------------------------------------------
    //Estado de configuración en el que se conecta con lorawan y se recibe el tiempo del temporizador. 
    case 1:

    Serial.println("Configurando dispositivo");
    Serial.println("A la espera de conexión con LoRaWAN...");
    lorawan_conf();
    
    break;



    //--------------------------------------------------------------------------------------------------
    //Estado de configuracion en el que se configura el bluetooth y se comienza a escanear nodos
    case 2:

    Serial.println("Configurando los demás nodos....");

    config_ble();
    while(1)
    {
      if(parar_ble == true){break;}
      sd_app_evt_wait();
      
    }
    parar_ble = false;
    break;
    //------------------------------------------------------------------------------------------------
    //Estado de configuracion en el que se envía el tiempo a todos los nodos conectados.
    case 3:
    ITimer1.stopTimer();
    Serial.println("Tiempo de escaneo superado");
    Serial.println("Enviando datos a los nodos conectados");
    sendAll(temporizador);
    desconexion_ble();
    uint8_t conexiones;
    
     
     while(1)
     {
      for(uint8_t id = 0; id<3; id++)
      {
        uint16_t conn_handle = prphs[id].clientUart.connHandle();
        if(Bluefruit.connected(conn_handle) == true)
        {
          conexiones++;
        }
        
      }
     
      if(conexiones == 0)
      {
        estado = 4;
        break;
      }
      else
      {
        desconexion_ble();
        
      }
     }
    
    break;

    //-----------------------------------------------------------------------------------------------------
    //Estado en el que se configura el temporizador con el tiempo recibido por LoRaWAN y se configura
    //también el acelerómetro.
    case 4:
    temporizador_variable = temporizador;
    numero_reinicio = tiempo_reinicio/temporizador_variable;
    Serial.print("Temporizador variable: ");
    Serial.println(temporizador_variable);
    //ITimer0.attachInterruptInterval(temporizador_variable*1000000, temporizador_handler);
    //ITimer2.attachInterruptInterval(temporizador_variable*1000000, temporizador_handler);
    temporizador_variable = temporizador;
    blinkTimer.begin(temporizador_variable*1000*60, funcion_tempo, NULL, false);
    blinkTimer.start();
    Serial.println("Temporizador iniciado");
    conf_acelerometro();
    estado = 5;
   
    break;

    //---------------------------------------------------------------------------------------------------------
    //Estado en de normalidad de bajo consumo en el que se está registrando datos hasta que salta el temporizador
    //y se sale de este estado al estado de recepción de los datos de los nodos. 
    case 5:

    while(1)
    {
      if(salir_estado5 == true){break;}
      sd_app_evt_wait();
    
    
    }
    salir_estado5 = false;
    break;

    //-------------------------------------------------------------------------------------------------------------
    //Estado en el que se configura el bluetooth para recibir los datos de los nodos, se conecta a los nodos 
    // y se reciben los datos. 
    case 6:

    Serial.println("Final de toma de datos");
    Serial.println("Configurando BLE");
    //ITimer0.stopTimer();
    blinkTimer.stop();
    config_ble();
    while(1)
    {
      if(salir_ble1 == true){break;}
      sd_app_evt_wait();
      
    }
    salir_ble1 = false;
    break;

    //----------------------------------------------------------------------------------------------------------------
    //Estado en el que se envían los datos recibidos por LoRaWAN
    case 7:
    Serial.println("Enviando datos a LoRaWAN...");
    parar_lorawan = false;
    lorawan_conf();
    while(1)
    {
      if(salir_lorawan1 == true){break;}
      sd_app_evt_wait();
      
      
    }
    salir_lorawan1 = false;
    break;

    //----------------------------------------------------------------------------------------------------------------
    //Estado en el que se desconectan todos los nodos.
    case 8:

     Serial.println("Desconectando todos los nodos....");
     desconexion_ble();
     conexiones = 0;
     while(1)
     {
      for(uint8_t id = 0; id<3; id++)
      {
        uint16_t conn_handle = prphs[id].clientUart.connHandle();
        if(Bluefruit.connected(conn_handle) == true)
        {
          conexiones++;
        }
        
        
      }
      
      if(conexiones == 0)
      {
        estado = 9;
        break;
      }
      else
      {
        desconexion_ble();
        
      }
     }
    
     break;

     //-----------------------------------------------------------------------------------------
     //Estado en el que se reinician todos los valores de las variables y el temporizador. 
     case 9: 

     contador_reinicio++;
     reinicio_datos();
     //ISR_Timer0.run();
     //ISR_Timer0.setTimeout(temporizador_variable*1000, funcion_tempo);
     
     
     blinkTimer.reset();
     //blinkTimer.begin(temporizador*1000, funcion_tempo);
     blinkTimer.begin(temporizador_variable*1000*60, funcion_tempo, NULL, false);
     blinkTimer.start();
     
     estado = 5;
     if(contador_reinicio == numero_reinicio){estado = 10;}
     break;
//----------------------------------------------------------------------------------------------------------------------
     //Estado en el que se reinicia el dispositivo para comenzar una nueva configuración.
     case 10:
     Serial.println("REINICIO");
     Serial.println("--------------------------------------------------------------------------------------------");
     NVIC_SystemReset();
     break;
     





  }


}
