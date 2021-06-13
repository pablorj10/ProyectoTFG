////////////LIBRERIAS, VARIABLES E INSTANCIAS///////////////
#include<Wire.h>
#include<ADXL345_WE.h>
#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "NRF52TimerInterrupt.h"
#include "NRF52_ISR_Timer.h"
#include <stdlib.h>
#define TIMER_INTERRUPT_DEBUG                0
#define _TIMERINTERRUPT_LOG_LEVEL_           0
#define ADXL345_I2CADDR 0x53 // 0x1D if SDO = HIGH
#define PININT1 14
//#define PINLED 17

ADXL345_WE myAcc(ADXL345_I2CADDR);
BLEUart bleuart;
NRF52Timer ITimer0 (NRF_TIMER_1);

SoftwareTimer blinkTimer;
volatile uint8_t datos = 0;
void registro_dato();

volatile bool salir_ble = false;
volatile bool salir_recibirdato = false;
volatile bool salir_estado4 = false;
volatile bool salir_ble1 = false;
volatile bool salir_desconexion = false;


volatile uint8_t estado = 1;
volatile uint32_t temporizador;
uint32_t  tiempo_reinicio = 30;
volatile uint32_t contador_reinicio = 0;
volatile uint32_t numero_reinicio;
unsigned long tiempoinicio;
unsigned long tiempofin;
unsigned long tiempo;
uint8_t tiempo_inactividad = 5;



String datos1 = "Nodo1";
char datos2[100];




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////VARIABLE DINAMICOS////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int* datostiempo;
size_t counta;
size_t capacity;


void reinicio_datos()
{
  datos1 = "";
  datos1 = "Nodo1";
  datos2[0] = '\0';
  datos = 0;
  Serial.println("Datos reiniciados");
}

void convertir_datos()
{
  int datos_len = datos1.length() + 1;
  datos1.toCharArray(datos2, 100);
  Serial.println("Datos convertidos");
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
   Serial.println("TIEMPO------------------------");
   if(estado == 4)
   {
    salir_estado4 = true;
    estado = 5;
   }
    Serial.println("Parar temporizador");
  
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////BLUETOOTH///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void conexion_callback(uint16_t conn_handle)
{
  Serial.println("Conectado");
  Bluefruit.Advertising.stop();
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  
  if(estado == 1)
  {
    salir_ble = true;
    estado = 2;
  }
  
  if(estado == 5)
  {
    delay(3000);
    convertir_datos();
    Serial.print("Datos enviados: ");
    Serial.println(datos2);
    bleuart.write(datos2);
    
    
  }
  
}

void desconexion_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.println("Desconectado del maestro");
  if(estado == 2)
  {
    salir_recibirdato = true;
    estado = 3;
  }
  if(estado == 5)
  {
    salir_ble1 = true;
    estado = 6;
  }
}

void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;
  Serial.println("Recibiendo datos...");
  uint8_t* prueba;
  while(bleuart.available())
  {
    *prueba = bleuart.read();
    temporizador = *prueba;
    Serial.print("El dato recibido es: ");
    Serial.println(temporizador);
    if (temporizador != 0)
    {
      break;
    }
  }
}
void startAdv()
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower(); //Se añade la potencia que tendra el paquete de Advertising
  Bluefruit.Advertising.addService(bleuart); //Se añade el servicio al paquete de advertising
  Bluefruit.ScanResponse.addName(); //Se añade el nombre al proceso de advertising
  Bluefruit.Advertising.restartOnDisconnect(true);  //Se activa el auto advertising en caso de desconexion
  Bluefruit.Advertising.setInterval(32, 244); //Se configura el intervalo del advertising
  Bluefruit.Advertising.setFastTimeout(30); 
  Bluefruit.Advertising.start(0); //0 significa que no para de realizar el advertising despues de n segundos. 
  Serial.println("Advertising...");
  
}


void config_ble()
{
 
  Bluefruit.autoConnLed(false); //Se desconecta el led de conexion
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); //Se configura el periferico con el maximo ancho de banda
  Bluefruit.begin(); //Comienza la configuracion de BLE
  Bluefruit.setTxPower(4);  //Se configura la potencia de transmision a 4 
  Bluefruit.setName("Nodo1"); //Se configura el nombre con el que se mostrara
  bleuart.setRxCallback(bleuart_rx_callback);
  Bluefruit.Periph.setConnectCallback(conexion_callback); //Se indica la funcion a la que se ira el programa cuando se realice la conexion
  Bluefruit.Periph.setDisconnectCallback(desconexion_callback);//Se indica la funcion a la que se ira el programa cuando se realice la desconexion.
  bleuart.begin(); //Comienza el servicio de BleUart.
  Serial.println("BLE configurado");
  startAdv(); //Comienza el proceso de advertising.
}

void enviar_datos_ble()
{
  delay(3000);
  convertir_datos();
  bleuart.print(datos2);
  Serial.print("Datos enviados: ");
  Serial.println(datos2);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////SETUP/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  //pinMode(PININT1, INPUT);
  //pinMode(PINLED, OUTPUT);
  //digitalWrite(PINLED, HIGH);
  delay(3000);
  //digitalWrite(PINLED, LOW);
  Serial.println("Inicio Dispositivo NODO");
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
    //digitalWrite(PINLED, HIGH);
    Serial.println("ACTIVIDAD ");
    
  }
  if(myAcc.checkInterrupt(intSource, ADXL345_INACTIVITY))
  {
    //digitalWrite(PINLED, LOW);
    tiempofin = millis();
    if(datos == 0){Serial.println("INACTIVIDAD");}
    else
    {
      Serial.println("INACTIVIDAD");
      tiempo = tiempofin - tiempoinicio;
      if(tiempo < tiempo_inactividad*1000)
      {
         Serial.println(tiempo/1000);
         datos1.concat(";" + String(tiempo/1000));
         Serial.print("Datos: ");
         Serial.println(datos1);
      }
      else
      {
        tiempo = tiempo - tiempo_inactividad*1000;
        Serial.println(tiempo/1000);
        datos1.concat(";" + String(tiempo/1000));
        Serial.print("Datos: ");
        Serial.println(datos1);
      }
     
    }
  }
  myAcc.readAndClearInterrupts();
}








void loop()
{
  switch(estado)
  {
    //------------------------------------------------------------------------------------------------------
    //Estado en el que se configura el Bluetooth y comienza el advertising para mostrarse hacia el maestro.
    case 1:

    Serial.println("Configurando BLE...");
    config_ble();
    while(1)
    {
      if(salir_ble == true){break;}
      sd_app_evt_wait();
      
    }
    salir_ble = false;
    break;
    //---------------------------------------------------------------------------------------------------------
    //Estado en el que se reciben los datos del temporizador desde el maestro
    case 2:
    while(1)
    {
      if(salir_recibirdato == true){break;}
      sd_app_evt_wait();
    }
    salir_recibirdato = false;
    break;

    //----------------------------------------------------------------------------------------------------------------------
    //Estado en el que se configura el temporizador para el envío de los datos, para el reinicio y 
    // también el acelerómetro.
    case 3:

    numero_reinicio = tiempo_reinicio/temporizador;
    blinkTimer.begin(temporizador*60000, funcion_tempo, NULL, false);
    blinkTimer.start();
    Serial.println("Temporizador iniciado");
    conf_acelerometro();
    estado = 4;
    break;

    //---------------------------------------------------------------------------------------------------------------------
    //Estado de normalidad de bajo consumo en el que se está registrando datos hasta que salta el temporizador
    // y se sale de este estado hacia el estado de envío de los datos al maestro.
    case 4:

    while(1)
    {
      if(salir_estado4 == true){break;}
      sd_app_evt_wait();
      
    }
    salir_estado4 = false;
    break;
    
    //----------------------------------------------------------------------------------------------------------------------
    //Estado en el que se configura el Bluetooth para el envío de los datos al maestro y comienza el advertising, se envían los datos, 
    // y se espera a que el maestro se desconecte.
    case 5:

    Serial.println("Final de toma de datos");
    Serial.println("Configurando BLE");
    blinkTimer.stop();
    startAdv();
    while(1)
    {
      if(salir_ble1 == true){break;}
      sd_app_evt_wait();
      
    }
    salir_ble1 = false;
    break;

    //------------------------------------------------------------------------------------------------------------------------
    //Estado en el que se reinician los datos, las variables y el temporizador para iniciar un nuevo ciclo
    case 6:
    contador_reinicio++;
    reinicio_datos();
    blinkTimer.reset();
    blinkTimer.begin(temporizador*1000*60, funcion_tempo, NULL, false);
    blinkTimer.start();
    Serial.println("Temporizador iniciado");
    estado = 4;
    if(contador_reinicio == numero_reinicio){estado = 7;}
    break;

    //----------------------------------------------------------------------------------------------------------------------------
    //Estado en el que se reinicia el dispositivo para una nueva configuración. 
    case 7:

    Serial.println("REINICIO");
    Serial.println("------------------------------------------------------------------------------------------------");
    NVIC_SystemReset();
    break;

  }




  
}
