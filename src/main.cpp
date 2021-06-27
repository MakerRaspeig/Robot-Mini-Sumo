/*

  Robot Mini Sumo
  Asociación Maker Raspeig (makerraspeig.com)
  Basado en el siguiente proyecto de Enrique Crespo:
  https://aprendiendoarduino.wordpress.com/2019/05/06/construccion-y-programacion-de-robot-minisumo

  Basado en info de:
  https://www.luisllamas.es/controlar-un-servo-de-rotacion-continua-con-arduino/



  Conexionado:
  - Servos de rotación continua FS90R: https://www.addicore.com/FS90R-Servo-p/ad314.htm
      With traditional servos you can control which position (angle of revolution) the servo's arm moves to. 
      With the same control signals used to control traditional servos you can control the FS90R servo's shaft 
      to be stationary or rotating either clockwise (CW) or counterclockwise (CCW) and the speed of rotation. 
      The signal you would use to tell a traditional servo to go to its middle position, 
        1.5ms pulse signal (position "90" when used with the Arduino Servo library), will cause the FS90R to stop, 
        a 2ms pulse signal (position "180") will cause the FS90R to rotate full speed counterclockwise, 
        and a 1ms pulse signal (position "0") will cause the FS90R to rotate clockwise at full speed.

      The FS90R servo includes an adjustment potentiometer to calibrate the middle-point (stopped) setting. 
      Calibrate this setting by sending a 1.5ms (1500µs) pulse signal (position "90" when used with the Arduino Servo library), 
      then slowly turn the middle-point adjustment potentiometer until the servo stops rotating. 
      Access the middle-point adjustment potentiometer by inserting a small screwdriver into the hole in the bottom of the servo, see diagram below.
    
      Marrón  -> GND
      Rojo    -> V+
      Naranja -> Señal

      Rueda derecha   -> Señal a la patilla 9
      Rueda izquierda -> Señal a la patilla 10 

  - Zumbador:
    Basado en: https://www.arduino.cc/reference/en/language/functions/advanced-io/tone/
      https://github.com/jecrespo/Robot-MiniSumo/blob/master/Firmware%20Arduino/Test_Minisumo/Test_Minisumo.ino
      
      Marrón  -> GND
      Rojo    -> V+
      Naranja -> Señal  -> Patilla 12

      
      
  - Sensores de infrarrojos para detectar bordes oscuros y no salir del tatami. Cuando detectan un color oscuro, el pin de salida se pone a LOW.
    Basado en información de: https://www.luisllamas.es/arduino-detector-lineas-tcrt5000l/ 
    
    - Sensor de infrarrojos trasero
        Rojo    -> GND
        Naranja -> V+
        Amarillo-> Señal  -> Patilla 4
    
    - Sensor de infrarrojos derecho
        Verde    -> GND
        Azul     -> V+
        Violeta  -> Señal  -> Patilla 3
    
    - Sensor de infrarrojos izquierdo
        Negro    -> GND
        Blanco   -> V+
        Gris     -> Señal  -> Patilla 2

  - Sensor de ultrasonidos
    Basado en la información de: http://programarfacil.com/blog/arduino-blog/sensor-ultrasonico-arduino-medir-distancia/ 

      Azul    -> GND
      Gris    -> V+
      Blanco  -> Echo     -> Patilla 6
      Negro   -> Trigger  -> Patilla 5

*/

#include <Arduino.h>
#include <Servo.h>

/**********************/
/* SERVO Y MOVIMIENTO */
/**********************/
Servo servoRuedaDerecha;    // crea el objeto servo para la rueda correspondiente
Servo servoRuedaIzquierda;  // crea el objeto servo para la rueda correspondiente

const unsigned int PIN_RUEDA_DERECHA   = 9;
const unsigned int PIN_RUEDA_IZQUIERDA = 10;

const unsigned int RUEDA_PARADA              = 90;   // parámetro para que la rueda se pare
const unsigned int RUEDA_DERECHA_ADELANTE    = 0;  // parámetro para que la rueda derecha se mueva hacia delante 
const unsigned int RUEDA_DERECHA_ATRAS       = 180;    // parámetro para que la rueda derecha se mueva hacia atrás
const unsigned int RUEDA_IZQUIERDA_ADELANTE  = 180;    // parámetro para que la rueda izquierda se mueva hacia atrás
const unsigned int RUEDA_IZQUIERDA_ATRAS     = 0;  // parámetro para que la rueda izquierda se mueva hacia atrás

const unsigned long TIEMPO_ACTIVACION        = 100;  // tiempo durante el cual se mantiene activo el movimiento correspondiente
unsigned long ahoraMillis                    = 0;    // variable usada para contar los millis de duración


/************/
/* ZUMBADOR */
/************/
const unsigned int PIN_ZUMBADOR   = 12;
const unsigned int DURACION_ESTANDAR_PITIDO = 500;

/**************************************/
/* SENSORES DE BORDES POR INFRARROJOS */
/**************************************/
const unsigned int PIN_IR_TRASERO   = 4;
const unsigned int PIN_IR_DERECHO   = 3;
const unsigned int PIN_IR_IZQUIERDO = 2;

bool bordeTraseroDetectado    = false;
bool bordeDerechoDetectado    = false;
bool bordeIzquierdoDetectado  = false;

/**************************/
/* SENSOR DE ULTRASONIDOS */
/**************************/
const unsigned int PIN_TRIGGER = 5;
const unsigned int PIN_ECHO    = 6;

const float VELOCIDAD_SONIDO_CM_S = 34300.0;  // Velocidad del sonido en cm/s
const float ANCHO_TATAMI          = 30.0;     // tamaño máximo del tatami en cm


void Parar(){
  
  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_PARADA ); 
    servoRuedaIzquierda.write( RUEDA_PARADA ); 
  }

}

void MoverAdelante(){

  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_DERECHA_ADELANTE ); 
    servoRuedaIzquierda.write( RUEDA_IZQUIERDA_ADELANTE ); 
  }

}

void GirarDerecha(){
  
  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_DERECHA_ATRAS ); 
    servoRuedaIzquierda.write( RUEDA_IZQUIERDA_ADELANTE ); 
  }

}

void GirarIzquierda(){
  
  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_DERECHA_ADELANTE ); 
    servoRuedaIzquierda.write( RUEDA_IZQUIERDA_ATRAS );
  }

}

void MoverAtras(){
  
  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_DERECHA_ATRAS ); 
    servoRuedaIzquierda.write( RUEDA_IZQUIERDA_ATRAS ); 
  }

}

void Pita( unsigned int vDuracion ){
  tone( PIN_ZUMBADOR, 
        262, 
        vDuracion );
  noTone( PIN_ZUMBADOR );
  }

void PruebaMotores(){

  for (size_t i = 0; i < 3; i++)
  {
    Pita( 500 );
    delay( 500 );
  }
    
  MoverAdelante();
  delay( 500 );

  GirarDerecha();
  delay( 500 );
  
  GirarIzquierda();
  delay( 500 );

  MoverAtras();
  delay( 500 );

  Parar();
  delay( 500 );

}

void leerSensoresBordes(){

  // los pines se ponen a LOW cuando detectan el borde negro
  if ( digitalRead( PIN_IR_TRASERO ) == LOW ){
    bordeTraseroDetectado = true;
  }
  else {
    bordeTraseroDetectado = false;
  }

  if ( digitalRead( PIN_IR_DERECHO ) == LOW ){
    bordeDerechoDetectado = true;
  }
  else {
    bordeDerechoDetectado = false;
  }

  if ( digitalRead( PIN_IR_IZQUIERDO ) == LOW ){
    bordeIzquierdoDetectado = true;
  }
  else {
    bordeIzquierdoDetectado = false;
  }

}

void moverSegunBordeDetectado(){

  if (    bordeDerechoDetectado
       && bordeIzquierdoDetectado )
  {
    
    MoverAtras();

  } else if ( bordeTraseroDetectado )
    {

      MoverAdelante();

    } else if ( bordeDerechoDetectado ) 
      {
        
        GirarIzquierda();

      } else if( bordeIzquierdoDetectado ){

                GirarDerecha();

              } else {

                MoverAdelante();              // si no se da ningún caso, nos movemos hacia adelante

              }
  
}

float medirDistancia(){
  // lanzamos la señal desde el trigger
  digitalWrite( PIN_TRIGGER, LOW );
  delayMicroseconds(2);
 
  digitalWrite( PIN_TRIGGER, HIGH );
  delayMicroseconds(10);

  digitalWrite( PIN_TRIGGER, LOW );

  // La función pulseIn obtiene el tiempo que tarda en cambiar entre estados, en este caso a HIGH
  unsigned long tiempo = pulseIn( PIN_ECHO, HIGH );  
  // Obtenemos la distancia en cm, hay que convertir el tiempo en segudos ya que está en microsegundos
  // por eso se multiplica por 0.000001, y se divide entre dos porque la señal va y vuelve
  float distancia = tiempo * 0.000001 * VELOCIDAD_SONIDO_CM_S / 2.0;

  return distancia;
}

void buscarEnemigoParaAtacar(){

 while (   medirDistancia() >= 2.0              // si detecta un objeto en este rango, se dirige hacia él
        && medirDistancia() <= ANCHO_TATAMI ) {
    MoverAdelante();
  }

}

void setup() {
  
  pinMode( PIN_IR_TRASERO   , INPUT );
  pinMode( PIN_IR_DERECHO   , INPUT );
  pinMode( PIN_IR_IZQUIERDO , INPUT );
  pinMode( PIN_TRIGGER      , OUTPUT );
  pinMode( PIN_ECHO         , INPUT );

  servoRuedaDerecha.attach( PIN_RUEDA_DERECHA  );     // vincula el servo de la rueda derecha al pin digital indicado
  servoRuedaIzquierda.attach( PIN_RUEDA_IZQUIERDA );  // vincula el servo de la rueda derecha al pin digital indicado

  //PruebaMotores();
  Pita( DURACION_ESTANDAR_PITIDO );
  MoverAdelante();
  Parar();

}

void loop() {

  leerSensoresBordes();

  moverSegunBordeDetectado();

  buscarEnemigoParaAtacar();

}