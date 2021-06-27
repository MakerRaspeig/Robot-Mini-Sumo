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
      Marrón  -> GND
      Rojo    -> Señal  -> Patilla 12
      Naranja -> V+

      https://www.arduino.cc/reference/en/language/functions/advanced-io/tone/
      Funcionamiento del zumbador usando el siguiente programa: https://github.com/jecrespo/Robot-MiniSumo/blob/master/Firmware%20Arduino/Test_Minisumo/Test_Minisumo.ino
      

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


void Parar(){
  
  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_PARADA ); 
    servoRuedaIzquierda.write( RUEDA_PARADA ); 
  }

}

void Adelante(){

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

void MarchaAtras(){
  
  ahoraMillis = millis();

  while ( millis() <= ( ahoraMillis + TIEMPO_ACTIVACION ) )
  {
    servoRuedaDerecha.write( RUEDA_DERECHA_ATRAS ); 
    servoRuedaIzquierda.write( RUEDA_IZQUIERDA_ATRAS ); 
  }

}

void PruebaMotores(){

  Adelante();
  delay(500);

  GirarDerecha();
  delay(500);
  
  GirarIzquierda();
  delay(500);

  MarchaAtras();
  delay(500);

  Parar();
  delay(500);

}

void Pita( unsigned int vDuracion ){
  tone( PIN_ZUMBADOR, 
        262, 
        vDuracion );
  }


void setup() {
  
  servoRuedaDerecha.attach( PIN_RUEDA_DERECHA  );     // vincula el servo de la rueda derecha al pin digital indicado
  servoRuedaIzquierda.attach( PIN_RUEDA_IZQUIERDA );  // vincula el servo de la rueda derecha al pin digital indicado

  PruebaMotores();
  Pita( DURACION_ESTANDAR_PITIDO );

}

void loop() {

  Pita( 100 );
  delay(1000);

}