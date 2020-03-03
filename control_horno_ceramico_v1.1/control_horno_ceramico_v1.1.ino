
/*

  #       _\|/_   A ver..., ¿que tenemos por aqui?
  #       (O-O)        
  # ---oOO-(_)-OOo-----------------------------------------
   
   
  ##########################################################
  # ****************************************************** #
  # *           DOMOTICA PARA PRINCIPIANTES              * #
  # *      CONTROL DE TEMPERATURA HORNO CERAMICO         * #
  # *          Autor:  Eulogio López Cayuela             * #
  # *                  https://github.com/inopya         * #
  # *                                                    * #
  # *       Versión v1.1      Fecha: 18/03/2019          * #
  # ****************************************************** #
  ##########################################################
  
*/

#define _VERSION_ "CONTROL DE HORNO PARA CERAMICA\n"


/*
 * DESCRIPCION DEL PROYECTO
  Control del proceso de coccion de piezas ceramicas en horno industrial

    MATERIALES NECESARIOS
    
    - MCU Arduino UNO, (Nano, MEGA...)
    - LCD 16x2 con conexion I2C
    - Potenciometro
    - Pulsador
    - Zumbador
    - Led RGB direccionable
    - modulo MAX6675 para termopar tipo K
    - Sonda termopar tipo K

  
  El programa recibe del usuario tres parametros:
  Duracion de la fase de coccion, Temperatura maxima y Tiempo de meseta / mantenimiento de temperatura maxima
  El proceso se realiza en 4 etapas:
  
  nº  Duracion de la etapa            Temperatura de la etapa
  1.- (1/5) del tiempode coccion -->  Se incrementara desde la temperatura ambiente hasta 150ºC
  2.- (1/5) del tiempode coccion -->  Se continua incrementando hasta los 500ºC 
  3.- (3/5) del tiempode coccion -->  nuevo incremento hasta T maxima de coccion
  4.- Tiempo de meseta           -->  Se mantiene el horno a la T maxima de coccion
  5.- Parada del horno y espera hasta enfriamiento a temperatura ambiente
      Durante esta etapa se ira mostrando la temperatura del horno como medida de seguridad.



 * NOTAS DE LA VERSION ACTUAL (informacion acumulada)
  v1.0 - Creacion del esqueleto del programa, menus, temporizadores y curvas de coccion
  v1.1 - añadido led RGB(direccionable) para indicar el estado durante el funcionamiento.


 * DETALLES DE COMPILACION CON ARDUINO IDE 1.8.5
   El Sketch usa 12910 bytes del espacio de almacenamiento de programa.
   Las variables Globales usan 800 bytes de RAM.

 
 *  CONEXIONES del modulo MAX6675 para termopar tipo K con ARDUINO
 *  
 *  MAX6675 Modulo  ==>   Arduino
 *      SCK         ==>     D4
 *      CS          ==>     D5
 *      SO          ==>     D6
 *      Vcc         ==>     Vcc (5v)
 *      Gnd         ==>     Gnd
 *  


 ---  COLORES DE REFERENCIA --- 

  (255, 000, 000);    // rojo
  (000, 255, 000);    // verde
  (000, 000, 255);    // azul
  (255, 255, 000);    // amarillo
  (255, 000, 255);    // morado / rosa
  (255, 140, 000);    // naranja
  (255, 105, 180);    // rosa
  (175, 255, 045);    // amarillo verdoso
  (000, 255, 255);    // celeste/agua
  (000, 180, 255);    // azul cielo
  (000, 000, 140);    // azul oscuro
  (000, 000, 128);    // navy
  (255, 255, 255);    // blanco
  (80, 80, 80);       // gris 
  
*/




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        IMPORTACION DE LIBRERIAS 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/
#include <Wire.h>                     // libreria para comunicaciones I2C, necesaria para el LCD
#include <LiquidCrystal_I2C.h>        // Biblioteca para el control del LCD
#include <Adafruit_NeoPixel.h>        // Incluir biblioteca Adafruit NeoPixel
#include <max6675.h>



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//------------------------------------------------------------------------
// Algunas definiciones personales para mi comodidad al escribir codigo
//------------------------------------------------------------------------
#define AND &&
#define OR ||
#define NOT !
#define ANDbit &
#define ORbit |
#define NOTbit ~
#define XORbit ^

//------------------------------------------------------------------------
// Otras definiciones para pines y variables
//------------------------------------------------------------------------
#define LED_OnBoard        13   // led OnBoard de la placa Arduino UNO
#define LCD_AZUL_ADDR    0x27   // Direccion I2C de nuestro LCD color azul  <-- Este es el que vamos a usar
#define LCD_VERDE_ADDR   0x3F   // Direccion I2C de nuestro LCD color verde

#define TIPO_LED            NEO_GRB + NEO_KHZ800 //tipo controlador de los led
#define PIN_TIRA_LED        6   // patilla para la tira

#define RELE_ON           LOW   //mis reles se activan con 0
#define RELE_OFF         HIGH

#define MAX6675_SCK        10  
#define MAX6675_CS          9  
#define MAX6675_SO          8

#define PIN_RELE            5   // salida a rele que controlara el contactor para el quemador del horno
#define TECLA_enter         7   // pulsador apra seleccion en los menus
#define POTENCIOMETRO      A0   // potenciometro que se usara para navegar por los menus           


/* ****  Definiciones relacionadas con el altavoz  ****  */
/* activarAlarma(PIN_ZUMBADOR, FRECUENCIA, TIEMPO_SONIDO, TIEMPO_SILENCIO, CICLOS) */
/*       EJEMPLO --->  activarAlarma(PIN_ZUMBADOR, 1800, 200, 150, 1) */
/* version humana ---> pitidos(numero_pitidos) */
#define PIN_ZUMBADOR        3       //patilla para el altavocillo
#define FRECUENCIA       1800       //frecuencia (Hz)del tono. Entre 2000 y 2100, muy cabrones y buenos para ALARMA
#define TIEMPO_SONIDO     200       //durarion del tono (milisegundos)
#define TIEMPO_SILENCIO   150       //durarion del silencio (milisegundos)
//      CICLOS                      //numero de veces que se repite byte(1-255). con valor CERO no sonaria



boolean FLAG_punto_segundero = true;  // controla si deben o no mostrarse los puntos de separacion de los segundos
                                      // para generar el tipico parpadeo de los relojes que acompaña a los segundos

boolean FLAG_update_reloj;            // indica si es el momento de refrescar el lcd para mostrar la hora
                                      // evitamos asi parpadeos innecesarios. La activa la ISR correspondiente 
                                      // al control del timer2 para el control de 'medios segundos'
                                  
unsigned char medioSegundo = 0;       // control de medios segundos. Por una parte controla el parpadeo del segundero
                                      // y por otra, cada dos ciclos realiza el cambio de segundo y actualiza el reloj 


int horas = -1;                       // contiene el valor horario del temporizador (iniciados fuera de rango)
int minutos = -1;                     // contiene el valor de los minutos del temporizador
int segundos = -1;                    // contiene el valor de los segundos del temporizador

int horas_fase1 = -1;                 // valores iniciados fuera de rango, lo que ahce que se muestren como "--"
int minutos_fase1 = -1;               // indicando que debemos introducir un valor valido
int segundos_fase1 = 0;

int horas_fase2 = 0;
int minutos_fase2 = -1;               //tiempo de mantenimiento despues de la coccion
int segundos_fase2 = 0;

int tolerancia_temperatura = 2;       //margen de temperatura aceptable tanto por arriba como por debajo
int temperatura_deseada = -1;         //contendra el valor introducido por el usuario(será la tmaxima y la de mantenimiento)
float temperatura_curva = 0;          //se ira actualizando con la temperatura de la curva de coccion
                                      //usamos un float porque el calculo de la curva produce decimales

int REG_PROCESO = 0;                  //contador para el control del 'momento' del programa en que nos encontramos


int temperatura_inicial;   //medimos la temperatura 'ambiente' del horno como referencia para la rampa inicial
float tiempo_coccion;      //minutos totales de fase1 --> horas_fase1*60 + minutos_fase1;
float unidad_rampa;        //el tiempo se divide en 5 partes que forman la rampa de coccion --> tiempo_coccion/5.0;

int temperatura_rampa[5]={0, 0, 0, 0, 0}; //array para las temperaturas de la rampa de coccion
int tiempo_rampa[5]={0, 0, 0, 0, 0};      //array para los tiempos de la rampa de coccion

int minuto_actual = 0; //lleva la cuenta del minuto en el que estamos para ir aplicando la rampa de coccion
boolean FLAG_update_temperatura_curva = false;
int LONGITUD_TIRA = 2; 

unsigned long momento_parpadeo = millis()+500;
boolean FLAG_parpadeo_led  = true;


//------------------------------------------------------
// Creamos las instancia de los objetos:
//------------------------------------------------------

//Creamos el objeto 'lcd' como una instancia del tipo "LiquidCrystal_I2C"
LiquidCrystal_I2C lcd(LCD_AZUL_ADDR, 16, 2);

// Crear el 'objeto' para controlar los led
Adafruit_NeoPixel tiraLEDS = Adafruit_NeoPixel(LONGITUD_TIRA , PIN_TIRA_LED, TIPO_LED);

// Crear el 'objeto' termopar
MAX6675 termopar(MAX6675_SCK, MAX6675_CS, MAX6675_SO);


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//         FUNCION DE CONFIGURACION
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void setup()  
{
  Serial.begin(115200);         // Se inicia el puerto serie para depuracion
  Serial.println(F(_VERSION_));
  lcd.begin();
  
  /* ------ MostarVersion ------------------------------------------ */
  lcd.clear();                  // Reset del display 
  lcd.setBacklight(true);       // Activamos la retroiluminacion
  lcd.setCursor(0, 0);          // posicionar cursor al inicio de la primera linea (0)
  //       ("0123456789ABCDEF") // guia apara escribir sin pasarnos del espacio disponible  :)
  lcd.print("CONTROL DE HORNO");
  lcd.setCursor(0, 1);          // posicionar cursor al inicio de la segunda linea (1)
  lcd.print(" inopya Systems");

  //SPI.begin();
  pinMode(MAX6675_SCK, OUTPUT);
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  
  pinMode(LED_OnBoard, OUTPUT);
  digitalWrite(LED_OnBoard, LOW);
  
  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, RELE_OFF);

  pinMode(PIN_TIRA_LED, OUTPUT);  // pin para la linea DATA de la tira de led (en este caso unsolo led direcionable)
  tiraLEDS.begin();               // Inicializar el objeto 'tiraLEDS'
  apagarTira();
  tiraLEDS.setBrightness(30); //establecer la cantidad de brillo de TODOS los led (1-255)
  tiraLEDS.setPixelColor(0, tiraLEDS.Color( 0, 0, 140));  //azul oscuro (inicio de sistema)
  tiraLEDS.show();  
  delay(2500);   //pausa mostrando el 'logo'
  
  /* ------ Mostar peticionde pulsacion de tecla apra iniciar ------------------------------- */
  lcd.clear();                  // Reset del display 
  lcd.setBacklight(true);       // Activamos la retroiluminacion
  lcd.setCursor(2, 0);          // posicionar cursor al inicio de la primera linea (0)
  //       ("0123456789ABCDEF") // guia apara escribir sin pasarnos del espacio disponible  :)
  lcd.print("Pulse tecla");
  lcd.setCursor(2, 1);          // posicionar cursor al inicio de la segunda linea (1)
  lcd.print("para iniciar");

  delay(500);  
  //stopEmergencia();   // ----> DEBUG

  
  pinMode(TECLA_enter, INPUT);
  while (read_pulsador(TECLA_enter)  == 0){
    if (millis()>momento_parpadeo){
      momento_parpadeo = millis()+500; 
      FLAG_parpadeo_led = !FLAG_parpadeo_led;
    }
    if (FLAG_parpadeo_led){
      tiraLEDS.setPixelColor(0, tiraLEDS.Color(  0,   0, 255));  //azul parpadeo(espera de pulsacion inicioal)
      tiraLEDS.show();
    }
    else{
      tiraLEDS.setPixelColor(0, tiraLEDS.Color(  0,   0,   0));  // (apagado)
      tiraLEDS.show();
    }
  }
  tiraLEDS.setPixelColor(0, tiraLEDS.Color(  0,   0, 255));  //azul (inicio/programacion)
  tiraLEDS.show();  
  delay(150); //para evitar rebotes

  /* medida de control para saber si la sonda esta conectada y funcionando */
  temperatura_curva = medir_temperatura();
}




//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  BUCLE PRINCIPAL DEL PROGRAMA   (SISTEMA VEGETATIVO)
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 


//   "0123456789ABCDEF" 
//   "  F1   F2   Temp"
//    00:00  00   1234
int temperatura_real = medir_temperatura();

void loop()
{
  if(REG_PROCESO==0){
    /* ----- programar tiempos y temperatura ----- */
    programar_tiempos();
    REG_PROCESO++;   //REG_PROCESO=1
    tiraLEDS.setPixelColor(0, tiraLEDS.Color(255, 0, 255));  //rosa, inicio de cosion
    tiraLEDS.show(); 
    /* ----- cargar el temporizador con el tiempo de la fase 1 ----- */
      horas = horas_fase1; 
      minutos = minutos_fase1; 
      segundos = 0;
      start_Timer2();
    //mmmmmmmmmmmmmmmmmmmmmmmm
    /* calcular rampa
     *  t0, t1, t2, t3, t4
     *  t0 a t3 es la fase 1 repartida en tres etapas de duracion 
     *  1 unidad, 1 unidad y 3 unidades (una unidad es el tiempo Fase1/5)
     *  t4 es el tiempo de mantenimiento, Fase2
     */
     tiempo_coccion = horas_fase1*60 + minutos_fase1;
     unidad_rampa = tiempo_coccion/5.0; //(1+1+3) proporciones entre los intervalos
     unidad_rampa = int(unidad_rampa);  //eliminamos los decimales
     temperatura_inicial = medir_temperatura();

     temperatura_rampa[0] = temperatura_inicial; //la que tiene el horno antes de empezar
     temperatura_rampa[1] = 150;
     temperatura_rampa[2] = 500;
     temperatura_rampa[3] = temperatura_deseada;
     temperatura_rampa[4] = temperatura_deseada;

     tiempo_rampa[0] = 0;
     tiempo_rampa[1] = unidad_rampa;      // 1*unidad_rampa              
     tiempo_rampa[2] = 2*unidad_rampa;    // 1*unidad_rampa + tiempo acumulado       
     tiempo_rampa[3] = tiempo_coccion;    // 3*unidad_rampa + acumulado = tiempo_coccion
     tiempo_rampa[4] = minutos_fase2;
  }

  if(REG_PROCESO==1 AND FLAG_update_temperatura_curva == true){
    FLAG_update_temperatura_curva = false;
    byte indice_rampa = 1;  //nos interesan los elementos del 1 al 4
    while(tiempo_rampa[indice_rampa] < minuto_actual){
      indice_rampa++;
    }
    float sub_temperatura = temperatura_rampa[indice_rampa] - temperatura_rampa[indice_rampa-1];
    float sub_tiempo = tiempo_rampa[indice_rampa] - tiempo_rampa[indice_rampa-1];
    temperatura_curva += (sub_temperatura/sub_tiempo);
    
    if(temperatura_curva > temperatura_deseada){
      temperatura_curva = temperatura_deseada;
    }
    /*
    //------------------- { DEBUG ----------------------------------------------------------------------
    Serial.print("Cuenta atras --> ");Serial.print(horas);Serial.print(":");Serial.println(minutos);
    Serial.print("indice rampa --> ");Serial.println(indice_rampa);
    Serial.print("rango temperaturas --> ");Serial.print(temperatura_rampa[indice_rampa]);
    Serial.print(" - ");Serial.println(temperatura_rampa[indice_rampa-1]);
    Serial.print("rango tiempo --> ");Serial.print(tiempo_rampa[indice_rampa]);
    Serial.print(" - ");Serial.println(tiempo_rampa[indice_rampa-1]);
    Serial.print("sub temperatura --> ");Serial.println(sub_temperatura);
    Serial.print("sub tiempo --> ");Serial.println(sub_tiempo);
    Serial.print("incremento temperatura --> ");Serial.println((sub_temperatura/sub_tiempo));
    Serial.print("temperatura de la curva --> ");Serial.println(temperatura_curva);
    Serial.println("---------------------------\n");
    //------------------- DEBUG } ----------------------------------------------------------------------
    */
  }
  
  /* comprobar si ha terminado la fase 1 y en ese caso iniciar la fase 2 */
  if(REG_PROCESO==2){
    /* ----- cargar el temporizador con el tiempo de la fase 2 ----- */
    horas = 0; 
    minutos = minutos_fase2; 
    segundos = 0;
    temperatura_curva = temperatura_deseada;
    REG_PROCESO++; //REG_PROCESO=3 al iniciar fase2, (para evitar que se vuelvan a cargar los valores
  }

  if(horas+minutos+segundos==0){
    REG_PROCESO++; //REG_PROCESO=2 al terminar fase1, REG_PROCESO=4 al terminar fase2
  }
      
    
  /* ----- lectura de la sonda de temperatura ----- */
  if (millis()>momento_parpadeo){  //reaprovechamos la variable momento_parpadeo
    momento_parpadeo = millis()+2500; 
    temperatura_real = medir_temperatura();
  }  

  /* actuar sobre los contactores de salida  para conectar y desconetar 
   * el horno segun las condiciones de temperatura, tiempo y fase del proceso */

  if(temperatura_real < (temperatura_curva - tolerancia_temperatura)){
    //encender resistencias
    digitalWrite(PIN_RELE, RELE_ON);
    if(REG_PROCESO < 3){
      tiraLEDS.setPixelColor(0, tiraLEDS.Color(255, 0, 255));  //rosa, quemador Aumentando temperatura
    }
    else{
      tiraLEDS.setPixelColor(0, tiraLEDS.Color(255, 140, 000));  //naranja, quemador Manteniendo temperatura
    }
    tiraLEDS.show(); 
  }

  if(temperatura_real > (temperatura_curva + 2*tolerancia_temperatura)){
    //apagar resistencias
    digitalWrite(PIN_RELE, RELE_OFF);
    tiraLEDS.setPixelColor(0, tiraLEDS.Color(100, 100, 100));  //blanco suave, quemador parado
    tiraLEDS.show(); 
  }


  /* -----  Presentar informacion en la pantalla LCD 16x2  ----- */  
  
  /* mostrar la cuenta atras (comun a fase 1 y fase 2) en el LCD */
  //usamos los medios segundos para refrescar el resto de informacion
  //y de esa manera que no se produzcan parpadeos indeseables en el LCD
  //ya que los LCD de color azul(usado aqui) tiene bastante mala follada a ese respecto
  if(FLAG_update_reloj == true){
    FLAG_update_reloj = false;
    refrescar_Reloj();
      
  
    /* linea de texto correspondiente a la fase actual */
    if(REG_PROCESO==1){
      lcd.setCursor(0, 0);
      //       ("0123456789ABCDEF");
      //lcd.print("FASE 1      TEMP");
      lcd.print("FASE 1            ");
      lcd.setCursor(9, 0);
      lcd.print(temperatura_curva);
    }
    if(REG_PROCESO==3){
      lcd.setCursor(0, 0);
      //       ("0123456789ABCDEF");
      lcd.print("FASE 2 >        ");  //lcd.print("FASE 2      TEMP");
      lcd.setCursor(12, 0);
      lcd.print(int(temperatura_curva));
    }
        
    /* ----- mostrar la temperatura actual en el LCD ----- */   
    lcd.setCursor(12, 1);
    if(temperatura_real>999){
      lcd.print(temperatura_real);
    }
    else{
      lcd.print("    ");
      lcd.setCursor(13, 1);
      lcd.print(temperatura_real);
    }
  }

  /* ----- PARAR el horno al terminar fase 2  y mantenernos en un bucle infinito----- */
  if(REG_PROCESO >= 4){
    // Apagar el horno
    //REG_PROCESO++;
    digitalWrite(PIN_RELE, RELE_OFF);
    delay(5000);
    lcd.clear();
    lcd.setCursor(0, 0);
    //       ("0123456789ABCDEF");
    lcd.print("FIN DE PROCESO  ");
    pitidos(2);
    delay(2000);
    pitidos(2);
    /* fin de cocion, esperar enfriamiento, mostramos luz en amarillo */
    tiraLEDS.setPixelColor(0, tiraLEDS.Color(255, 255, 000));  //amarillo, proceso de enfriamiento
    tiraLEDS.show(); 
    while(true){
      //mostramos la temperatura conforme el horno se va enfriando
      temperatura_real = medir_temperatura();
      lcd.setCursor(0, 1);
      lcd.print("Temp:  ");lcd.print(temperatura_real);lcd.print("    ");
      delay(2000);    //evitamos parpademos innecesarios y molestos
      if(temperatura_real<30){ 
        //si el horno se enfria como para poder tocar las piezas mostramos luz verde
        tiraLEDS.setPixelColor(0, tiraLEDS.Color(0, 255, 0));  //verde, ya se puede meter la mano
        tiraLEDS.show(); 
      }
    }
  }
}




//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  BLOQUE DE FUNCIONES: LECTURA SENSORES, TOMA DE DECISIONES, ...
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    SONDA PIROMETRICA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  LECTRUA DE TERMOPAR CON MAX6675
//========================================================
int medir_temperatura() 
{
  float temperatura = termopar.readCelsius();
  if (temperatura != temperatura){
    Serial.println(F(">>> Fallo en el termopar"));
    stopEmergencia();
  }  
  //Serial.print(F("Temperatura C: ")); Serial.println(temperatura,2);
  return int(temperatura);  //despreciamos los decimales por ahora
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//       MENU PRINCIPAL ---  PROGRAMAR TIEMPOS DE COCCION
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// ASIGNAR VALORES DE COCCION
//========================================================
void programar_tiempos()
/*
 * Generacion del menu para entrada de tiempos de coccion
 * Para los valores e los tiempo dispomenos del rango entre 0 y 59 para cada uno de los campos HH:mm
 * El valor de un campo queda aceptado al activar el pulsador, 'saltando' automaticamente al siguente campo.
 * --- Podemos corregir un valor ya aceptado de la siguiente manera:
 *          En la posicion minima del potenciometro el valor que obtenemos es -1, que el programa  
 *          nos representa como'--'. Si pulsamos en ese estado el programa interpetra que queremos 
 *          corregir el valor anterior y nos mueve a dicho campo
 */
{

  byte estado_tecla_enter;
  int REG_MENU = 1;                     // variable que indicara en que campo del menu de seleccion estamos
  boolean FLAG_program_mode = true;     // bandera para indicar que aun no hemos terminado de introducir el tiempo
                                        // en el temporizador.
                                        
  horas_fase1   = -1;      // valores iniciados fuera de rango, lo que hace que se muestren como "--"
  minutos_fase1 = -1;      // indicando que debemos introducir un valor valido
  minutos_fase2 = -1;  
  temperatura_deseada = -1; 
    
//   "0123456789ABCDEF" 
//   "  F1   F2   Temp"
//   "00:00  00   1234"

  int valor_mapeado;
  int valorPotenciometro;
  lcd.clear();
  while (FLAG_program_mode == true){
     valorPotenciometro= analogRead(POTENCIOMETRO);   // el potenciometro nos permitira ingresar valores 
                                                      // del rango -1 y 59. 
                                                      // aceptar el valor ('--') le indica a la rutina que deseamos 
                                                      // volver sobre el valor anterior para corregirlo

    //---------------- seleccion de horas y minutos fase 1, fase 2
    if(REG_MENU<=3){
      valor_mapeado = map(valorPotenciometro,5,4020, -4,60);
      if (valor_mapeado == -1){valor_mapeado=0;}
      if (valor_mapeado <= -2){valor_mapeado=-1;}     
      if (valor_mapeado > 59){valor_mapeado=59;}
      
      if(REG_MENU < 1){REG_MENU = 1;}
      if(REG_MENU == 1){horas_fase1 = valor_mapeado;}
      if(REG_MENU == 2){minutos_fase1 = valor_mapeado;}
      if(REG_MENU == 3){minutos_fase2 = valor_mapeado;} 
    }
    if(REG_MENU<3){
      //lcd.clear();
      lcd.setCursor(0, 0);
      //       ("0123456789ABCDEF");
      lcd.print(" hh:mm    FASE 1");
      lcd.setCursor(1, 1);
      //--print horas fase 1
      if(horas_fase1 == -1){
        lcd.print("--");
      }
      else{
        if(horas_fase1<10){
          lcd.print("0");
        }
        lcd.print(horas_fase1);
      }
      lcd.print(":");
      //--print minutos fase 1
      if(minutos_fase1 == -1){
        lcd.print("--");
      }
      else{
        if(minutos_fase1<10){
          lcd.print("0");
        }
        lcd.print(minutos_fase1);
      }
    }
    //---------------- seleccion de minutos fase 2
    if(REG_MENU==3){
      //lcd.clear();
      lcd.setCursor(0, 0);
      //       ("0123456789ABCDEF");
      lcd.print("minutos FASE 2  ");
      lcd.setCursor(2, 1);
      //--print minutos_fase2
      if(minutos_fase2 == -1){
        lcd.print("--");
      }
      else{
        if(minutos_fase2<10){
          lcd.print("0");
        }
        lcd.print(minutos_fase2);
      }
    }
    //---------------- seleccion de la temperatura
    if(REG_MENU==4){
      valor_mapeado = map(valorPotenciometro,5,4020, -4,190); // ahora el potenciometro nos permitira ingresar valores de -1 a 9999
      if (valor_mapeado >=-1 AND valor_mapeado< 50){valor_mapeado=50;}
      if (valor_mapeado <= -2){valor_mapeado=-1;}         
      if (valor_mapeado > 190){valor_mapeado=190;}
      temperatura_deseada = valor_mapeado;
      //lcd.clear();
      lcd.setCursor(0, 0);
      //       ("0123456789ABCDEF");
      lcd.print("TEMPERATURA");
      lcd.setCursor(2, 1);
      //--print temperatura deseada
      if(temperatura_deseada == -1){
        lcd.print("----");
      }
      else{
        temperatura_deseada *= 10; 
        if(temperatura_deseada<100){
          lcd.print(" ");
        }
        lcd.print(temperatura_deseada);lcd.print("    ");
      }
    }

    //delay(20);


    /* ---- ACCESO A TECLAS PARA RECOGER PULSACIONES ---- */
    estado_tecla_enter = read_pulsador(TECLA_enter);

    if (estado_tecla_enter == 1 AND (valor_mapeado >=0)){
      delay(350);         //para evitar rebotes
      REG_MENU++;         // si las condiciones son correctas se acepta el valor para ese campo y se salta al siguiente
      lcd.clear();
    }
    
    if (estado_tecla_enter == 1 AND valor_mapeado <0){
      REG_MENU--;         // si el campo sobre el que se activa el pulsador contiene '--' (-1)
                          // se vuelve al campo anterior
      lcd.clear();
    }

    
    if ((REG_MENU == 5) AND (temperatura_deseada >=0)){
      valorPotenciometro = analogRead(POTENCIOMETRO);
      valor_mapeado = map(valorPotenciometro,5,1020, -10,10);     
        
      //si se completan todos los campos correctamente se muestra un mensaje 
      //pidiendo confirmacion de salvar informacion
      lcd.setCursor(0, 0);      
      if(valor_mapeado<0){
        //       ("0123456789ABCDEF");
        lcd.print("*CORREGIR     OK");
      }
      else{
        //       ("0123456789ABCDEF");
        lcd.print(" CORREGIR    *OK");
      }

      lcd.setCursor(0, 1);
      //----------------------------------------------
      if(horas_fase1<10){
        lcd.print("0");
      }      
      lcd.print(horas_fase1);lcd.print(":");
      //----------------------------------------------
      if(minutos_fase1<10){
        lcd.print("0");
      }
      lcd.print(minutos_fase1);lcd.print("  ");
      //----------------------------------------------
      if(minutos_fase2<10){
        lcd.print("0");
      }
      lcd.print(minutos_fase2);lcd.print("   ");
      //----------------------------------------------
      lcd.print(temperatura_deseada);      
    }

      
    if (REG_MENU==6){                 // completado el proceso de introducion del tiempos y temperatura
      FLAG_program_mode = false;      // se desactiva la posibilidad de volver atras para modificar datos
    } 
  }
  
  delay(800);
  lcd.setCursor(0, 0);
  //       ("0123456789ABCDEF");
  lcd.print(" OK para INICIAR");
  
  while (read_pulsador(TECLA_enter) == 0){     
  }
  delay(350); //para evitar rebotes
  return;
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//      SONIDO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  FUNCION BASE PARA GENERAR PITIDOS CON EL ZUMBADOR
//========================================================

void activarAlarma(byte pin_Zumbador, int frecuencia, int tiempoON, int tiempoOFF, byte repeticiones)
/*
 * Funcion para generar los pitidos de aviso y pulsacion de teclas
 */
{
  for (byte i=0; i<repeticiones; i++){
    tone(pin_Zumbador, frecuencia);
    delay (tiempoON);
    noTone(pin_Zumbador);
    delay (tiempoOFF);
  }
} 


//========================================================
//  FUNCION 'HUMANA' para generar PITIDOS
//========================================================

void pitidos(byte numero_pitidos)
{
  //activarAlarma(PIN_ZUMBADOR, FRECUENCIA, TIEMPO_SONIDO, TIEMPO_SILENCIO, numero_pitidos);
  for (byte i=0; i<numero_pitidos; i++){
    tone(PIN_ZUMBADOR, FRECUENCIA);
    delay (TIEMPO_SONIDO);
    noTone(PIN_ZUMBADOR);
    if(numero_pitidos>1){
      delay (TIEMPO_SILENCIO);
    }
  }
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//     PULSADORES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  ATENCION DE PULSADOR SIN REBOTES
//========================================================


byte read_pulsador(byte _PIN_entrada)   
{
  if(digitalRead(_PIN_entrada) == false){
    //si no hay pulsacion pasamos de perder el tiempo
    return 0;      
  }        
  unsigned long inicioPulsacion = millis();
  
  /* si entramos y nos encontramos con una pulsacion desplegamos el algoritmo para interpretarla */
  while(digitalRead(_PIN_entrada) == 1){
  }

  if(millis() - inicioPulsacion >= 450){  //tiempoPulsacionLarga (350 ms esta bien)
    pitidos(2);
    return 2;
  }
  else{
    pitidos(1);
    return 1;
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//     LCD
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  RELOJ EN EL LCD
//========================================================

void refrescar_Reloj()
{
  lcd.setCursor(0, 1);
 
  if(horas<10){
    lcd.print("0");   // para que todas las cantidades aparezcan siempre con dos cifras 
  }                   // y no tengamos un 'baile' de numero
  lcd.print(horas);
  lcd.print(":");
  if(minutos<10){
    lcd.print("0");
  }    
  lcd.print(minutos);
  
  //--- control de parpadeo de segundero
  if(FLAG_punto_segundero==true){
    lcd.print(":");
  }
  else {
    lcd.print(" "); 
  }
  //--- FIN control de parpadeo de segundero
  
  if(segundos<10){
    lcd.print("0");
  }
  lcd.print(segundos);lcd.print(" ");
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//       ATENCION A INTERRUPCIONES  SOFTWARE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// INICIAR TIMER 2
//========================================================

void start_Timer2()
{
  /* Setup Timer2  para que junto a la funcion de atencion a su interrupcion
   * poder obtener una 'señal util' cada 500 ms */
  TCCR2B = 0x00;        // deshabilita Timer2 mientras lo estamos ajustando
  TCNT2  = 5 ;          // establecer el contador iniciandolo con el valor 5 (para contar 250)
  TIFR2  = 0x00;        // Timer2 INT Flag Reg: borrar la bandera de desbordamiento
  TIMSK2 = 0x01;        // Timer2 INT Reg: habilita la interrupcion de desbordamiento de Timer2
  TCCR2A = 0x00;        // Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x07;        // Timer2 Control Reg B: Timer Prescaler set to 1024
}



//========================================================
// PARAR TIMER 2  (sin uso)
//========================================================

void stop_Timer2()
{
 TCCR2B = 0x00;         // deshabilita Timer2 mientras lo estamos ajustando
 TIMSK2=0 ;             // Timer2 INT Reg: deshabilita la interrupcion de desbordamiento de Timer2
}



//========================================================
// RUTINA DE INTERRUPCION PARA DESBORDAMIENTO DE TIMER 2
// El vector de desbordamineto es -->  TIMER2_OVF_vect
//========================================================

unsigned int timer2_ovf_count = 0;    //contador de numero de interrupciones (desbordamientos de Timer2)

ISR(TIMER2_OVF_vect) 
{
  /* Con el preescales a 1024, contando 250 en TCNT2 y 31 en timer2_count,
   * con esta funcion de atencion a su interrupcion se obtiene una 'señal util' cada 500 ms 
   Este Reloj por software atrasa 1 segundo cada 3 horas*/
  
  timer2_ovf_count++;             // Incremento del contador de numero de interrupciones
  if(timer2_ovf_count >= 31){
    timer2_ovf_count = 0;         // reset del contador de numero de interrupciones
    CuentaAtras_ISR();
    //Reloj_ISR();   
  }
  TCNT2 = 5;                      // reset del contador iniciandolo con el valor 5 para contar 250
  TIFR2 = 0x00;                   // Timer2 INT Flag Reg: borrar la bandera de desbordamiento
}



//========================================================
// RELOJ CON INTERRUPCION SOFTWARE DEL TIMER02  (sin uso)
//========================================================

//void Reloj_ISR()
///*
// * Reloj relativametne preciso mediante software
// * y el uso de interrupciones intermas del timer02 
// * Podemos ponerlo en hora actuando sobre las variables globales 
// * 'horas', 'minutos' y 'segundos'  definidas al inicio del codigo
// * 
// * Usado para mostrar tiempo en pantalla unicamente como un contador creciente
// */
//{
//  medioSegundo ++;
//  FLAG_update_reloj = true;
//  if(medioSegundo == 2){
//    segundos ++;
//    if(segundos == 60){
//      segundos = 0;
//      minutos ++;
//      if(minutos == 60){
//        minutos = 0;
//        horas ++;
//        if(horas == 24){
//          horas = 0;
//        }
//      }
//    }
//    medioSegundo = 0;  
//  }
//  FLAG_punto_segundero = !FLAG_punto_segundero;
//}



//========================================================
// CUENTA ATRAS CON INTERRUPCION SOFTWARE DEL TIMER02
//========================================================

void CuentaAtras_ISR()
/*
 * Contador descendente mediante el uso de interrupciones intermas del timer02
 */
{
  if(horas==0 AND minutos==0 AND segundos==0){
    /* por si se queda activo el servicio de ISR, salimos de esta rutina si el tiepo ha llegado a cero
     evitando asi que se produzcan valores negativos que generen erroes de representacion el el LCD */                                
    FLAG_update_reloj = false;                    // y desactivamos la correspondiente al refresco del reloj
    return;
  }

  medioSegundo ++;                                // contabilizamos medios segundos
  FLAG_update_reloj = true;                       // que son los que generan el refresco del segundero (:)
  if(medioSegundo == 2){                          // cada dos medios segundos, 
    segundos --;                                  // descontamos 1 segundo
    if(segundos == -1){                           // Despues del segundo CERO,...
      segundos = 59;                              // viene el 59
      minutos --;                                 // y con ello descontar 1 minuto
      minuto_actual++;                            //    <---- para la cuenta de minutos de la curva de coccion
      FLAG_update_temperatura_curva = true;
      if(minutos == -1){                          
        minutos = 59;                             // El paso de CERO minutos lleva al 59
        horas --;                                 // y con ello descontar 1 hora
      }
    }
    medioSegundo = 0;                             // cada dos medios segundos, reiniciamos el proceso 
  }
  /* cada medio segundo se activa/desactiva la bandera para la gestion y refresco del segundero en el LCD */
  FLAG_punto_segundero = !FLAG_punto_segundero;   
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DE LA TIRA DE LEDS PARA CREAR LOS EFECTOS DE COLOR
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//   FUNCION PARA APAGAR LA TIRA DE LED POR COMPLETO
//========================================================

void apagarTira()
{ 
  /* 
    esta funcion se encarga de apagar la tira de led recorriendo 
    todos los pixeles que la forman y los pone a Cero en sus tres componentes
  */

  for (int i=0; i<= LONGITUD_TIRA +1; i++){ 
    tiraLEDS.setPixelColor(i, tiraLEDS.Color(0, 0, 0));
  } 
  tiraLEDS.show(); 
}

//========================================================
//   MOSTRAR UN COLOR EN UN LED UNICO
//========================================================

void colorearPIXEL(int pixel, byte red, byte green, byte blue)
{ 
  tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(red, green, blue));
  tiraLEDS.show();  //actualizar 'en bloque' el nuevo estado de los leds que se han de modificar 
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//       CONTROL DE SEGURIDAD / APAGADO DE EMERGENCIA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void stopEmergencia()
{
  /* respuesta de seguridad y apagado de emergencia del horno*/
  
  digitalWrite(PIN_RELE, RELE_OFF);  //orden de apagado del horno por seguridad
  lcd.clear();
  Serial.println(F(">> ERROR TERMOPAR - PARADA DE SEGURIDAD\n"));
  lcd.setCursor(0, 0);
  lcd.print("  ERROR SONDA");
  lcd.setCursor(0, 1); 
  lcd.print("STOP x SEGURIDAD");
  while(true){
    if (millis()>momento_parpadeo){
      momento_parpadeo = millis()+1000; 
      FLAG_parpadeo_led = !FLAG_parpadeo_led;
      pitidos(2);
    }
    if (FLAG_parpadeo_led){
      tiraLEDS.setPixelColor(0, tiraLEDS.Color(255,   0,   0));  //rojo
      tiraLEDS.show();
    }
    else{
      tiraLEDS.setPixelColor(0, tiraLEDS.Color(  0,   0,   0));  //apagado
      tiraLEDS.show();
    }
  }
}

//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
