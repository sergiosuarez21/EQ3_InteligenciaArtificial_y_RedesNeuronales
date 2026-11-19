/*
 * CLASIFICADOR DE MONEDAS - Arduino UNO (compuertas + banda)
 * Recibe de la ESP32-CAM: '#' seguido de '1' (1 peso), '2' (2 pesos) o '3' (10 pesos)
 * Espera a que la moneda llegue a la compuerta correspondiente (sensor IR)
 * y acciona el servo para desviarla a su cajón.
 *
 * Conexiones:
 *   ESP32-CAM IO13 (TX) -> UNO D2 (RX SoftwareSerial)   + GND común
 *   IR compuerta 1 (1 peso)   -> A0
 *   IR compuerta 2 (2 pesos)  -> A1
 *   IR compuerta 3 (10 pesos) -> A2
 *   Servo 1 (2.2 kgf)  -> D9
 *   Servo 2 (2.2 kgf)  -> D10
 *   Servo 3 (1 Nm)     -> D11
 *   Gate del MOSFET (motor de la banda) -> D6 (PWM) con resistencia de 100 ohm
 *   Servos y motor se alimentan del adaptador de 5 V (no del pin del UNO).
 *   Todos los GND unidos.
 */

/* EXPLICACIÓN: Visión general del programa
 * Este código corre en el Arduino UNO y es el que mueve la banda y las compuertas. Funciona así:
 *   1. Enciende el motor de la banda transportadora.
 *   2. Espera un mensaje de la ESP32-CAM: '#' seguido de '1', '2' o '3' (la compuerta destino).
 *   3. Espera a que el objeto llegue a esa compuerta, detectándolo con un sensor infrarrojo (IR).
 *   4. Mueve el servo para desviar el objeto a su cajón y lo regresa a su posición de reposo.
 * Nota: los comentarios originales hablan de monedas (1, 2 y 10 pesos), pero el proyecto ahora clasifica pastillas. El funcionamiento es el mismo; con el código de la ESP32-CAM, '1' = COLOR CÁPSULA, '2' = PASTILLA CIRCULAR y '3' = PASTILLA CON JARABE.
 *
 * Librerías incluidas
 * - Servo.h: controla los servomotores con ángulos de 0 a 180 grados.
 * - SoftwareSerial.h: crea un puerto serial por software en pines normales del UNO. Se usa para recibir los mensajes de la ESP32-CAM sin ocupar el puerto serial principal, que queda libre para el Monitor Serie.
 */
#include <Servo.h>
#include <SoftwareSerial.h>

/* EXPLICACIÓN: Pines
 * - PIN_RX_ESP = 2: recibe los datos de la ESP32-CAM (se conecta a su IO13).
 * - PIN_TX_DUMMY = 3: SoftwareSerial exige indicar un pin de transmisión aunque no se use; este queda sin conexión.
 * - PIN_IR = {A0, A1, A2}: un sensor infrarrojo por compuerta, que detecta cuando el objeto llega.
 * - PIN_SERVO = {9, 10, 11}: un servo por compuerta.
 * - PIN_BANDA = 6: pin PWM que controla la velocidad del motor de la banda por medio del MOSFET.
 * Los arreglos permiten usar el mismo código para las 3 compuertas, usando el índice 0, 1 o 2.
 */
// ---------- Pines ----------
const uint8_t PIN_RX_ESP   = 2;
const uint8_t PIN_TX_DUMMY = 3;                 // no se usa, SoftwareSerial lo exige
const uint8_t PIN_IR[3]    = {A0, A1, A2};
const uint8_t PIN_SERVO[3] = {9, 10, 11};
const uint8_t PIN_BANDA    = 6;                 // PWM

/* EXPLICACIÓN: Ajustes (se calibran con pruebas reales)
 * - VELOCIDAD_BANDA 140: velocidad del motor de 0 a 255. Bajarla da más tiempo a la cámara para clasificar.
 * - IR_ACTIVO_EN_BAJO true: la mayoría de módulos IR dan LOW (nivel bajo) cuando detectan un objeto. Si el tuyo hace lo contrario, ponlo en false.
 * - ANGULO_REPOSO {0, 0, 0}: posición de cada compuerta cuando está abierta (paralela a la banda, deja pasar el objeto).
 * - ANGULO_DESVIO {60, 60, 60}: posición cuando está cerrada y atravesada (desvía el objeto al cajón).
 * - TIEMPO_ESPERA_MAX_MS 12000: si el objeto no llega a la compuerta en 12 s, se cancela la acción.
 * - RETARDO_ANTES_DESVIO_MS 40: pequeña espera después de que el IR detecta, para que el objeto quede frente a la paleta.
 * - TIEMPO_DESVIO_MS 700: tiempo que la compuerta permanece cerrada empujando el objeto.
 */
// ---------- Ajustes (calibrar en la práctica) ----------
const uint8_t VELOCIDAD_BANDA = 140;            // 0-255. Bájala si la cámara no alcanza a clasificar
const bool IR_ACTIVO_EN_BAJO = true;            // la mayoría de módulos IR dan LOW al detectar
const int  ANGULO_REPOSO[3]  = {0,  0,  0};     // compuerta paralela a la banda
const int  ANGULO_DESVIO[3]  = {60, 60, 60};    // compuerta atravesada
const unsigned long TIEMPO_ESPERA_MAX_MS    = 12000; // si la moneda no llega, se cancela
const unsigned long RETARDO_ANTES_DESVIO_MS = 40;    // ajusta para que la moneda quede frente a la paleta
const unsigned long TIEMPO_DESVIO_MS        = 700;   // tiempo con la compuerta cerrada

/* EXPLICACIÓN: Objetos y variable global
 * - espSerial: el puerto serial por software que recibe los datos de la ESP32-CAM.
 * - servos[3]: un objeto Servo por compuerta.
 * - esperandoCodigo: indica si ya llegó el carácter '#' y falta el carácter del código (empieza en false).
 */
SoftwareSerial espSerial(PIN_RX_ESP, PIN_TX_DUMMY);
Servo servos[3];
bool esperandoCodigo = false;

/* EXPLICACIÓN: irDetecta(i): indica si el sensor IR de la compuerta i detecta un objeto
 * Lee el pin del sensor y lo interpreta según IR_ACTIVO_EN_BAJO: si es true, devuelve true cuando la lectura es LOW; si es false, cuando es HIGH. Así el resto del código no depende del tipo de módulo IR que se use.
 */
bool irDetecta(uint8_t i) {
  bool lectura = digitalRead(PIN_IR[i]);
  return IR_ACTIVO_EN_BAJO ? (lectura == LOW) : (lectura == HIGH);
}

/* EXPLICACIÓN: setup(): se ejecuta una sola vez al encender
 * 1. Serial.begin(9600) abre el Monitor Serie y espSerial.begin(9600) abre la comunicación con la ESP32-CAM (misma velocidad que en su código).
 * 2. Configura los 3 pines de los sensores IR como entradas y el pin de la banda como salida.
 * 3. analogWrite(PIN_BANDA, VELOCIDAD_BANDA) enciende la banda a la velocidad configurada (140 de 255, aproximadamente 55 % de la potencia).
 * Resultado esperado en el Monitor Serie: "Arduino listo". La macro F() guarda el texto en la memoria flash y ahorra memoria RAM.
 */
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  for (uint8_t i = 0; i < 3; i++) pinMode(PIN_IR[i], INPUT);
  pinMode(PIN_BANDA, OUTPUT);
  analogWrite(PIN_BANDA, VELOCIDAD_BANDA);      // banda encendida

  Serial.println(F("Arduino listo"));
}

/* EXPLICACIÓN: accionarCompuerta(i): desvía el objeto en la compuerta i (0, 1 o 2)
 * Se explica por pasos dentro de la función. Duración aproximada de la acción una vez que el objeto llega: 40 + 60 + 700 + 400 = 1.2 s.
 */
void accionarCompuerta(uint8_t i) {
  unsigned long inicio = millis();

  /* EXPLICACIÓN: Paso 1: esperar a que el objeto llegue
   * Guarda el instante de inicio con millis() y repite la lectura del IR hasta que detecte algo. Si pasan más de 12 s sin detectar nada, imprime "Tiempo agotado: la moneda no llego" y sale de la función sin mover el servo (por ejemplo, si el objeto se cayó o se atoró).
   */
  // Esperar a que la moneda llegue a la compuerta
  while (!irDetecta(i)) {
    if (millis() - inicio > TIEMPO_ESPERA_MAX_MS) {
      Serial.println(F("Tiempo agotado: la moneda no llego"));
      return;
    }
  }
  /* EXPLICACIÓN: Paso 2: pequeña espera
   * Una pausa corta (40 ms) para que el objeto avance hasta quedar frente a la compuerta antes de moverla.
   */
  delay(RETARDO_ANTES_DESVIO_MS);

  /* EXPLICACIÓN: Paso 3: mover el servo
   * - attach() conecta el servo al pin solo mientras se mueve, y detach() lo desconecta al final. La razón: la librería Servo genera pulsos constantes que interfieren con SoftwareSerial y pueden corromper los datos recibidos; además, el servo no vibra en reposo.
   * - Secuencia: se fija el ángulo de reposo antes de conectar, se conecta, se espera 60 ms, se manda al ángulo de desvío (60°) durante 700 ms, se regresa a reposo (0°), se espera 400 ms para que llegue y se desconecta.
   */
  // Los servos solo se conectan mientras se mueven: sin pulsos en reposo,
  // la recepcion serial no se corrompe y los servos no vibran.
  servos[i].write(ANGULO_REPOSO[i]);
  servos[i].attach(PIN_SERVO[i]);
  delay(60);
  servos[i].write(ANGULO_DESVIO[i]);
  delay(TIEMPO_DESVIO_MS);
  servos[i].write(ANGULO_REPOSO[i]);
  delay(400);
  servos[i].detach();

  /* EXPLICACIÓN: Paso 4: avisar
   * Imprime en el Monitor Serie, por ejemplo: "Compuerta 2 accionada" (i + 1 convierte el índice 0, 1 o 2 en el número de compuerta).
   */
  Serial.print(F("Compuerta "));
  Serial.print(i + 1);
  Serial.println(F(" accionada"));
}

/* EXPLICACIÓN: loop(): el ciclo principal, que se repite sin parar
 * Lee byte por byte lo que llega de la ESP32-CAM:
 * - Si el carácter es '#', marca que el siguiente será el código.
 * - Si no es '#' y se estaba esperando el código, lo toma: si está entre '1' y '3', imprime "Recibido: X", llama a accionarCompuerta(c - '1') (que convierte '1' en 0, '2' en 1 y '3' en 2) y después vacía lo que se haya acumulado en el buffer, para no reaccionar a mensajes repetidos que llegaron mientras la compuerta se movía.
 * - Cualquier otro carácter se ignora.
 * Resultado esperado en el Monitor Serie cuando llega un objeto: "Recibido: 2" y luego "Compuerta 2 accionada" (o "Tiempo agotado..." si el objeto nunca llegó).
 */
void loop() {
  while (espSerial.available()) {
    char c = espSerial.read();
    if (c == '#') {
      esperandoCodigo = true;
    } else if (esperandoCodigo) {
      esperandoCodigo = false;
      if (c >= '1' && c <= '3') {
        Serial.print(F("Recibido: "));
        Serial.println(c);
        accionarCompuerta(c - '1');             // '1'->0, '2'->1, '3'->2
        while (espSerial.available()) espSerial.read();  // limpiar lo acumulado
      }
    }
  }
}
