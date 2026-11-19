/*
 * CLASIFICADOR DE PASTILLAS - ESP32-CAM clásica (AI-Thinker, OV3660)
 * Modelo: Edge Impulse (clasificación de imágenes)
 *
 * Etiquetas del modelo:
 *   BANDA TRANSPORTADORA  -> banda vacía (se ignora)
 *   COLOR CÁPSULA         -> compuerta 1
 *   PASTILLA CIRCULAR     -> compuerta 2
 *   PASTILLA CON JARABE   -> compuerta 3
 *
 * Envía por Serial2 dos bytes al Arduino UNO: '#' + código
 *   '1' -> compuerta 1 (servo D9, IR A0)
 *   '2' -> compuerta 2 (servo D10, IR A1)
 *   '3' -> compuerta 3 (servo D11, IR A2)
 *
 * Conexión:  ESP32-CAM IO13 (TX) -> UNO D2 (RX), GND común, 5V.
 *
 * Arduino IDE:
 *   Placa: "AI Thinker ESP32-CAM"
 *   Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
 *   Core ESP32 recomendado: 2.0.17
 */

/* EXPLICACIÓN: Visión general del programa
 * Este código corre en la ESP32-CAM y funciona como los "ojos" del clasificador. Repite este ciclo sin parar:
 *   1. La cámara toma una foto de la banda transportadora.
 *   2. Un modelo de inteligencia artificial (entrenado en Edge Impulse) dice qué objeto ve y con qué confianza.
 *   3. Si la confianza es alta y el mismo objeto se detecta varias veces seguidas, envía por cable (Serial2) dos caracteres al Arduino UNO: '#' y un número ('1', '2' o '3') que indica a qué compuerta debe ir el objeto.
 *   4. Espera unos segundos para no contar dos veces el mismo objeto.
 * El Arduino UNO (archivo arduino_compuertas.ino) es el que mueve los servos.
 * Nota eléctrica: la ESP32 transmite a 3.3 V y el UNO trabaja a 5 V. En este sentido (ESP32 -> UNO) funciona, porque el UNO interpreta 3.3 V como nivel alto. El pin RX de la ESP32 no se usa.
 *
 * Librerías incluidas
 * - AlbertoArMtz-project-1_inferencing.h: librería que exporta Edge Impulse con el modelo ya entrenado. Aporta run_classifier(), el tamaño de entrada del modelo (EI_CLASSIFIER_INPUT_WIDTH/HEIGHT) y la cantidad de etiquetas (EI_CLASSIFIER_LABEL_COUNT).
 * - image.hpp (Edge Impulse): funciones para recortar y redimensionar imágenes.
 * - esp_camera.h: controla la cámara (inicializar, tomar fotos).
 * - img_converters.h: convierte la foto JPEG de la cámara a píxeles RGB, que es lo que necesita el modelo.
 * - ctype.h: aporta tolower(), que se usa para comparar texto sin importar mayúsculas.
 * Resultado: esta parte no imprime nada; si falta alguna librería, el programa no compila.
 */
#include <AlbertoArMtz-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "img_converters.h"
#include <ctype.h>

/* EXPLICACIÓN: Pines de la cámara
 * Estos #define le dicen a la librería de la cámara a qué pin físico de la ESP32 está conectada cada señal del sensor: datos (Y2 a Y9), reloj (XCLK y PCLK), sincronía (VSYNC y HREF), comunicación con el sensor (SIOD y SIOC) y apagado (PWDN). Son los valores fijos de la placa AI-Thinker ESP32-CAM, así que normalmente no se cambian. RESET en -1 significa que ese pin no se usa.
 */
// ---------- Pines de cámara (AI-Thinker ESP32-CAM) ----------
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

/* EXPLICACIÓN: Comunicación con el Arduino UNO
 * - UART_TX_PIN 13: la ESP32 transmite por el GPIO13, que se conecta al pin D2 del UNO.
 * - UART_RX_PIN 15: se declara porque la función de inicio lo pide, pero no se conecta a nada.
 * - UART_BAUD 9600: velocidad de la comunicación. Debe ser igual en el Arduino (espSerial.begin(9600)); si no coinciden, el UNO recibe caracteres basura.
 */
// ---------- Serial hacia el Arduino UNO ----------
#define UART_TX_PIN 13        // -> UNO D2
#define UART_RX_PIN 15        // sin usar
#define UART_BAUD   9600

/* EXPLICACIÓN: Parámetros de decisión (se ajustan sin tocar el resto del código)
 * - UMBRAL_CONFIANZA 0.80: el modelo debe estar al menos 80 % seguro para que se tome en cuenta una detección.
 * - FRAMES_CONSECUTIVOS 2: el mismo objeto debe detectarse en 2 lecturas seguidas antes de enviarlo. Subirlo a 3 reduce los falsos positivos, pero responde más lento.
 * - ENFRIAMIENTO_MS 3500: después de enviar un objeto, el programa se detiene 3.5 s para no clasificar la misma pastilla otra vez mientras sigue frente a la cámara.
 * - MOSTRAR_DETECCIONES 1: si vale 1, cada lectura se imprime en el Monitor Serie (útil para calibrar); con 0 solo se ven los envíos.
 */
// ---------- Parámetros de decisión ----------
#define UMBRAL_CONFIANZA     0.80f
#define FRAMES_CONSECUTIVOS  2        // sube a 3 si ves falsos positivos
#define ENFRIAMIENTO_MS      3500     // pausa tras enviar un objeto
#define MOSTRAR_DETECCIONES  1        // 1 = imprime cada lectura en el Monitor Serie; 0 = solo los envíos

/* EXPLICACIÓN: Tabla de clases: traduce lo que ve el modelo a un número de compuerta
 * Cada fila de la tabla es un par "clave -> código". El programa busca la clave dentro del nombre de la etiqueta del modelo, sin importar mayúsculas:
 *   "color"    -> '1'  (etiqueta COLOR CÁPSULA)
 *   "circular" -> '2'  (etiqueta PASTILLA CIRCULAR)
 *   "jarabe"   -> '3'  (etiqueta PASTILLA CON JARABE)
 * La etiqueta BANDA TRANSPORTADORA (banda vacía) no está en la tabla, por eso se ignora sola. NUM_CLASES calcula cuántas filas tiene la tabla (tamaño total / tamaño de una fila), así que si agregas una fila no hay que cambiar nada más.
 * Para cambiar a qué compuerta va cada objeto, basta con cambiar el código ('1', '2' o '3').
 */
// ---------- TABLA DE CLASES ----------
// Se busca la CLAVE dentro de la etiqueta del modelo, sin importar mayúsculas.
// (Se usan palabras sin acentos para evitar problemas con la Á de CÁPSULA.)
// La banda vacía no está en la tabla: se ignora sola.
// Si quieres cambiar a qué compuerta va cada objeto, cambia el código ('1','2','3').
struct Clase { const char *clave; char codigo; };
const Clase CLASES[] = {
  {"color",    '1'},   // COLOR CÁPSULA
  {"circular", '2'},   // PASTILLA CIRCULAR
  {"jarabe",   '3'},   // PASTILLA CON JARABE
};
const int NUM_CLASES = sizeof(CLASES) / sizeof(CLASES[0]);

/* EXPLICACIÓN: Tamaño de la imagen y buffer
 * RAW_W y RAW_H (320 x 240) son el tamaño de la foto que captura la cámara (formato QVGA). snapshot_buf es el espacio de memoria donde se guarda esa foto ya convertida a píxeles RGB. Empieza en nullptr (vacío) y se reserva en setup().
 */
#define RAW_W 320
#define RAW_H 240

static uint8_t *snapshot_buf = nullptr;

/* EXPLICACIÓN: contiene(texto, clave): busca una palabra dentro de un texto
 * Recorre el texto posición por posición y compara letra por letra con la clave, pasando cada letra a minúscula con tolower(). Devuelve true si la clave aparece en cualquier parte. Ejemplo: contiene("PASTILLA CON JARABE", "jarabe") devuelve true.
 * Sirve porque las etiquetas del modelo tienen mayúsculas y espacios; así no hace falta que coincidan exactamente.
 */
// true si "texto" contiene "clave" (clave en minúsculas), sin distinguir mayúsculas
static bool contiene(const char *texto, const char *clave) {
  size_t n = strlen(clave);
  for (size_t i = 0; texto[i]; i++) {
    size_t j = 0;
    while (j < n && texto[i + j] && tolower((unsigned char)texto[i + j]) == clave[j]) j++;
    if (j == n) return true;
  }
  return false;
}

/* EXPLICACIÓN: iniciarCamara(): configura y enciende la cámara
 * - Llena una estructura camera_config_t con los pines de la cámara, la frecuencia del reloj (20 MHz) y el canal PWM que usa.
 * - pixel_format = PIXFORMAT_JPEG y frame_size = FRAMESIZE_QVGA: fotos de 320x240 comprimidas en JPEG. jpeg_quality = 12 es un balance entre calidad y tamaño (menor número = mejor calidad).
 * - fb_count = 1 y fb_location = PSRAM: usa un solo buffer de imagen, guardado en la memoria PSRAM de la ESP32.
 * - grab_mode = WHEN_EMPTY: toma una foto nueva solo cuando el buffer está vacío.
 * - Devuelve true si esp_camera_init() tuvo éxito y false si falló.
 * Nota: los comentarios sobre pin_sscb_sda indican que, con una versión vieja del core de ESP32, esos nombres cambian.
 */
static bool iniciarCamara() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;   // si tu core es viejo y da error: pin_sscb_sda
  c.pin_sccb_scl = SIOC_GPIO_NUM;   // idem: pin_sscb_scl
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QVGA;     // 320x240
  c.jpeg_quality = 12;
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  return esp_camera_init(&c) == ESP_OK;
}

/* EXPLICACIÓN: ei_camera_get_data(): entrega la imagen al modelo de Edge Impulse
 * Edge Impulse no recibe la imagen completa de golpe: llama a esta función (callback) pidiendo un tramo de píxeles, indicado por offset (dónde empezar) y length (cuántos). Por cada píxel combina sus 3 bytes (R, G, B) en un solo número de tipo float, que es el formato que espera el modelo, y lo coloca en out_ptr. Devuelve 0 para indicar que todo salió bien.
 */
// Callback que usa Edge Impulse para leer la imagen
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t out_ix = 0;
  while (length--) {
    out_ptr[out_ix++] = (snapshot_buf[pixel_ix + 2] << 16) +
                        (snapshot_buf[pixel_ix + 1] << 8) +
                         snapshot_buf[pixel_ix];
    pixel_ix += 3;
  }
  return 0;
}

/* EXPLICACIÓN: capturar(): toma una foto y la deja lista para el modelo
 * 1. esp_camera_fb_get() pide a la cámara la foto actual (en JPEG). Si no hay foto, devuelve false.
 * 2. fmt2rgb888() convierte el JPEG a píxeles RGB y los guarda en snapshot_buf.
 * 3. esp_camera_fb_return() devuelve el buffer a la cámara; es obligatorio, o la cámara se queda sin memoria para la siguiente foto.
 * 4. Si el tamaño que pide el modelo (w, h) es distinto a 320x240, crop_and_interpolate_rgb888() recorta y redimensiona la imagen.
 * Devuelve true si todo salió bien y false si algún paso falló.
 */
static bool capturar(uint32_t w, uint32_t h, uint8_t *out) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);
  if (!ok) return false;
  if (w != RAW_W || h != RAW_H) {
    ei::image::processing::crop_and_interpolate_rgb888(out, RAW_W, RAW_H, out, w, h);
  }
  return true;
}

/* EXPLICACIÓN: setup(): se ejecuta una sola vez al encender
 * 1. Serial.begin(115200) abre el Monitor Serie para ver mensajes en la computadora.
 * 2. Serial2.begin(...) abre la comunicación hacia el Arduino UNO (9600 baudios, 8N1, con los pines TX y RX definidos arriba).
 * 3. Si la cámara no inicia, imprime "Error al iniciar la camara" y se queda detenida (while(true)); en ese caso conviene revisar la conexión del módulo y la alimentación.
 * 4. Si el sensor es un OV3660 (el que trae este módulo), aplica los mismos ajustes que el ejemplo CameraWebServer (imagen volteada, brillo +1, saturación -2). Así las fotos con las que se entrenó el modelo y las que se clasifican ahora se ven igual; si fueran distintas, el modelo se confundiría.
 * 5. ps_malloc() reserva en la PSRAM la memoria del buffer (320 x 240 x 3 bytes, uno por cada color). Si no hay PSRAM disponible, avisa y se detiene.
 * Resultado esperado en el Monitor Serie: "Listo. Clasificando..." 
 */
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (!iniciarCamara()) {
    Serial.println("Error al iniciar la camara");
    while (true) delay(1000);
  }

  // Tu módulo trae sensor OV3660. Estos ajustes son los mismos que aplica el
  // ejemplo CameraWebServer, para que las fotos del dataset y las de la
  // clasificación salgan iguales.
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  snapshot_buf = (uint8_t *)ps_malloc(RAW_W * RAW_H * 3);
  if (!snapshot_buf) {
    Serial.println("No hay memoria para el buffer (revisa PSRAM)");
    while (true) delay(1000);
  }
  Serial.println("Listo. Clasificando...");
}

/* EXPLICACIÓN: loop(): el ciclo principal, que se repite sin parar
 * Cada vuelta toma una foto, la clasifica y decide si hay que enviar algo al Arduino. Los pasos se explican dentro de la función.
 * Resultados esperados en el Monitor Serie (si MOSTRAR_DETECCIONES vale 1):
 *   Lectura: BANDA TRANSPORTADORA (0.97)   -> banda vacía, no se envía nada
 *   Lectura: PASTILLA CIRCULAR (0.91)      -> primera detección válida (1 de 2 frames)
 *   ENVIADO: PASTILLA CIRCULAR (0.93) -> '2'   -> se confirmó y se mandó al UNO
 * (Los valores de ejemplo son ilustrativos; dependen de lo que vea la cámara.)
 */
void loop() {
  /* EXPLICACIÓN: Variables que se conservan entre vueltas (static)
   * - ultimaClase: índice de la última clase detectada (-1 = ninguna).
   * - repeticiones: cuántas lecturas seguidas se ha detectado esa misma clase.
   */
  static int ultimaClase = -1;
  static int repeticiones = 0;

  /* EXPLICACIÓN: Paso 1: capturar la imagen
   * Toma una foto con el tamaño que necesita el modelo. Si falla, imprime "Fallo la captura", espera 100 ms y vuelve a intentar en la siguiente vuelta.
   */
  if (!capturar(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
    Serial.println("Fallo la captura");
    delay(100);
    return;
  }

  /* EXPLICACIÓN: Paso 2: ejecutar el modelo
   * Se arma una "señal" que le indica a Edge Impulse cuántos píxeles hay y qué función usar para leerlos (ei_camera_get_data). run_classifier() corre la red neuronal y llena result con un valor entre 0 y 1 por cada etiqueta (la confianza). Si el clasificador falla, se salta esta vuelta.
   */
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

  /* EXPLICACIÓN: Paso 3: encontrar la etiqueta ganadora
   * Recorre las confianzas de todas las etiquetas y se queda con el índice (mejor) y el valor (mejorValor) de la más alta.
   */
  int mejor = -1;
  float mejorValor = 0;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > mejorValor) {
      mejorValor = result.classification[i].value;
      mejor = i;
    }
  }

/* EXPLICACIÓN: Paso 4: mostrar la lectura (solo si MOSTRAR_DETECCIONES vale 1)
 * Imprime la etiqueta ganadora y su confianza. Es útil para calibrar el umbral y ver qué tan seguro está el modelo.
 */
#if MOSTRAR_DETECCIONES
  if (mejor >= 0) {
    Serial.printf("Lectura: %s (%.2f)\n", result.classification[mejor].label, mejorValor);
  }
#endif

  /* EXPLICACIÓN: Paso 5: filtrar por confianza
   * Si la confianza es menor a 0.80 (o no hubo ganadora), la detección no es confiable: se reinicia el conteo de repeticiones y se pasa a la siguiente vuelta sin enviar nada.
   */
  if (mejor < 0 || mejorValor < UMBRAL_CONFIANZA) {
    repeticiones = 0; ultimaClase = -1;
    return;
  }

  /* EXPLICACIÓN: Paso 6: traducir la etiqueta a código de compuerta
   * Se recorre la tabla CLASES; en cuanto una clave aparece en la etiqueta ganadora, se guarda su código ('1', '2' o '3') y se detiene la búsqueda con break.
   */
  const char *etiqueta = result.classification[mejor].label;
  char codigo = 0;
  for (int k = 0; k < NUM_CLASES; k++) {
    if (contiene(etiqueta, CLASES[k].clave)) {
      codigo = CLASES[k].codigo;
      break;
    }
  }

  /* EXPLICACIÓN: Paso 7: ignorar banda vacía
   * Si no se encontró código (codigo vale 0), la etiqueta es BANDA TRANSPORTADORA o una que no está en la tabla: se reinicia el conteo y no se envía nada.
   */
  // Banda vacía o etiqueta que no está en la tabla: no se envía nada
  if (codigo == 0) { repeticiones = 0; ultimaClase = -1; return; }

  /* EXPLICACIÓN: Paso 8: confirmar con lecturas consecutivas
   * Si es la misma clase que la lectura anterior, suma una repetición; si cambió, reinicia el conteo en 1 con la nueva clase. Así una detección aislada (por ejemplo, por una sombra) no dispara el envío.
   */
  if (mejor == ultimaClase) repeticiones++;
  else { ultimaClase = mejor; repeticiones = 1; }

  /* EXPLICACIÓN: Paso 9: enviar al Arduino UNO
   * Con 2 lecturas seguidas de la misma clase:
   * - Serial2.write('#') y Serial2.write(codigo) mandan los dos bytes al UNO: '#' marca el inicio del mensaje y el segundo carácter dice a qué compuerta ir.
   * - Se imprime "ENVIADO..." en el Monitor Serie con la etiqueta, la confianza y el código.
   * - Se reinicia el conteo y se hace delay(ENFRIAMIENTO_MS): el programa se detiene 3.5 s para no contar el mismo objeto dos veces. Durante esa pausa no se toman fotos.
   */
  if (repeticiones >= FRAMES_CONSECUTIVOS) {
    Serial2.write('#');
    Serial2.write(codigo);
    Serial.printf("ENVIADO: %s (%.2f) -> '%c'\n", etiqueta, mejorValor, codigo);
    repeticiones = 0; ultimaClase = -1;
    delay(ENFRIAMIENTO_MS);   // evita contar el mismo objeto dos veces
  }
}
