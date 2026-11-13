const int PIN_TRIG          = 9;
const int PIN_ECHO          = 10;
const int PIN_BOMBA_VACIAR  = 3;
const int PIN_BOMBA_LLENAR  = 5;
const bool HAY_BOMBA_LLENAR = false;

const float DIST_SENSOR_FONDO_CM = 14.6;
const float NIVEL_MAX_CM = 11.0;
const float NIVEL_MIN_CM = 4.0;

float velocidad = 40.0;
float setpoint  = 7.0;
float Kp        = 15.0;
float Ti        = 20.0;

const float KP_INESTABLE = 40.0;
const float RETARDO_S    = 8.0;
const float PWM_MINIMO   = 35.0;
const float TS           = 0.2;

const bool PWM_ALTA_FRECUENCIA = false;

const int MAX_RETARDO = 100;
float bufferNivel[MAX_RETARDO];
int   idxBuffer = 0;
int   modo = 0;
bool  llenar = true;
float integral = 0;
float nivelAnterior = 0;
bool  primeraLectura = true;
const float FILTRO = 0.25;
float ultimaDistancia = -1;
bool  soloMedir = false;
int   fallosSensor = 0;
unsigned long tAnterior = 0;

float leerDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (duracion == 0) return -1.0;
  return duracion * 0.0343 / 2.0;
}

float leerNivelCm() {
  float v[5];
  int n = 0;
  for (int i = 0; i < 5; i++) {
    float d = leerDistanciaCm();
    if (d > 0) v[n++] = d;
    delay(4);
  }
  if (n == 0) { fallosSensor++; return nivelAnterior; }
  fallosSensor = 0;
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  ultimaDistancia = v[n / 2];
  float crudo = DIST_SENSOR_FONDO_CM - v[n / 2];
  if (crudo < 0) crudo = 0;

  float nivel = primeraLectura ? crudo : nivelAnterior + FILTRO * (crudo - nivelAnterior);
  primeraLectura = false;
  nivelAnterior = nivel;
  return nivel;
}

int pwmDe(float u) {
  if (u <= 0.5) return 0;
  float pwm = PWM_MINIMO + u * (100.0 - PWM_MINIMO) / 100.0;
  return (int)(pwm * 255.0 / 100.0);
}

float aplicarBomba(float u) {
  u = constrain(u, 0, 100);
  if (llenar) {
    analogWrite(PIN_BOMBA_VACIAR, 0);
    if (HAY_BOMBA_LLENAR) analogWrite(PIN_BOMBA_LLENAR, pwmDe(u));
    else u = 0;
  } else {
    if (HAY_BOMBA_LLENAR) analogWrite(PIN_BOMBA_LLENAR, 0);
    analogWrite(PIN_BOMBA_VACIAR, pwmDe(u));
  }
  return u;
}

void apagarBombas() {
  analogWrite(PIN_BOMBA_VACIAR, 0);
  if (HAY_BOMBA_LLENAR) analogWrite(PIN_BOMBA_LLENAR, 0);
}

void cambiarModo(int m) {
  modo = m;
  integral = 0;
  for (int i = 0; i < MAX_RETARDO; i++) bufferNivel[i] = nivelAnterior;
  Serial.print(llenar ? F("# LLENAR - modo ") : F("# VACIAR - modo "));
  Serial.print(m);
  if (m == 0) Serial.println(F(" - detenido"));
  if (m == 1) { Serial.print(F(" - lazo abierto al ")); Serial.print(velocidad, 0); Serial.println(F(" %")); }
  if (m == 2) { Serial.print(F(" - control P con Kp = ")); Serial.println(Kp); }
  if (m == 3) { Serial.print(F(" - control PI con Kp = ")); Serial.print(Kp); Serial.print(F(" y Ti = ")); Serial.println(Ti); }
  if (m == 4) Serial.println(F(" - control P con retardo"));
}

bool elegirSentido(bool quiereLlenar) {
  if (quiereLlenar && !HAY_BOMBA_LLENAR) {
    Serial.println(F("# No hay bomba de llenado conectada. La bomba del tanque solo puede vaciar."));
    Serial.println(F("# Llena el tanque desde el deposito o escribe V para vaciar."));
    return false;
  }
  llenar = quiereLlenar;
  apagarBombas();
  cambiarModo(0);
  return true;
}

void preguntarSentido() {
  float nivel = leerNivelCm();
  Serial.println(F("# ----------------------------------------"));
  Serial.print(F("# Nivel actual = ")); Serial.print(nivel, 1);
  Serial.print(F(" cm    Setpoint = ")); Serial.print(setpoint, 1); Serial.println(F(" cm"));
  if (!HAY_BOMBA_LLENAR) Serial.println(F("# Solo hay bomba de vaciar: escribe V (o M para solo medir)"));
  else if (nivel < setpoint) Serial.println(F("# El nivel esta por DEBAJO del setpoint (sugerido L)"));
  else                  Serial.println(F("# El nivel esta por ENCIMA del setpoint (sugerido V)"));
  Serial.println(F("# Que quieres hacer?  L = llenar   V = vaciar   M = solo medir"));
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if ((c == 'L' || c == 'l') && elegirSentido(true))  break;
      if ((c == 'V' || c == 'v') && elegirSentido(false)) break;
      if (c == 'M' || c == 'm') {
        elegirSentido(false);
        soloMedir = true;
        Serial.println(F("# SOLO MEDIR - bomba apagada. Escribe M para salir"));
        return;
      }
    }
  }
  Serial.println(F("# Escribe 1 (lazo abierto), 2 (P), 3 (PI) o 4 (inestable) para empezar"));
}

void leerComandos() {
  if (Serial.available() == 0) return;
  char c = Serial.read();
  if (c == 'X' || c == 'x') { soloMedir = false; apagarBombas(); cambiarModo(0); Serial.println(F("# PARO - bombas apagadas")); return; }
  if (c == 'M' || c == 'm') {
    soloMedir = !soloMedir;
    apagarBombas();
    if (soloMedir) Serial.println(F("# SOLO MEDIR - bomba apagada. Escribe M para salir"));
    else           cambiarModo(0);
    return;
  }
  if (c >= '0' && c <= '4') { cambiarModo(c - '0'); return; }
  if (c == '\n' || c == '\r' || c == ' ') return;
  if (c == 'L' || c == 'l') { elegirSentido(true);  return; }
  if (c == 'V' || c == 'v') {

    delay(20);
    if (Serial.peek() >= '0' && Serial.peek() <= '9') {
      velocidad = constrain(Serial.parseFloat(), 0, 100);
      cambiarModo(1);
    } else {
      elegirSentido(false);
    }
    return;
  }
  float x = Serial.parseFloat();
  if (c == 'r' || c == 'R') {
    if (x > NIVEL_MIN_CM && x < NIVEL_MAX_CM) { setpoint = x; Serial.print(F("# Setpoint = ")); Serial.println(setpoint); }
    else Serial.println(F("# Setpoint fuera de rango"));
  }
  if ((c == 'k' || c == 'K') && x > 0) { Kp = x; Serial.print(F("# Kp = ")); Serial.println(Kp); }
  if ((c == 'i' || c == 'I') && x > 0) { Ti = x; integral = 0; Serial.print(F("# Ti = ")); Serial.println(Ti); }
}

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BOMBA_VACIAR, OUTPUT);
  pinMode(PIN_BOMBA_LLENAR, OUTPUT);
  if (PWM_ALTA_FRECUENCIA) TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  apagarBombas();
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(300);
  preguntarSentido();
}

void loop() {
  leerComandos();

  unsigned long ahora = millis();
  if (ahora - tAnterior < (unsigned long)(TS * 1000)) return;
  tAnterior = ahora;

  float nivel = leerNivelCm();

  if (soloMedir) {
    apagarBombas();
    Serial.print(F("Distancia:")); Serial.print(ultimaDistancia, 2);
    Serial.print(F(",Nivel:"));    Serial.println(nivel, 2);
    return;
  }

  bufferNivel[idxBuffer] = nivel;
  int nRet = min((int)(RETARDO_S / TS), MAX_RETARDO - 1);
  float nivelViejo = bufferNivel[(idxBuffer - nRet + MAX_RETARDO) % MAX_RETARDO];
  idxBuffer = (idxBuffer + 1) % MAX_RETARDO;

  float signo = llenar ? 1.0 : -1.0;
  float e      = signo * (setpoint - nivel);
  float eViejo = signo * (setpoint - nivelViejo);
  float u = 0;

  if (modo == 1) u = velocidad;
  if (modo == 2) u = Kp * e;
  if (modo == 3) {
    float prueba = Kp * (e + (integral + e * TS) / Ti);
    if ((prueba > 0 && prueba < 100) || (prueba >= 100 && e < 0) || (prueba <= 0 && e > 0))
      integral += e * TS;
    u = Kp * (e + integral / Ti);
  }
  if (modo == 4) u = KP_INESTABLE * eViejo;

  if (llenar  && nivel >= NIVEL_MAX_CM) u = 0;
  if (!llenar && nivel <= NIVEL_MIN_CM) u = 0;
  if (fallosSensor >= 10) u = 0;

  u = aplicarBomba(u);

  Serial.print(F("Nivel:"));     Serial.print(nivel, 2);
  Serial.print(F(",Setpoint:")); Serial.print(setpoint, 1);
  Serial.print(F(",Bomba:"));    Serial.println(u / 10.0, 2);
}
