#define IN1 6
#define IN2 5

#define ENC_A 2   // INT0
#define ENC_B 3   // INT1 (lo usamos como entrada digital)

const int N = 20;
float rpmBuffer[N];
int idx = 0;
bool bufferFilled = false;

unsigned long t0;

const float CPR = 1800.0f;

// === PID ===
float Kp = 0.047;
float Ki = 0.32;
float Kd = 0.002;

float setpoint = 0;      // RPM objetivo (eje de salida)
float rpmActual = 0;
float error = 0;
float errorPrev = 0;
float errorSum = 0;
float errorDiff = 0;

float pidOutput = 0;
float pwmPercent = 0;
int pwmValue = 0;
bool direccion = true;

float pidInterval = 50;   // ms

float integralMax = 1000;
float outputMax = 100;

String cmd = "";
bool stringComplete = false;

// === Encoder contador (se actualiza en ISR) ===
volatile long encCount = 0;

void ISR_A() {
  bool b = digitalRead(ENC_B);
  // Si A cambia y B está en cierto estado, eso indica sentido.
  // La regla exacta depende del cableado/fase. Si va al revés, invierte +/-
  if (b) encCount++;
  else   encCount--;
}

// Aplica PWM y dirección al motor
void setMotor(float percent, bool forward) {
  percent = constrain(percent, 0, 100);
  pwmValue = map((int)percent, 0, 100, 0, 255);

  if (forward) {
    analogWrite(IN1, pwmValue);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, pwmValue);
  }
}

// PID (igual que el tuyo)
float computePID(float dt) {
  if (dt <= 0) return pidOutput;

  error = setpoint - rpmActual;
  float P = Kp * error;

  errorSum += error * dt;
  errorSum = constrain(errorSum, -integralMax, integralMax);
  float I = Ki * errorSum;

  errorDiff = (error - errorPrev) / dt;
  float D = Kd * errorDiff;

  pidOutput = P + I + D;
  errorPrev = error;

  pidOutput = constrain(pidOutput, -outputMax, outputMax);
  return pidOutput;
}

void setup() {
  Serial.begin(115200);
  t0 = millis();
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  // Encoder
  pinMode(ENC_A, INPUT);   // si tu encoder NO trae pull-ups internas, cambia a INPUT_PULLUP
  pinMode(ENC_B, INPUT);   // idem
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_A, CHANGE);

  for (int i = 0; i < N; i++) rpmBuffer[i] = 0;

  cmd.reserve(50);

  delay(500);
}

void loop() {

  while (Serial.available()) {
    char inChar = (char)Serial.read();
  
    if (inChar == '\n' || inChar == '\r') {   // acepta Enter y CRLF
      stringComplete = true;
    } else {
      cmd += inChar;
    }
  }
  
  if (stringComplete) {
    cmd.trim();                 // elimina espacios y saltos
    if (cmd.length() > 0) {     // solo procesa si hay algo válido
      float valor = cmd.toFloat();
      setpoint = constrain(valor, 0, 300);  // RPM objetivo
    }
  
    cmd = "";                   // limpia buffer SIEMPRE
    stringComplete = false;     // reinicia bandera
  }

  static long prevCount = 0;
  static unsigned long prevTimeMs = 0;

  unsigned long nowMs = millis();
  if (prevTimeMs == 0) { prevTimeMs = nowMs; }

  float rpmInst = rpmActual;

  if (nowMs - prevTimeMs >= (unsigned long)pidInterval) {
    noInterrupts();
    long c = encCount;
    interrupts();

    long dCount = c - prevCount;
    float dt = (nowMs - prevTimeMs) / 1000.0f;

    prevCount = c;
    prevTimeMs = nowMs;

    // RPM salida = (dCount/CPR) * (60/dt)
    rpmInst = (dCount / CPR) * (60.0f / dt);

    // Guardar en buffer para promedio móvil
    rpmBuffer[idx] = rpmInst;
    idx++;
    if (idx >= N) { idx = 0; bufferFilled = true; }

    float sum = 0;
    int samples = bufferFilled ? N : idx;
    for (int i = 0; i < samples; i++) sum += rpmBuffer[i];
    rpmActual = sum / samples;

    pidOutput = computePID(dt);

    if (pidOutput >= 0) { pwmPercent = abs(pidOutput); direccion = true; }
    else               { pwmPercent = abs(pidOutput); direccion = false; }
    
    setMotor(pwmPercent, direccion);

    // Telemetría
    float t = (millis() - t0) / 1000.0f;
    Serial.print(t, 2); Serial.print("\t");
    Serial.print("PV:"); Serial.print(rpmActual, 2);
    Serial.print(",SP:"); Serial.print(setpoint, 2);
    Serial.print(",OP:"); Serial.print(pwmPercent, 2);
    Serial.print(",Err:"); Serial.print(error, 2);
    Serial.print(",Cnt:"); Serial.println(c);
  }
  delay(5);
}
