// ============================================
// Control de Motor DC con Encoder JGB37-3530
// Control PID Lazo Cerrado
// Modelo: PID Ideal con Derivativo sobre la Medición
// ============================================

// Pines del motor
#define IN1 6
#define IN2 5

// Pines del encoder (interrupciones)
#define ENC_A 2   // INT0
#define ENC_B 3   // INT1

// Parámetros del motor
const float PWM_MIN = 20.0;          // % mínimo donde el motor gira
const float GEAR_RATIO = 56.25;      // Relación de reducción
const int PULSES_PER_REV = 16;       // Pulsos por revolución del encoder (por canal)
const int CPR_OUTPUT = PULSES_PER_REV * 4 * GEAR_RATIO;  // x4 = 3600 pulsos en la salida

// Intervalo de muestreo para PID
// CAMBIAR ESTO DE ACUERDO AL ROVER:
// entre mayor el tiempo de muestra menos preciso a pocas RPMs y más preciso a altas rpms, y viceversa
const unsigned long SAMPLE_MS = 200; 

// Variables de control PID
float setpointRPM = 0.0;             // RPM deseadas
float currentRPM = 0.0;              // RPM medidas
float Kp = 0.0;                      // Ganancia proporcional
float Ki = 1.0;                      // Ganancia integral
float Kd = 0.0;                      // Ganancia derivativa

// Variables internas del PID
float error = 0.0;                   // Error actual
float errorSum = 0.0;                // Acumulador integral
float errorPrev = 0.0;               // Error previo para derivativo
float measuredPrev = 0.0;            // Medición previa (para derivativo)
float pidOutput = 0.0;               // Salida del PID
const float INTEGRAL_MAX = 200.0;    // Anti-windup (outputMax/Ki = 100/0.5)

// Variables de control del motor
float pwmPercent = 0.0;              // 0-100%
bool direccion = true;               // true=adelante, false=atrás

// Variables del encoder (volátiles porque se usan en ISR)
volatile long encoderTicks = 0;

// Variables de tiempo
unsigned long lastSampleTime = 0;

// Buffer serial
String inputBuffer = "";

// ============================================
// ISR - Lectura del encoder con cuadratura x4
// ============================================
void ISR_encoderA() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);
  
  if (A == B) {
    encoderTicks--;
  } else {
    encoderTicks++;
  }
}

void ISR_encoderB() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);
  
  if (A != B) {
    encoderTicks--;
  } else {
    encoderTicks++;
  }
}

// ============================================
// Función: Calcular RPM actuales
// ============================================
float calcularRPM(long ticks, float dt) {
  return (ticks / (float)CPR_OUTPUT) * (60.0 / dt);
}

// ============================================
// Función: Calcular PID
// ============================================
float computePID(float dt) {
  // Protección contra dt inválido
  if (dt <= 0) return pidOutput;
  
  // Calcula error (usa valor absoluto de RPM)
  error = setpointRPM - abs(currentRPM);
  
  // Término proporcional
  float P = Kp * error;
  
  // Término integral con anti-windup
  errorSum += error * dt;
  errorSum = constrain(errorSum, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = Ki * errorSum;
  
  // Término derivativo
  float errorDiff = (error - errorPrev) / dt;
  float D = Kd * errorDiff;
  errorPrev = error;
  
  // Salida PID
  pidOutput = P + I + D;
  pidOutput = constrain(pidOutput, 0.0, 100.0);
  
  return pidOutput;
}

// ============================================
// Función: Controlar motor
// ============================================
void setMotor(float percent, bool forward) {
  percent = constrain(percent, 0.0, 100.0);
  
  // Caso especial: apagar motor
  if (percent < 0.1) {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    return;
  }
  
  // Reescalamiento: 0-100% lógico → PWM_MIN-100% físico
  float percentReal = map(percent * 10, 0, 1000, PWM_MIN * 10, 1000) / 10.0;
  int pwmValue = (int)(percentReal * 2.55);
  
  // Aplica PWM según dirección
  if (forward) {
    analogWrite(IN1, pwmValue);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, pwmValue);
  }
}

// ============================================
// Función: Procesar comandos seriales
// ============================================
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.length() == 0) return;
  
  cmd.replace(",", " ");
  cmd.replace("  ", " ");
  
  // Comando S (Setpoint RPM)
  int sIdx = cmd.indexOf('S');
  if (sIdx != -1) {
    int endIdx = cmd.indexOf(' ', sIdx);
    if (endIdx == -1) endIdx = cmd.length();
    
    String sStr = cmd.substring(sIdx + 1, endIdx);
    sStr.trim();
    
    float newSetpoint = constrain(sStr.toFloat(), 0.0, 67.0);
    
    // Solo actualiza si hay cambio significativo
    if (abs(newSetpoint - setpointRPM) > 0.1) {
      setpointRPM = newSetpoint;
    }
  }
  
  // Comando D (Dirección)
  int dIdx = cmd.indexOf('D');
  if (dIdx != -1) {
    String dStr = cmd.substring(dIdx + 1);
    dStr.trim();
    int dirValue = dStr.toInt();
    bool newDir = (dirValue == 1);
    
    // Solo actualiza si hay cambio
    if (newDir != direccion) {
      direccion = newDir;
    }
  }
  
  // ========================================
  // COMANDOS OPCIONALES PARA AJUSTAR PID
  // Si no los necesitas, puedes eliminar esta sección completa
  // ========================================
  
  // Comando KP (Ajustar Kp)
  if (cmd.indexOf("KP") != -1) {
    int kpIdx = cmd.indexOf("KP");
    String kpStr = cmd.substring(kpIdx + 2);
    int endIdx = kpStr.indexOf(' ');
    if (endIdx == -1) endIdx = kpStr.length();
    Kp = kpStr.substring(0, endIdx).toFloat();
  }
  
  // Comando KI (Ajustar Ki)
  if (cmd.indexOf("KI") != -1) {
    int kiIdx = cmd.indexOf("KI");
    String kiStr = cmd.substring(kiIdx + 2);
    int endIdx = kiStr.indexOf(' ');
    if (endIdx == -1) endIdx = kiStr.length();
    Ki = kiStr.substring(0, endIdx).toFloat();
  }
  
  // Comando KD (Ajustar Kd)
  if (cmd.indexOf("KD") != -1) {
    int kdIdx = cmd.indexOf("KD");
    String kdStr = cmd.substring(kdIdx + 2);
    int endIdx = kdStr.indexOf(' ');
    if (endIdx == -1) endIdx = kdStr.length();
    Kd = kdStr.substring(0, endIdx).toFloat();
  }
  
  // ========================================
  // FIN COMANDOS OPCIONALES
  // ========================================
}

// ============================================
// Setup
// ============================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_encoderB, CHANGE);
  
  lastSampleTime = millis();
  inputBuffer.reserve(50);

//  Serial.println("\n========================================");
//  Serial.println("Control PID Motor DC - Lazo Cerrado");
//  Serial.println("Modelo: PID Ideal (Derivativo sobre PV)");
//  Serial.println("========================================");
//  Serial.println("Comandos:");
//  Serial.println("  S<rpm>    - Setpoint (0-67 RPM)");
//  Serial.println("  D<0/1>    - Direccion");
//  Serial.println("  KP/KI/KD  - Ajustar ganancias (opcional)");
//  Serial.println("\nEjemplos:");
//  Serial.println("  S50       - 50 RPM");
//  Serial.println("  S0        - Detener");
//  Serial.println("  D0        - Invertir");
//  Serial.println("========================================");
//  Serial.print("PID -> Kp:");
//  Serial.print(Kp, 2);
//  Serial.print(" Ki:");
//  Serial.print(Ki, 2);
//  Serial.print(" Kd:");
//  Serial.println(Kd, 2);
//  Serial.println("========================================\n");
}

// ============================================
// Loop principal
// ============================================
void loop() {
  // Procesa entrada serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
  
  // Ejecuta control PID periódicamente
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_MS) {
    // Lee encoder de forma atómica
    noInterrupts();
    long ticks = encoderTicks;
    encoderTicks = 0;
    interrupts();
    
    // Calcula tiempo transcurrido
    float dt = (now - lastSampleTime) / 1000.0;
    
    // Calcula RPM actuales
    currentRPM = calcularRPM(ticks, dt);
    
    pwmPercent = computePID(dt);
    setMotor(pwmPercent, direccion);
      
    // Salida para Serial Plotter: UNA línea con todas las señales
    Serial.print("Setpoint:");
    Serial.print(setpointRPM, 2);
    Serial.print(" Actual:");
    Serial.print(currentRPM, 2);
    Serial.print(" PWM:");
    Serial.print(pwmPercent, 1);
    Serial.print(" Error:");
    Serial.println(error, 2);   // <-- solo el último lleva println()  
    
    lastSampleTime = now;
  }
}
