#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =========================================================================
// MOTOR PIN DEFINITIONS
// =========================================================================
const int STBY = 19; 
const int PWMA = 23, AIN1 = 21, AIN2 = 22, motorAChannel = 0;
const int PWMB = 2, BIN1 = 18, BIN2 = 16, motorBChannel = 1;
const int freq = 30000, resolution = 8;
const int MAX_DUTY = 255;

int BASE_SPEED = 200;
float Kp = 4.0, Ki = 0.00, Kd = 0.50;
int lastError = 0;
float errorSum = 0;

// =========================================================================
// SENSOR DEFINITIONS
// =========================================================================
const int NUM_SENSORS = 6;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33};
const int sensorWeights[NUM_SENSORS] = {-200, -120, -40, 40, 120, 200};
const int WHITE_VALUE = 200, BLACK_VALUE = 4000;

// =========================================================================
// BLE CONFIGURATION
// =========================================================================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_CONTROL   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_STATUS    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

BLEServer* pServer = NULL;
BLECharacteristic* pCharControl = NULL;
BLECharacteristic* pCharStatus = NULL;
bool deviceConnected = false;

// Operating modes
enum Mode { MODE_STOP, MODE_FOLLOWING };
Mode currentMode = MODE_STOP;

// =========================================================================
// BLE CALLBACKS
// =========================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE client connected");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
        delay(500);
        pServer->getAdvertising()->start();
        Serial.println("Advertising restarted");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        
        if (value.length() > 0) {
            Serial.print("Command received: ");
            Serial.println(value.c_str());
            
            String cmd = String(value.c_str());
            cmd.trim();
            cmd.toLowerCase();
            
            // RUN command
            if (cmd == "run") {
                currentMode = MODE_FOLLOWING;
                Serial.println("Line following started");
            }
            // STOP command
            else if (cmd == "stop") {
                currentMode = MODE_STOP;
                Serial.println("Robot stopped");
            }
            // SET KP value
            else if (cmd.startsWith("set kp ")) {
                float newKp = cmd.substring(7).toFloat();
                Kp = newKp;
                Serial.printf("Kp set to %.2f\n", Kp);
            }
            // SET KI value
            else if (cmd.startsWith("set ki ")) {
                float newKi = cmd.substring(7).toFloat();
                Ki = newKi;
                errorSum = 0; // Reset integral
                Serial.printf("Ki set to %.2f\n", Ki);
            }
            // SET KD value
            else if (cmd.startsWith("set kd ")) {
                float newKd = cmd.substring(7).toFloat();
                Kd = newKd;
                Serial.printf("Kd set to %.2f\n", Kd);
            }
            // SPEED value (base speed)
            else if (cmd.startsWith("speed ")) {
                int newSpeed = cmd.substring(6).toInt();
                if (newSpeed >= 50 && newSpeed <= 255) {
                    BASE_SPEED = newSpeed;
                    Serial.printf("Speed set to %d\n", BASE_SPEED);
                } else {
                    Serial.println("Error: Speed must be between 50 and 255");
                }
            }
            // STATUS command (display parameters)
            else if (cmd == "status") {
                Serial.println("\n--- CURRENT PARAMETERS ---");
                Serial.printf("Mode: %d (0=Stop, 1=Following)\n", currentMode);
                Serial.printf("Kp: %.2f\n", Kp);
                Serial.printf("Ki: %.2f\n", Ki);
                Serial.printf("Kd: %.2f\n", Kd);
                Serial.printf("Base speed: %d\n", BASE_SPEED);
                Serial.printf("Current error: %d\n", lastError);
                Serial.println("-------------------------\n");
                
                // Send via BLE too
                char bleStatus[150];
                sprintf(bleStatus, "Mode:%d Kp:%.2f Ki:%.2f Kd:%.2f Speed:%d Err:%d", 
                        currentMode, Kp, Ki, Kd, BASE_SPEED, lastError);
                pCharStatus->setValue(bleStatus);
                pCharStatus->notify();
            }
            else {
                Serial.println("Unknown command");
            }
        }
    }
};

// =========================================================================
// MOTOR FUNCTIONS
// =========================================================================
void setMotor(int dir1_pin, int dir2_pin, int pwm_channel, int speed) {
    digitalWrite(STBY, HIGH);
    int abs_speed = constrain(abs(speed), 0, MAX_DUTY);

    if (speed > 0) {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
    } else if (speed < 0) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
    } else {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, LOW);
    }
    ledcWrite(pwm_channel, abs_speed);
}

void stopMotors() {
    digitalWrite(STBY, LOW);
    ledcWrite(motorAChannel, 0);
    ledcWrite(motorBChannel, 0);
}

// =========================================================================
// SENSOR FUNCTION
// =========================================================================
int readLine() {
    long weightedSum = 0, valueSum = 0;
    int activeSensors = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int rawValue = analogRead(sensorPins[i]);
        int value = constrain(BLACK_VALUE - rawValue, 0, 4095);
        
        if (value > 100) { 
            weightedSum += (long)value * sensorWeights[i];
            valueSum += value;
            activeSensors++;
        }
    }

    if (activeSensors < 2 || valueSum == 0) { 
        return lastError; 
    }
    return weightedSum / valueSum;
}

// =========================================================================
// SEND BLE STATUS (only on demand with "status" command)
// =========================================================================
void sendStatus() {
    // Status sent only when requested by "status" command
    // No more automatic notifications
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("   LINE FOLLOWER + BLE");
    Serial.println("========================================\n");

    // BLE INITIALIZATION
    Serial.println("Initializing BLE...");
    BLEDevice::init("LineFollower");
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Control characteristic (write)
    pCharControl = pService->createCharacteristic(
        CHAR_UUID_CONTROL,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharControl->setCallbacks(new MyCallbacks());
    
    // Status characteristic (read + notification)
    pCharStatus = pService->createCharacteristic(
        CHAR_UUID_STATUS,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharStatus->addDescriptor(new BLE2902());

    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE active");
    Serial.println("Device name: LineFollower");
    Serial.println("Service UUID: " + String(SERVICE_UUID));
    Serial.println("\nAVAILABLE COMMANDS:");
    Serial.println("  run           - Start line following");
    Serial.println("  stop          - Stop the robot");
    Serial.println("  set kp 4.5    - Change Kp");
    Serial.println("  set ki 0.01   - Change Ki");
    Serial.println("  set kd 0.8    - Change Kd");
    Serial.println("  speed 150     - Change speed (50-255)");
    Serial.println("  status        - Display parameters\n");

    // MOTOR SETUP
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    ledcSetup(motorAChannel, freq, resolution);
    ledcAttachPin(PWMA, motorAChannel);
    ledcSetup(motorBChannel, freq, resolution);
    ledcAttachPin(PWMB, motorBChannel);
    
    // SENSOR SETUP
    analogSetAttenuation(ADC_11db);
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    
    Serial.println("Hardware configured");
    Serial.print("Kp: "); Serial.print(Kp);
    Serial.print(", Ki: "); Serial.print(Ki);
    Serial.print(", Kd: "); Serial.println(Kd);
    Serial.println("\nWaiting for connection...\n");
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
    static unsigned long lastStatus = 0;
    
    // Display status every 2 seconds
    if (millis() - lastStatus > 2000) {
        if (deviceConnected) {
            Serial.print("Connected | Mode: ");
            if (currentMode == MODE_STOP) Serial.println("STOP");
            else Serial.println("FOLLOWING");
        } else {
            Serial.println("Waiting for BLE connection...");
        }
        lastStatus = millis();
    }

    // Execute according to mode
    switch(currentMode) {
        case MODE_STOP:
            stopMotors();
            break;
            
        case MODE_FOLLOWING: {
            int error = readLine();
            float P = Kp * error;
            errorSum += error;
            float I = Ki * errorSum;
            float D = Kd * (error - lastError);
            lastError = error;
            
            float correction = P + I + D;
            int leftSpeed = constrain(BASE_SPEED - correction, 0, MAX_DUTY);
            int rightSpeed = constrain(BASE_SPEED + correction, 0, MAX_DUTY);

            setMotor(AIN1, AIN2, motorAChannel, -leftSpeed);
            setMotor(BIN1, BIN2, motorBChannel, rightSpeed);
            break;
        }
    }

    sendStatus();
    delay(10);
}