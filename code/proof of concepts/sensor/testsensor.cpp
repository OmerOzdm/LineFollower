#include <Arduino.h>

const int NUM_SENSORS = 6;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33};
const int VALEUR_NOIR = 4000; 
const int sensorWeights[NUM_SENSORS] = {-200, -120, -40, 40, 120, 200};

int readLineAndDisplay() {
    long sumWeights = 0;
    long sumValues = 0;
    int activeSensors = 0;

    Serial.print("Raw(0-4095): ");
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int rawValue = analogRead(sensorPins[i]);
        
        Serial.printf("A%d:%4d ", i, rawValue); 

        int value = VALEUR_NOIR - rawValue;
        
        if (value < 0) value = 0;
        if (value > 4095) value = 4095;
        
        if (value > 100) { 
            sumWeights += (long)value * sensorWeights[i];
            sumValues += value;
            activeSensors++;
        }
    }
    
    int error = 0;
    if (sumValues > 0) { 
        error = sumWeights / sumValues;
    } else {
        error = 0; 
    }

    Serial.print(" | COG Error: ");
    Serial.println(error);
    
    return error;
}

void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db); 
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    readLineAndDisplay();
    delay(200); 
}