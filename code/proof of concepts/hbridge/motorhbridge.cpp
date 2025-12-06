#include <Arduino.h>

// =========================================================================
// 1. DEFINITIONS DES BROCHES MOTEURS 
// =========================================================================

// Broche de veille partagée (Active HIGH)
const int STBY = 19; 

// MOTEUR A (Gauche)
const int PWMA = 23;      // Vitesse (PWM)
const int AIN1 = 21;      // Direction 1
const int AIN2 = 22;      // Direction 2
const int motorAChannel = 0; // Canal PWM 0

// MOTEUR B (Droit)
const int PWMB = 2;       // Vitesse (PWM)
const int BIN1 = 18;      // Direction 1
const int BIN2 = 16;      // Direction 2
const int motorBChannel = 1; // Canal PWM 1

// Paramètres PWM pour l'ESP32
const int freq = 30000;       // Fréquence PWM en Hz
const int resolution = 8;     // Résolution 8 bits (0-255)
const int MAX_DUTY = (1 << resolution) - 1; // 255


// -------------------------------------------------------------------------
// CONSTANTES DE CALIBRAGE DU MOUVEMENT & PID
// -------------------------------------------------------------------------
// Vitesse de base pour la calibration (changée à 50)
#define VITESSE_BASE 255

// GAINS PID : À CALIBRER SUR LA PISTE !
// On garde Kp à 4.0 pour l'oscillation.
const float Kp = 4.0;   
const float Ki = 0.00;  // Commencer à 0.00
const float Kd = 0.50;  // Commencer bas

// Variables pour le PID
int derniereErreur = 0;
float sommeErreur = 0;


// =========================================================================
// 2. DEFINITIONS DES BROCHES ET PARAMÈTRES CAPTEURS
// =========================================================================

// Pins des 6 capteurs actifs
const int NUM_SENSORS = 6;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33};

// Poids attribués aux capteurs pour le calcul du Centre de Gravité (COG)
const int sensorWeights[NUM_SENSORS] = {-200, -120, -40, 40, 120, 200};

// Seuils de calibrage des capteurs
const int VALEUR_BLANC = 200; 
const int VALEUR_NOIR  = 4000; 


// =========================================================================
// 3. FONCTIONS MOTEURS
// =========================================================================

void setMotor(int dir1_pin, int dir2_pin, int pwm_channel, int speed) {
    digitalWrite(STBY, HIGH);
    int abs_speed = abs(speed);
    if (abs_speed > MAX_DUTY) abs_speed = MAX_DUTY;

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


// =========================================================================
// 4. FONCTION CAPTEURS
// =========================================================================

/**
 * Lit les 6 capteurs analogiques et calcule la position d'erreur de la ligne (COG).
 */
int lireLigne() {
    long sommePoids = 0;
    long sommeValeurs = 0;
    int valeurs[NUM_SENSORS];
    int nbCapteursActifs = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int rawValue = analogRead(sensorPins[i]);
        
        // Inversement de la logique : Noir donne valeur élevée
        int value = VALEUR_NOIR - rawValue;
        
        if (value < 0) value = 0;
        if (value > 4095) value = 4095;
        
        valeurs[i] = value;
        
        if (value > 100) { 
            sommePoids += (long)value * sensorWeights[i];
            sommeValeurs += value;
            nbCapteursActifs++;
        }
    }

    // Si on a perdu la ligne, on garde la dernière erreur connue pour ne pas s'arrêter
    if (nbCapteursActifs < 2 || sommeValeurs == 0) { 
        return derniereErreur; 
    }

    int erreur = sommePoids / sommeValeurs;
    
    return erreur;
}


// =========================================================================
// 5. SETUP ET LOOP (CONTRÔLE PID)
// =========================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("--- DEMARRAGE ROBOT (VITESSE 50, Kp=4.0) ---");
    Serial.print("Kp: "); Serial.print(Kp);
    Serial.print(", Ki: "); Serial.print(Ki);
    Serial.print(", Kd: "); Serial.println(Kd);

    // Setup Pins Moteurs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    ledcSetup(motorAChannel, freq, resolution);
    ledcAttachPin(PWMA, motorAChannel);

    ledcSetup(motorBChannel, freq, resolution);
    ledcAttachPin(PWMB, motorBChannel);
    
    // Setup Pins Capteurs
    analogSetAttenuation(ADC_11db);
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    // 1. Lire la position d'erreur
    int erreur = lireLigne();

    // 2. Calcul du terme PID
    float P = Kp * erreur;
    
    // Terme Intégral (accumulation)
    sommeErreur += erreur;
    float I = Ki * sommeErreur;
    
    // Terme Dérivé (anticipation)
    float D = Kd * (erreur - derniereErreur);

    derniereErreur = erreur;
    
    // Calcul de la correction totale
    float correction = P + I + D; 
    
    // 3. Application de la correction aux moteurs
    int vitesseGauche = VITESSE_BASE - correction;
    int vitesseDroite = VITESSE_BASE + correction;

    // S'assurer que les vitesses restent dans la plage [0, 255]
    if (vitesseGauche > MAX_DUTY) vitesseGauche = MAX_DUTY;
    if (vitesseDroite > MAX_DUTY) vitesseDroite = MAX_DUTY;
    if (vitesseGauche < 0) vitesseGauche = 0;
    if (vitesseDroite < 0) vitesseDroite = 0;

    // 4. Commande des moteurs
    // Logique AVANCER : -GAUCHE, +DROITE
    setMotor(AIN1, AIN2, motorAChannel, -vitesseGauche);
    setMotor(BIN1, BIN2, motorBChannel, vitesseDroite);

    delay(10); 
}