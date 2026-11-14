
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define BAUD_RATE 115200

#define COMMANDES_DEBUG 0

#define BLE_SUR_SERIAL2 0
#define BLE_BAUD_RATE 115200

#define LEDNUM 12
#define LED_PIN 44

#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

const int VITESSE = 100;
const int VITESSE_PIVOT = 50;

// 4. PARAMETRES DES MOUVEMENTS
const float TOLERANCE = 2.0;
const float ANGLE_CALIBRATION = 360.0;
const float ANGLE_DEMI_TOUR = 180.0;

// 5. TEMPS DES ETATS 
const unsigned long TEMPS_BOOT = 3000;
const unsigned long TEMPS_AVERTISSEMENT = 3000;

// 6. PARAMETRES DU PID
const float KP = 6.5;
const float KI = 0.0;
const float KD = 1.1;

// 7. SUIVI DE LIGNE
const unsigned long PERIODE_PID_MS       = 10;   
const int   SUIVI_VITESSE_BASE           = 60;  
const float SUIVI_CORRECTION_MAX         = 32.0f;
const int   SUIVI_VITESSE_MAX            = 95;
const int   SUIVI_VITESSE_RECHERCHE      = 30;   
const float SUIVI_SEUIL_COTE             = 1200.0f; 
const unsigned long SUIVI_CONFIRM_FIN_MS = 120;  
const unsigned long SUIVI_PERTE_COTE_MS  = 1500; 
const unsigned long SUIVI_RECHERCHE_MS   = 3000; 

// 8. LABO 03 

const float LAB3_DISTANCE_CIBLE_CM        = 90.0f;  
const float LAB3_DISTANCE_ARRET_CM        = 30.0f; 
const float LAB3_DISTANCE_RALENTIR_CM     = 50.0f;
const float LAB3_DIAMETRE_ROUE_CM         = 6.5f;

const int LAB3_VITESSE_MOTEUR             = 100;
const int LAB3_VITESSE_LENTE              = 65;
const int LAB3_VITESSE_PIVOT              = 70;
const int LAB3_VITESSE_PIVOT_LENT         = 50;

const float LAB3_CORRECTION_GYRO          = 1.5f;
const float LAB3_CORRECTION_ENCODEURS     = 0.15f;
const int   LAB3_CORRECTION_MAX           = 35;
const int   LAB3_COMPENSATION_DROITE      = 3;

const unsigned long LAB3_DELAI_DEPART_MS    = 3000; 
const unsigned long LAB3_PERIODE_CAPTEUR_MS = 50;
const unsigned long LAB3_TEMPS_MAX_PIVOT_MS = 6000;
const unsigned long LAB3_DUREE_FIN_MS       = 3000;  

// 9. COULEURS
#define COULEUR_ROUGE 1
#define COULEUR_JAUNE 2
#define COULEUR_VERT 3

#endif
