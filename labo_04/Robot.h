#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Configuration.h"
#include "Calibration.h"
#include "SuiviLigne.h"

void initialiserMoteursLab3();
void initialiserLab3();
void actualiserLab3();
bool lab3EstTermine();
void arreterLab3();

void commencerRotationCalibration();
bool actualiserRotationCalibration();
void arreterRotationCalibration();
bool rotationCalibrationReussie();

void commanderMoteursLigne(int vitesseGauche, int vitesseDroite);
void inverserSensMoteurGauche();
void inverserSensMoteurDroit();
void arreterMoteursLigne();
void commencerPivot180();
bool actualiserPivot180();
void arreterPivot180();

void afficherCouleurEtat(int rouge, int vert, int bleu);
void eteindreLedsEtat();
void initialiserLedsEtat();

enum EtatRobot {
    BOOT,
    ATTENTE,
    LAB3,
    CALIB,
    SUIVRE_LIGNE,
    CMD_INVALIDE,
    NON_CALIB
};

class Robot {
private:
    EtatRobot etatActuel;
    EtatRobot etatPrecedent;
    unsigned long tempsDebut;
    void changerEtat(EtatRobot nouvelEtat);

    Calibration calibration;
    bool calibrationEnCours;
    bool estCalibre;
    SuiviLigne suiviLigne;
    unsigned long dernierAffichageLigne;

    enum PhaseSuivi { AVANCER_LIGNE, PIVOTER_180 };
    PhaseSuivi phaseSuivi;
    unsigned long debutPerteLigne;
    bool ligneDejaVue = false;
    float dernierePosition = 0.0f;          
    float derniereCorrectionValide = 0.0f; 
    bool correctionInverse = false;
    bool testAvanceActif = false;
    unsigned long debutTestAvance = 0;
    unsigned long dernierClignotement;
    bool ledsAllumees;

    void actualiserClignotement(int rouge, int vert, int bleu);
    String commandeRecue;
    void lireCommandes();
    void lirePort(Stream &port, uint8_t *tampon, uint8_t &longueur,
                  unsigned long &dernierOctet);
    void traiterCommande(String commande);

    void executerBoot();
    void executerAttente();
    void executerLab3();
    void executerCalibration();
    void executerSuiviLigne();
    void executerCmdInvalide();
    void executerNonCalib();

public:
    Robot();
    void initialiser();
    void actualiser();
};

#endif
