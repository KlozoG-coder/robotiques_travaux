#ifndef LAB3_H
#define LAB3_H

#include <Arduino.h>
#include <MeAuriga.h>
#include <math.h>
#include "Configuration.h"

class Lab3 {
private:
    MeUltrasonicSensor ultraSensor;
    MeRGBLed led;
    MeGyro gyro;
    MeEncoderOnBoard encoderRight;
    MeEncoderOnBoard encoderLeft;

    static Lab3* instance;
    static void rightEncoderInterrupt();
    static void leftEncoderInterrupt();

    unsigned long debutRotationCalibration;
    float angleDebutCalibration;
    float anglePrecedentCalibration;
    float angleCumuleCalibration;
    bool rotationCalibrationActive;
    bool rotationCalibrationReussie;

    float anglePrecedentPivot180;
    float angleCumulePivot180;
    unsigned long debutPivot180;
    bool pivot180Actif;

    //  Machine a etats 
    enum State {
        ATTENTE_DEPART,
        SEGMENT1,
        PIVOT_GAUCHE,
        SEGMENT2,
        PIVOT_DROITE,
        SEGMENT3,
        PIVOT_180,
        RETOUR_SEG3,
        RETOUR_PIVOT_GAUCHE,
        RETOUR_SEG2,
        RETOUR_PIVOT_DROITE,
        RETOUR_SEG1,
        FIN,
        ERREUR_PIVOT
    };

    State state;
    bool premier;      
    bool termine;    
    unsigned long debutEtat;

    long departGauche;
    long departDroite;
    float angleDepartSegment;

    long ticksCibleSeg2;
    long ticksReelsSeg2;
    unsigned long dernierCapteur;
    bool distancePrete;
    int vitesseCourante;

    float anglePrecedentPivot;
    float angleCumulePivot;
    unsigned long debutPivot;

    unsigned long dernierArcEnCiel;
    int etapeArcEnCiel;
    int derniereBarre;

    void encoderConfig();
    void offMotors();
    void offLeds();

    long calculerTicks(float distanceCm);
    long ticksGauche();
    long ticksDroite();
    long ticksMoyens();
    void debuterSegment();
    void avancerDroit(int vitesse);

    void afficherSegment(int numeroDel);
    void afficherProgression(long courant, long cible);
    void afficherArcEnCiel(unsigned long ct);

    void changerEtatLab3(State nouvelEtat);
    void etatAttenteDepart(unsigned long ct);
    void etatSegmentDistance(int numeroDel, const char* nom,
                             State suivant, unsigned long ct);
    void etatSegment2(unsigned long ct);
    void executerPivot(int direction, float angleCible, State suivant,
                       bool arcEnCiel, unsigned long ct);
    void etatRetour(long ticksCible, State suivant, unsigned long ct);
    void etatFin(unsigned long ct);
    void gererMachine(unsigned long ct);

public:
    Lab3();
    void initialiser();
    void actualiser();
    bool estTermine();
    void arreterLab3();

    void commencerRotationCalibration();
    bool actualiserRotationCalibration();
    void arreterRotationCalibration();
    bool calibrationRotationReussie();

    void commanderMoteursLigne(int vitesseGauche, int vitesseDroite);
    void arreterMoteursLigne();
    void commencerPivot180();
    bool actualiserPivot180();
    void arreterPivot180();

    void afficherCouleurEtat(int rouge, int vert, int bleu);
    void eteindreLedsEtat();
    void initialiserLedsEtat();
};

#endif
