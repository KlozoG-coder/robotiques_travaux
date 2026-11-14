
#include "Lab3.h"

Lab3* Lab3::instance = nullptr;

static float ecartAngle(float a, float b) {
    float e = a - b;

    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;

    return e;
}

// CONSTRUCTEUR
Lab3::Lab3()
    : ultraSensor(PORT_10),
      led(0, LEDNUM),
      gyro(0, 0x69),
      encoderRight(SLOT2),
      encoderLeft(SLOT1),
      debutRotationCalibration(0),
      angleDebutCalibration(0),
      anglePrecedentCalibration(0),
      angleCumuleCalibration(0),
      rotationCalibrationActive(false),
      rotationCalibrationReussie(false),
      anglePrecedentPivot180(0),
      angleCumulePivot180(0),
      debutPivot180(0),
      pivot180Actif(false),
      state(ATTENTE_DEPART),
      premier(true),
      termine(false),
      debutEtat(0),
      departGauche(0),
      departDroite(0),
      angleDepartSegment(0),
      ticksCibleSeg2(0),
      ticksReelsSeg2(0),
      dernierCapteur(0),
      distancePrete(false),
      vitesseCourante(0),
      anglePrecedentPivot(0),
      angleCumulePivot(0),
      debutPivot(0),
      dernierArcEnCiel(0),
      etapeArcEnCiel(0),
      derniereBarre(-1) {

    instance = this;
}

// INITIALISATION (au demarrage et a chaque lancement de lab3)

void Lab3::initialiser() {

    // Reinitialiser toute la machine a etats : un lab3 relance apres un
    // « stop » repart toujours du debut.
    state = ATTENTE_DEPART;
    premier = true;
    termine = false;

    ticksCibleSeg2 = 0;
    ticksReelsSeg2 = 0;
    distancePrete = false;
    vitesseCourante = 0;
    derniereBarre = -1;

    // Configuration du materiel
    led.setpin(LED_PIN);

    encoderConfig();

    gyro.begin();

    offMotors();
    offLeds();
}

// INTERRUPTIONS DES ENCODEURS

void Lab3::rightEncoderInterrupt() {

    if (instance == nullptr) return;

    if (digitalRead(instance->encoderRight.getPortB()) == 0) {
        instance->encoderRight.pulsePosMinus();
    }
    else {
        instance->encoderRight.pulsePosPlus();
    }
}

void Lab3::leftEncoderInterrupt() {

    if (instance == nullptr) return;

    if (digitalRead(instance->encoderLeft.getPortB()) == 0) {
        instance->encoderLeft.pulsePosMinus();
    }
    else {
        instance->encoderLeft.pulsePosPlus();
    }
}

// CONFIGURATION DES ENCODEURS

void Lab3::encoderConfig() {

    attachInterrupt(
        encoderRight.getIntNum(),
        rightEncoderInterrupt,
        RISING
    );

    attachInterrupt(
        encoderLeft.getIntNum(),
        leftEncoderInterrupt,
        RISING
    );

    encoderRight.setPulse(PULSE);
    encoderLeft.setPulse(PULSE);

    encoderRight.setRatio(RATIO);
    encoderLeft.setRatio(RATIO);

    encoderRight.setPosPid(1.8, 0, 1.2);
    encoderLeft.setPosPid(1.8, 0, 1.2);

    encoderRight.setSpeedPid(0.18, 0, 0);
    encoderLeft.setSpeedPid(0.18, 0, 0);

    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

// ARRET DES MOTEURS ET DES DEL

void Lab3::offMotors() {

    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
}

void Lab3::offLeds() {

    led.setColor(0, 0, 0);
    led.show();
}

void Lab3::arreterLab3() {
    offMotors();
    rotationCalibrationActive = false;
    pivot180Actif = false;

    state = ATTENTE_DEPART;
    premier = true;
    termine = false;

    offLeds();
}

long Lab3::calculerTicks(float distanceCm) {
    float ticksParTour = PULSE * RATIO;
    float circonference = LAB3_DIAMETRE_ROUE_CM * PI;

    return round(distanceCm / circonference * ticksParTour);
}

long Lab3::ticksGauche() {
    return abs(encoderLeft.getPulsePos() - departGauche);
}

long Lab3::ticksDroite() {
    return abs(encoderRight.getPulsePos() - departDroite);
}

long Lab3::ticksMoyens() {
    return (ticksGauche() + ticksDroite()) / 2;
}

void Lab3::debuterSegment() {
    offMotors();

    departGauche = encoderLeft.getPulsePos();
    departDroite = encoderRight.getPulsePos();

    angleDepartSegment = gyro.getAngleZ();
}

void Lab3::avancerDroit(int vitesse) {
    float angle = ecartAngle(gyro.getAngleZ(), angleDepartSegment);

    float correctionGyro = angle * LAB3_CORRECTION_GYRO;
    float correctionEncodeurs =
        (ticksGauche() - ticksDroite()) * LAB3_CORRECTION_ENCODEURS;

    int correction = correctionGyro + correctionEncodeurs;

    correction = constrain(
        correction,
        -LAB3_CORRECTION_MAX,
        LAB3_CORRECTION_MAX
    );

    int vitesseGauche = constrain(vitesse + correction, 0, 255);
    int vitesseDroite = constrain(
        vitesse - correction + LAB3_COMPENSATION_DROITE,
        0,
        255
    );

    encoderLeft.setMotorPwm(-vitesseGauche);
    encoderRight.setMotorPwm(vitesseDroite);
}

void Lab3::afficherSegment(int numeroDel) {
    for (int i = 1; i <= 7; i++) {
        led.setColor(i, 0, 0, 0);
    }

    led.setColor(numeroDel, 0, 255, 0);
    led.show();

    derniereBarre = -1;  
}

// Barre de progression sur les DEL 1 a 7 
void Lab3::afficherProgression(long courant, long cible) {
    int nb = 0;

    if (cible > 0) {
        nb = courant * 7L / cible;
    }

    nb = constrain(nb, 0, 7);

    if (nb == derniereBarre) {
        return;
    }

    derniereBarre = nb;

    for (int i = 1; i <= 7; i++) {
        if (i <= nb) {
            led.setColor(i, 0, 255, 0);
        } else {
            led.setColor(i, 0, 0, 0);
        }
    }

    led.show();
}

// Bonus 
void Lab3::afficherArcEnCiel(unsigned long ct) {
    if (ct - dernierArcEnCiel < 70) return;

    dernierArcEnCiel = ct;
    etapeArcEnCiel = (etapeArcEnCiel + 1) % 12;

    const byte couleurs[6][3] = {
        {255, 0, 0}, {255, 100, 0}, {255, 255, 0},
        {0, 255, 0}, {0, 0, 255}, {150, 0, 255}
    };

    for (int i = 1; i <= 12; i++) {
        int indice = ((i - 1 + etapeArcEnCiel) / 2) % 6;
        led.setColor(i, couleurs[indice][0],
                        couleurs[indice][1],
                        couleurs[indice][2]);
    }

    led.show();
}

// ETATS DU PARCOURS 

void Lab3::changerEtatLab3(State nouvelEtat) {
    offMotors();
    state = nouvelEtat;
    premier = true;
}

// Attente de 3 secondes avant le depart
void Lab3::etatAttenteDepart(unsigned long ct) {
    if (premier) {
        offMotors();
        offLeds();

        debutEtat = ct;
        premier = false;
        return;
    }

    if (ct - debutEtat >= LAB3_DELAI_DEPART_MS) {
        changerEtatLab3(SEGMENT1);
    }
}

// Segments
void Lab3::etatSegmentDistance(int numeroDel, const char* nom,
                               State suivant, unsigned long ct) {
    if (premier) {
        debuterSegment();
        afficherSegment(numeroDel);

        Serial.println(nom);

        premier = false;
        return;
    }

    long courant = ticksMoyens();
    long cible = calculerTicks(LAB3_DISTANCE_CIBLE_CM);

    afficherProgression(courant, cible);

    if (courant >= cible) {
        changerEtatLab3(suivant);
        return;
    }

    avancerDroit(LAB3_VITESSE_MOTEUR);
}

// Segment 2
void Lab3::etatSegment2(unsigned long ct) {
    if (premier) {
        debuterSegment();
        afficherSegment(11);

        distancePrete = false;
        vitesseCourante = 0;
        ticksCibleSeg2 = 0;

        dernierCapteur = ct;
        premier = false;

        Serial.println("Segment 2");
        return;
    }

    if (ct - dernierCapteur >= LAB3_PERIODE_CAPTEUR_MS) {
        dernierCapteur = ct;

        float distance = ultraSensor.distanceCm();

#if COMMANDES_DEBUG
        Serial.print("Distance : ");
        Serial.println(distance);
#endif

        if (distance <= 0 || distance >= 400) {
            vitesseCourante = 0;
        } else {
            if (!distancePrete) {
                ticksCibleSeg2 =
                    calculerTicks(distance - LAB3_DISTANCE_ARRET_CM);
                distancePrete = true;
            }

            if (distance <= LAB3_DISTANCE_ARRET_CM) {
                ticksReelsSeg2 = ticksMoyens();
                afficherProgression(1, 1);

                changerEtatLab3(PIVOT_DROITE);
                return;
            }

            if (distance <= LAB3_DISTANCE_RALENTIR_CM) {
                vitesseCourante = LAB3_VITESSE_LENTE;
            } else {
                vitesseCourante = LAB3_VITESSE_MOTEUR;
            }
        }
    }

    afficherProgression(ticksMoyens(), ticksCibleSeg2);

    if (vitesseCourante == 0) {
        offMotors();
    } else {
        avancerDroit(vitesseCourante);
    }
}

// Pivot commun 
void Lab3::executerPivot(int direction, float angleCible, State suivant,
                         bool arcEnCiel, unsigned long ct) {
    if (arcEnCiel) {
        afficherArcEnCiel(ct);
    }

    if (premier) {
        offMotors();

        anglePrecedentPivot = gyro.getAngleZ();
        angleCumulePivot = 0.0f;
        debutPivot = ct;

        Serial.print("Pivot de ");
        Serial.println(angleCible);

        premier = false;
        return;
    }

    // Cumul des variations d'angle 
    float angleActuel = gyro.getAngleZ();
    angleCumulePivot += ecartAngle(angleActuel, anglePrecedentPivot);
    anglePrecedentPivot = angleActuel;

    float angle = fabs(angleCumulePivot);

    // Pivot termine
    if (angle >= angleCible) {
        Serial.println("Pivot termine");

        changerEtatLab3(suivant);
        return;
    }

    // Empecher la rotation infinie
    if (ct - debutPivot >= LAB3_TEMPS_MAX_PIVOT_MS) {
        Serial.println("ERREUR : pivot trop long");

        changerEtatLab3(ERREUR_PIVOT);
        return;
    }

    int vitesse = LAB3_VITESSE_PIVOT;

    if (angle >= angleCible - 20.0f) {
        vitesse = LAB3_VITESSE_PIVOT_LENT;
    }

    encoderLeft.setMotorPwm(direction * vitesse);
    encoderRight.setMotorPwm(direction * vitesse);
}

// Bonus : segments du retour (arc-en-ciel)
void Lab3::etatRetour(long ticksCible, State suivant, unsigned long ct) {
    if (premier) {
        debuterSegment();
        dernierArcEnCiel = ct - 70;

        premier = false;
        return;
    }

    afficherArcEnCiel(ct);

    if (ticksMoyens() >= ticksCible) {
        changerEtatLab3(suivant);
        return;
    }

    avancerDroit(LAB3_VITESSE_MOTEUR);
}

// Fin du parcours : arc-en-ciel quelques secondes, puis lab3 est « done »
void Lab3::etatFin(unsigned long ct) {
    if (premier) {
        offMotors();

        debutEtat = ct;
        dernierArcEnCiel = ct - 70;
        premier = false;

        Serial.println("Mission terminee");
    }

    afficherArcEnCiel(ct);

    if (ct - debutEtat >= LAB3_DUREE_FIN_MS) {
        termine = true;
    }
}

// MACHINE A ETATS INTERNE DU LAB3

void Lab3::gererMachine(unsigned long ct) {

    switch (state) {

        case ATTENTE_DEPART:
            etatAttenteDepart(ct);
            break;

        case SEGMENT1:
            etatSegmentDistance(12, "Segment 1", PIVOT_GAUCHE, ct);
            break;

        case PIVOT_GAUCHE:
            executerPivot(-1, 90.0f, SEGMENT2, false, ct);
            break;

        case SEGMENT2:
            etatSegment2(ct);
            break;

        case PIVOT_DROITE:
            executerPivot(1, 90.0f, SEGMENT3, false, ct);
            break;

        case SEGMENT3:
            etatSegmentDistance(10, "Segment 3", PIVOT_180, ct);
            break;

        case PIVOT_180:
            executerPivot(1, 180.0f, RETOUR_SEG3, false, ct);
            break;

        case RETOUR_SEG3:
            etatRetour(calculerTicks(LAB3_DISTANCE_CIBLE_CM),
                       RETOUR_PIVOT_GAUCHE, ct);
            break;

        case RETOUR_PIVOT_GAUCHE:
            executerPivot(-1, 90.0f, RETOUR_SEG2, true, ct);
            break;

        case RETOUR_SEG2:
            etatRetour(ticksReelsSeg2, RETOUR_PIVOT_DROITE, ct);
            break;

        case RETOUR_PIVOT_DROITE:
            executerPivot(1, 90.0f, RETOUR_SEG1, true, ct);
            break;

        case RETOUR_SEG1:
            etatRetour(calculerTicks(LAB3_DISTANCE_CIBLE_CM), FIN, ct);
            break;

        case FIN:
            etatFin(ct);
            break;

        case ERREUR_PIVOT:
            offMotors();
            break;
    }
}

void Lab3::actualiser() {

    unsigned long ct = millis();

    gyro.update();

    gererMachine(ct);
}

// Vrai quand le parcours est fini
bool Lab3::estTermine() {

    return termine || state == ERREUR_PIVOT;
}

// Reglages temporaires des polarites de marche.
static int sensMoteurGauche = -1;
static int sensMoteurDroit = 1;
void inverserSensMoteurGauche() { sensMoteurGauche *= -1; }
void inverserSensMoteurDroit() { sensMoteurDroit *= -1; }

void Lab3::commanderMoteursLigne(int vitesseGauche, int vitesseDroite) {
    encoderLeft.loop();
    encoderRight.loop();
    encoderLeft.setMotorPwm(sensMoteurGauche * constrain(vitesseGauche, 0, 100));
    encoderRight.setMotorPwm(sensMoteurDroit * constrain(vitesseDroite, 0, 100));
}

void Lab3::arreterMoteursLigne() {
    offMotors();
}

void Lab3::commencerPivot180() {
    encoderConfig();

    gyro.resetData();
    gyro.update();

    anglePrecedentPivot180 = gyro.getAngleZ();
    angleCumulePivot180 = 0.0f;
    debutPivot180 = millis();
    pivot180Actif = true;

    offMotors();

    Serial.println("Pivot 180 demarre");
}

bool Lab3::actualiserPivot180() {
    if (!pivot180Actif) {
        return true;
    }

    gyro.update();
    encoderLeft.loop();
    encoderRight.loop();

    // Cumul des variations d'angle.
    float angleActuel = gyro.getAngleZ();
    angleCumulePivot180 += ecartAngle(angleActuel, anglePrecedentPivot180);
    anglePrecedentPivot180 = angleActuel;

    if (fabs(angleCumulePivot180) >= ANGLE_DEMI_TOUR) {
        arreterPivot180();

        Serial.println("Pivot 180 termine");
        return true;
    }

    if (millis() - debutPivot180 >= LAB3_TEMPS_MAX_PIVOT_MS) {
        arreterPivot180();

        Serial.println("ERREUR : demi-tour trop long");
        return true;
    }

    // Rotation sur place
    encoderLeft.setMotorPwm(-60);
    encoderRight.setMotorPwm(-60);

    return false;
}

void Lab3::arreterPivot180() {
    offMotors();
    pivot180Actif = false;
}

void Lab3::afficherCouleurEtat(int rouge, int vert, int bleu) {
    for (int i = 1; i <= LEDNUM; i++) {
        led.setColor(i, rouge, vert, bleu);
    }

    led.show();
}

void Lab3::eteindreLedsEtat() {
    afficherCouleurEtat(0, 0, 0);
}

bool Lab3::calibrationRotationReussie() {
    return rotationCalibrationReussie;
}

void Lab3::initialiserLedsEtat() {
    led.setpin(LED_PIN);
    led.setNumber(LEDNUM);
    led.setColor(0, 0, 0, 0);
    led.show();

    Serial.println("DEL initialisees");
}
static Lab3 instanceLab3;

void initialiserLedsEtat() {
    instanceLab3.initialiserLedsEtat();
}

bool rotationCalibrationReussie() {
    return instanceLab3.calibrationRotationReussie();
}

void afficherCouleurEtat(int rouge, int vert, int bleu) {
    instanceLab3.afficherCouleurEtat(rouge, vert, bleu);
}

void eteindreLedsEtat() {
    instanceLab3.eteindreLedsEtat();
}

void arreterLab3() {
    instanceLab3.arreterLab3();
}

void initialiserMoteursLab3() {
    instanceLab3.initialiser();
}

void initialiserLab3() {
    instanceLab3.initialiser();
}

void actualiserLab3() {
    instanceLab3.actualiser();
}

bool lab3EstTermine() {
    return instanceLab3.estTermine();
}

void Lab3::commencerRotationCalibration() {
    rotationCalibrationReussie = false;

    encoderConfig();
    gyro.begin();
    gyro.update();

    angleDebutCalibration = gyro.getAngleZ();
    anglePrecedentCalibration = angleDebutCalibration;
    angleCumuleCalibration = 0;

    debutRotationCalibration = millis();
    rotationCalibrationActive = false;

    if (isnan(angleDebutCalibration) || isinf(angleDebutCalibration)) {
        Serial.println("ERREUR : gyroscope invalide");
        offMotors();
        return;
    }

    rotationCalibrationActive = true;

    Serial.println("Rotation calibration demarree");
}


bool Lab3::actualiserRotationCalibration() {
    if (!rotationCalibrationActive) {
        return true;
    }

    gyro.update();
    encoderRight.loop();
    encoderLeft.loop();

    float angleActuel = gyro.getAngleZ();

    if (isnan(angleActuel) || isinf(angleActuel)) {
    Serial.println("ERREUR : lecture gyroscope invalide");
    arreterRotationCalibration();
    return true;
    }
    float difference = angleActuel - anglePrecedentCalibration;

    if (difference > 180.0f) {
        difference -= 360.0f;
    } else if (difference < -180.0f) {
        difference += 360.0f;
    }

    angleCumuleCalibration += difference;
    anglePrecedentCalibration = angleActuel;


    // Arrêt après 360 degrés
   if (fabs(angleCumuleCalibration) >= 360.0f) {
    rotationCalibrationReussie = true;

    Serial.println("Rotation 360 terminee");
    arreterRotationCalibration();
    return true;
    }

    if (millis() - debutRotationCalibration >= 10000UL) {
    rotationCalibrationReussie = false;

    Serial.println("ERREUR : rotation trop longue");
    arreterRotationCalibration();
    return true;
    }

    encoderLeft.setMotorPwm(-60);
    encoderRight.setMotorPwm(-60);

    return false;
}


void Lab3::arreterRotationCalibration() {
    offMotors();
    rotationCalibrationActive = false;
}

void commencerRotationCalibration() {
    instanceLab3.commencerRotationCalibration();
}

bool actualiserRotationCalibration() {
    return instanceLab3.actualiserRotationCalibration();
}

void arreterRotationCalibration() {
    instanceLab3.arreterRotationCalibration();
}

void commanderMoteursLigne(int vitesseGauche, int vitesseDroite) {
    instanceLab3.commanderMoteursLigne(vitesseGauche, vitesseDroite);
}

void arreterMoteursLigne() {
    instanceLab3.arreterMoteursLigne();
}

void commencerPivot180() {
    instanceLab3.commencerPivot180();
}

bool actualiserPivot180() {
    return instanceLab3.actualiserPivot180();
}

void arreterPivot180() {
    instanceLab3.arreterPivot180();
}