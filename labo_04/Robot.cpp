
#include "Robot.h"

// Constructeur

Robot::Robot()
    : etatActuel(ATTENTE),
      etatPrecedent(ATTENTE),
      tempsDebut(0),
      calibrationEnCours(false),
      estCalibre(false),
      dernierAffichageLigne(0),
      phaseSuivi(AVANCER_LIGNE),
      debutPerteLigne(0),
      dernierClignotement(0),
      ledsAllumees(false) {
}

void Robot::initialiser() {
    Serial.begin(BAUD_RATE);
#if BLE_SUR_SERIAL2
    Serial2.begin(BLE_BAUD_RATE);
#endif

    initialiserMoteursLab3();
    initialiserLedsEtat();
    eteindreLedsEtat();

    calibration.initialiser();
    suiviLigne.initialiser(&calibration);

    dernierAffichageLigne = 0;

    changerEtat(BOOT);
}

void Robot::actualiser() {
    lireCommandes();
    if (testAvanceActif && millis() - debutTestAvance >= 2000UL) {
        testAvanceActif = false;
        arreterMoteursLigne();
        Serial.println("TEST AVANCE termine");
    }
    if (testAvanceActif) {
        commanderMoteursLigne(45, 45);
        return;
    }
    switch (etatActuel) {
        case BOOT:
            executerBoot();
            break;

        case ATTENTE:
            executerAttente();
            break;

        case LAB3:
            executerLab3();
            break;

        case CALIB:
            executerCalibration();
            break;

        case SUIVRE_LIGNE:
            executerSuiviLigne();
            break;

        case CMD_INVALIDE:
            executerCmdInvalide();
            break;

        case NON_CALIB:
            executerNonCalib();
            break;
    }
}

void Robot::changerEtat(EtatRobot nouvelEtat) {

    if (etatActuel == nouvelEtat && nouvelEtat != BOOT) {
        return;
    }

    if (etatActuel == LAB3) {
        arreterLab3();
    }

    if (etatActuel == SUIVRE_LIGNE) {
        if (phaseSuivi == PIVOTER_180) {
            arreterPivot180();
        }

        arreterMoteursLigne();
    }

    if (etatActuel == CALIB) {
        arreterRotationCalibration();
        calibrationEnCours = false;
    }

    etatPrecedent = etatActuel;
    etatActuel = nouvelEtat;
    tempsDebut = millis();

    switch (etatActuel) {
        case BOOT:
            Serial.println("État : BOOT");
            eteindreLedsEtat();
            break;

        case ATTENTE:
            Serial.println("État : ATTENTE");
            arreterMoteursLigne();
            eteindreLedsEtat();
            break;

        case LAB3:
            Serial.println("État : LAB3");
            eteindreLedsEtat();
            initialiserLab3();
            break;

        case CALIB:
            Serial.println("État : CALIB");
            eteindreLedsEtat();

            calibrationEnCours = calibration.estConnecte();

            if (calibrationEnCours) {
                calibration.commencer();
                estCalibre = false;
                commencerRotationCalibration();
            } else {
                Serial.println(
                    "Calibration impossible : LyneTracker absent"
                );
            }
            break;

        case SUIVRE_LIGNE:
            Serial.println("État : SUIVRE_LIGNE");
            eteindreLedsEtat();

            suiviLigne.commencer();

            phaseSuivi = AVANCER_LIGNE;
            debutPerteLigne = 0;
            ligneDejaVue = false;
            dernierePosition = 0.0f;
            derniereCorrectionValide = 0.0f;
            dernierAffichageLigne = 0;
            break;

        case CMD_INVALIDE:
            Serial.println("État : CMD_INVALIDE");

            arreterMoteursLigne();

            dernierClignotement = millis() - 250UL;
            ledsAllumees = false;
            eteindreLedsEtat();
            break;

        case NON_CALIB:
            Serial.println("État : NON_CALIB");

            arreterMoteursLigne();

            dernierClignotement = millis() - 250UL;
            ledsAllumees = false;
            eteindreLedsEtat();
            break;
    }
}

void Robot::executerBoot() {
    if (millis() - tempsDebut >= TEMPS_BOOT) {
        changerEtat(ATTENTE);
    }
}

void Robot::executerAttente() {
    // rester en ATTENTE.
}

void Robot::executerLab3() {
    actualiserLab3();

    if (lab3EstTermine()) {
        changerEtat(ATTENTE);
    }
}

void Robot::executerCalibration() {
    if (!calibrationEnCours) {
        changerEtat(ATTENTE);
        return;
    }

    calibration.calibrer();

    if (actualiserRotationCalibration()) {
        calibrationEnCours = false;

        if (rotationCalibrationReussie()) {
            calibration.calibrer();
            calibration.terminer();

            estCalibre = calibration.estCalibre();

            Serial.println("Calibration reussie !");
        } else {
            estCalibre = false;

            Serial.println("ERREUR : calibration echouee");
        }

        changerEtat(ATTENTE);
    }
}

void Robot::executerSuiviLigne() {
    static unsigned long dernierCalcul = 0;

    if (phaseSuivi == PIVOTER_180) {
        if (actualiserPivot180()) {
            phaseSuivi = AVANCER_LIGNE;
            debutPerteLigne = 0;

            ligneDejaVue = false;
            dernierePosition = 0.0f;
            derniereCorrectionValide = 0.0f;
            suiviLigne.commencer();

            Serial.println("Reprise du suivi de ligne");
        }

        return;
    }

    if (millis() - dernierCalcul < PERIODE_PID_MS) {
        return;
    }

    dernierCalcul = millis();

    suiviLigne.actualiser();

    if (!suiviLigne.estLigneDetectee()) {
        if (debutPerteLigne == 0) {
            debutPerteLigne = millis();

            dernierePosition = suiviLigne.getPosition();
        }

        const unsigned long dureePerte = millis() - debutPerteLigne;

        if (!ligneDejaVue) {
            if (dureePerte >= SUIVI_RECHERCHE_MS) {
                Serial.println("Ligne introuvable");
                changerEtat(ATTENTE);
            } else {
                commanderMoteursLigne(SUIVI_VITESSE_RECHERCHE, SUIVI_VITESSE_RECHERCHE);
            }
            return;
        }

        if (fabs(dernierePosition) > SUIVI_SEUIL_COTE && dureePerte < SUIVI_PERTE_COTE_MS) {
            float aide = (derniereCorrectionValide >= 0.0f)
                             ? SUIVI_CORRECTION_MAX
                             : -SUIVI_CORRECTION_MAX;
            commanderMoteursLigne(
                constrain((int)(SUIVI_VITESSE_RECHERCHE + aide), 0, 90),
                constrain((int)(SUIVI_VITESSE_RECHERCHE - aide), 0, 90));
            return;
        }

        if (fabs(dernierePosition) <= SUIVI_SEUIL_COTE &&
            dureePerte < SUIVI_CONFIRM_FIN_MS) {
            commanderMoteursLigne(SUIVI_VITESSE_BASE, SUIVI_VITESSE_BASE);
            return;
        }

        Serial.println("Fin de ligne detectee");
        arreterMoteursLigne();
        phaseSuivi = PIVOTER_180;
        commencerPivot180();
        return;
    }

    debutPerteLigne = 0;
    ligneDejaVue = true;

    float correction = constrain(-suiviLigne.getCorrection(),
                                 -SUIVI_CORRECTION_MAX,
                                  SUIVI_CORRECTION_MAX);
    if (correctionInverse) correction = -correction;
    derniereCorrectionValide = correction;

    int vitesseGauche = constrain(
        (int)(SUIVI_VITESSE_BASE + correction), 0, SUIVI_VITESSE_MAX);
    int vitesseDroite = constrain(
        (int)(SUIVI_VITESSE_BASE - correction), 0, SUIVI_VITESSE_MAX);

    commanderMoteursLigne(vitesseGauche, vitesseDroite);
}

void Robot::executerCmdInvalide() {
    actualiserClignotement(255, 0, 0);

    if (millis() - tempsDebut >= TEMPS_AVERTISSEMENT) {
        eteindreLedsEtat();
        changerEtat(ATTENTE);
    }
}

void Robot::executerNonCalib() {
    actualiserClignotement(255, 255, 0);

    if (millis() - tempsDebut >= TEMPS_AVERTISSEMENT) {
        eteindreLedsEtat();
        changerEtat(ATTENTE);
    }
}

void Robot::actualiserClignotement(
    int rouge,
    int vert,
    int bleu
) {
    if (millis() - dernierClignotement >= 250UL) {
        dernierClignotement = millis();
        ledsAllumees = !ledsAllumees;

        if (ledsAllumees) {
            afficherCouleurEtat(rouge, vert, bleu);
        } else {
            eteindreLedsEtat();
        }
    }
}

void Robot::lirePort(Stream &port, uint8_t *tampon, uint8_t &longueur,
                     unsigned long &dernierOctet) {
    if (longueur && millis() - dernierOctet > 1000UL) longueur = 0;
    while (port.available() > 0) {
        uint8_t octet = (uint8_t)port.read();
        dernierOctet = millis();
        if (octet == '\r' || octet == '\n') {
            if (!longueur) continue;
            uint8_t debut = 0, fin = longueur;
            if (longueur >= 4 && tampon[0] == 0xFF && tampon[1] == 0x55) {
                uint8_t crc = 0;
                for (uint8_t i = 0; i < longueur - 1; ++i) crc ^= tampon[i];
                if (crc != tampon[longueur - 1]) {
                    Serial.println("Trame ignoree : CRC incorrect");
                    longueur = 0;
                    continue;
                }
                debut = 2;
                fin = longueur - 1;
            }
            String commande;
            for (uint8_t i = debut; i < fin; ++i) commande += (char)tampon[i];
            longueur = 0;
            commande.trim();
            if (commande.length()) traiterCommande(commande);
        } else if (longueur < 48) {
            tampon[longueur++] = octet;
        } else {
            longueur = 0;
            Serial.println("Commande trop longue");
        }
    }
}

void Robot::lireCommandes() {
    static uint8_t tamponUSB[48];
    static uint8_t longueurUSB = 0;
    static unsigned long dernierUSB = 0;
    lirePort(Serial, tamponUSB, longueurUSB, dernierUSB);
#if BLE_SUR_SERIAL2
    static uint8_t tamponBLE[48];
    static uint8_t longueurBLE = 0;
    static unsigned long dernierBLE = 0;
    lirePort(Serial2, tamponBLE, longueurBLE, dernierBLE);
#endif
}

void Robot::traiterCommande(String commande) {
    commande.trim();
    commande.toLowerCase();

    if (commande.length() == 0) {
        return;
    }

    Serial.print("cmd reçu : ");
    Serial.println(commande);

    if (false) {
    
    }
#if COMMANDES_DEBUG
    else if (commande == "sensg") {
        testAvanceActif = false;
        changerEtat(ATTENTE);
        inverserSensMoteurGauche();
        Serial.println("Sens moteur gauche inverse. Testez avance.");
    }
    else if (commande == "sensd") {
        testAvanceActif = false;
        changerEtat(ATTENTE);
        inverserSensMoteurDroit();
        Serial.println("Sens moteur droit inverse. Testez avance.");
    }
    else if (commande == "pidinv") {
        testAvanceActif = false;
        changerEtat(ATTENTE);
        correctionInverse = !correctionInverse;
        Serial.println("Sens de correction PID inverse. Refaire ligne.");
    }
    else if (commande == "noirinv") {
        testAvanceActif = false;
        changerEtat(ATTENTE);
        suiviLigne.inverserNoirBlanc();
        Serial.println("Polarite noir/blanc inversee. Refaire ligne.");
    }
    else if (commande == "diag") {
        changerEtat(ATTENTE);
        calibration.lireCapteurs();
        calibration.afficherCapteurs();
        calibration.afficherCalibration();
        Serial.println("Diagnostic ponctuel termine.");
    }
    else if (commande == "avance") {
        changerEtat(ATTENTE);
        testAvanceActif = true;
        debutTestAvance = millis();
        Serial.println("TEST AVANCE : 2 secondes");
    }
#endif
    else if (commande == "stop" || commande == "arreter" || commande == "s") {
        testAvanceActif = false;
        arreterMoteursLigne();
        changerEtat(ATTENTE);
    }
    else if (commande == "lab3") {
        changerEtat(LAB3);
    }
    else if (commande == "cal" || commande == "calibrer") {
        changerEtat(CALIB);
    }
    else if (commande == "ligne" || commande == "suivre_ligne") {
        if (estCalibre) {
            changerEtat(SUIVRE_LIGNE);
        } else {
            changerEtat(NON_CALIB);
        }
    }
    else {
        Serial.println("cmd INCONNUE");
        changerEtat(CMD_INVALIDE);
    }
}
