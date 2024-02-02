/*----------------------APPEL DES LIBRAIRIES------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include <LiquidCrystal.h>                                                  //Appel de la librairie pour l'ecran LCD.
#include <LcdBarGraph.h>                                                    //Appel de la librairie pour afficher une bargraph sur l'ecran.
#include <Wire.h>                                                           //Appel de la librairie pour communication I2C.
#include "DetecMet.h"                                                       //Appel de la librairie pour le controle de l'alimenation, de la configuration des registres et de l'affichage de données liées au projet.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------DECLARATION DES PINS UTILISEES POUR LE CONTROLE DE L'ALIMENTATION-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
alimentation arduinoNano(8,A1);                                             //Les valeurs en argument de l'appel de classe sont, dans l'ordre : la pin de maintien de l'alimentation et la pin de lecture du bouton stop.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------DECLARATION DES PINS UTILISEES POUR L'ECRAN LCD ET LE BARGRAPH-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
LiquidCrystal lcd(6, 7, 10, 11, 12, 13);                                    //Attribution des pins pour l'écran LCD.
LcdBarGraph lbg(&lcd, 16, 0, 1);                                            //Attribution des pins pour le bargraph de l'écran LCD
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------DECLARATION DES DIRECTIVES DE PREPROCESSEUR-----------------------------------------------------------------------------------------------------------------------------------------------------------*/  
#define max_AmpMoyenne 200                                                  //Cette variable sera utilisée pour définir la valeur de réference pour le remplissage du bargraph, exemple : max max_AmpMoyenne est mis à 200 et que la variable à tester atteint la valeur de 100 alors le bargraph fera 50% de la taille de l'écran.
#define TIMER1_TOP (259)                                                    //Nous pouvons modifier la valeur du timertop pour affiner le reglage de la frequence de debordement du timer1.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------DECLARATION DES PIN NUMERIQUES--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
const int bouttonCalibrage = 2;                                             //Boutton pour le calibrage numérique.
const int Pinbuzzer = 3;                                                    //Buzzer piloté par le regisre OCR2B pour la géneration de la tonalité.
const int EntrerTimer0 = 4;                                                 //Bouclage de la pin 4(intput du timer0) avec la pin 9.
const int SortieTimer0 = 9;                                                 //Bouclage de la pin 9(output du timer0) avec la pin 4.
const int BobineTX = 5;                                                     //Pin qui alimente la bobine primaire du détecteur de métaux en PWM généré à l'aide du timer0.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------VARIABLES UNIQUEMENT UTILISEES PAR L'INTERRUPT SUB-ROUTINE (ISR) LORS DES DEBORDEMENTS DU TIMER 1-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
int16_t ValeurBrut[4];                                                      //Les "ValeurBrut[]"" sont utilisées pour accumuler la lecture des données du CAN, une pour chacune des 4 phases.
uint16_t Echantillon = 0;                                                   //variable utilisée pour gérer les séquences d'échantillonnage.
const uint16_t EchantillonageMax = 1024;                                    //Constante qui sert à définir un plafond pour la variable "Echantillon", ici 1024 (10bits) représente le nombre de bits du CAN.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------VARIABLES UTILISEES A LA FOIS PAR L'INTERRUPT SUB-ROUTINE (ISR) LORS DES DEBORDEMENTS DU TIMER 1 ET EGALEMENT A L'EXTERIEUR-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
volatile int16_t Moyenne[4];                                                //Quand nous avons accumulé suffisamment de lectures dans les "ValeursBrutes[]", l'ISR les copie dans cette variable et recommence.
volatile uint32_t ticks = 0;                                                //Compteur de tick pour le chronometrage.
volatile bool EchantillonPret = false;                                      //Indique si le tableau des échantillons recueillis a été mis à jour et qu'il n'est pas vide.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------VARIABLES UTILISEES UNIQUEMENT A L'EXTERIEUR DE L'INTERRUPT SUB-ROUTINE (ISR)-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
int16_t calib[4];                                                           //Valeur (réglée pendant la calibration) que l'on extrait de la variable "Moyenne" et qui servira pour recalibrer le détecteur.
volatile uint8_t dernierCompteur;                                           //Variable qui stocke la dernière lecture du registre TCNT0 qui contient la valeur du compteur du Timer0.
volatile uint16_t echec = 0;                                                //Cette variable compte combien de fois l'ISR a été exécutée en retard, elle doit donc rester à 0 si tout fonctionne bien.
const double Racine_De_Un_Demi = 0.707106781;                               //La constante stocke la valeur de la moitié de la racine carrée de 2, cette valeur est utilisée pour effectuer des transformations de coordonnées (trigonométriques), par exemple pour effectuer une rotation de 45 degrés.
const double QuartDePI = 0.785398163;                                       //π/4 correspond à 45 degrés. Cette variable sera utilisée pour des calculs de phase.
const double ConvRadianVersDegres = 57.2957795;                             //Conversion des radians vers les degrés pour calculer le déphasage en degrés.
const double f = 200.0;                                                     //Variables utilisées pour le filtrage et le traitement numérique des données.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------VARIABLES UTILISEES POUR AFFINER LA DETECTION LORS DES ESSAIS MATERIELS-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
const float Ajustement_De_Phase = (45.0 * 32.0)/(float)(TIMER1_TOP + 1);   //Correction des erreurs de phase que nous corrigeons par les calculs.
float Seuil = 5.0;                                                         //Variable pour contrôler la sensibilité de la détection, une valeur faible correspond à une plus grande sensibilité. Par exemple, 10 est utilisable uniquement si nous avons une bobine parfaitement équilibrée.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


void setup()
{
/*----------------------INITIALISATION DES PINS, DU PORT SERIE ET DE L'ECRAN LCD--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  arduinoNano.alimentation_init();                                          //Appel de la fonction de contrôle de l'alimentation.
  Serial.begin(19200);                                                      //Démarrage de la liaison série.
  lcd.begin(16, 2);                                                         //Démarrage de l'écran LCD.
  pinMode(bouttonCalibrage, INPUT_PULLUP);                                  //Boutton pour la calibration numérique. 
  digitalWrite(SortieTimer0, LOW);                                          //Initialisation de la pin du TIMER0 à l'état bas.
  pinMode(SortieTimer0, OUTPUT);                                            //Initialisation de la broche du Timer1 utilisée pour alimenter le Timer0.
  digitalWrite(BobineTX, LOW);                                              //Pin qui alimente la bobine primaire du détecteur de métaux en PWM généré à l'aide du Timer0 en fonction de la valeur de "TIMER1_TOP()".
  pinMode(BobineTX, OUTPUT);                                                
  lcd.setCursor(2, 0);                        
  lcd.print("c'est parti !");
  delay(3000);
  lcd.clear();      
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/  


/*----------------------CONFIGURATION ET AFFICHAGE DES REGISTRES--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ConfigRegistre(TIMER1_TOP,1);
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------ATTENTE DU PREMIERE APPEL DE L'ISR ET EFFACEMENT DU 1ER ECHANTILLON-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
  while (!EchantillonPret) {}                                               //Tant que la condition EchantillonPret est évaluée comme fausse, nous restons dans la boucle. Cette condition a été initialement déclarée comme fausse, et il faudra attendre qu'elle soit modifiée lors d'un appel de l'ISR.
  echec = 0;                                                                //Variable qui compte combien de fois l'ISR a été exécutée en retard, elle doit donc rester à 0 si tout fonctionne bien.                 
  EchantillonPret = false;                                                  //Définie l'opération booléenne de la variable EchantillonPret comme fausse. 
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------EXECUTION DE L'ISR LORS DU DEBORDEMENT DU TIMER1, C'EST DANS CELUI-CI QUE NOUS ALLONS COLLECTER LES DONNEES LUE PAR LA PIN A0 A TRAVERS LE REGISTRE ADCH-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
ISR(TIMER1_OVF_vect)                                                        //Interruption de débordement du Timer1.
{
  ++ticks;                                                                  //Incrémentation du compteur de ticks.
  uint8_t compteur = TCNT0;                                                 //Le registre TCNT0 stocke la valeur du compteur de temporisation du Timer0.
  int16_t val = (int16_t)(uint16_t)ADCH;                                    //Nous avons uniquement besoin de lire les 8 bits les plus significatifs.
  if (compteur != ((dernierCompteur + 1) & 7))                              //Test de condition pour vérifier si la valeur du 'compteur' a changé d'une unité depuis la dernière itération de la boucle.
  {
    ++echec;                                                                //Variable qui compte combien de fois l'ISR a été exécutée en retard, elle doit donc rester à 0 si tout fonctionne bien.
  }
  dernierCompteur = compteur;                                               //Copie la valeur de 'compteur' dans 'dernierCompteur' pour pouvoir faire la comparaison lors de la prochaine itération de la boucle.
  int16_t *p = &ValeurBrut[compteur & 3];                                   //Crée un pointeur 'p' qui pointe vers 'ValeurBrut'. Le masque '&' (ET binaire) permet de limiter le nombre d'opérations jusqu'à ce que 'compteur' atteigne 3.
  if (compteur < 4)                                                         //Exécution du code tant que 'compteur' ne dépasse pas 4.                            
  {
    *p += (val);                                                            //Incrémente la valeur stockée à l'adresse du pointeur par la valeur stockée dans 'val'.   
    if (*p > 15000) *p = 15000;                                             //Plafonne la valeur maximale stockée à l'adresse du pointeur à 15 000.
  }
  else    //éxecution du code lors ce que compteur dépasse 45               //Exécution du code si 'compteur' dépasse 4.  
  {
    *p -= val;                                                              //Décrémente la valeur stockée à l'adresse du pointeur par la valeur stockée dans 'val'.
    if (*p < -15000) *p = -15000;                                           //Plafonne la valeur minimale stockée à l'adresse du pointeur à - 15 000.
  } 
  if (compteur == 7)                                                        //Exécution du code si 'compteur' est égal à 7.
  {
    ++Echantillon;                                                          //Incrémentation de la valeur 'Echantillon' qui permet de contrôler le nombre de fois que l'échantillonnage a été exécuté.
    if (Echantillon == EchantillonageMax)                                   //Si l'échantillonnage a atteint le plafond défini par la constante EchantillonageMax (1024), alors nous allons exécuter des tests de conditions pour copier les données collectées, et nous remettons également la variable 'Echantillon' à zéro pour la suite.
    {
      Echantillon = 0;                                                      //Remise à zéro de la variable 'Echantillon' pour les prochaines exécutions de l'ISR.
      if (!EchantillonPret)                                                 //Test de condition pour vérifier si l'échantillon précédent a été consommé, et ainsi savoir si l'ISR a déjà été exécuté précédemment.
      {
        memcpy((void*)Moyenne, ValeurBrut, sizeof(Moyenne));                //Copie les données depuis l'emplacement mémoire 'ValeurBrut' vers l'emplacement mémoire 'Moyenne'. La fonction 'memcpy()' est utilisée pour copier une séquence d'octets d'un emplacement mémoire à un autre.
        EchantillonPret = true;                                             //Mise à zéro de la variable 'EchantillonPret'. Il faudra attendre le traitement des données dans le code principal pour que celle-ci soit remise à zéro, permettant ainsi de récupérer à nouveau les données brutes lors d'un nouveau traitement de l'ISR.
      }
      memset(ValeurBrut, 0, sizeof(ValeurBrut));                            //Initialise à zéro tous les éléments du tableau 'ValeurBrut'. La fonction 'memset()' est utilisée pour remplir une zone de mémoire avec une valeur donnée.
    }
  }
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


void loop()
{
/*----------------------APPEL DE LA FONCTION POUR LE CONTROLE DE L'ALIMENTATION--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  arduinoNano.stop_alimentation(3);                                       //La valeur entre parenthèses représente le temps en secondes nécessaire pour l'appui sur le bouton d'alimentation afin de couper l'alimentation.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------BOUCLE DE CONTROLE DE LA VALIDITE DES ECHANTILLONS COLLECTES SUR LA PIN A0--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  while (!EchantillonPret) {}                                             //Si 'EchantillonPret' est faux, alors nous resterons bloqués dans la boucle. Il faudra attendre une nouvelle exécution de l'ISR et la mise à jour de la variable 'EchantillonPret' à l'intérieur de celui-ci pour pouvoir sortir de la boucle 'while'. Cela évite de fausser les calculs ultérieurs avec des échantillons incomplets.
  uint32_t oldTicks = ticks;                                              //Nous copions la valeur de la variable 'ticks' dans 'oldticks'. Cela nous permettra de connaître le nombre de fois que la variable 'ticks' s'est incrémentée depuis cet instant.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------CALIBRATION NUMERIQUE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  if (digitalRead(bouttonCalibrage) == LOW)                               //Lorsque nous appuyons sur le bouton d'étalonnage, nous sauvegardons les sorties du détecteur de phase actuel et les soustrayons des résultats futurs. Il est préférable de prendre plusieurs échantillons au lieu d'en prendre qu'un seul pour plus de précision
  {
    for (int i = 0; i < 4; ++i)                                           //Boucle 'for' nous permettant d'exécuter le code pour le nombre d'échantillons (4).
    {
      calib[i] = Moyenne[i];                                              //Stocke la valeur de 'Moyenne[i]' dans 'calib[i]' afin de pouvoir exécuter la calibration.
    }
    EchantillonPret = false;                                              //Mise à zéro de 'EchantillonPret' pour pouvoir ensuite réexécuter une boucle de l'ISR et récupérer de nouvelles données une fois la calibration terminée.
    Serial.print("Calibré: ");                                            
    
    lcd.setCursor(0,0);
    lcd.print("Calibration...  ");    
    for (int i = 0; i < 4; ++i)                                           //Boucle 'for' pour l'affichage des données de calibration sur l'écran LCD et le port série.
    {
      Serial.write(' ');
      
      Serial.print(calib[i]);
    
    lcd.setCursor(0,1);    
    lcd.print(' ');    
    lcd.print(calib[4]); 
    lcd.print("        ");     
    }
    Serial.println();
  }
  else                                                                    //Une fois le bouton de calibration relâché, nous soustrayons les données de calibration aux données collectées par le capteur (tare).
  {  
    for (int i = 0; i < 4; ++i)                                           //Boucle 'for' nous permettant d'exécuter le code pour le nombre de copies à exécuter (4).                                          
    {
      Moyenne[i] -= calib[i];                                             //Soustrait la valeur contenue dans 'calib[i]' à 'Moyenne[i]'.
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------FILTRAGE ET TRAITEMENT NUMÉRIQUE DES ÉCHANTILLONS POUR ÉLIMINER LA SENSIBILITÉ À LA 3ÈME HARMONIQUE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/                                       
    double ValeurAffiner0 = (Moyenne[0] + Racine_De_Un_Demi * (Moyenne[1] - Moyenne[3]))/f;              //Traitement des données en fonction des valeurs de 'Moyenne[0]', 'Moyenne[1]' et 'Moyenne[3]' et nous stockons le résultat dans 'ValeurAffinée0'.        
    double ValeurAffiner1 = (Moyenne[1] + Racine_De_Un_Demi * (Moyenne[0] + Moyenne[2]))/f;              //Traitement des données en fonction des valeurs de 'Moyenne[1]', 'Moyenne[0]' et 'Moyenne[2]' et nous stockons le résultat dans 'ValeurAffinée1'.
    double ValeurAffiner2 = (Moyenne[2] + Racine_De_Un_Demi * (Moyenne[1] + Moyenne[3]))/f;              //Traitement des données en fonction des valeurs de 'Moyenne[2]', 'Moyenne[1]' et 'Moyenne[3]' et nous stockons le résultat dans 'ValeurAffinée2'.
    double ValeurAffiner3 = (Moyenne[3] + Racine_De_Un_Demi * (Moyenne[2] - Moyenne[0]))/f;              //Traitement des données en fonction des valeurs de 'Moyenne[3]', 'Moyenne[2]' et 'Moyenne[0]' et nous stockons le résultat dans 'ValeurAffinée3'.
    EchantillonPret = false;                                                                             //Les moyennes ont été traitées à l'intérieur de nouvelles variables, l'ISR est donc libre d'écraser les anciennes pour les prochains échantillonnages.
    double amp1 = sqrt((ValeurAffiner0 * ValeurAffiner0) + (ValeurAffiner2 * ValeurAffiner2));           //Traitement des données pour définir l'amplitude du signal en fonction de 'ValeurAffinée0' et 'ValeurAffinée2', et nous stockons le résultat dans la variable 'Amp1'.
    double amp2 = sqrt((ValeurAffiner1 * ValeurAffiner1) + (ValeurAffiner3 * ValeurAffiner3));           //Traitement des données pour définir l'amplitude du signal en fonction de 'ValeurAffinée1' et 'ValeurAffinée3', et nous stockons le résultat dans la variable 'Amp2'.
    double AmpMoyenne = (amp1 + amp2)/2.0;                                                               //Nous réalisons le calcul de la moyenne pour les variables 'amp1' et 'amp2' et nous stockons cette valeur dans 'AmpMoyenne'. Celle-ci nous servira à établir la distance du métal détecté.
    double phase1 = atan2(ValeurAffiner0, ValeurAffiner2) * ConvRadianVersDegres + 45.0;                 //Calcul de la première phase à l'aide des variables 'ValeurAffinée0' et 'ValeurAffinée2', en ajoutant une correction de phase de 45 degrés, et nous stockons le résultat dans la variable 'phase1'. Cette variable sera utilisée avec la variable 'phase2' pour la discrimination.            
    double phase2 = atan2(ValeurAffiner1, ValeurAffiner3) * ConvRadianVersDegres;                        //Calcul de la première phase à l'aide des variables 'ValeurAffinée1' et 'ValeurAffinée3' et nous stockons le résultat dans la variable 'phase2'. Cette variable sera utilisée avec la variable 'phase1' pour la discrimination.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------CALCUL ET TRAITEMENT DES PHASES DU SIGNAL POUR LA DISCRIMINATION--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    if (phase1 > phase2)                                                  //Si 'phase1' est supérieur à 'phase2', alors nous inverserons leurs valeurs, 'phase1' prendra la valeur de 'phase2' et vice versa. Cette opération permet de maintenir une valeur de 'phase1' inférieure à celle de 'phase2', évitant ainsi de perturber les calculs de moyennes, car dans les calculs suivants, nous rencontrerons des opérations de la forme 'phase2 - phase1'.
    {
      double temp = phase1;                                               //Code pour inverser les valeurs de phase1 et phase2.                                              
      phase1 = phase2;
      phase2 = temp;
    }
    double phaseMoyenne = ((phase1 + phase2)/2.0) - Ajustement_De_Phase;  //Nous effectuons le calcul de la moyenne des phases tout en soustrayant le déphasage.
    if (phase2 - phase1 > 180.0)                                          //Test de conditions pour exécuter le code afin de maintenir la valeur de 'PhaseMoyenne' entre 180 et -180.
    { 
      if (phaseMoyenne < 0.0)                                             //Le cas où 'PhaseMoyenne' est inférieur à zéro.
      {
        phaseMoyenne += 180.0;                                            //Mise à 180 de la valeur de 'PhaseMoyenne' lorsque celle-ci dépasse 0.
      }
      else                                                                //Le cas où 'PhaseMoyenne' est supérieur à zéro.                                                                
      {
        phaseMoyenne -= 180.0;                                            //Nous soustrayons 180 de la valeur de 'PhaseMoyenne'.
      }
    }
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------AFFICHAGE DES ECHANTILLONS SUR LE PORT SERIE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
affiEchantillon(echec, ValeurAffiner0, ValeurAffiner1, ValeurAffiner2, ValeurAffiner3, amp1, amp2, phase1, phase2, AmpMoyenne, phaseMoyenne);       
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------AFFICHAGE DU BARGRAPH SUR L'ECRAN LCD EN FONCTION DE LA VALEUR DE LA VARIABLE AMPMOYENNE"--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        lcd.setCursor(0,0);
        lcd.print("          ");
        lcd.print(AmpMoyenne);        
        lcd.setCursor(0,1);
        lbg.drawValue(AmpMoyenne, max_AmpMoyenne);                        //Affichage du bargraph en fonction de la valeur de AmpMoyenne.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------TEST DE CONDITION POUR DETERMINER LA NATURE DE L'OBJET DETECTE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
    if (AmpMoyenne >= Seuil)                                              //Si la variable AmpMoyenne est supérieure à notre constante seuil, alors nous pourrons effectuer les tests de conditions pour la détection. Par conséquent, si nous augmentons la valeur de la constante seuil, AmpMoyenne devra avoir une valeur plus élevée pour déclencher la détection. Plus la valeur de seuil est élevée, moins le détecteur sera sensible.
    {
      if (phaseMoyenne < -20.0)                                           //Si PhaseMoyenne est inférieure à -20, alors nous sommes en présence d'un métal non-ferreux.
      {
        Serial.println("Non-ferreux");                                    //Affichage du type de métal découvert sur l'écran LCD et le port série en fonction de la valeur de 'phaseMoyenne'.
        lcd.setCursor(0,0);        
        lcd.print("Non-ferreux ");
        Serial.println ();
        Serial.println("---------------------------------------------------------------------------");
        Serial.println ();      
      }
      else
      {
        Serial.println("ferreux");                                        //Affichage du type de métal découvert sur l'écran LCD et le port série en fonction de la valeur de 'phaseMoyenne'.   
        lcd.setCursor(0,0);       
        lcd.print("ferreux    ");
        Serial.println ();
        Serial.println("---------------------------------------------------------------------------");
        Serial.println ();                 
      }
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
      

/*----------------------MODULATION DE LA SONORITE DU BUZZER EN FONCTION DE LA VALEUR DE LA VARIABLE AMPMOYENNE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
      float temp = AmpMoyenne;                                            //Stockage de la valeur AmpMoyenne, qui nous servait à la détection, dans la variable 'temp' qui est une variable de type flottant.
              
       int PilotageBuzzer = map (temp, 10, 200, 100, 1500);               //Pilotage du buzzer en fonction de 'temp' (valeur de AmpMoyenne en flottant) que nous utilisons à travers la fonction map qui permet de redimensionner une valeur d'une plage de valeurs à une autre plage de valeurs, et nous stockons le résultat dans la variable 'PilotageBuzzer'.
       tone(Pinbuzzer, PilotageBuzzer,120);                               //Fait sonner le buzzer en fonction de la valeur de notre variable 'PilotageBuzzer' définie précédemment.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------BOUCLE PERMETTANT DE CADANCER L'EXECUTION DU PROGRAMME--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
      while (temp > Seuil)                                                //Boucle tant que 'temp' est supérieur à 'seuil'. Cela permet de cadencer le code en fonction de la valeur que nous attribuons à notre variable seuil. Plus cette valeur est grande, plus nous accumulerons d'échantillons en restant bloqués ici. 
      {
        temp -= (Seuil/2);                                                //Décrémente la valeur de 'temp' pour pouvoir sortir de la boucle.
      }
    }   
   }
  while (ticks - oldTicks < 8000)                                         //Force le programme à boucler ici tant que le nombre de 'ticks' n'a pas atteint 8000. Ces derniers sont incrémentés à chaque exécution de l'ISR, ce qui permet d'attendre de collecter suffisamment d'échantillons avant de réexécuter le programme.
  {
  }   
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
