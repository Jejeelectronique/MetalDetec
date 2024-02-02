#include "DetecMet.h"



/*----------------------Classe Alimentation------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

alimentation::alimentation(int pinAlimentation,int pinAlimentationStop)
{
  _pinAlimentation = pinAlimentation;
  _pinAlimentationStop = pinAlimentationStop;
}

void alimentation::alimentation_init()
{

pinMode(_pinAlimentation,OUTPUT);
digitalWrite(_pinAlimentation,HIGH);
pinMode(_pinAlimentationStop,INPUT);
}


void alimentation::stop_alimentation(int duree)
{
    
    if(_pinAlimentationStop >= 0 && _pinAlimentationStop <=13){
      _lecture_Boutton_off = digitalRead(_pinAlimentationStop);                        
      if(_lecture_Boutton_off == HIGH)
      {
        _tempsAppui++;
        delay(1000);

        if(_tempsAppui > duree)
        {
          digitalWrite(_pinAlimentation,LOW);
        }
      }

      if(_tempsAppui > 0 && _lecture_Boutton_off == LOW)
      {
        _tempsAppui=0;
      }
      }

      else {

        _lecture_Boutton_off = analogRead(_pinAlimentationStop);   
          if(_lecture_Boutton_off >= _Seuil_Boutton_off)       
          {
            _tempsAppui++;                                     
            delay(1000);
    
          if(_tempsAppui > duree)
          {                                                                 //Si la variable atteint la valeur définie alors on coupe l'alimentation
            digitalWrite(_pinAlimentation,L sW);                            //coupure de l'alimentation        
          }
        }

        if(_tempsAppui > 0 && _lecture_Boutton_off < _Seuil_Boutton_off)    //Test de condition pour voire si le compteur du temps d'appuie est supérieur à 0 et que le boutton a etais relaché, alors il faut réinitialisé le compteur.
        {
          _tempsAppui=0;                                                    //réinitialisation du compteur.
        }
        }

    }
/*----------------------FIN DE CLASSE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
















/*----------------------Fonction de déclaration et affichage des registres------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


void Registre(int TIMER1_TOP, int _refV)
{
/*--------------------------------------------------DESACTIVATION DE TOUTES LES INTERRUPTIONS PENDANT L'INITIALISATION DES REGISTRES----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  cli();
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------REINITIALISATION DU TIMER0 INITIALISER AUTOMATIQUEMENT PAR L'ARDUINO----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  TCCR0B = 0;                                                         // arrete le timer0 qui a etais configurer par defaut par l'arduino.
  TIMSK0 = 0;                                                         // desactive les interruption du timer0
  TIFR0 = 0x07;                                                       // efface les interruption en attente du timer0
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------SELECTION DE LA TENSION DE REFERENCE DE LA PIN AREF----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
if (_refV == 1 ){                                                     //utilise la pin AREF (connecté au 3.3v) comme tension de reference si la commande #define USE_3V3_AREF est utilisé au début du code
  ADMUX = (1 << ADLAR);                                               //Le registre ADMUX (Analog-to-Digital Multiplexer) contrôle les entrées analogiques qui sont lues par le CAN, ADLAR (ADjust Left ARithmetic),active la lecture alignée à gauche des données ADC. Cela signifie que les 8 bits les plus significatifs de la conversion ADC sont stockés dans le registre ADCH, tandis que les 2 bits les moins significatifs sont stockés dans le registre ADCL.
}
else{
  ADMUX = (1 << REFS0) | (1 << ADLAR);                                //REFS0 signifie que l'on sélectionne la tension de référence AVCC.
} 
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------CONFIGURATION DES REGISTRES DE L'ADC POUR LA LECTURE DE LA BROCHE A0----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  ADCSRB = (1 << ADTS2) | (1 << ADTS1);                               // Auto-déclenchement du CAN sur débordement du Timer/counter1.
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2);   // Active le CAN, Active l'auto-trigger,prescaler = 16 (1Mhz CAN clock).
  DIDR0 = 1;                                                          // la broche A0 ne sera pas utilisée comme entrée numérique et sera réservée pour une utilisation analogique avec le CAN.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------CONFIGURATION DU TIMER 1----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  TCCR1A = (1 << COM1A1) | (1 << WGM11);  
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
  TCCR1C = 0;                                                         //initialise le registre TCCR1C à zéro, ce qui configure le Timer1 en mode normal. En mode normal, le Timer1 compte jusqu'a 65535 avant de redémarrer à zéro.
  OCR1AH = (TIMER1_TOP/2 >> 8);                                       //OCR1AH est utilisé pour stocker la valeur de la partie supérieure (MSB) de la valeur de comparaison du TIMER1. OCR1AH est définie en prenant la moitié de la valeur de TIMER1_TOP, puis en décalant cette valeur de 8 bits vers la droite. Le décalage de bits vers la droite de 8 bits signifie que la valeur est divisée par 256. pour ensuite etre stocke dans OCR1AH pour être utilisée par le Timer1
  OCR1AL = (TIMER1_TOP/2 & 0xFF);                                     //OCR1AL seras égals a la moitié de la valeur maximale du Timer1 (TIMER1_TOP) dans un format de 8 bits. "OCR1AL" est un registre de comparaison de sortie pour le Timer1.                                                                   
  ICR1H = (TIMER1_TOP >> 8);                                          // définie la valeur de comparaison pour le mode de compteur ICR1, qui est utilisé pour mesurer la durée entre deux événements externes.la partie haute de la valeur de comparaison est définie en utilisant le décalage de la constante TIMER1_TOP de 8 bits vers la droite.nous obtenons les 8 bits les plus significatifs de cette valeur, qui sont ensuite stockés dans le registre ICR1H. Les 8 bits les moins significatifs de TIMER1_TOP sont stockés dans le registre ICR1L.
  ICR1L = (TIMER1_TOP & 0xFF);                                        //écrit la valeur basse du registre ICR1 en appliquant un masque bit à bit pour extraire les 8 bits de poids faible du registre TIMER1_TOP, En utilisant un masque, cela garantit que seuls les 8 bits de poids faible sont écrits dans le registre ICR1L, tandis que les 8 bits de poids fort sont ignorés.                                             
  TCNT1H = 0;                                                         //réinitialisation des 8bit de poids fort du timer1.                                                
  TCNT1L = 0;                                                         //réinitialisation des 8bit de poids faible du timer1.    
  TIFR1 = 0x07;                                                       // efface toute interruption en attente du timer1.
  TIMSK1 = (1 << TOIE1);                                              //active l'interruption de dépassement de temps pour le timer1.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 

/*--------------------------------------------------CONFIGURATION DU TIMER 0----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);               //TCCR0A contrôle le fonctionnement du timer0 ,COM0B1 : définit la configuration de la sortie OC0B (broche B de comparaison de sortie) pour le PWM du mode non inverseur. WGM01 et WGM00 : définie le timer 0 en Fast PWM ce qui permet de générer des signaux de sortie avec une résolution de 8 bits.
  TCCR0B = (1 << CS00) | (1 << CS01) | (1 << CS02) | (1 << WGM02);    //configure le timer0 pour qu'il utilise un prescaler de 1024 et génère une sortie PWM avec une résolution de 8 bits.
  OCR0A = 7;                                                          //configue le signal PWM qui sera généré avec une valeur de comparaison de sortie de 7, ce qui affecte la durée de la période active du signal et donc la quantité d'énergie délivrée à la charge connectée.
  OCR0B = 3;                                                          //En assignant la valeur 3 à OCR0B, cela signifie que le signal PWM généré aura un rapport cyclique de 11,72% (3/255), ce qui correspond à un signal avec une largeur d'impulsion de 11,72% de la période du signal.
  TCNT0 = 0;                                                          //mets à zéro le Timer0. Cette instruction permet de démarrer le compteur à partir de zéro et de compter à partir de ce point.
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------AFFICHAGE DES REGISTRES----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  Serial.println("Registre TCCR0B = " + String(TCCR0B,BIN));  
  Serial.println("Registre TIMSK0 = " + String(TIMSK0,BIN));
  Serial.println("Registre TIFR0 = " + String(TIFR0,BIN));
  Serial.println("Registre ADMUX = " + String(ADMUX,BIN));
  Serial.println("Registre ADCSRB = " + String(ADCSRB,BIN));
  Serial.println("Registre ADCSRA = " + String(ADCSRA,BIN));
  Serial.println("Registre DIDR0 = " + String(DIDR0,BIN));
  Serial.println("Registre TCCR1A = " + String(TCCR1A,BIN));
  Serial.println("Registre TCCR1B = " + String(TCCR1B,BIN));
  Serial.println("Registre TCCR1C = " + String(TCCR1C,BIN));
  Serial.println("Registre OCR1AH = " + String(OCR1AH,BIN));
  Serial.println("Registre OCR1AL = " + String(OCR1AL,BIN)); 
  Serial.println("Registre ICR1L = " + String(ICR1L,BIN));
  Serial.println("Registre TCNT1H = " + String(TCNT1H,BIN));
  Serial.println("Registre TCNT1L = " + String(TCNT1L,BIN));
  Serial.println("Registre TIFR1 = " + String(TIFR1,BIN));
  Serial.println("Registre TIMSK1 = " + String(TIMSK1,BIN));
  Serial.println("Registre TCCR0A = " + String(TCCR0A,BIN));
  Serial.println("Registre TCCR0B = " + String(TCCR0B,BIN));
  Serial.println("Registre OCR0A = " + String(OCR0A,BIN));
  Serial.println("Registre OCR0B = " + String(OCR0B,BIN));
  Serial.println("Registre TCNT0 = " + String(TCNT0,BIN));
  Serial.println ();
  Serial.println("---------------------------------------------------------------------------");
  Serial.println ();
  sei();
}
/*----------------------FIN DE FONCTION REGISTRE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


















/*----------------------Fonction d'affichage des échantillons------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void affiEchantillon(double echec, double ValeurAffiner0, double ValeurAffiner1, double ValeurAffiner2, double ValeurAffiner3, double amp1, double amp2, double phase1, double phase2, double AmpMoyenne, double phaseMoyenne)
  { 

    /*--------------------------------------------------AFFICHAGE DES VALEUR RECOLTER PAR LE REGISTRE ADCH----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/                                                       
    Serial.println("echec : " +String(echec));
    Serial.write(' ');
    
    if (ValeurAffiner0 >= 0.0) Serial.write(' ');
    Serial.println("ValeurAffiner0 : " +String(ValeurAffiner0, 2));                       //Affiche la valeur de ValeurAffiner0 avec une précision de 2décimals
    Serial.write(' ');                                                                    //envoie un espace au port série pour aligné l'affichage des valeurs
    if (ValeurAffiner1 >= 0.0) Serial.write(' ');
    Serial.println("ValeurAffiner1 : " +String (ValeurAffiner1, 2)); /
    Serial.write(' ');  
    if (ValeurAffiner2 >= 0.0) Serial.write(' ');
    Serial.println("ValeurAffiner2 :" +String (ValeurAffiner2, 2)); 
    Serial.write(' '); 
    if (ValeurAffiner3 >= 0.0) Serial.write(' ');
    Serial.println("ValeurAffiner3 : " +String(ValeurAffiner3, 2)); 
    Serial.print("    ");  
    Serial.println("Amp1 : " +String(amp1, 2));                                           //Affiche la valeur de amp1 avec une précision de 2décimals
    Serial.write(' '); 
    Serial.println("Amp2 : " +String(amp2, 2));                                           //Affiche la valeur de amp2 avec une précision de 2décimals
    Serial.write(' ');  
    if (phase1 >= 0.0) Serial.write(' ');
    Serial.println("Phase1 : " +String(phase1, 2));                                       //Affiche la valeur de phase1 avec une précision de 2décimals
    Serial.write(' ');  
    if (phase2 >= 0.0) Serial.write(' ');
    Serial.println("Phase2 : " +String(phase2, 2));                                       //Affiche la valeur de phase2 avec une précision de 2décimals
    if (AmpMoyenne >= 0.0) Serial.write(' ');
    Serial.println("AmpMoyenne : " +String(AmpMoyenne, 1));                               //Affiche la valeur de AmpMoyenne avec une précision de 1décimal
    Serial.write(' ');
    if (phaseMoyenne >= 0.0) Serial.write(' ');
    Serial.println("Phase Moyenne : " +String((int)phaseMoyenne));
  Serial.println ();
  Serial.println("---------------------------------------------------------------------------");
  Serial.println ();
  }
/*----------------------FIN DE FONCTION AFFIECHANTILLON--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
