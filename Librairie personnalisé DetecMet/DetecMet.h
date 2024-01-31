#ifndef DetecMet_h
#define DetecMet_h


#include "Arduino.h"


class alimentation
{
  public:
    alimentation(int pinAlimentation, int pinAlimentationStop);
    void alimentation_init();
    void stop_alimentation(int duree);


  private:
  const int  _Seuil_Boutton_off = 850;
  int _pinAlimentation;
  int _pinAlimentationStop;
  int _lecture_Boutton_off;
  int _tempsAppui = 0;
  int _refV;
};

void Registre(int TIMER1_TOP,int RefV);


void affiEchantillon(double echec, double ValeurAffiner0, double ValeurAffiner1, double ValeurAffiner2, double ValeurAffiner3, double amp1, double amp2, double phase1, double phase2, double AmpMoyenne, double phaseMoyenne);


#endif