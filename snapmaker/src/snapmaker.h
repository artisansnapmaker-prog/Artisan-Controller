
#ifndef SNAPMAKER_H_
#define SNAPMAKER_H_

#include <stdio.h>
#include "config.h"

class SnapmakerPrinter
{
  public:
    SnapmakerPrinter() {}

    void init(void (*marlin)());

  private:
    TaskHandle_t thandle_marlin;
};

extern SnapmakerPrinter smprinter;

#endif  // #ifndef SNAPMAKER_H_
