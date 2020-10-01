#ifndef _VVC_GLOBAL_H
#define _VVC_GLOBAL_H

// Device header file.
#include "stm32h7xx.h"
// Standard library includes.
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Memory section boundaries, etc.
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _siitcm,
                _sidtcm, _sitcm, _sdtcm, _eitcm, _edtcm, _estack;

#endif
