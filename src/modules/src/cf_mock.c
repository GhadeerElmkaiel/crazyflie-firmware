
#include "cf_mock.h"
#include "param.h"


static bool isInit = false;


PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Lighthouse positioning deck](%https://store.bitcraze.io/collections/decks/products/lighthouse-positioning-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBuzzer, &isInit)

PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &isInit)

PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &isInit)

PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)

PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &isInit)

PARAM_GROUP_STOP(deck)