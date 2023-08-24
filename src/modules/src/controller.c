#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_mellinger.h"
#include "controller_indi.h"
#include "controller_floaty.h"

#include "autoconf.h"

#define DEFAULT_CONTROLLER ControllerTypePID
static ControllerType currentController = ControllerTypeAny;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} ControllerFcns;


void controllerInit(ControllerType controller) {
  if (controller < 0 || controller >= ControllerType_COUNT) {
    return;
  }

  currentController = controller;

  if (ControllerTypeAny == currentController) {
    currentController = DEFAULT_CONTROLLER;
  }

  #if defined(CONFIG_CONTROLLER_PID)
    #define CONTROLLER ControllerTypePID
  #elif defined(CONFIG_CONTROLLER_INDI)
    #define CONTROLLER ControllerTypeINDI
  #elif defined(CONFIG_CONTROLLER_MELLINGER)
    #define CONTROLLER ControllerTypeMellinger
  #elif defined(CONFIG_CONTROLLER_FLOATY)
    #define CONTROLLER ControllerTypeFloaty
  #else
    #define CONTROLLER ControllerTypeAny
  #endif

  ControllerType forcedController = CONTROLLER;
  if (forcedController != ControllerTypeAny) {
    DEBUG_PRINT("Controller type forced\n");
    currentController = forcedController;
  }

  initController();

  DEBUG_PRINT("Using %s (%d) controller\n", controllerGetName(), currentController);
}

ControllerType getControllerType(void) {
  return currentController;
}

static void initController() {
  // Forcing the use of Floaty controller
  // controllerFunctions[currentController].init();
  controllerFloatyInit();
}

bool controllerTest(void) {
  // Forcing the use of Floaty controller
  // return controllerFunctions[currentController].test();
  return controllerFloatyTest();
}

void controller(floaty_control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Forcing the use of Floaty controller
  // controllerFunctions[currentController].update(control, setpoint, sensors, state, tick);
  controllerFloaty(control, setpoint, sensors, state, tick);
}

const char* controllerGetName() {
  // Forcing the use of Floaty controller
  // return controllerFunctions[currentController].name;
  return "Floaty";
}
