// libsurvive_cli.cpp
#include <survive.h>
#include <signal.h>
#include <stdlib.h>

static volatile int keepRunning = 1;

#ifdef __linux__
void intHandler(int dummy) {
  if (keepRunning == 0)
    exit(-1);
  keepRunning = 0;
}
#endif

// Button callback function
SURVIVE_EXPORT void button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
                                   const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {
  printf("Button event: Object %s, Button %d\n", so->serial_number, buttonId);  // Simple print for standalone CLI
}

int runLibsurvive(int argc, char **argv) {
#ifdef __linux__
  signal(SIGINT, intHandler);
  signal(SIGTERM, intHandler);
  signal(SIGKILL, intHandler);
#endif

  // Initialize libsurvive
  SurviveContext *ctx = survive_init(argc, argv);
  if (ctx == 0) // implies -help or similar
    return 0;

  // Start tracking
  survive_startup(ctx);

  // Install button handler
  survive_install_button_fn(ctx, button_process);

  // Poll loop
  while (keepRunning && survive_poll(ctx) == 0) {
  }

  // Clean up
  survive_close(ctx);
  return 0;
}
