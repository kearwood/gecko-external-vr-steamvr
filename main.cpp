#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include <chrono>
#include <thread>

#if defined(__APPLE__)
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <errno.h>
#include <unistd.h>
#endif // defined(__APPLE__)

#include <openvr.h>
#include <kraken-math.h>
#include "gecko_vr.h"
#include "OpenVRSession.h"

using namespace mozilla::gfx;
using namespace kraken;

int main(int argc, char **argv) {
  OpenVRSession session;
  VRSystemState systemState;
  memset(&systemState, 0, sizeof(systemState));

  if (!session.Initialize(systemState)) {
    fprintf(stderr, "Unable to start OpenVR session.\n");
    return EXIT_FAILURE;
  }

  if (!gecko_vr_init()) {
    fprintf(stderr, "Failied to initialize Gecko VR.\n");
    return EXIT_FAILURE;
  }

  while (!session.ShouldQuit()) {
    fprintf(stdout, "Frame %lli\n", systemState.sensorState.inputFrameID);
    session.StartFrame(systemState);
    gecko_vr_push_state(systemState);
  }
/*
    std::this_thread::sleep_for(std::chrono::milliseconds(11));
*/
  gecko_vr_shutdown();
}
