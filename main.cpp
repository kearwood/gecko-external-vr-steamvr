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

#include "gecko_vr.h"

#define M_PI 3.14159265358979323846264338327950288

using namespace mozilla::gfx;

int main(int argc, char **argv) {
  if (!gecko_vr_init()) {
    fprintf(stderr, "Failied to initialize Gecko VR\n");
    return EXIT_FAILURE;
  }

  VRSystemState state;

  VRDisplayState& displayState = state.displayState;
  memset(&displayState, 0, sizeof(displayState));

  strncpy_s(displayState.mDisplayName, "HelloVR HMD", kVRDisplayNameMaxLen);
  displayState.mIsConnected = true;
  displayState.mIsMounted = true;
  displayState.mCapabilityFlags = (VRDisplayCapabilityFlags)((int)(VRDisplayCapabilityFlags::Cap_None) |
                                  (int)VRDisplayCapabilityFlags::Cap_Orientation |
                                  (int)VRDisplayCapabilityFlags::Cap_Position |
                                  (int)VRDisplayCapabilityFlags::Cap_External |
                                  (int)VRDisplayCapabilityFlags::Cap_Present |
                                  (int)VRDisplayCapabilityFlags::Cap_StageParameters |
                                  (int)VRDisplayCapabilityFlags::Cap_MountDetection);

  displayState.mEyeResolution.width = 1280;
  displayState.mEyeResolution.height = 800;

  for (uint32_t eye = 0; eye < 2; ++eye) {
    float left, right, up, down;
    // TODO - Implement real values
    left = -0.785398f; // 45 degrees
    right = 0.785398f; // 45 degrees
    up = -0.785398f; // 45 degrees
    down = 0.785398f; // 45 degrees

    displayState.mEyeFOV[eye].upDegrees = atan(up) * 180.0 / M_PI;
    displayState.mEyeFOV[eye].rightDegrees = atan(right) * 180.0 / M_PI;
    displayState.mEyeFOV[eye].downDegrees = atan(down) * 180.0 / M_PI;
    displayState.mEyeFOV[eye].leftDegrees = atan(left) * 180.0 / M_PI;
  }

  displayState.mStageSize.width = 1.0f;
  displayState.mStageSize.height = 1.0f;

  displayState.mSittingToStandingTransform[0] = 1.0f;
  displayState.mSittingToStandingTransform[1] = 0.0f;
  displayState.mSittingToStandingTransform[2] = 0.0f;
  displayState.mSittingToStandingTransform[3] = 0.0f;

  displayState.mSittingToStandingTransform[4] = 0.0f;
  displayState.mSittingToStandingTransform[5] = 1.0f;
  displayState.mSittingToStandingTransform[6] = 0.0f;
  displayState.mSittingToStandingTransform[7] = 0.0f;

  displayState.mSittingToStandingTransform[8] = 0.0f;
  displayState.mSittingToStandingTransform[9] = 0.0f;
  displayState.mSittingToStandingTransform[10] = 1.0f;
  displayState.mSittingToStandingTransform[11] = 0.0f;

  displayState.mSittingToStandingTransform[12] = 0.0f;
  displayState.mSittingToStandingTransform[13] = 0.75f;
  displayState.mSittingToStandingTransform[14] = 0.0f;
  displayState.mSittingToStandingTransform[15] = 1.0f;

  VRHMDSensorState& sensorState = state.sensorState;
  sensorState.flags = (VRDisplayCapabilityFlags)(
    (int)VRDisplayCapabilityFlags::Cap_Orientation |
    (int)VRDisplayCapabilityFlags::Cap_Position);
  sensorState.orientation[3] = 1.0f; // Default to an identity quaternion

  for(int frame = 0; frame < 1000; frame++) {
    fprintf(stdout, "Frame %i\n", frame);
    gecko_vr_push_state(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(11));

    sensorState.inputFrameID++;
    sensorState.timestamp += 1.0f / 90.0f;
  }

  gecko_vr_shutdown();
}
