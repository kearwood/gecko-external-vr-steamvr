#ifndef OPENVR_SESSION_H
#define OPENVR_SESSION_H

#include <openvr.h>
#include <kraken-math.h>
#include "gecko_vr.h"

class OpenVRSession
{
public:
  OpenVRSession();
  ~OpenVRSession();

  bool Initialize(mozilla::gfx::VRSystemState& aSystemState);
  void Shutdown();
  void StartFrame(mozilla::gfx::VRSystemState& aSystemState);
  bool ShouldQuit() const;

private:
  // OpenVR State
  ::vr::IVRSystem* mVRSystem = nullptr;
  ::vr::IVRChaperone* mVRChaperone = nullptr;
  ::vr::IVRCompositor* mVRCompositor = nullptr;
  bool mShouldQuit;

  bool InitState(mozilla::gfx::VRSystemState& aSystemState);
  void UpdateStageParameters(mozilla::gfx::VRDisplayState& state);
  void UpdateEyeParameters(mozilla::gfx::VRDisplayState& state, kraken::Matrix4* headToEyeTransforms = nullptr);
  void CalcViewMatrices(mozilla::gfx::VRSystemState& state, const kraken::Matrix4* aHeadToEyeTransforms);
  void GetSensorState(mozilla::gfx::VRSystemState& state);
  void ProcessEvents(mozilla::gfx::VRSystemState& aSystemState);
};

#endif // OPENVR_SESSION_H
