#pragma once
#include "Types.h"

bool readSerialTcpCommand(bool& debugPrint,
						  bool havePose,
						  const Vec3& lastTcpPosBase,
						  const Mat3& lastRBaseTcp,
						  Vec3& outPos,
						  float& outGrip01);