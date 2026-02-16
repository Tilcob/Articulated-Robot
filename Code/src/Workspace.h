#pragma once

#include "Types.h"

// Workspace checks/clamping for the 2-link planar arm with base rotation.
// This clamps in spherical distance within [|L1-L2|, L1+L2] and z within [Z_MIN, Z_MAX].
struct WorkspaceResult {
	bool wasClamped;
	bool wasOutOfBounds;
	Vec3 clamped;
};

WorkspaceResult clampToWorkspace(const Vec3 &target);
