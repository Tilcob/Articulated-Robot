#include "Workspace.h"
#include <math.h>
#include "Config.h"

static Vec3 clampReachDistance(Vec3 t) {
	const float r = sqrtf(t.x * t.x + t.y * t.y);
	const float z = t.z - cfg::h;
	const float d = sqrtf(r * r + z * z);

	const float dMin = fabsf(cfg::L1 - cfg::L2);
	constexpr float dMax = cfg::L1 + cfg::L2;
	constexpr float EPS = 1e-4f;

	if (d < EPS) return t;

	if (d > dMax) {
		const float s = dMax / d;
		t.x *= s;
		t.y *= s;
		t.z = cfg::h + z * s;
	} else if (d < dMin) {
		const float s = dMin / d;
		t.x *= s;
		t.y *= s;
		t.z = cfg::h + z * s;
	}
	return t;
}

WorkspaceResult clampToWorkspace(const Vec3 &target) {
	WorkspaceResult out{};
	out.wasClamped = false;
	out.wasOutOfBounds = false;
	out.clamped = target;

	constexpr float EPS = 1e-4f;

	// Z bounds
	const bool zBad = (target.z < cfg::Z_MIN - EPS) || (target.z > cfg::Z_MAX + EPS);

	// Distance bounds
	const float r = sqrtf(target.x * target.x + target.y * target.y);
	const float z = target.z - cfg::h;
	const float d = sqrtf(r * r + z * z);
	const float dMin = fabsf(cfg::L1 - cfg::L2);
	constexpr float dMax = cfg::L1 + cfg::L2;
	const bool dBad = (d < dMin - EPS) || (d > dMax + EPS);

	out.wasOutOfBounds = zBad || dBad;
	if (!out.wasOutOfBounds) return out;

	// Clamp: keep direction, scale distance into reachable annulus.
	out.clamped = clampReachDistance(target);
	out.wasClamped = true;
	return out;
}
