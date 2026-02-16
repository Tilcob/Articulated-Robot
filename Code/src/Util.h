#pragma once
#include <math.h>
#include "Types.h"


namespace util {

	inline float clampf(const float x, const float lo, const float hi) {
		return (x < lo) ? lo : (x > hi) ? hi : x;
	}

	inline float clamp01(const float x) { return clampf(x, 0.0f, 1.0f); }

	inline bool nearf(const float a, const float b, const float eps) {
		return fabsf(a - b) <= eps;
	}

	inline float lerpf(const float a, const float b, const float t) { return a + (b - a) * t; }

	inline Vec3 lerpVec3(const Vec3& a, const Vec3& b, const float t) {
		return Vec3{lerpf(a.x, b.x, t), lerpf(a.y, b.y, t), lerpf(a.z, b.z, t)};
	}

	// Smoothstep 5th order (0..1 -> 0..1)
	inline float smoothstep5(float u) {
		u = clamp01(u);
		return u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);
	}

	inline float mapRangeClamped(const int v, const int inMin, const int inMax, const float outMin, const float outMax) {
		const float denom = static_cast<float>(inMax - inMin);
		if (fabsf(denom) < 1e-9f) return outMin;
		float t = static_cast<float>(v - inMin) / denom;
		t = clamp01(t);
		return outMin + t * (outMax - outMin);
	}

}

namespace math3 {

	inline Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
	inline Vec3 sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

	inline Vec3 mul(const Mat3& R, const Vec3& v) {
		return {R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
				R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
				R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z};
	}

	inline Vec3 cross(const Vec3& a, const Vec3& b) {
		return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
	}

}