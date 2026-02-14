#pragma once

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Angles {
  float q1;
  float q2;
  float q3;
};

struct Mat3 {
  float m[3][3];
};

inline Vec3 add(const Vec3& a, const Vec3& b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 sub(const Vec3& a, const Vec3& b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }

inline Vec3 mul(const Mat3& R, const Vec3& v) {
  return {
    R.m[0][0]*v.x + R.m[0][1]*v.y + R.m[0][2]*v.z,
    R.m[1][0]*v.x + R.m[1][1]*v.y + R.m[1][2]*v.z,
    R.m[2][0]*v.x + R.m[2][1]*v.y + R.m[2][2]*v.z};
}