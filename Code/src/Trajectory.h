#pragma once
#include "Types.h"

struct Waypoint {
    Vec3 targetPos;  // (m)
    float gripper; // 0..1
};

class Trajectory {
public:
    void reset();
    Waypoint sample(float deltaTime);
    bool finished() const;

private:
    int segmentIndex = 0;
    float interpParam = 0.0f;
};

extern const Waypoint TRAJ_POINTS[];
extern const int TRAJ_COUNT;
