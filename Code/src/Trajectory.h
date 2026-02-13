#pragma once
#include "Types.h"

struct Waypoint {
    Vec3 targetPos;  // (m)
    float gripper; // 0..1
};

class Trajectory {
public:
    void reset();
    const Waypoint& current() const;
    bool advanceIfArrived(bool arrived);
    bool finished() const;

private:
    int idx = 0;
};

extern const Waypoint TRAJ_POINTS[];
extern const int TRAJ_COUNT;
