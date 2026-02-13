#pragma once
#include "Types.h"

struct Waypoint {
    Vec3 p;          // (m)
    float gripper01; // 0..1
};

class Trajectory {
public:
    void reset();
    const Waypoint& current() const;
    bool advanceIfArrived(bool arrived); // returns true if advanced
    bool finished() const;

private:
    int idx = 0;
};

extern const Waypoint TRAJ_POINTS[];
extern const int TRAJ_COUNT;
