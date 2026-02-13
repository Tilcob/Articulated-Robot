#include "Trajectory.h"

// Definition of the target positions
const Waypoint TRAJ_POINTS[] = {
    { {0.20f, 0.00f, 0.10f}, 0.5f },
    { {0.23f, 0.03f, 0.14f}, 0.5f },
    { {0.25f, -0.02f, 0.12f}, 0.2f },
    { {0.20f, 0.00f, 0.10f}, 0.8f },
  };

const int TRAJ_COUNT = (sizeof(TRAJ_POINTS)/sizeof(TRAJ_POINTS[0]));

void Trajectory::reset() { idx = 0; }

const Waypoint& Trajectory::current() const {
    if (idx < 0) return TRAJ_POINTS[0];
    if (idx >= TRAJ_COUNT) return TRAJ_POINTS[TRAJ_COUNT-1];
    return TRAJ_POINTS[idx];
}

bool Trajectory::finished() const {
    return idx >= TRAJ_COUNT;
}

bool Trajectory::advanceIfArrived(const bool arrived) {
    if (!arrived) return false;
    if (idx < TRAJ_COUNT) idx++;
    return true;
}
