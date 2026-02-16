#include "Trajectory.h"
#include "Config.h"
#include "Util.h"

// Definition of the target positions
const Waypoint TRAJ_POINTS[] = {
    { { 0.13320f, -0.00000f, 0.13040f }, 0.50f },
    { { 0.11535f, -0.06660f, 0.13040f }, 0.50f },
    { { 0.06660f, -0.11535f, 0.13040f }, 0.50f },
    { { 0.00000f, -0.13320f, 0.13040f }, 0.50f },
    { { -0.06660f, -0.11535f, 0.13040f }, 0.50f },
    { { -0.11535f, -0.06660f, 0.13040f }, 0.50f },
    { { -0.13320f, -0.00000f, 0.13040f }, 0.50f },
    { { -0.11535f,  0.06660f, 0.13040f }, 0.50f },
    { { -0.06660f,  0.11535f, 0.13040f }, 0.50f },
    { { 0.00000f,  0.13320f, 0.13040f }, 0.50f },
    {{0,0,0.175f}, 0.5f},
    {{0,0,0.175f}, 0.5f},
    {{0,0,0.175f}, 0.5f},
    { { 0.06660f,  0.11535f, 0.13040f }, 0.50f },
    { { 0.11535f,  0.06660f, 0.13040f }, 0.50f },
    { { 0.05500f, -0.00000f, 0.07620f }, 0.50f },
    { { 0.04763f, -0.02750f, 0.08620f }, 0.50f },
    { { 0.02750f, -0.04763f, 0.09352f }, 0.50f },
    { { 0.00000f, -0.05500f, 0.09620f }, 0.50f },
    { { 0.02750f, -0.04763f, 0.09352f }, 0.50f },
    { { 0.04763f, -0.02750f, 0.08620f }, 0.50f },
    { { 0.05500f, -0.00000f, 0.07620f }, 0.50f },
    { { 0.04763f,  0.02750f, 0.06620f }, 0.50f },
    { { 0.02750f,  0.04763f, 0.13040f }, 0.50f },
    { { 0.00000f,  0.05500f, 0.13040f }, 0.50f },
    { { 0.02750f,  0.04763f, 0.13040f }, 0.50f },
    { { 0.04763f,  0.02750f, 0.13040f }, 0.50f },
    { { 0.13320f, -0.00000f, 0.13040f }, 0.50f },
    {{0,0,0.175f}, 0.5f},
    {{0,0,0.175f}, 0.5f},
    {{0,0,0.175f}, 0.5f},
  };

const int TRAJ_COUNT = sizeof(TRAJ_POINTS)/sizeof(TRAJ_POINTS[0]);

void Trajectory::reset() {
    segmentIndex = 0;
    interpParam = 0.0f;
}

bool Trajectory::finished() const {
    return (TRAJ_COUNT < 2) ? true : (segmentIndex >= TRAJ_COUNT - 1);
}

Waypoint Trajectory::sample(const float deltaTime) {
    if (TRAJ_COUNT <= 0) return Waypoint{Vec3{0,0,0}, 0.5f};
    if (TRAJ_COUNT == 1) return TRAJ_POINTS[0];

    if (finished()) return TRAJ_POINTS[TRAJ_COUNT - 1];

    constexpr float segTime = (cfg::TRAJ_SEG_TIME_S > 0.05f) ? cfg::TRAJ_SEG_TIME_S : 0.05f;
    interpParam += deltaTime / segTime;

    while (interpParam >= 1.0f && segmentIndex < TRAJ_COUNT - 1) {
        interpParam -= 1.0f;
        segmentIndex++;
    }

    if (finished()) return TRAJ_POINTS[TRAJ_COUNT - 1];

    const Waypoint& a = TRAJ_POINTS[segmentIndex];
    const Waypoint& b = TRAJ_POINTS[segmentIndex + 1];

    const float t = util::smoothstep5(interpParam);

    Waypoint out{};
    out.targetPos = util::lerpVec3(a.targetPos, b.targetPos, t);
    out.gripper   = util::lerpf(a.gripper, b.gripper, t);
    return out;
}
