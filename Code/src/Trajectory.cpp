#include "Trajectory.h"
#include "Config.h"
#include "Util.h"
#include "TrajectoryData.h"

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
