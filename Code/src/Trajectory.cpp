#include "Trajectory.h"

#include "Config.h"

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
    { { -0.02750f, -0.04763f, 0.09352f }, 0.50f },
    { { -0.04763f, -0.02750f, 0.08620f }, 0.50f },
    { { -0.05500f, -0.00000f, 0.07620f }, 0.50f },
    { { -0.04763f,  0.02750f, 0.06620f }, 0.50f },
    { { -0.02750f,  0.04763f, 0.13040f }, 0.50f },
    { { 0.00000f,  0.05500f, 0.13040f }, 0.50f },
    { { 0.02750f,  0.04763f, 0.13040f }, 0.50f },
    { { 0.04763f,  0.02750f, 0.13040f }, 0.50f },
    { { 0.13320f, -0.00000f, 0.13040f }, 0.50f },
{{0,0,0.18f}, 0.5f},
    {{0,0,0.18f}, 0.5f},
    {{0,0,0.18f}, 0.5f},
{{0,0,0.3f}, 0.5f},
{{0,0,0.3f}, 0.5f},
{{0,0,0.3f}, 0.5f},
  };

const int TRAJ_COUNT = sizeof(TRAJ_POINTS)/sizeof(TRAJ_POINTS[0]);

static float clamp01(const float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

static float interpolateStep(float u) {
    u = clamp01(u);
    return u*u*u*(u*(u*6.0f - 15.0f) + 10.0f);
}

static Vec3 lerpVec3(const Vec3& a, const Vec3& b, const float t) {
    return Vec3{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

static float lerpf(const float a, const float b, const float t) {
    return a + (b - a) * t;
}

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

    const float segTime = (TRAJ_SEG_TIME_S > 0.05f) ? TRAJ_SEG_TIME_S : 0.05f;
    interpParam += deltaTime / segTime;

    while (interpParam >= 1.0f && segmentIndex < TRAJ_COUNT - 1) {
        interpParam -= 1.0f;
        segmentIndex++;
    }

    if (finished()) return TRAJ_POINTS[TRAJ_COUNT - 1];

    const Waypoint& a = TRAJ_POINTS[segmentIndex];
    const Waypoint& b = TRAJ_POINTS[segmentIndex + 1];

    const float t = interpolateStep(interpParam);

    Waypoint out{};
    out.targetPos = lerpVec3(a.targetPos, b.targetPos, t);
    out.gripper   = lerpf(a.gripper, b.gripper, t);
    return out;
}
