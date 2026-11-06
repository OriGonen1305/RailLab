#pragma once

#include "raillab/track.hpp"
#include <iosfwd>
#include <string>
#include <vector>

namespace rail {

struct Settings {
    double initialSpeed = 4.0; // m/s
    double mass = 300.0;       // kg
    double gravity = 9.80665;  // m/s^2
    double resistance = 0.025; // constant slowdown, m/s^2
    double dragArea = 0.35;    // Cd*A, m^2
    double airDensity = 1.225; // kg/m^3
    double step = 0.15;        // biggest step, meters
};

enum class RunStatus { Complete, Stalled, CannotStart, SampleLimit };

struct Sample {
    double distance{}, time{};
    TrackSample track;
    double speed{}, tangentialAcceleration{}, normalAcceleration{}, acceleration{};
    double normalG{}, feltG{}, railForce{};
    double kineticEnergy{}, potentialEnergy{}, dissipatedEnergy{}, mechanicalEnergy{};
    double forwardForce{},
        propulsionWork{}; // Signed N along the forward tangent; cumulative applied work, J.
};

struct Analysis {
    std::vector<Sample> samples;
    RunStatus status = RunStatus::Complete;
    double trackLength{}, maxSpeed{}, maxFeltG{}, minNormalG{}, maxNormalG{};
    double energyResidual{};            // biggest energy mismatch, joules
    bool timeResolutionLimited = false; // Near equilibrium or nonfinite time; also reported by CSV/CLI.
    std::string message;
};

void validate(const Settings &settings);
// An empty list means zero force everywhere; otherwise one value per segment is required.
void validateForwardForces(const std::vector<double> &forces, std::size_t segmentCount);
Analysis analyze(const Track &track, const Settings &settings, const std::vector<double> &forwardForces = {});
void writeCsv(std::ostream &out, const Analysis &analysis);
std::vector<Vec2> readAnchorsCsv(std::istream &in);
std::string statusName(RunStatus status);

} // namespace rail
