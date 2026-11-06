#include "raillab/analysis.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rail {
void validate(const Settings &p) {
    const double values[]{p.initialSpeed, p.mass, p.gravity, p.resistance, p.dragArea, p.airDensity, p.step};
    for (double x : values)
        if (!std::isfinite(x))
            throw std::invalid_argument("Physics settings must be finite.");
    if (p.initialSpeed < 0 || p.initialSpeed > 200 || p.mass < 1 || p.mass > 100000 || p.gravity < 0.1 ||
        p.gravity > 30 || p.resistance < 0 || p.resistance > 10 || p.dragArea < 0 || p.dragArea > 20 ||
        p.airDensity < 0 || p.airDensity > 20 || p.step < 0.01 || p.step > 2)
        throw std::invalid_argument("Physics settings are outside supported ranges.");
}

void validateForwardForces(const std::vector<double> &forces, std::size_t segmentCount) {
    if (!forces.empty() && forces.size() != segmentCount)
        throw std::invalid_argument("Provide exactly one forward force per track segment.");
    for (double force : forces)
        if (!std::isfinite(force))
            throw std::invalid_argument("Segment forces must be finite numbers in newtons.");
}

std::string statusName(RunStatus status) {
    switch (status) {
    case RunStatus::Complete:
        return "Complete";
    case RunStatus::Stalled:
        return "Stalled";
    case RunStatus::CannotStart:
        return "Cannot start";
    case RunStatus::SampleLimit:
        return "Sample limit";
    }
    return "Unknown";
}

Analysis analyze(const Track &track, const Settings &p, const std::vector<double> &forwardForces) {
    validate(p);
    validateForwardForces(forwardForces, track.segments().size());
    Analysis result;
    result.trackLength = track.length();
    const double y0 = track.sample(0).position.y;
    const double initialSpecific = p.initialSpeed * p.initialSpeed / 2 + p.gravity * y0;
    const double drag = p.airDensity * p.dragArea / (2 * p.mass);
    auto forceAt = [&](std::size_t segment) { return forwardForces.empty() ? 0.0 : forwardForces[segment]; };
    auto criticalDistances = track.criticalDistances();
    for (std::size_t i = 0; i < track.segments().size(); ++i) {
        if (forceAt(i) == 0 && p.resistance == 0)
            continue;
        // Propulsion and resistance shift energy barriers away from height extrema. At zero speed,
        // net acceleration changes sign where g*T_y = F/m - resistance.
        const auto barriers =
            track.tangentLevelDistances(i, (forceAt(i) / p.mass - p.resistance) / p.gravity);
        criticalDistances.insert(criticalDistances.end(), barriers.begin(), barriers.end());
    }
    std::sort(criticalDistances.begin(), criticalDistances.end());
    // Propagate the remaining kinetic energy, not initial energy minus nearly equal losses.
    // w' + 2*b*w = 2*a(s). The exponential part and constant a are integrated exactly;
    // five-point Gauss quadrature integrates only the changing gravity component.
    auto advance = [&](double s, double w, double work, double ds, double force, double *roundoff = nullptr) {
        double next;
        if (drag == 0) {
            // Without drag, the energy formula is exact and avoids accumulated step error.
            const auto energySpeed = [&](double distance, double motorWork) {
                return p.initialSpeed * p.initialSpeed +
                       2 * p.gravity * (y0 - track.sample(distance).position.y) -
                       2 * p.resistance * distance + 2 * (motorWork / p.mass);
            };
            // Carry any boundary roundoff correction into the restarted segment.
            next = energySpeed(s + ds, work + force * ds) + (w - energySpeed(s, work));
        } else {
            const double startY = track.sample(s).tangent.y;
            const double a = force / p.mass - p.resistance - p.gravity * startY;
            const double z = 2 * drag * ds, decay = std::exp(-z);
            const double response = z > 0 ? 2 * ds * (-std::expm1(-z) / z) : 2 * ds;
            constexpr std::array<double, 5> nodes{0, -0.5384693101056831, 0.5384693101056831,
                                                  -0.9061798459386640, 0.9061798459386640};
            constexpr std::array<double, 5> weights{0.5688888888888889, 0.4786286704993665,
                                                    0.4786286704993665, 0.2369268850561891,
                                                    0.2369268850561891};
            double variation = 0, variationScale = 0;
            for (std::size_t i = 0; i < nodes.size(); ++i) {
                const double q = ds * (1 + nodes[i]) / 2;
                const double term = weights[i] * std::exp(-2 * drag * (ds - q)) * p.gravity *
                                    (startY - track.sample(s + q).tangent.y);
                variation += term;
                variationScale += std::abs(term);
            }
            next = decay * w + response * a + ds * variation;
            if (roundoff) {
                // Earlier arithmetic error decays with the same integrating factor as w.
                // Keep this bound only for exact-boundary restarts, never as a speed cutoff.
                const double scale =
                    std::abs(decay * w) +
                    response * (std::abs(force / p.mass) + p.resistance + p.gravity * std::abs(startY)) +
                    ds * variationScale;
                *roundoff = decay * *roundoff + 8 * std::numeric_limits<double>::epsilon() * scale;
            }
        }
        if (!std::isfinite(next))
            throw std::overflow_error("Analysis exceeds the numeric range. Reduce the force magnitude.");
        return next;
    };
    auto append = [&](double s, double w, double work) {
        Sample point;
        point.distance = s;
        point.track = track.sample(s);
        w = std::max(0.0, w);
        point.speed = std::sqrt(w);
        const double opposing = p.resistance + drag * w;
        point.forwardForce = forceAt(point.track.segment);
        const double drive = point.forwardForce / p.mass;
        point.tangentialAcceleration = -p.gravity * point.track.tangent.y - opposing + drive;
        point.normalAcceleration = w * point.track.curvature;
        point.acceleration = std::hypot(point.tangentialAcceleration, point.normalAcceleration);
        // Gravity matters here too.
        point.normalG = point.normalAcceleration / p.gravity + point.track.tangent.x;
        point.feltG = std::hypot(point.normalG, (drive - opposing) / p.gravity);
        point.railForce = point.normalG * p.mass * p.gravity;
        point.kineticEnergy = p.mass * w / 2;
        point.potentialEnergy = p.mass * p.gravity * point.track.position.y;
        // Recover losses for reporting, never use this subtraction to determine a tiny speed.
        const double loss =
            drag == 0 ? p.resistance * s
                      : std::max(0.0, (p.initialSpeed * p.initialSpeed - w) / 2 +
                                          p.gravity * (y0 - point.track.position.y) + work / p.mass);
        point.dissipatedEnergy = p.mass * loss;
        point.mechanicalEnergy = point.kineticEnergy + point.potentialEnergy;
        point.propulsionWork = work;
        for (double value : {point.acceleration, point.feltG, point.railForce, point.kineticEnergy,
                             point.potentialEnergy, point.dissipatedEnergy, point.mechanicalEnergy, work})
            if (!std::isfinite(value))
                throw std::overflow_error("Analysis exceeds the numeric range. Reduce the force magnitude.");
        if (!result.samples.empty()) {
            const auto &last = result.samples.back();
            const double sum = last.speed + point.speed;
            const double ds = s - last.distance;
            double dt = 0;
            if (ds > 0) {
                dt = sum > 0 ? 2 * ds / sum : std::numeric_limits<double>::infinity();
                const auto &curve = track.segments()[last.track.segment];
                if (drag > 0 && last.speed > 0 && forceAt(last.track.segment) / p.mass == p.resistance &&
                    curve.p0.y == curve.p1.y && curve.p1.y == curve.p2.y && curve.p2.y == curve.p3.y) {
                    // On a horizontal force-balanced segment v = v_start * exp(-b*ds).
                    // Integrate its clock exactly, including when v is far below 1e-12 m/s.
                    const double z = drag * ds;
                    dt = (ds / last.speed) * (z > 0 ? std::expm1(z) / z : 1);
                }
            }
            point.time = last.time + dt;
            result.timeResolutionLimited |= !std::isfinite(point.time);
        }
        result.maxSpeed = std::max(result.maxSpeed, point.speed);
        result.maxFeltG = std::max(result.maxFeltG, point.feltG);
        if (result.samples.empty())
            result.minNormalG = result.maxNormalG = point.normalG;
        result.minNormalG = std::min(result.minNormalG, point.normalG);
        result.maxNormalG = std::max(result.maxNormalG, point.normalG);
        result.energyResidual =
            std::max(result.energyResidual, std::abs(point.mechanicalEnergy + point.dissipatedEnergy -
                                                     point.propulsionWork - p.mass * initialSpecific));
        result.samples.push_back(point);
    };

    append(0, p.initialSpeed * p.initialSpeed, 0);
    // Retain the legacy unpowered start tolerance, while allowing small positive motor forces.
    const double startTolerance = forceAt(0) == 0 ? 1e-10 : 0;
    if (p.initialSpeed == 0 && result.samples.front().tangentialAcceleration <= startTolerance) {
        result.status = RunStatus::CannotStart;
        result.message =
            "The cart cannot start forward from rest. Add initial speed, forward force on the first "
            "segment, or start on a downhill slope.";
        return result;
    }

    double s = 0, w = p.initialSpeed * p.initialSpeed, work = 0, roundoff = 0;
    std::size_t critical = 0;
    while (s < track.length()) {
        if (result.samples.size() >= 200000) {
            result.status = RunStatus::SampleLimit;
            result.message =
                "Analysis reached 200,000 samples. Increase the distance step or shorten the track.";
            return result;
        }
        while (critical < criticalDistances.size() && criticalDistances[critical] <= s)
            ++critical;
        // Limit changes within the gravity quadrature and sampled plots under strong drag.
        const double stableStep = drag > 0 ? std::min(p.step, 0.1 / drag) : p.step;
        const std::size_t segment = track.sample(s).segment;
        const double force = forceAt(segment);
        double end = std::min(s + stableStep, track.segmentDistances()[segment + 1]);
        if (critical < criticalDistances.size())
            end = std::min(end, criticalDistances[critical]);
        const double ds = end - s;
        const double halfWork = work + force * (ds / 2), endWork = work + force * ds;
        const double halfW = advance(s, w, work, ds / 2, force, &roundoff);
        const double endW = advance(s + ds / 2, halfW, halfWork, ds / 2, force, &roundoff);
        // Only arithmetic-scale roundoff can be snapped at an actual point boundary.
        // The drag path scales this bound to its remaining energy, not its initial energy.
        const double energyScale =
            drag == 0 ? p.initialSpeed * p.initialSpeed +
                            2 * (std::abs(p.gravity * (y0 - track.sample(end).position.y)) +
                                 p.resistance * end + std::abs(endWork / p.mass))
                      : std::abs(w) + 2 * ds *
                                          (std::abs(force / p.mass) + p.resistance +
                                           p.gravity * std::abs(track.sample(end).tangent.y));
        const double boundaryTolerance =
            std::max(roundoff, 8 * std::numeric_limits<double>::epsilon() * energyScale);
        // Reaching a powered boundary at exactly zero speed can start the next segment.
        // A stop strictly before that boundary must still terminate the run.
        if (halfW > 0 && std::abs(endW) <= boundaryTolerance && segment + 1 < track.segments().size() &&
            end == track.segmentDistances()[segment + 1] &&
            force / p.mass - p.gravity * track.sample(end).tangent.y - p.resistance < 0 &&
            forceAt(segment + 1) / p.mass - p.gravity * track.sample(end).tangent.y - p.resistance > 0) {
            s = end;
            w = 0;
            roundoff = 0;
            work = endWork;
            append(s, w, work);
            continue;
        }
        // A small positive speed is still motion. Only bisect a bracket reaching v^2 <= 0.
        if (halfW <= 0 || endW <= 0) {
            double start = s, startW = w, startWork = work, width = ds / 2;
            if (halfW > 0) {
                start += ds / 2;
                startW = halfW;
                startWork = halfWork;
                append(start, startW, startWork);
            }
            double lo = 0, hi = width;
            for (int i = 0; i < 60; ++i) {
                const double mid = (lo + hi) / 2;
                if (advance(start, startW, startWork, mid, force) > 0)
                    lo = mid;
                else
                    hi = mid;
            }
            const double stop = start + (lo + hi) / 2;
            append(stop, 0, startWork + force * (stop - start));
            // Cannot make it over this hill. Stop here.
            result.status = RunStatus::Stalled;
            result.timeResolutionLimited |= std::abs(result.samples.back().tangentialAcceleration) < 1e-4;
            result.message =
                "The cart runs out of forward speed. Graphs stop here; the remaining track is not traversed.";
            if (result.timeResolutionLimited)
                result.message += " Near equilibrium, the reported time is resolution dependent (the limit "
                                  "may be infinite).";
            return result;
        }
        s = end;
        w = endW;
        work = endWork;
        append(s, w, work);
    }
    result.message = "The cart reaches the end of the track. All graphs describe one forward pass.";
    if (result.timeResolutionLimited)
        result.message += " Travel time is unresolved at the available numeric precision.";
    return result;
}
} // namespace rail
