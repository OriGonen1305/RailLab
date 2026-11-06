#include "raillab/analysis.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void check(bool ok, const std::string &message) {
    if (!ok)
        throw std::runtime_error(message);
}
void near(double a, double b, double tolerance, const std::string &message) {
    check(std::abs(a - b) <= tolerance, message + ": " + std::to_string(a) + " versus " + std::to_string(b));
}
template <class F> void rejects(F action) {
    bool threw = false;
    try {
        action();
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    check(threw, "invalid input was accepted");
}
rail::Settings ideal(double speed = 4) {
    rail::Settings s;
    s.initialSpeed = speed;
    s.resistance = 0;
    s.dragArea = 0;
    return s;
}
} // namespace

int main() {
    using namespace rail;
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"Bezier endpoints and analytical derivatives",
         [] {
             Bezier b{{0, 2}, {3, 8}, {8, -1}, {12, 4}};
             near(norm(b.position(0) - b.p0), 0, 1e-12, "start");
             near(norm(b.position(1) - b.p3), 0, 1e-12, "end");
             for (double u : {0.1, 0.3, 0.7, 0.9}) {
                 const double h = 1e-5;
                 near(norm((b.position(u + h) - b.position(u - h)) / (2 * h) - b.derivative(u)), 0, 1e-7,
                      "derivative");
                 near(norm((b.derivative(u + h) - b.derivative(u - h)) / (2 * h) - b.secondDerivative(u)), 0,
                      1e-7, "second derivative");
             }
         }},
        {"Arc length and equal-distance sampling on a line",
         [] {
             Track t({{0, 0}, {30, 40}});
             near(t.length(), 50, 1e-9, "length");
             for (int i = 0; i <= 50; ++i) {
                 const auto p = t.sample(i);
                 near(p.position.x, i * 0.6, 1e-8, "x");
                 near(p.position.y, i * 0.8, 1e-8, "y");
                 near(p.curvature, 0, 1e-12, "curvature");
             }
             near(t.sample(-1).position.x, 0, 1e-9, "start clamp");
             near(t.sample(100).position.x, 30, 1e-9, "end clamp");
         }},
        {"Interpolation and curvature continuity at unequal chords",
         [] {
             const std::vector<Vec2> points{{0, 30}, {3, 24}, {40, 2}, {53, 19}, {100, 5}};
             Track t(points);
             double s = 0;
             for (std::size_t i = 0; i < t.segments().size(); ++i) {
                 near(norm(t.sample(s).position - points[i]), 0, 1e-7, "anchor interpolation");
                 s += t.segments()[i].arcLength();
                 if (i + 1 < t.segments().size()) {
                     const auto left = t.sample(s - 1e-6), right = t.sample(s + 1e-6);
                     near(norm(left.tangent - right.tangent), 0, 1e-6, "continuous tangent");
                     near(left.curvature, right.curvature, 1e-6, "continuous curvature");
                 }
             }
             near(norm(t.sample(t.length()).position - points.back()), 0, 1e-7, "last anchor");
         }},
        {"A loop permits non-monotone X",
         [] {
             Track t({{-35, 30}, {-15, 0}, {0, 0}, {12, 12}, {0, 24}, {-12, 12}, {0, 0}, {30, 0}});
             bool leftward = false, upward = false;
             for (double s = 0; s < t.length(); s += 0.1) {
                 const auto p = t.sample(s);
                 check(std::isfinite(p.curvature), "finite loop curvature");
                 leftward |= p.tangent.x < -0.9;
                 upward |= p.tangent.y > 0.9;
             }
             check(leftward && upward, "loop turns through vertical and leftward tangents");
         }},
        {"Horizontal frictionless speed, time and one G",
         [] {
             Track t({{0, 10}, {100, 10}});
             const auto r = analyze(t, ideal(10));
             check(r.status == RunStatus::Complete, "complete");
             near(r.samples.back().speed, 10, 1e-10, "speed");
             near(r.samples.back().time, 10, 1e-8, "time");
             near(r.samples[100].normalG, 1, 1e-10, "normal load");
             near(r.samples[100].acceleration, 0, 1e-10, "acceleration");
             near(r.energyResidual, 0, 1e-7, "energy balance");
         }},
        {"Very slow frictionless motion reaches the end at constant speed",
         [] {
             Track t({{0, 0}, {10, 0}});
             auto p = ideal(1e-7);
             for (double step : {0.01, 0.15, 2.0}) {
                 p.step = step;
                 const auto r = analyze(t, p);
                 check(r.status == RunStatus::Complete, "small positive speed is not a stall");
                 check(!r.timeResolutionLimited, "travel time is resolved");
                 near(r.samples.back().distance, 10, 1e-9, "reaches the track end");
                 near(r.samples.back().time, 10 / p.initialSpeed, 1e-4, "constant-speed travel time");
                 for (const auto &sample : r.samples)
                     near(sample.speed, p.initialSpeed, 1e-19, "speed stays constant");
             }
         }},
        {"Incline agrees with constant-acceleration mechanics",
         [] {
             Track t({{0, 40}, {30, 0}});
             const auto p = ideal(2);
             const auto r = analyze(t, p);
             const double v = std::sqrt(4 + 2 * p.gravity * 40);
             near(r.samples.back().speed, v, 1e-8, "terminal speed");
             near(r.samples.back().time, 100 / (2 + v), 1e-8, "travel time");
             near(r.samples[10].tangentialAcceleration, p.gravity * 0.8, 1e-9, "downhill gravity");
             near(r.samples[10].normalG, 0.6, 1e-10, "incline load");
         }},
        {"Horizontal drag and resistance match the exact solution",
         [] {
             Track t({{0, 0}, {100, 0}});
             auto p = ideal(20);
             p.dragArea = 2;
             p.resistance = 0.1;
             const auto r = analyze(t, p);
             const double b = p.airDensity * p.dragArea / (2 * p.mass);
             const double expected = (400 + p.resistance / b) * std::exp(-2 * b * 100) - p.resistance / b;
             near(r.samples.back().speed, std::sqrt(expected), 1e-8, "drag solution");
             check(r.samples.back().dissipatedEnergy > 0, "positive energy loss");
             near(r.energyResidual, 0, 1e-7, "energy balance with drag");
         }},
        {"Strong drag remains stable at the largest user step",
         [] {
             Track t({{0, 0}, {0.25, 0}});
             auto p = ideal(20);
             p.mass = 1;
             p.dragArea = 20;
             p.step = 2;
             const auto r = analyze(t, p);
             const double b = p.airDensity * p.dragArea / (2 * p.mass);
             check(r.status == RunStatus::Complete, "strong drag does not cause a spurious stop");
             near(r.samples.back().speed, 20 * std::exp(-b * 0.25), 1e-5, "strong drag speed");
         }},
        {"Uphill stopping event matches analytical distance and time",
         [] {
             Track t({{0, 0}, {60, 80}});
             auto p = ideal(10);
             // Exercise a stop in either half of an integration step.
             for (double step : {0.7, 0.2}) {
                 p.step = step;
                 const auto r = analyze(t, p);
                 check(r.status == RunStatus::Stalled, "stalled");
                 near(r.samples.back().distance, 100 / (2 * p.gravity * 0.8), 1e-8, "stopping distance");
                 near(r.samples.back().time, 10 / (p.gravity * 0.8), 1e-7, "stopping time");
                 near(r.samples.back().speed, 0, 1e-12, "zero final speed");
                 near(r.energyResidual, 0, 1e-6, "stop energy");
             }
         }},
        {"Rest distinguishes downhill start from no forward motion",
         [] {
             check(analyze(Track({{0, 0}, {10, 0}}), ideal(0)).status == RunStatus::CannotStart,
                   "level rest");
             check(analyze(Track({{0, 0}, {10, 10}}), ideal(0)).status == RunStatus::CannotStart,
                   "uphill rest");
             const auto r = analyze(Track({{0, 10}, {10, 0}}), ideal(0));
             check(r.status == RunStatus::Complete, "downhill rest");
             near(r.samples.back().speed, std::sqrt(20 * ideal().gravity), 1e-8, "release speed");
         }},
        {"Conservative curved-track energy and signed normal force",
         [] {
             Track t({{0, 40}, {20, 10}, {40, 2}, {60, 15}, {90, 5}});
             const auto p = ideal(8);
             const auto r = analyze(t, p);
             for (const auto &s : r.samples) {
                 near(s.speed * s.speed, 64 + 2 * p.gravity * (40 - s.track.position.y), 1e-8,
                      "energy speed");
                 near(s.railForce / p.mass,
                      s.speed * s.speed * s.track.curvature + p.gravity * s.track.tangent.x, 1e-9,
                      "rail force includes gravity");
             }
         }},
        {"Numerical convergence with drag on a curved track",
         [] {
             Track t({{0, 40}, {20, 5}, {40, 1}, {65, 20}, {90, 0}});
             auto p = ideal(8);
             p.dragArea = 1.5;
             p.resistance = 0.05;
             p.step = 0.5;
             const auto coarse = analyze(t, p);
             p.step = 0.05;
             const auto fine = analyze(t, p);
             check(coarse.status == RunStatus::Complete && fine.status == RunStatus::Complete,
                   "complete runs");
             near(coarse.samples.back().speed, fine.samples.back().speed, 1e-5, "speed convergence");
             near(coarse.samples.back().time, fine.samples.back().time, 0.005, "time convergence");
         }},
        {"100 N over 5 m adds 500 J from rest, followed by an unpowered coast",
         [] {
             Track t({{0, 0}, {5, 0}, {10, 0}});
             auto p = ideal(0);
             p.mass = 100;
             for (double step : {0.01, 0.7, 2.0}) {
                 p.step = step;
                 const auto r = analyze(t, p, {100, 0});
                 check(r.status == RunStatus::Complete, "motor starts the cart");
                 near(r.samples.back().propulsionWork, 500, 1e-8, "motor work");
                 near(r.samples.back().speed, std::sqrt(10), 1e-10, "final speed 3.16227766 m/s");
                 near(r.samples.back().time, 15 / std::sqrt(10), 1e-8, "acceleration and coast time");
                 near(r.energyResidual, 0, 1e-8, "energy balance includes motor work");
                 bool boundary = false;
                 for (const auto &s : r.samples) {
                     const bool powered = s.distance < t.segmentDistances()[1];
                     boundary |= s.distance == t.segmentDistances()[1];
                     near(s.forwardForce, powered ? 100 : 0, 0, "force switches at Point 2");
                     near(s.tangentialAcceleration, powered ? 1 : 0, 1e-12, "local acceleration");
                     near(s.propulsionWork, 100 * std::min(s.distance, 5.0), 1e-8, "work along segment");
                     near(s.normalG, 1, 1e-12, "propulsion has no normal component");
                     near(s.feltG, std::hypot(1, powered ? 1 / p.gravity : 0), 1e-12, "felt propulsion");
                     if (!powered)
                         near(s.speed, std::sqrt(10), 1e-10, "unpowered speed stays constant");
                 }
                 check(boundary, "sample lands exactly at force boundary");
             }
         }},
        {"Adjacent forces switch exactly even when segments are shorter than the step",
         [] {
             Track t({{0, 0}, {0.6, 0}, {1.4, 0}, {3, 0}});
             auto p = ideal(0);
             p.mass = 100;
             p.step = 2;
             const std::vector<double> forces{100, 0, 250};
             const auto r = analyze(t, p, forces);
             check(r.status == RunStatus::Complete, "complete");
             near(r.samples.back().propulsionWork, 460, 1e-9, "sum of three independent segment works");
             near(r.samples.back().speed, std::sqrt(9.2), 1e-10, "different adjacent forces");
             for (std::size_t i = 1; i < t.segmentDistances().size(); ++i) {
                 const double boundary = t.segmentDistances()[i];
                 const auto it = std::find_if(r.samples.begin(), r.samples.end(),
                                              [boundary](const auto &s) { return s.distance == boundary; });
                 check(it != r.samples.end(), "exact boundary sample");
                 near(it->forwardForce, forces[std::min(i, forces.size() - 1)], 0, "right-hand force");
             }
         }},
        {"Propulsion and drag match the piecewise analytical solution",
         [] {
             Track t({{0, 0}, {0.6, 0}, {1.4, 0}, {3, 0}});
             auto p = ideal(2);
             p.mass = 100;
             p.step = 2;
             p.dragArea = 2;
             p.resistance = 0.1;
             const std::vector<double> forces{100, 0, 250};
             const auto r = analyze(t, p, forces);
             double w = 4;
             const double b = p.airDensity * p.dragArea / (2 * p.mass);
             for (std::size_t i = 0; i < forces.size(); ++i) {
                 const double length = t.segmentDistances()[i + 1] - t.segmentDistances()[i];
                 const double equilibrium = (forces[i] / p.mass - p.resistance) / b;
                 w = equilibrium + (w - equilibrium) * std::exp(-2 * b * length);
             }
             near(r.samples.back().speed, std::sqrt(w), 2e-9, "piecewise drag solution");
             near(r.samples.back().dissipatedEnergy, 200 + 460 - p.mass * w / 2, 1e-6, "dissipation");
             near(r.energyResidual, 0, 1e-8, "work plus losses");
         }},
        {"Propulsion follows a curved local tangent and uses arc length",
         [] {
             Track t({{-35, 30}, {-15, 0}, {0, 0}, {12, 12}, {0, 24}, {-12, 12}, {0, 0}, {30, 0}});
             auto p = ideal(40);
             p.mass = 100;
             const std::vector<double> forces{100, 0, 250, 350, 150, 200, 0};
             const auto r = analyze(t, p, forces);
             check(r.status == RunStatus::Complete, "powered loop complete");
             bool leftward = false, upward = false;
             for (const auto &s : r.samples) {
                 double work = 0;
                 for (std::size_t i = 0; i < forces.size(); ++i)
                     work += forces[i] * std::clamp(s.distance - t.segmentDistances()[i], 0.0,
                                                    t.segmentDistances()[i + 1] - t.segmentDistances()[i]);
                 const double expectedW =
                     1600 + 2 * p.gravity * (30 - s.track.position.y) + 2 * work / p.mass;
                 const double drive = forces[s.track.segment] / p.mass;
                 near(s.propulsionWork, work, 1e-7, "arc length work");
                 near(s.speed * s.speed, expectedW, 1e-8, "curved energy");
                 near(s.tangentialAcceleration, drive - p.gravity * s.track.tangent.y, 1e-10,
                      "forward tangent");
                 near(s.railForce, p.mass * (expectedW * s.track.curvature + p.gravity * s.track.tangent.x),
                      1e-7, "normal load uses propelled speed");
                 near(s.feltG, std::hypot(s.normalG, drive / p.gravity), 1e-10, "felt load includes drive");
                 leftward |= drive > 0 && s.track.tangent.x < -0.9;
                 upward |= drive > 0 && s.track.tangent.y > 0.9;
             }
             check(leftward && upward, "powered tangent turns through vertical and leftward directions");
         }},
        {"Unreachable powered segments never contribute work or prevent a stop",
         [] {
             Track t({{0, 0}, {5, 0}, {10, 0}});
             auto p = ideal(2);
             p.resistance = 1;
             p.mass = 100;
             p.step = 2;
             const auto r = analyze(t, p, {0, 1e6});
             check(r.status == RunStatus::Stalled, "stops before motor");
             near(r.samples.back().distance, 2, 1e-9, "stop at 2 m");
             near(r.samples.back().time, 2, 1e-8, "stop time");
             for (const auto &s : r.samples) {
                 near(s.propulsionWork, 0, 0, "no future motor work");
                 near(s.forwardForce, 0, 0, "no future force");
             }
             p.initialSpeed = 0;
             check(analyze(t, p, {0, 1e6}).status == RunStatus::CannotStart,
                   "remote motor cannot start cart");
         }},
        {"Stopping inside a powered uphill segment counts only the traveled work",
         [] {
             Track t({{0, 0}, {60, 80}});
             auto p = ideal(10);
             p.mass = 100;
             const double deceleration = p.gravity * 0.8 - 1;
             for (double step : {0.7, 2.0}) {
                 p.step = step;
                 const auto r = analyze(t, p, {100});
                 check(r.status == RunStatus::Stalled, "motor cannot overcome hill");
                 near(r.samples.back().distance, 100 / (2 * deceleration), 1e-8, "powered stop distance");
                 near(r.samples.back().time, 10 / deceleration, 1e-7, "powered stop time");
                 near(r.samples.back().propulsionWork, 100 * r.samples.back().distance, 1e-8,
                      "partial motor work");
                 near(r.energyResidual, 0, 1e-6, "stopping energy balance");
             }
         }},
        {"A motor starts at a reached zero-speed boundary, but never before it",
         [] {
             auto p = ideal(2);
             p.mass = 100;
             p.resistance = 1;
             for (double step : {0.7, 2.0}) {
                 p.step = step;
                 const auto reached = analyze(Track({{0, 0}, {2, 0}, {7, 0}}), p, {0, 200});
                 check(reached.status == RunStatus::Complete, "starts at reached boundary");
                 near(reached.samples.back().speed, std::sqrt(10), 1e-8, "powered restart speed");
                 near(reached.samples.back().propulsionWork, 1000, 1e-8, "only the next segment is powered");
                 near(reached.samples.back().time, 2 + std::sqrt(10), 1e-7, "stop and restart time");
                 const auto missed = analyze(Track({{0, 0}, {2.0001, 0}, {7, 0}}), p, {0, 200});
                 check(missed.status == RunStatus::Stalled, "stops just before motor");
                 near(missed.samples.back().propulsionWork, 0, 0, "no work across the gap");
             }
         }},
        {"A small positive force can start a frictionless cart",
         [] {
             auto p = ideal(0);
             p.mass = 100;
             const auto r = analyze(Track({{0, 0}, {5, 0}}), p, {1e-9});
             check(r.status == RunStatus::Complete, "small positive acceleration is not zero");
             near(r.samples.back().speed, 1e-5, 1e-15, "small-force energy speed");
         }},
        {"A powered curve cannot skip a narrow energy barrier between samples",
         [] {
             Track t({{0, 0}, {1, 1}, {2, 0}});
             auto p = ideal(0.01);
             p.mass = 1;
             p.gravity = 10;
             p.step = 0.01;
             const auto fine = analyze(t, p, {8.3, 10000});
             p.step = 2;
             const auto coarse = analyze(t, p, {8.3, 10000});
             check(fine.status == RunStatus::Stalled && coarse.status == RunStatus::Stalled,
                   "stop before an unreachable motor even at the largest step");
             check(coarse.samples.back().distance < 0.1, "barrier is near the start, not at the crest");
             near(coarse.samples.back().distance, fine.samples.back().distance, 1e-8,
                  "barrier stop convergence");
             near(coarse.samples.back().propulsionWork, 8.3 * coarse.samples.back().distance, 1e-8,
                  "future 10000 N segment does no work");
         }},
        {"Explicit zero forces preserve all unpowered sample outputs",
         [] {
             Track t({{0, 40}, {20, 5}, {40, 1}, {65, 20}, {90, 0}});
             Settings p;
             std::ostringstream implicit, explicitZero;
             writeCsv(implicit, analyze(t, p));
             writeCsv(explicitZero, analyze(t, p, {0, 0, 0, 0}));
             check(implicit.str() == explicitZero.str(), "zero-force and default runs are identical");
             check(analyze(Track({{0, 1e-12}, {10, 0}}), ideal(0), {0}).status == RunStatus::CannotStart,
                   "legacy unpowered start tolerance is unchanged");
         }},
        {"Review: boundary restart is invariant under step changes and rejects a real gap",
         [] {
             auto p = ideal(2);
             p.mass = 100;
             p.gravity = 10;
             p.resistance = 0.5;
             for (double step : {0.01, 0.15, 0.7, 2.0}) {
                 p.step = step;
                 const auto reached = analyze(Track({{0, 0}, {4, 0}, {9, 0}}), p, {0, 100});
                 check(reached.status == RunStatus::Complete, "reached motor must restart");
                 near(reached.samples.back().speed, std::sqrt(5), 1e-10, "boundary final speed");
                 near(reached.samples.back().time, 4 + 2 * std::sqrt(5), 1e-6, "boundary time");
                 near(reached.samples.back().propulsionWork, 500, 1e-8, "reached motor work");
                 for (double gap : {1e-12, 1e-9, 1e-7, 1e-4}) {
                     const auto missed = analyze(Track({{0, 0}, {4 + gap, 0}, {9, 0}}), p, {0, 100});
                     check(missed.status == RunStatus::Stalled, "a real gap cannot be snapped to the motor");
                     near(missed.samples.back().distance, 4, 1e-11, "stopping distance before motor");
                     near(missed.samples.back().propulsionWork, 0, 0, "unreached motor work");
                 }
             }
         }},
        {"Review: resistance shifts an unpowered hill's energy barrier",
         [] {
             Track t({{0, 0}, {1, 1}, {2, 0}});
             auto p = ideal(5.09);
             p.gravity = 10;
             p.resistance = 2;
             for (double step : {0.01, 0.15, 0.5, 2.0}) {
                 p.step = step;
                 const auto r = analyze(t, p);
                 check(r.status == RunStatus::Stalled, "unpowered hill is unreachable at every step");
                 near(r.samples.back().distance, 1.4813381943, 1e-8, "review stopping point");
                 near(10 * r.samples.back().track.position.y + 2 * r.samples.back().distance, 5.09 * 5.09 / 2,
                      1e-10, "effective potential equals initial specific energy");
             }
         }},
        {"Review: strong drag preserves tiny kinetic energy and the analytical travel time",
         [] {
             auto p = ideal(10);
             p.mass = 1;
             p.gravity = 10;
             p.dragArea = 2;
             p.airDensity = 1;
             for (double step : {0.01, 0.15, 2.0}) {
                 p.step = step;
                 const auto r = analyze(Track({{0, 0}, {30, 0}}), p);
                 check(r.status == RunStatus::Complete, "quadratic drag has no finite-distance stop");
                 const double speed = 10 * std::exp(-30.0), time = std::expm1(30.0) / 10;
                 near(r.samples.back().speed / speed, 1, 1e-8, "tiny exponential drag speed");
                 near(r.samples.back().time / time, 1, 1e-8, "exponential drag time");
                 check(!r.timeResolutionLimited, "finite drag time is resolved");
                 near(r.energyResidual, 0, 1e-8, "strong-drag accounting");
             }
         }},
        {"Review: very slow positive uniform motion has a nonzero finite travel time",
         [] {
             const auto r = analyze(Track({{0, 0}, {10, 0}}), ideal(1e-13));
             check(r.status == RunStatus::Complete, "slow motion completes");
             check(!r.timeResolutionLimited, "positive constant speed has a resolved time");
             near(r.samples.back().time / 1e14, 1, 1e-13, "10 m at 1e-13 m/s takes 1e14 s");
             for (const auto &s : r.samples) {
                 near(s.speed / 1e-13, 1, 1e-13, "tiny speed remains constant");
                 near(s.time / 1e14, s.distance / 10, 1e-13, "every timestamp is distance / speed");
             }
         }},
        {"Curved propulsion with drag agrees with an independent fine-step RK4 reference",
         [] {
             Track t({{0, 40}, {20, 5}, {40, 1}, {65, 20}, {90, 0}});
             auto p = ideal(100);
             p.mass = 100;
             p.dragArea = 2;
             p.resistance = 0.1;
             p.step = 0.5;
             const std::vector<double> forces{50, 300, 0, 200};
             const auto r = analyze(t, p, forces);
             check(r.status == RunStatus::Complete, "reference track completes");
             const double drag = p.airDensity * p.dragArea / (2 * p.mass);
             double s = 0, w = p.initialSpeed * p.initialSpeed;
             while (s < t.length()) {
                 const auto segment = t.sample(s).segment;
                 const double end = std::min(s + 0.005, t.segmentDistances()[segment + 1]);
                 const double h = end - s;
                 const auto derivative = [&](double distance, double speedSquared) {
                     return 2 * (forces[segment] / p.mass - p.resistance -
                                 p.gravity * t.sample(distance).tangent.y - drag * speedSquared);
                 };
                 const double k1 = derivative(s, w);
                 const double k2 = derivative(s + h / 2, w + h * k1 / 2);
                 const double k3 = derivative(s + h / 2, w + h * k2 / 2);
                 const double k4 = derivative(end, w + h * k3);
                 w += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
                 check(w > 0, "independent reference remains reachable");
                 s = end;
             }
             near(r.samples.back().speed, std::sqrt(w), 1e-7, "independent curved-drag speed");
         }},
        {"Braking stops at 2 m and removes only 200 J without reversing",
         [] {
             auto p = ideal(2);
             p.mass = 100;
             for (double step : {0.01, 0.15, 2.0}) {
                 p.step = step;
                 const auto r = analyze(Track({{0, 0}, {5, 0}, {10, 0}}), p, {-100, 250});
                 check(r.status == RunStatus::Stalled, "brake stops before the next motor");
                 near(r.samples.back().distance, 2, 1e-10, "braking stopping distance");
                 near(r.samples.back().time, 2, 1e-7, "braking stopping time");
                 near(r.samples.back().speed, 0, 0, "stopped without rollback");
                 near(r.samples.back().propulsionWork, -200, 1e-8, "partial negative work");
                 near(r.energyResidual, 0, 1e-8, "signed work energy balance");
                 for (const auto &s : r.samples) {
                     near(s.forwardForce, -100, 0, "unreached propulsion never applies");
                     near(s.propulsionWork, -100 * s.distance, 1e-8, "negative work along travel");
                     near(s.tangentialAcceleration, -1, 1e-12, "braking acceleration");
                     near(s.feltG, std::hypot(1, 1 / p.gravity), 1e-12, "felt braking load");
                     near(s.dissipatedEnergy, 0, 0, "braking work is not counted again as drag loss");
                 }
             }
             p.initialSpeed = 0;
             const auto rest = analyze(Track({{0, 0}, {5, 0}}), p, {-100});
             check(rest.status == RunStatus::CannotStart, "brake cannot launch a resting cart backward");
             near(rest.samples.back().propulsionWork, 0, 0, "no work while stationary");
         }},
        {"Short braking segment hands off to an unpowered coast at positive speed",
         [] {
             auto p = ideal(2);
             p.mass = 100;
             for (double step : {0.01, 0.15, 2.0}) {
                 p.step = step;
                 const auto r = analyze(Track({{0, 0}, {1, 0}, {4, 0}}), p, {-100, 0});
                 check(r.status == RunStatus::Complete, "short brake leaves enough kinetic energy");
                 near(r.samples.back().speed, std::sqrt(2), 1e-10, "coasting speed");
                 near(r.samples.back().propulsionWork, -100, 1e-8, "only 1 m of braking work");
                 near(r.samples.back().time, 2 - std::sqrt(2) + 3 / std::sqrt(2), 1e-8, "brake/coast time");
                 bool boundary = false;
                 for (const auto &s : r.samples) {
                     boundary |= s.distance == 1;
                     if (s.distance >= 1) {
                         near(s.forwardForce, 0, 0, "brake switches off at the exact endpoint");
                         near(s.speed, std::sqrt(2), 1e-10, "coast speed remains constant");
                         near(s.tangentialAcceleration, 0, 0, "no coasting acceleration");
                     }
                 }
                 check(boundary, "samples include the braking endpoint");
             }
         }},
        {"Braking with drag matches the analytical stopping distance and work",
         [] {
             auto p = ideal(2);
             p.mass = 100;
             p.dragArea = 20;
             p.airDensity = 1;
             const double b = 0.1, stop = std::log1p(b * 4) / (2 * b);
             for (double step : {0.01, 0.15, 2.0}) {
                 p.step = step;
                 const auto r = analyze(Track({{0, 0}, {5, 0}}), p, {-100});
                 check(r.status == RunStatus::Stalled, "brake and drag stop the cart");
                 near(r.samples.back().distance, stop, 1e-9, "analytical brake/drag distance");
                 near(r.samples.back().propulsionWork, -100 * stop, 1e-7, "brake/drag signed work");
                 near(r.samples.back().dissipatedEnergy, 200 - 100 * stop, 1e-7, "drag loss excludes brake");
             }
         }},
        {"Braking on a curve follows the tangent and detects the effective energy barrier",
         [] {
             const Track t({{0, 0}, {1, 1}, {2, 0}});
             auto p = ideal(5.09);
             p.mass = 100;
             p.gravity = 10;
             for (double step : {0.01, 0.5, 2.0}) {
                 p.step = step;
                 const auto r = analyze(t, p, {-200, -200});
                 check(r.status == RunStatus::Stalled, "braking hill cannot be skipped");
                 near(r.samples.back().distance, 1.4813381943, 1e-8, "braking effective potential barrier");
                 for (const auto &s : r.samples) {
                     near(s.tangentialAcceleration, -2 - 10 * s.track.tangent.y, 1e-12, "local tangent");
                     near(s.propulsionWork, -200 * s.distance, 1e-8, "curved arc length work");
                     near(s.kineticEnergy + s.potentialEnergy - s.propulsionWork, 100 * 5.09 * 5.09 / 2, 1e-7,
                          "curved signed work balance");
                 }
             }
         }},
        {"A brake can hand off at zero speed to a reached motor but cannot bridge a gap",
         [] {
             auto p = ideal(2);
             p.mass = 100;
             p.airDensity = 1;
             for (double dragArea : {0.0, 20.0}) {
                 p.dragArea = dragArea;
                 const double b = dragArea / 200;
                 const double stop = b > 0 ? std::log1p(b * 4) / (2 * b) : 2;
                 for (double step : {0.01, 0.15, 2.0}) {
                     p.step = step;
                     const auto reached = analyze(Track({{0, 0}, {stop, 0}, {stop + 1, 0}}), p, {-100, 100});
                     check(reached.status == RunStatus::Complete,
                           "motor restarts at braking boundary, dragArea=" + std::to_string(dragArea) +
                               ", step=" + std::to_string(step));
                     near(reached.samples.back().propulsionWork, -100 * stop + 100, 1e-8,
                          "braking and propulsion net work");
                     const auto missed =
                         analyze(Track({{0, 0}, {stop + 1e-12, 0}, {stop + 1, 0}}), p, {-100, 100});
                     check(missed.status == RunStatus::Stalled, "motor cannot reach across a gap");
                     near(missed.samples.back().propulsionWork, -100 * stop, 1e-7, "braking only before gap");
                 }
             }
         }},
        {"Invalid segment forces are rejected",
         [] {
             Track t({{0, 0}, {5, 0}, {10, 0}});
             for (const auto &forces :
                  std::vector<std::vector<double>>{{1},
                                                   {1, 2, 3},
                                                   {-std::numeric_limits<double>::infinity(), 0},
                                                   {0, std::nan("")},
                                                   {std::numeric_limits<double>::infinity(), 0}})
                 rejects([&] { analyze(t, ideal(), forces); });
         }},
        {"Invalid geometry and settings rejected",
         [] {
             rejects([] { Track t({{0, 0}}); });
             rejects([] { Track t({{0, 0}, {0, 0}}); });
             rejects([] { Track t({{0, 0}, {1, 0}, {0, 0}}); });
             rejects([] { Track t({{0, 0}, {std::numeric_limits<double>::infinity(), 2}}); });
             rejects([] {
                 auto p = ideal();
                 p.mass = 0;
                 validate(p);
             });
             rejects([] {
                 auto p = ideal();
                 p.step = 0;
                 validate(p);
             });
             rejects([] {
                 auto p = ideal();
                 p.initialSpeed = std::nan("");
                 validate(p);
             });
         }},
        {"CSV parses strict coordinates and exports physical units", [] {
             std::istringstream in("x,y\n0,12\n10,4\n");
             const auto points = readAnchorsCsv(in);
             check(points.size() == 2 && points[1].y == 4, "read points");
             std::ostringstream out;
             writeCsv(out, analyze(Track(points), ideal()));
             check(out.str().find("rail_force_N") != std::string::npos, "units in header");
             check(out.str().find("forward_force_N,propulsion_work_J") != std::string::npos,
                   "propulsion units");
             rejects([] {
                 std::istringstream bad("0,1,2");
                 readAnchorsCsv(bad);
             });
             rejects([] {
                 std::istringstream bad("0,1junk");
                 readAnchorsCsv(bad);
             });
         }}};
    int failures = 0;
    for (const auto &[name, test] : tests) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception &e) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << e.what() << '\n';
        }
    }
    std::cout << tests.size() - static_cast<std::size_t>(failures) << '/' << tests.size()
              << " checks passed.\n";
    return failures ? 1 : 0;
}
