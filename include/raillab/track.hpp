#pragma once

#include <cstddef>
#include <vector>

namespace rail {

struct Vec2 {
    double x{}, y{};
    Vec2 operator+(Vec2 b) const { return {x + b.x, y + b.y}; }
    Vec2 operator-(Vec2 b) const { return {x - b.x, y - b.y}; }
    Vec2 operator*(double k) const { return {x * k, y * k}; }
    Vec2 operator/(double k) const { return {x / k, y / k}; }
};
double dot(Vec2 a, Vec2 b);
double norm(Vec2 a);

struct Bezier {
    Vec2 p0, p1, p2, p3;
    Vec2 position(double u) const;
    Vec2 derivative(double u) const;
    Vec2 secondDerivative(double u) const;
    double arcLength(double begin = 0, double end = 1) const;
};

struct TrackSample {
    Vec2 position, tangent;
    double curvature{}; // Positive toward the left normal (-Ty, Tx).
    std::size_t segment{};
    double u{};
};

// Natural spline, stored as Bezier pieces.
// Chord-length parameter keeps the joins smooth.
class Track {
  public:
    explicit Track(const std::vector<Vec2> &anchors);
    TrackSample sample(double distance) const;
    double length() const { return offsets_.back(); }
    const std::vector<Bezier> &segments() const { return segments_; }
    // Arc distances of the confirmed points, including 0 and the track end.
    const std::vector<double> &segmentDistances() const { return offsets_; }
    const std::vector<double> &criticalDistances() const { return criticalDistances_; }
    // Interior arc distances where a segment's forward tangent has the given Y component.
    std::vector<double> tangentLevelDistances(std::size_t segment, double tangentY) const;

  private:
    std::vector<Bezier> segments_;
    std::vector<std::vector<double>> arcTables_;
    std::vector<double> offsets_{0};
    std::vector<double> criticalDistances_;
};

} // namespace rail
