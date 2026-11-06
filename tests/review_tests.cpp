#include "window.hpp"

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok, const char *message) {
    if (!ok)
        throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char *message) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
} // namespace

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    if (argc < 2) {
        std::cerr << "Usage: review_tests CASES_DIRECTORY [--report-only]\n";
        return 1;
    }
    const bool reportOnly = argc == 3 && QString(argv[2]) == "--report-only";
    const QDir cases(QString::fromLocal8Bit(argv[1]));
    const QString artifactDirectory = qEnvironmentVariable("RAILLAB_REVIEW_OUTPUT_DIR");
    if (!artifactDirectory.isEmpty())
        QDir().mkpath(artifactDirectory);
    const QStringList names{"01-boundary-step-0.15.json",
                            "01-boundary-step-2.json",
                            "02-hill-step-0.01.json",
                            "02-hill-step-0.5.json",
                            "03-drag.json",
                            "04-slow-clock.json"};
    std::cout << std::setprecision(17);
    int failures = 0;
    for (const auto &name : names) {
        try {
            Window window;
            window.openFrom(cases.filePath(name));
            check(window.analysis().has_value(), "project produced no analysis");
            const auto &r = *window.analysis();
            const auto &last = r.samples.back();
            std::cout << name.toStdString() << " | " << rail::statusName(r.status)
                      << " | distance=" << last.distance << " | speed=" << last.speed
                      << " | time=" << last.time << " | motor_work=" << last.propulsionWork
                      << " | time_limited=" << r.timeResolutionLimited << '\n';
            if (!artifactDirectory.isEmpty()) {
                const QString base = artifactDirectory + "/" + name.chopped(5);
                window.exportCsv(base + ".csv");
                window.exportSvg(base + ".svg");
                window.show();
                QApplication::processEvents();
                check(window.grab().save(base + ".png"), "fixture screenshot");
            }
            if (reportOnly)
                continue;
            if (name.startsWith("01-")) {
                check(r.status == rail::RunStatus::Complete, "exact boundary must restart");
                near(last.distance, 9, 1e-10, "powered track end");
                near(last.speed, std::sqrt(5), 1e-10, "powered final speed");
                near(last.time, 4 + 2 * std::sqrt(5), 1e-6, "stop/restart travel time");
                near(last.propulsionWork, 500, 1e-8, "500 J motor work");
            } else if (name.startsWith("02-")) {
                check(r.status == rail::RunStatus::Stalled, "hill must stop at both steps");
                near(last.distance, 1.4813381943, 1e-8, "hill stopping point");
                near(last.speed, 0, 0, "stopped hill speed");
                near(last.propulsionWork, 0, 0, "unpowered hill work");
            } else if (name.startsWith("03-")) {
                check(r.status == rail::RunStatus::Complete, "drag cannot stop at finite distance");
                const double speed = 10 * std::exp(-30.0), time = std::expm1(30.0) / 10;
                near(last.speed, speed, speed * 1e-8, "exponential drag speed");
                near(last.time, time, time * 1e-8, "exponential drag travel time");
            } else {
                check(r.status == rail::RunStatus::Complete, "slow positive speed completes");
                near(last.speed, 1e-13, 1e-26, "slow constant speed");
                near(last.time, 1e14, 10, "slow travel time is 1e14 seconds");
            }
            check(!r.timeResolutionLimited, "these fixtures have resolvable finite times");
            QTemporaryDir output;
            check(output.isValid(), "temporary export directory");
            const auto path = output.filePath("analysis.csv");
            window.exportCsv(path);
            QFile csv(path);
            check(csv.open(QIODevice::ReadOnly), "fixture CSV opens");
            const auto lines = csv.readAll().trimmed().split('\n');
            check(lines.front().endsWith("time_resolution_limited"), "CSV includes time reliability");
            const auto fields = lines.back().split(',');
            near(fields[1].toDouble(), last.time, std::abs(last.time) * 1e-12, "CSV preserves sample time");
            check(fields.back() == "0", "fixture CSV reports a resolved time");
        } catch (const std::exception &e) {
            ++failures;
            std::cerr << "FAIL " << name.toStdString() << ": " << e.what() << '\n';
        }
    }
    return failures ? 1 : 0;
}
