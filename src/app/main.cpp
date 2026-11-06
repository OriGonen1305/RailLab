#include "window.hpp"
#include <QApplication>
#include <QDir>
#include <QTimer>
#include <iostream>

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("RailLab");
    QApplication::setApplicationVersion("1.0.0");
    QApplication::setStyle("Fusion");
    try {
        Window window;
        QString screenshot, report;
        const auto args = app.arguments();
        for (int i = 1; i < args.size(); ++i) {
            if (args[i] == "--screenshot" && i + 1 < args.size())
                screenshot = args[++i];
            else if (args[i] == "--report" && i + 1 < args.size())
                report = args[++i];
            else if (args[i] == "--project" && i + 1 < args.size())
                window.openFrom(args[++i]);
            else if (args[i] == "--help") {
                std::cout << "RailLab [--project FILE.json] [--screenshot FILE.png] [--report FILE.svg]\n";
                return 0;
            } else
                throw std::invalid_argument("Unknown or incomplete argument: " + args[i].toStdString());
        }
        window.show();
        if (!screenshot.isEmpty() || !report.isEmpty()) {
            QTimer::singleShot(250, &window, [&] {
                try {
                    if (!screenshot.isEmpty() && !window.grab().save(screenshot))
                        throw std::runtime_error("Could not save the screenshot.");
                    if (!report.isEmpty())
                        window.exportSvg(report);
                    app.exit(0);
                } catch (const std::exception &e) {
                    std::cerr << e.what() << '\n';
                    app.exit(1);
                }
            });
        }
        return app.exec();
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
