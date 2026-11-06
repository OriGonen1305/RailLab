#include "raillab/analysis.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
    try {
        if (argc < 2 || std::string(argv[1]) == "--help") {
            std::cout << "RailLab — static Bezier track analysis\n"
                         "Usage: rail_analyze POINTS.csv [--output analysis.csv] [--speed M/S]\n"
                         "       [--mass KG] [--resistance M/S2] [--drag-area M2] [--step M]\n"
                         "Exit codes: 0 = complete, 2 = stalled/cannot start/limit, 1 = invalid input.\n";
            return argc < 2 ? 1 : 0;
        }
        rail::Settings settings;
        std::string output;
        for (int i = 2; i < argc; i += 2) {
            if (i + 1 >= argc)
                throw std::invalid_argument("Missing option value.");
            const std::string option = argv[i], value = argv[i + 1];
            if (option == "--output") {
                output = value;
                continue;
            }
            std::size_t consumed = 0;
            const double number = std::stod(value, &consumed);
            if (consumed != value.size())
                throw std::invalid_argument("Invalid number: " + value);
            if (option == "--speed")
                settings.initialSpeed = number;
            else if (option == "--mass")
                settings.mass = number;
            else if (option == "--resistance")
                settings.resistance = number;
            else if (option == "--drag-area")
                settings.dragArea = number;
            else if (option == "--step")
                settings.step = number;
            else
                throw std::invalid_argument("Unknown option: " + option);
        }
        std::ifstream input(argv[1]);
        if (!input)
            throw std::runtime_error("Could not open points CSV.");
        const rail::Track track(rail::readAnchorsCsv(input));
        const auto result = rail::analyze(track, settings);
        if (!output.empty()) {
            // Different names can still be the same file.
            if (std::filesystem::exists(output) && std::filesystem::equivalent(argv[1], output))
                throw std::invalid_argument("Output CSV must differ from the input points file.");
            std::ofstream file(output);
            if (!file)
                throw std::runtime_error("Could not open output CSV.");
            rail::writeCsv(file, result);
            file.close();
            if (!file)
                throw std::runtime_error("Could not finish writing the analysis CSV.");
        }
        std::cout << "Status: " << rail::statusName(result.status) << "\nTrack length: " << track.length()
                  << " m\nTraversed: " << result.samples.back().distance
                  << " m\nMaximum speed: " << result.maxSpeed
                  << " m/s\nMaximum felt load: " << result.maxFeltG << " g\n";
        if (result.timeResolutionLimited)
            std::cout << "Elapsed time: unresolved (resolution-limited)\n";
        else
            std::cout << "Elapsed time estimate: " << result.samples.back().time << " s\n";
        std::cout << "Energy balance residual: " << result.energyResidual << " J\n" << result.message << '\n';
        return result.status == rail::RunStatus::Complete ? 0 : 2;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
