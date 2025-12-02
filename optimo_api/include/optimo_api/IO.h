#pragma once

#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace optimo::utils {
    // A helper function that reads a trajectory file into a vector of vectors
    // The function will choose which method to use based on the file extension
    // Currently, .txt, and .bin are supported
    // For a file that has N waypoints, each row will have 7 values (for 7 joints)
    // Each joint value is separated by either "," or " " for .txt files
    // For .bin files, one joint waypoint is stored as 7 consecutive doubles
    std::vector<std::vector<double>> read_trajectory_file(const std::filesystem::path& filepath) {
        std::vector<std::vector<double>> trajectory;
        const std::string ext = filepath.extension().string();

        if (ext == ".txt") {
            std::ifstream file(filepath);
            if (!file.is_open())
                throw std::runtime_error("Failed to open trajectory file: " + filepath.string());

            std::string line;
            while (std::getline(file, line)) {
                if (line.empty()) continue;

                // Replace commas with spaces
                std::replace(line.begin(), line.end(), ',', ' ');

                std::istringstream iss(line);
                std::vector<double> waypoint;
                double value;
                while (iss >> value) {
                    waypoint.push_back(value);
                }

                if (!waypoint.empty())
                    trajectory.push_back(std::move(waypoint));
            }
        }

        else if (ext == ".bin") {
            std::ifstream file(filepath, std::ios::binary);
            if (!file.is_open())
                throw std::runtime_error("Failed to open binary trajectory file: " + filepath.string());

            constexpr size_t num_joints = 7;
            double buffer[num_joints];
            while (file.read(reinterpret_cast<char*>(buffer), sizeof(buffer))) {
                std::vector<double> waypoint(buffer, buffer + num_joints);
                trajectory.push_back(std::move(waypoint));
            }
        }

        else {
            throw std::runtime_error("Unsupported trajectory file extension: " + ext);
        }
        RCLCPP_INFO(rclcpp::get_logger("optimo_api::IO"), "Read %zu waypoints from trajectory file.", trajectory.size());

        return trajectory;
    }
}