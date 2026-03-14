#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <iomanip>

#include <Eigen/Dense>

namespace labrob {

struct WBCEntry {
    double time_ms = 0.0;

    double com_x = 0.0;
    double com_y = 0.0;
    double com_z = 0.0;

    double com_x_des = 0.0;
    double com_y_des = 0.0;
    double com_z_des = 0.0;

    double wheel_l_x = 0.0;
    double wheel_l_y = 0.0;
    double wheel_l_z = 0.0;

    double wheel_l_x_des = 0.0;
    double wheel_l_y_des = 0.0;
    double wheel_l_z_des = 0.0;

    double wheel_r_x = 0.0;
    double wheel_r_y = 0.0;
    double wheel_r_z = 0.0;

    double wheel_r_x_des = 0.0;
    double wheel_r_y_des = 0.0;
    double wheel_r_z_des = 0.0;
};

struct MPCEntry {
    double time_ms = 0.0;     // used to name the folder

    Eigen::MatrixXd X;        // rows = horizon steps, cols = state dimension
    Eigen::MatrixXd U;        // rows = horizon steps - 1, cols = input dimension
};

struct TimingEntry {
    long time_mpc_us = 0;
    long time_wbc_us = 0;
    long total_time_us = 0;
};

class Logger {
public:
    Logger() = default;

    explicit Logger(std::size_t reserve_size) {
        reserve(reserve_size);
    }

    void reserve(std::size_t n) {
        wbc_entries_.reserve(n);
        mpc_entries_.reserve(n);
        timing_entries_.reserve(n);
    }

    void clear() {
        wbc_entries_.clear();
        mpc_entries_.clear();
        timing_entries_.clear();
    }

    void log_wbc_data(const WBCEntry& entry) {
        wbc_entries_.emplace_back(entry);
    }

    void log_wbc_data(WBCEntry&& entry) {
        wbc_entries_.emplace_back(std::move(entry));
    }

    void log_mpc_data(const MPCEntry& entry) {
        mpc_entries_.emplace_back(entry);
    }

    void log_mpc_data(MPCEntry&& entry) {
        mpc_entries_.emplace_back(std::move(entry));
    }

    void log_timing_data(const TimingEntry& entry) {
        timing_entries_.emplace_back(entry);
    }

    void log_timing_data(TimingEntry&& entry) {
        timing_entries_.emplace_back(std::move(entry));
    }

    void save_log_data(const std::string& directory = "/tmp") const {
        namespace fs = std::filesystem;
        fs::create_directories(directory);

        auto start_wbc_log = std::chrono::system_clock::now();
        
        save_wbc_logs(directory + "/wbc_log.txt");
        auto end_wbc_log = std::chrono::system_clock::now();
        auto wbc_log_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_wbc_log - start_wbc_log).count();
        std::cout << "WBC logs saved in " << wbc_log_time << " ms" << std::endl;
        
        save_timing_logs(directory + "/timing_log.txt");
        auto end_timing_log = std::chrono::system_clock::now();
        auto timing_log_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_timing_log - end_wbc_log).count();
        std::cout << "timing logs saved in " << timing_log_time << " ms" << std::endl;

        save_mpc_logs(directory + "/mpc_data");
        auto end_mpc_log = std::chrono::system_clock::now();
        auto mpc_log_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_mpc_log - end_timing_log).count();
        std::cout << "MPC logs saved in " << mpc_log_time << " ms" << std::endl;
    }

    std::size_t wbc_size() const { return wbc_entries_.size(); }
    std::size_t mpc_size() const { return mpc_entries_.size(); }
    std::size_t timing_size() const { return timing_entries_.size(); }

private:
    static void save_matrix_txt(const Eigen::MatrixXd& M, const std::filesystem::path& filename) {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename.string());
        }

        for (int i = 0; i < M.rows(); ++i) {
            for (int j = 0; j < M.cols(); ++j) {
                out << M(i, j);
                if (j + 1 < M.cols()) {
                    out << ' ';
                }
            }
            out << '\n';
        }
    }

    void save_wbc_logs(const std::string& filename) const {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        out << "time_ms,"
            << "com_x,com_y,com_z,"
            << "com_x_des,com_y_des,com_z_des,"
            << "wheel_l_x,wheel_l_y,wheel_l_z,"
            << "wheel_l_x_des,wheel_l_y_des,wheel_l_z_des,"
            << "wheel_r_x,wheel_r_y,wheel_r_z,"
            << "wheel_r_x_des,wheel_r_y_des,wheel_r_z_des\n";

        for (const auto& e : wbc_entries_) {
            out << e.time_ms << ","
                << e.com_x << "," << e.com_y << "," << e.com_z << ","
                << e.com_x_des << "," << e.com_y_des << "," << e.com_z_des << ","
                << e.wheel_l_x << "," << e.wheel_l_y << "," << e.wheel_l_z << ","
                << e.wheel_l_x_des << "," << e.wheel_l_y_des << "," << e.wheel_l_z_des << ","
                << e.wheel_r_x << "," << e.wheel_r_y << "," << e.wheel_r_z << ","
                << e.wheel_r_x_des << "," << e.wheel_r_y_des << "," << e.wheel_r_z_des
                << "\n";
        }
    }

    void save_mpc_logs(const std::string& root_dir) const {
        namespace fs = std::filesystem;

        fs::create_directories(root_dir);

        for (const auto& e : mpc_entries_) {
            // Folder name based on t_msec
            long long t_msec = static_cast<long long>(e.time_ms);

            fs::path cycle_dir = fs::path(root_dir) / std::to_string(t_msec);
            fs::create_directories(cycle_dir);

            save_matrix_txt(e.X, cycle_dir / "x.txt");
            save_matrix_txt(e.U, cycle_dir / "u.txt");
        }
    }

    void save_timing_logs(const std::string& filename) const {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        out << "time_mpc_us,time_wbc_us,total_time_us\n";

        for (const auto& e : timing_entries_) {
            out << e.time_mpc_us << ","
                << e.time_wbc_us << ","
                << e.total_time_us
                << "\n";
        }
    }

private:
    std::vector<WBCEntry> wbc_entries_;
    std::vector<MPCEntry> mpc_entries_;
    std::vector<TimingEntry> timing_entries_;
};

} // namespace labrob