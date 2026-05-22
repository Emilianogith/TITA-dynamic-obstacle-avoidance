#pragma once

#include <vector>
#include <string>
#include <array>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <iomanip>

#include <Eigen/Dense>

namespace labrob {

struct GenericTask {
    
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;

    double px_des = 0.0;
    double py_des = 0.0;
    double pz_des = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

    double vx_des = 0.0;
    double vy_des = 0.0;
    double vz_des = 0.0;

    double ax_des = 0.0;
    double ay_des = 0.0;
    double az_des = 0.0;
};

struct WBCJointData {
    std::array<double, 8> pos    = {};
    std::array<double, 8> vel    = {};
    std::array<double, 8> effort = {};
};

struct WBCSolution {
    WBCJointData joints;

    // aggiungi contct forces, ma non si sa il numero esatto, dipende dai contatti
    
};

struct WBCEntry {
    double time_ms = 0.0;

    GenericTask com;
    GenericTask wheel_l;
    GenericTask wheel_r;
    GenericTask torso;
    WBCSolution solution;
    WBCJointData feedback;
};

struct MPCEntry {
    double time_ms = 0.0;     // used to name the folder

    Eigen::MatrixXd X;        // rows = horizon steps, cols = state dimension
    Eigen::MatrixXd U;        // rows = horizon steps - 1, cols = input dimension
};

struct TimingEntry {
    long time_mpc_us    = 0;
    long time_wbc_us    = 0;
    long time_filter_us = 0;
    long total_time_us  = 0;
};

struct ImuEntry {
    double time_s = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double wx = 0.0, wy = 0.0, wz = 0.0;
    double ax = 0.0, ay = 0.0, az = 0.0;
};

struct JointStateEntry {
    double time_stamp_s = 0.0;   // from message header
    double time_now_s   = 0.0;   // from ros clock
    WBCJointData joints;
};

struct TauCommandedEntry {
    double time_s = 0.0;
    std::array<double, 8> tau = {};
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
        imu_entries_.reserve(n);
        joint_entries_.reserve(n);
        tau_entries_.reserve(n);
    }

    void clear() {
        wbc_entries_.clear();
        mpc_entries_.clear();
        timing_entries_.clear();
        imu_entries_.clear();
        joint_entries_.clear();
        tau_entries_.clear();
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

    void set_joint_names(std::vector<std::string> names) {
        joint_names_ = std::move(names);
    }

    void log_imu_data(const ImuEntry& entry) {
        imu_entries_.emplace_back(entry);
    }

    void log_imu_data(ImuEntry&& entry) {
        imu_entries_.emplace_back(std::move(entry));
    }

    void log_joint_state_data(const JointStateEntry& entry) {
        joint_entries_.emplace_back(entry);
    }

    void log_joint_state_data(JointStateEntry&& entry) {
        joint_entries_.emplace_back(std::move(entry));
    }

    void log_tau_commanded_data(const TauCommandedEntry& entry) {
        tau_entries_.emplace_back(entry);
    }

    void log_tau_commanded_data(TauCommandedEntry&& entry) {
        tau_entries_.emplace_back(std::move(entry));
    }

    void save_feedback_logs(const std::string& directory = "/tmp") const {
        namespace fs = std::filesystem;
        fs::create_directories(directory);

        save_imu_logs(directory + "/imu_log.txt");
        std::cout << "IMU logs saved" << std::endl;

        save_joint_state_logs(directory + "/joint_state_log.txt");
        std::cout << "Joint state logs saved" << std::endl;

        save_tau_commanded_logs(directory + "/tau_commanded_log.txt");
        std::cout << "Tau commanded logs saved" << std::endl;
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

        save_imu_logs(directory + "/imu_log.txt");
        auto end_imu_log = std::chrono::system_clock::now();
        auto imu_log_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_imu_log - end_mpc_log).count();
        std::cout << "IMU logs saved in " << imu_log_time << " ms" << std::endl;

        save_joint_state_logs(directory + "/joint_state_log.txt");
        auto end_js_log = std::chrono::system_clock::now();
        auto js_log_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_js_log - end_imu_log).count();
        std::cout << "Joint state logs saved in " << js_log_time << " ms" << std::endl;

        save_tau_commanded_logs(directory + "/tau_commanded_log.txt");
        auto end_tau_log = std::chrono::system_clock::now();
        auto tau_log_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_tau_log - end_js_log).count();
        std::cout << "Tau commanded logs saved in " << tau_log_time << " ms" << std::endl;
    }

    std::size_t wbc_size()         const { return wbc_entries_.size(); }
    std::size_t mpc_size()         const { return mpc_entries_.size(); }
    std::size_t timing_size()      const { return timing_entries_.size(); }
    std::size_t imu_size()         const { return imu_entries_.size(); }
    std::size_t joint_state_size() const { return joint_entries_.size(); }
    std::size_t tau_commanded_size() const { return tau_entries_.size(); }

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

        auto task_header = [&](const std::string& p) {
            out << p << "_x,"  << p << "_y,"  << p << "_z,"
                << p << "_x_des," << p << "_y_des," << p << "_z_des,"
                << p << "_vx," << p << "_vy," << p << "_vz,"
                << p << "_vx_des," << p << "_vy_des," << p << "_vz_des,"
                << p << "_ax_des," << p << "_ay_des," << p << "_az_des";
        };

        out << "time_ms,";
        task_header("com");     out << ",";
        task_header("wheel_l"); out << ",";
        task_header("wheel_r"); out << ",";
        task_header("torso");
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : ("j" + std::to_string(i));
            out << ",sol_" << n << "_pos";
        }
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : ("j" + std::to_string(i));
            out << ",sol_" << n << "_vel";
        }
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : ("j" + std::to_string(i));
            out << ",sol_" << n << "_tau";
        }
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : ("j" + std::to_string(i));
            out << ",fb_" << n << "_pos";
        }
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : ("j" + std::to_string(i));
            out << ",fb_" << n << "_vel";
        }
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size())) ? joint_names_[i] : ("j" + std::to_string(i));
            out << ",fb_" << n << "_effort";
        }
        out << "\n";

        out << std::fixed << std::setprecision(9);

        auto write_task = [&](const GenericTask& t) {
            out << t.px     << "," << t.py     << "," << t.pz     << ","
                << t.px_des << "," << t.py_des << "," << t.pz_des << ","
                << t.vx     << "," << t.vy     << "," << t.vz     << ","
                << t.vx_des << "," << t.vy_des << "," << t.vz_des << ","
                << t.ax_des << "," << t.ay_des << "," << t.az_des;
        };

        for (const auto& e : wbc_entries_) {
            out << e.time_ms << ",";
            write_task(e.com);     out << ",";
            write_task(e.wheel_l); out << ",";
            write_task(e.wheel_r); out << ",";
            write_task(e.torso);
            for (int i = 0; i < 8; ++i) out << "," << e.solution.joints.pos[i];
            for (int i = 0; i < 8; ++i) out << "," << e.solution.joints.vel[i];
            for (int i = 0; i < 8; ++i) out << "," << e.solution.joints.effort[i];
            for (int i = 0; i < 8; ++i) out << "," << e.feedback.pos[i];
            for (int i = 0; i < 8; ++i) out << "," << e.feedback.vel[i];
            for (int i = 0; i < 8; ++i) out << "," << e.feedback.effort[i];
            out << "\n";
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

        out << "time_mpc_us,time_wbc_us,time_filter_us,total_time_us\n";

        for (const auto& e : timing_entries_) {
            out << e.time_mpc_us    << ","
                << e.time_wbc_us    << ","
                << e.time_filter_us << ","
                << e.total_time_us  << "\n";
        }
    }

    void save_imu_logs(const std::string& filename) const {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        out << "time_s,qx,qy,qz,qw,wx,wy,wz,ax,ay,az\n";
        out << std::fixed << std::setprecision(9);
        for (const auto& e : imu_entries_) {
            out << e.time_s << ","
                << e.qx << "," << e.qy << "," << e.qz << "," << e.qw << ","
                << e.wx << "," << e.wy << "," << e.wz << ","
                << e.ax << "," << e.ay << "," << e.az << "\n";
        }
    }

    void save_joint_state_logs(const std::string& filename) const {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        out << "time_stamp_s,time_now_s";
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size()))
                                  ? joint_names_[i]
                                  : ("j" + std::to_string(i));
            out << "," << n << "_pos," << n << "_vel," << n << "_effort";
        }
        out << "\n";
        out << std::fixed << std::setprecision(9);
        for (const auto& e : joint_entries_) {
            out << e.time_stamp_s << "," << e.time_now_s;
            for (int i = 0; i < 8; ++i) {
                out << "," << e.joints.pos[i] << "," << e.joints.vel[i] << "," << e.joints.effort[i];
            }
            out << "\n";
        }
    }

    void save_tau_commanded_logs(const std::string& filename) const {
        std::ofstream out(filename);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        out << "time_s";
        for (int i = 0; i < 8; ++i) {
            const std::string n = (i < static_cast<int>(joint_names_.size()))
                                  ? joint_names_[i]
                                  : ("j" + std::to_string(i));
            out << "," << n << "_effort";
        }
        out << "\n";
        out << std::fixed << std::setprecision(9);
        for (const auto& e : tau_entries_) {
            out << e.time_s;
            for (int i = 0; i < 8; ++i) {
                out << "," << e.tau[i];
            }
            out << "\n";
        }
    }

private:
    std::vector<WBCEntry> wbc_entries_;
    std::vector<MPCEntry> mpc_entries_;
    std::vector<TimingEntry> timing_entries_;
    std::vector<ImuEntry> imu_entries_;
    std::vector<JointStateEntry> joint_entries_;
    std::vector<TauCommandedEntry> tau_entries_;
    std::vector<std::string> joint_names_;
};

} // namespace labrob