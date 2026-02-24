#include <simcore/simcore.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <sstream>

namespace {

struct Config {
  std::string model_path = "E:/workspace/NexDyn-Diff/models/fuzajiaolian.xml";
  std::string output_path_gear = "E:/workspace/NexDyn-Diff/output/Link_Gear_Vel_Rx.csv";
  std::string output_path_drive = "E:/workspace/NexDyn-Diff/output/Gear21_Vel_Rx.csv";
  std::string output_path_arm = "E:/workspace/NexDyn-Diff/output/Swing_Arm_Vel_Rx.csv";
  std::string input_profile_path = ""; // Optional input profile
  std::string preferred_actuator_name = "RevJoint1.RMotion";
  double sim_time = 0.501; 
  double dt = 0.0001; // Simulation step
  double out_dt = 0.0005; // Output step
  double default_target_velocity = 1.0;  // rad/s
  double startup_ramp_time = 0.05;       // s
  bool require_velocity_actuator = true;
};

// Profile data
struct ProfilePoint {
    double t;
    double v;
};
std::vector<ProfilePoint> input_profile;

void LoadProfile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Failed to open profile: " << path << "\n";
        return;
    }
    std::string line;
    std::getline(f, line); // Skip header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        double t, v;
        if (ss >> t >> v) {
            input_profile.push_back({t, v});
        }
    }
    std::cout << "Loaded " << input_profile.size() << " profile points.\n";
}

double GetTargetVelocity(double t) {
    if (input_profile.empty()) return 0.0;
    if (t <= input_profile.front().t) return input_profile.front().v;
    if (t >= input_profile.back().t) return input_profile.back().v;
    
    // Linear interp
    auto it = std::lower_bound(input_profile.begin(), input_profile.end(), t, 
        [](const ProfilePoint& p, double val) { return p.t < val; });
    
    if (it == input_profile.begin()) return it->v;
    const auto& p2 = *it;
    const auto& p1 = *(it - 1);
    
    double alpha = (t - p1.t) / (p2.t - p1.t);
    return p1.v + alpha * (p2.v - p1.v);
}

struct DriveInfo {
  int actuator_id = -1;
  int joint_id = -1;
  int dof_adr = -1;
  bool is_velocity_servo = false;
};

DriveInfo ResolveDriveInfo(const sim_model_t* model, const Config& cfg) {
  DriveInfo info;

  info.actuator_id = sim_name2id(model, SIM_OBJ_ACTUATOR, cfg.preferred_actuator_name.c_str());
  if (info.actuator_id < 0) {
    info.actuator_id = sim_name2id(model, SIM_OBJ_ACTUATOR, "motor_rev1");
  }
  if (info.actuator_id < 0 && model->nu > 0) {
    info.actuator_id = 0;
  }

  if (info.actuator_id >= 0 &&
      (model->actuator_trntype[info.actuator_id] == SIM_TRN_JOINT ||
       model->actuator_trntype[info.actuator_id] == SIM_TRN_JOINTINPARENT)) {
    info.joint_id = model->actuator_trnid[2 * info.actuator_id];
  }

  if (info.joint_id < 0) {
    info.joint_id = sim_name2id(model, SIM_OBJ_JOINT, "RevJoint1");
  }
  if (info.joint_id < 0) {
    info.joint_id = sim_name2id(model, SIM_OBJ_JOINT, "RevJoint1.RMotion");
  }

  if (info.joint_id >= 0) {
    info.dof_adr = model->jnt_dofadr[info.joint_id];
  }

  if (info.actuator_id >= 0) {
    int aid = info.actuator_id;
    const sim_scalar_t gain = model->actuator_gainprm[aid * SIM_NGAIN];
    const sim_scalar_t bias_vel = model->actuator_biasprm[aid * SIM_NBIAS + 2];
    info.is_velocity_servo =
        model->actuator_biastype[aid] == SIM_BIAS_AFFINE &&
        std::abs(gain) > 0 &&
        std::abs(gain + bias_vel) < 1e-9;
  }

  return info;
}

void WriteHeader(std::ofstream& out, const std::string& col_name) {
  out << "Time," << col_name << "\n";
}

void WriteRow(std::ofstream& out, double time, double val) {
  out << std::setprecision(9) << std::fixed
      << time << ','
      << val << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  
  if (argc > 1) {
      cfg.model_path = argv[1];
  }
  
  double ignored_kp = 0.0;
  double target_vel_scale = 1.0;
  
  if (argc > 2) {
      ignored_kp = std::stod(argv[2]);
  }
  if (argc > 3) {
      target_vel_scale = std::stod(argv[3]);
  }
  if (argc > 4) {
      cfg.input_profile_path = argv[4];
      LoadProfile(cfg.input_profile_path);
  }
  
  // Create output directory if not exists
  std::filesystem::create_directories("E:/workspace/NexDyn-Diff/output");

  char error[1024] = {0};
  sim_model_t* model = sim_loadXML(cfg.model_path.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    std::cerr << "sim_loadXML failed: " << error << "\n";
    return 1;
  }

  sim_data_t* data = sim_makeData(model);
  if (!data) {
    std::cerr << "sim_makeData failed\n";
    sim_deleteModel(model);
    return 1;
  }

  if (model->opt.timestep > 0) {
    cfg.dt = model->opt.timestep;
  } else {
    model->opt.timestep = static_cast<sim_scalar_t>(cfg.dt);
  }

  DriveInfo drive = ResolveDriveInfo(model, cfg);
  
  // Bodies
  int gear22_id = sim_name2id(model, SIM_OBJ_BODY, "GEAR22");
  int swingarm_id = sim_name2id(model, SIM_OBJ_BODY, "SWINGARM000");
  int gear21_id = sim_name2id(model, SIM_OBJ_BODY, "GEAR21");

  if (gear22_id < 0 || swingarm_id < 0 || gear21_id < 0) {
    std::cerr << "Could not find GEAR21, GEAR22, or SWINGARM000\n";
    return 1;
  }

  if (drive.actuator_id < 0) {
    std::cerr << "Could not resolve drive actuator.\n";
    return 1;
  }

  if (cfg.require_velocity_actuator && !drive.is_velocity_servo) {
    std::cerr << "Drive actuator is not a velocity servo. "
              << "Use a <velocity ...> actuator for RevJoint1.\n";
    return 1;
  }

  if (ignored_kp != 0.0) {
    std::cout << "Note: kp argument is ignored; velocity actuator mode is active.\n";
  }

  std::ofstream f_gear(cfg.output_path_gear);
  std::ofstream f_drive(cfg.output_path_drive);
  std::ofstream f_arm(cfg.output_path_arm);
  
  // Headers to match baseline somewhat
  f_gear << "X:Vel_RX-GEAR22-fuzajiaolian(rad/s),Y:Vel_RX-GEAR22-fuzajiaolian(rad/s)\n";
  f_drive << "X:Vel_RX-GEAR21-fuzajiaolian(rad/s),Y:Vel_RX-GEAR21-fuzajiaolian(rad/s)\n";
  f_arm << "X:Vel_RX-SWINGARM000-fuzajiaolian(rad/s),Y:Vel_RX-SWINGARM000-fuzajiaolian(rad/s)\n";

  int steps = static_cast<int>(std::ceil(cfg.sim_time / cfg.dt));
  int out_interval = 50; // Force 50 steps (0.0005s) if dt=1e-5
  if (cfg.dt > 0) out_interval = static_cast<int>(std::round(cfg.out_dt / cfg.dt));

  for (int i = 0; i <= steps; ++i) {
    double time = static_cast<double>(data->time);
    
    double current_target = 0.0;
    
    if (!input_profile.empty()) {
        current_target = GetTargetVelocity(time) * target_vel_scale;
    } else {
        current_target = cfg.default_target_velocity * target_vel_scale;
    }

    if (cfg.startup_ramp_time > 0.0) {
        const double ramp = std::clamp(time / cfg.startup_ramp_time, 0.0, 1.0);
        current_target *= ramp;
    }
    
    data->ctrl[drive.actuator_id] = static_cast<sim_scalar_t>(current_target);

    if (i % out_interval == 0) {
        // Capture output
        sim_scalar_t v_gear21[6];
        sim_objectVelocity(model, data, SIM_OBJ_BODY, gear21_id, v_gear21, 0);

        sim_scalar_t v_gear22[6];
        sim_objectVelocity(model, data, SIM_OBJ_BODY, gear22_id, v_gear22, 0);
        
        sim_scalar_t v_swingarm[6];
        sim_objectVelocity(model, data, SIM_OBJ_BODY, swingarm_id, v_swingarm, 0);
        
        // v[0] is wx (Angular Velocity X)
        WriteRow(f_drive, time, v_gear21[0]);
        WriteRow(f_gear, time, v_gear22[0]);
        WriteRow(f_arm, time, v_swingarm[0]);
        
        if (i % 1000 == 0) {
             std::cout << "Step " << i << " T=" << time
                       << " Gear21_Rx=" << v_gear21[0]
                       << " Gear22_Rx=" << v_gear22[0] << "\n";
        }
    }

    sim_step(model, data);
  }

  sim_deleteData(data);
  sim_deleteModel(model);
  
  std::cout << "Simulation completed.\n";
  return 0;
}
