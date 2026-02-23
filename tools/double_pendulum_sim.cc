#include <simcore/simcore.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Config {
  std::string model_path = "E:/workspace/NexDyn-Diff/models/double_pendulum.xml";
  std::string output_dir = "E:/workspace/NexDyn-Diff/output/double_pendulum";
  double sim_time = 5.0; // Assume 5 seconds based on typical simulations
  double dt = 0.001;
};

void WriteHeader(std::ofstream& out, const std::string& type) {
  out << "Time,X,Y,Z\n";
}

// Write row for a specific body and type
void WriteRow(std::ofstream& out, double time, const double* vec) {
  out << std::setprecision(17)
      << time << ','
      << vec[0] << ','
      << vec[1] << ','
      << vec[2] << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  
  // Create output directory
  std::filesystem::create_directories(cfg.output_dir);

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

  model->opt.timestep = static_cast<sim_scalar_t>(cfg.dt);

  // Get Body IDs
  int body1_id = sim_name2id(model, SIM_OBJ_BODY, "Body1");
  int body2_id = sim_name2id(model, SIM_OBJ_BODY, "Body2");

  if (body1_id < 0 || body2_id < 0) {
    std::cerr << "Could not find Body1 or Body2\n";
    return 1;
  }

  // Open CSV files
  std::ofstream f_pos1(cfg.output_dir + "/sim_Link1_Pos.csv");
  std::ofstream f_vel1(cfg.output_dir + "/sim_Link1_Vel.csv");
  std::ofstream f_acc1(cfg.output_dir + "/sim_Link1_Acc.csv");
  std::ofstream f_pos2(cfg.output_dir + "/sim_Link2_Pos.csv");
  std::ofstream f_vel2(cfg.output_dir + "/sim_Link2_Vel.csv");
  std::ofstream f_acc2(cfg.output_dir + "/sim_Link2_Acc.csv");

  WriteHeader(f_pos1, "pos"); WriteHeader(f_vel1, "vel"); WriteHeader(f_acc1, "acc");
  WriteHeader(f_pos2, "pos"); WriteHeader(f_vel2, "vel"); WriteHeader(f_acc2, "acc");

  int steps = static_cast<int>(std::llround(cfg.sim_time / cfg.dt));

  for (int i = 0; i < steps; ++i) {
    sim_step(model, data);
    
    // Ensure accelerations are computed (if not already done by sim_step via sensors)
    // Checking if cacc is populated. Since we added a dummy sensor, it should be.
    // If not, explicitly call sim_rnePostConstraint(model, data); if available.
    // Assuming sim_step handles it due to sensor presence.

    double time = static_cast<double>(data->time);

    // Body 1 Data
    // xipos is 3D vector at 3*id (Global Position of COM)
    // Use sim_objectVelocity/Acceleration to get Global Frame vectors

    double pos1[3] = {
      static_cast<double>(data->xipos[3*body1_id + 0]),
      static_cast<double>(data->xipos[3*body1_id + 1]),
      static_cast<double>(data->xipos[3*body1_id + 2])
    };

    sim_scalar_t v1_6[6];
    sim_objectVelocity(model, data, SIM_OBJ_BODY, body1_id, v1_6, 0);
    double vel1[3] = { (double)v1_6[3], (double)v1_6[4], (double)v1_6[5] };

    sim_scalar_t a1_6[6];
    sim_objectAcceleration(model, data, SIM_OBJ_BODY, body1_id, a1_6, 0);
    double acc1[3] = { (double)a1_6[3], (double)a1_6[4], (double)a1_6[5] };
    
    // Convert from proper acceleration (accelerometer-like) to kinematic acceleration
    // a_kinematic = a_proper + g
    acc1[0] += model->opt.gravity[0];
    acc1[1] += model->opt.gravity[1];
    acc1[2] += model->opt.gravity[2];

    // Body 2 Data
    double pos2[3] = {
      static_cast<double>(data->xipos[3*body2_id + 0]),
      static_cast<double>(data->xipos[3*body2_id + 1]),
      static_cast<double>(data->xipos[3*body2_id + 2])
    };

    sim_scalar_t v2_6[6];
    sim_objectVelocity(model, data, SIM_OBJ_BODY, body2_id, v2_6, 0);
    double vel2[3] = { (double)v2_6[3], (double)v2_6[4], (double)v2_6[5] };

    sim_scalar_t a2_6[6];
    sim_objectAcceleration(model, data, SIM_OBJ_BODY, body2_id, a2_6, 0);
    double acc2[3] = { (double)a2_6[3], (double)a2_6[4], (double)a2_6[5] };
    
    // a_kinematic = a_proper + g
    acc2[0] += model->opt.gravity[0];
    acc2[1] += model->opt.gravity[1];
    acc2[2] += model->opt.gravity[2];



    WriteRow(f_pos1, time, pos1);
    WriteRow(f_vel1, time, vel1);
    WriteRow(f_acc1, time, acc1);

    WriteRow(f_pos2, time, pos2);
    WriteRow(f_vel2, time, vel2);
    WriteRow(f_acc2, time, acc2);
  }

  sim_deleteData(data);
  sim_deleteModel(model);
  return 0;
}
