#include <simcore/simcore.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

struct Config {
  std::string model_path = "E:/workspace/NexDyn-Diff/models/slider_crank.xml";
  std::string output_dir = "E:/workspace/NexDyn-Diff/output/slider_crank";
  double sim_time = 1.0;
  double dt = 0.001;
};

double StepValue(double time, double t0, double h0, double t1, double h1) {
  if (time <= t0) {
    return h0;
  }
  if (time >= t1) {
    return h1;
  }
  const double tau = (time - t0) / (t1 - t0);
  const double blend = tau * tau * (3.0 - 2.0 * tau);
  return h0 + (h1 - h0) * blend;
}

double StepVelocity(double time, double t0, double h0, double t1, double h1) {
  if (time <= t0 || time >= t1) {
    return 0.0;
  }
  const double tau = (time - t0) / (t1 - t0);
  return (h1 - h0) * (6.0 * tau * (1.0 - tau) / (t1 - t0));
}

void WritePosHeader(std::ofstream& out) {
  out << "Time,X,Y,Z\n";
}

void WriteVelHeader(std::ofstream& out) {
  out << "Time,Rz\n";
}

void WritePosRow(std::ofstream& out, double time, const double* xyz) {
  out << std::setprecision(17)
      << time << ','
      << xyz[0] << ','
      << xyz[1] << ','
      << xyz[2] << '\n';
}

void WriteVelRow(std::ofstream& out, double time, double rz) {
  out << std::setprecision(17)
      << time << ','
      << rz << '\n';
}

void SampleAndWrite(
    const sim_model_t* model,
    const sim_data_t* data,
    int body1_id,
    int body2_id,
    int body3_id,
    int body5_id,
    std::ofstream& body1_pos,
    std::ofstream& body2_pos,
    std::ofstream& body3_pos,
    std::ofstream& body5_pos,
    std::ofstream& body2_vel_rz,
    std::ofstream& body5_vel_rz) {
  const double time = static_cast<double>(data->time);

  const double pos1[3] = {
      static_cast<double>(data->xipos[3 * body1_id + 0]),
      static_cast<double>(data->xipos[3 * body1_id + 1]),
      static_cast<double>(data->xipos[3 * body1_id + 2])};
  const double pos2[3] = {
      static_cast<double>(data->xipos[3 * body2_id + 0]),
      static_cast<double>(data->xipos[3 * body2_id + 1]),
      static_cast<double>(data->xipos[3 * body2_id + 2])};
  const double pos3[3] = {
      static_cast<double>(data->xipos[3 * body3_id + 0]),
      static_cast<double>(data->xipos[3 * body3_id + 1]),
      static_cast<double>(data->xipos[3 * body3_id + 2])};
  const double pos5[3] = {
      static_cast<double>(data->xipos[3 * body5_id + 0]),
      static_cast<double>(data->xipos[3 * body5_id + 1]),
      static_cast<double>(data->xipos[3 * body5_id + 2])};

  sim_scalar_t vel2_6[6] = {0, 0, 0, 0, 0, 0};
  sim_scalar_t vel5_6[6] = {0, 0, 0, 0, 0, 0};
  sim_objectVelocity(model, data, SIM_OBJ_BODY, body2_id, vel2_6, 0);
  sim_objectVelocity(model, data, SIM_OBJ_BODY, body5_id, vel5_6, 0);

  WritePosRow(body1_pos, time, pos1);
  WritePosRow(body2_pos, time, pos2);
  WritePosRow(body3_pos, time, pos3);
  WritePosRow(body5_pos, time, pos5);
  WriteVelRow(body2_vel_rz, time, static_cast<double>(vel2_6[2]));
  WriteVelRow(body5_vel_rz, time, static_cast<double>(vel5_6[2]));
}

}  // namespace

int main() {
  Config cfg;
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

  const int body1_id = sim_name2id(model, SIM_OBJ_BODY, "Body1");
  const int body2_id = sim_name2id(model, SIM_OBJ_BODY, "Body2");
  const int body3_id = sim_name2id(model, SIM_OBJ_BODY, "Body3");
  const int body5_id = sim_name2id(model, SIM_OBJ_BODY, "Body5");
  const int rev1_id = sim_name2id(model, SIM_OBJ_JOINT, "RevJoint1");
  const int rev2_id = sim_name2id(model, SIM_OBJ_JOINT, "RevJoint2");
  const int tra1_id = sim_name2id(model, SIM_OBJ_JOINT, "TraJoint1");
  const int tra2_id = sim_name2id(model, SIM_OBJ_JOINT, "TraJoint2");

  if (body1_id < 0 || body2_id < 0 || body3_id < 0 || body5_id < 0 ||
      rev1_id < 0 || rev2_id < 0 || tra1_id < 0 || tra2_id < 0) {
    std::cerr << "Model is missing required body or joint names.\n";
    sim_deleteData(data);
    sim_deleteModel(model);
    return 1;
  }

  const int rev1_qpos = model->jnt_qposadr[rev1_id];
  const int rev2_qpos = model->jnt_qposadr[rev2_id];
  const int tra1_qpos = model->jnt_qposadr[tra1_id];
  const int tra2_qpos = model->jnt_qposadr[tra2_id];

  const int rev1_dof = model->jnt_dofadr[rev1_id];
  const int rev2_dof = model->jnt_dofadr[rev2_id];
  const int tra1_dof = model->jnt_dofadr[tra1_id];
  const int tra2_dof = model->jnt_dofadr[tra2_id];

  std::ofstream body1_pos(cfg.output_dir + "/sim_Link_Body1_Pos.csv");
  std::ofstream body2_pos(cfg.output_dir + "/sim_Link_Body2_Pos.csv");
  std::ofstream body3_pos(cfg.output_dir + "/sim_Link_Body3_Pos.csv");
  std::ofstream body5_pos(cfg.output_dir + "/sim_Sphere_Body5_Pos.csv");
  std::ofstream body2_vel_rz(cfg.output_dir + "/sim_Link_Body2_Vel_Rz.csv");
  std::ofstream body5_vel_rz(cfg.output_dir + "/sim_Sphere_Body5_Vel_Rz.csv");

  WritePosHeader(body1_pos);
  WritePosHeader(body2_pos);
  WritePosHeader(body3_pos);
  WritePosHeader(body5_pos);
  WriteVelHeader(body2_vel_rz);
  WriteVelHeader(body5_vel_rz);

  const int steps = static_cast<int>(std::llround(cfg.sim_time / cfg.dt));

  for (int i = 0; i <= steps; ++i) {
    const double t = i * cfg.dt;

    const double rev1_pos = StepValue(t, 0.0, 0.0, 1.0, 1.0);
    const double rev2_pos = StepValue(t, 0.0, 0.0, 1.0, -1.0);
    const double tra1_pos = StepValue(t, 0.0, 0.0, 1.0, 1.0);
    const double tra2_pos = StepValue(t, 0.0, 0.0, 1.0, 1.0);

    const double rev1_vel = StepVelocity(t, 0.0, 0.0, 1.0, 1.0);
    const double rev2_vel = StepVelocity(t, 0.0, 0.0, 1.0, -1.0);
    const double tra1_vel = StepVelocity(t, 0.0, 0.0, 1.0, 1.0);
    const double tra2_vel = StepVelocity(t, 0.0, 0.0, 1.0, 1.0);

    data->time = static_cast<sim_scalar_t>(t);
    data->qpos[rev1_qpos] = static_cast<sim_scalar_t>(rev1_pos);
    data->qpos[rev2_qpos] = static_cast<sim_scalar_t>(rev2_pos);
    data->qpos[tra1_qpos] = static_cast<sim_scalar_t>(tra1_pos);
    data->qpos[tra2_qpos] = static_cast<sim_scalar_t>(tra2_pos);

    data->qvel[rev1_dof] = static_cast<sim_scalar_t>(rev1_vel);
    data->qvel[rev2_dof] = static_cast<sim_scalar_t>(rev2_vel);
    data->qvel[tra1_dof] = static_cast<sim_scalar_t>(tra1_vel);
    data->qvel[tra2_dof] = static_cast<sim_scalar_t>(tra2_vel);

    sim_forward(model, data);

    SampleAndWrite(model, data,
                   body1_id, body2_id, body3_id, body5_id,
                   body1_pos, body2_pos, body3_pos, body5_pos,
                   body2_vel_rz, body5_vel_rz);
  }

  sim_deleteData(data);
  sim_deleteModel(model);
  return 0;
}
