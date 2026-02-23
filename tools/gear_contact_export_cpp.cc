#include <simcore/simcore.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Config {
  std::string model_path = "models/gear_selfcontained.xml";
  std::string out_csv = "output/gear_example_cpp.csv";
  std::string debug_contacts_csv;
  std::string plugin_lib;
  bool dump_model = false;
  double ctrl = 1.0;
  double sim_time = 5.0;
  double dt = 0.001;
};

struct Row {
  double time = 0.0;
  double fx = 0.0;
  double fy = 0.0;
  double fz = 0.0;
  double fn_sum = 0.0;
  double fworld_mag = 0.0;
  int ncon = 0;
  double freewheel_vel = 0.0;
  double drive_vel = 0.0;
};

void PrintUsage(const char* exe) {
  std::cerr
      << "Usage: " << exe
      << " [--model <path>] [--csv <path>] [--debug-contacts-csv <path>] [--plugin-lib <path>] [--ctrl <float>] "
      << "[--sim-time <float>] [--dt <float>] [--dump-model]\n";
}

const char* DefaultPluginLibName() {
#if defined(_WIN32)
  return "simcore_sdf_plugin.dll";
#elif defined(__APPLE__)
  return "libsimcore_sdf_plugin.dylib";
#else
  return "libsimcore_sdf_plugin.so";
#endif
}

bool ParseArgs(int argc, char** argv, Config& cfg) {
  const std::filesystem::path exe_path = std::filesystem::absolute(argv[0]);
  cfg.plugin_lib = (exe_path.parent_path() / DefaultPluginLibName()).string();

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--model") {
      const char* v = require_value("--model");
      if (!v) return false;
      cfg.model_path = v;
    } else if (arg == "--csv") {
      const char* v = require_value("--csv");
      if (!v) return false;
      cfg.out_csv = v;
    } else if (arg == "--debug-contacts-csv") {
      const char* v = require_value("--debug-contacts-csv");
      if (!v) return false;
      cfg.debug_contacts_csv = v;
    } else if (arg == "--plugin-lib") {
      const char* v = require_value("--plugin-lib");
      if (!v) return false;
      cfg.plugin_lib = v;
    } else if (arg == "--no-plugin-load") {
      cfg.plugin_lib.clear();
    } else if (arg == "--dump-model") {
      cfg.dump_model = true;
    } else if (arg == "--ctrl") {
      const char* v = require_value("--ctrl");
      if (!v) return false;
      cfg.ctrl = std::stod(v);
    } else if (arg == "--sim-time") {
      const char* v = require_value("--sim-time");
      if (!v) return false;
      cfg.sim_time = std::stod(v);
    } else if (arg == "--dt") {
      const char* v = require_value("--dt");
      if (!v) return false;
      cfg.dt = std::stod(v);
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      PrintUsage(argv[0]);
      return false;
    }
  }
  return true;
}

int RequireId(const sim_model_t* m, int type, const char* name) {
  const int id = sim_name2id(m, type, name);
  if (id < 0) {
    std::ostringstream oss;
    oss << "Missing object '" << name << "' with type " << type;
    throw std::runtime_error(oss.str());
  }
  return id;
}

int GeomA(const sim_contact_t& c) {
  return c.geom[0] >= 0 ? c.geom[0] : c.geom1;
}

int GeomB(const sim_contact_t& c) {
  return c.geom[1] >= 0 ? c.geom[1] : c.geom2;
}

void WriteHeader(std::ofstream& out) {
  out << "time,fx,fy,fz,fn_sum,fworld_mag,ncon,freewheel_vel,drive_vel\n";
}

void WriteRow(std::ofstream& out, const Row& r) {
  out << std::setprecision(17)
      << r.time << ','
      << r.fx << ','
      << r.fy << ','
      << r.fz << ','
      << r.fn_sum << ','
      << r.fworld_mag << ','
      << r.ncon << ','
      << r.freewheel_vel << ','
      << r.drive_vel << '\n';
}

void DumpModelInfo(const sim_model_t* model) {
  std::cout << "model:nbody=" << model->nbody
            << ",njnt=" << model->njnt
            << ",nu=" << model->nu
            << ",nmesh=" << model->nmesh
            << ",nmeshvert=" << model->nmeshvert
            << ",nmeshface=" << model->nmeshface
            << ",timestep=" << static_cast<double>(model->opt.timestep)
            << ",integrator=" << model->opt.integrator
            << ",solver=" << model->opt.solver
            << ",cone=" << model->opt.cone
            << ",jacobian=" << model->opt.jacobian
            << ",iterations=" << model->opt.iterations
            << ",ls_iterations=" << model->opt.ls_iterations
            << ",sdf_iterations=" << model->opt.sdf_iterations
            << ",sdf_initpoints=" << model->opt.sdf_initpoints
            << "\n";

  for (int i = 0; i < model->nbody; ++i) {
    const char* bname = sim_id2name(model, SIM_OBJ_BODY, i);
    std::cout << "body[" << i << "]"
              << ",name=" << (bname ? bname : "")
              << ",mass=" << static_cast<double>(model->body_mass[i])
              << ",inertia_x=" << static_cast<double>(model->body_inertia[3 * i + 0])
              << ",inertia_y=" << static_cast<double>(model->body_inertia[3 * i + 1])
              << ",inertia_z=" << static_cast<double>(model->body_inertia[3 * i + 2])
              << "\n";
  }

  for (int i = 0; i < model->nmesh; ++i) {
    const char* mname = sim_id2name(model, SIM_OBJ_MESH, i);
    std::cout << "mesh[" << i << "]"
              << ",name=" << (mname ? mname : "")
              << ",vertnum=" << model->mesh_vertnum[i]
              << ",facenum=" << model->mesh_facenum[i]
              << ",scale_x=" << static_cast<double>(model->mesh_scale[3 * i + 0])
              << ",scale_y=" << static_cast<double>(model->mesh_scale[3 * i + 1])
              << ",scale_z=" << static_cast<double>(model->mesh_scale[3 * i + 2])
              << ",pos_x=" << static_cast<double>(model->mesh_pos[3 * i + 0])
              << ",pos_y=" << static_cast<double>(model->mesh_pos[3 * i + 1])
              << ",pos_z=" << static_cast<double>(model->mesh_pos[3 * i + 2])
              << "\n";
  }

  for (int i = 0; i < model->njnt; ++i) {
    const char* jname = sim_id2name(model, SIM_OBJ_JOINT, i);
    const int dof = model->jnt_dofadr[i];
    std::cout << "joint[" << i << "]"
              << ",name=" << (jname ? jname : "")
              << ",dof=" << dof
              << ",damping=" << static_cast<double>(model->dof_damping[dof])
              << ",armature=" << static_cast<double>(model->dof_armature[dof])
              << "\n";
  }

  for (int i = 0; i < model->nu; ++i) {
    const char* aname = sim_id2name(model, SIM_OBJ_ACTUATOR, i);
    std::cout << "actuator[" << i << "]"
              << ",name=" << (aname ? aname : "")
              << ",gear0=" << static_cast<double>(model->actuator_gear[6 * i + 0])
              << ",gain0=" << static_cast<double>(model->actuator_gainprm[SIM_NGAIN * i + 0])
              << ",bias0=" << static_cast<double>(model->actuator_biasprm[SIM_NBIAS * i + 0])
              << ",ctrl0=" << static_cast<double>(model->actuator_ctrlrange[2 * i + 0])
              << ",ctrl1=" << static_cast<double>(model->actuator_ctrlrange[2 * i + 1])
              << ",ctrllimited=" << static_cast<int>(model->actuator_ctrllimited[i])
              << "\n";
  }

  for (int i = 0; i < model->nplugin; ++i) {
    const char* pname = sim_id2name(model, SIM_OBJ_PLUGIN, i);
    const int adr = model->plugin_attradr[i];
    const int next = (i + 1 < model->nplugin) ? model->plugin_attradr[i + 1] : model->npluginattr;
    std::string flattened;
    for (int j = adr; j < next; ++j) {
      const char ch = model->plugin_attr[j];
      flattened.push_back(ch ? ch : '|');
    }
    std::cout << "plugin[" << i << "]"
              << ",name=" << (pname ? pname : "")
              << ",slot=" << model->plugin[i]
              << ",attradr=" << adr
              << ",attrlen=" << (next - adr)
              << ",attrs=\"" << flattened << "\""
              << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  if (!ParseArgs(argc, argv, cfg)) {
    return 1;
  }

  try {
    if (!cfg.plugin_lib.empty()) {
      sim_loadPluginLibrary(cfg.plugin_lib.c_str());
    }

    char error[1024] = {0};
    sim_model_t* model = sim_loadXML(cfg.model_path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
      std::cerr << "sim_loadXML failed: " << error << "\n";
      return 1;
    }

    if (cfg.dump_model) {
      DumpModelInfo(model);
    }

    sim_data_t* data = sim_makeData(model);
    if (!data) {
      std::cerr << "sim_makeData failed\n";
      sim_deleteModel(model);
      return 1;
    }

    model->opt.timestep = static_cast<sim_scalar_t>(cfg.dt);

    const int actuator_drive = RequireId(model, SIM_OBJ_ACTUATOR, "drive");
    const int joint_freewheel = RequireId(model, SIM_OBJ_JOINT, "freewheel");
    const int joint_drive = RequireId(model, SIM_OBJ_JOINT, "drive");
    const int geom_gear1 = RequireId(model, SIM_OBJ_GEOM, "gear1");
    const int geom_gear2 = RequireId(model, SIM_OBJ_GEOM, "gear2");

    const int dof_freewheel = model->jnt_dofadr[joint_freewheel];
    const int dof_drive = model->jnt_dofadr[joint_drive];
    const int steps = static_cast<int>(std::llround(cfg.sim_time / cfg.dt));

    std::filesystem::path out_path(cfg.out_csv);
    if (!out_path.parent_path().empty()) {
      std::filesystem::create_directories(out_path.parent_path());
    }

    std::ofstream out(cfg.out_csv, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      std::cerr << "Failed to open output csv: " << cfg.out_csv << "\n";
      sim_deleteData(data);
      sim_deleteModel(model);
      return 1;
    }

    WriteHeader(out);

    std::ofstream debug_contacts_out;
    if (!cfg.debug_contacts_csv.empty()) {
      std::filesystem::path debug_path(cfg.debug_contacts_csv);
      if (!debug_path.parent_path().empty()) {
        std::filesystem::create_directories(debug_path.parent_path());
      }
      debug_contacts_out.open(cfg.debug_contacts_csv, std::ios::out | std::ios::trunc);
      if (!debug_contacts_out.is_open()) {
        std::cerr << "Failed to open debug contacts csv: " << cfg.debug_contacts_csv << "\n";
        sim_deleteData(data);
        sim_deleteModel(model);
        return 1;
      }
      debug_contacts_out
          << "step,time,ncon_total,ncon_pair,contact_index,geom1,geom2,dist,efc_address,"
          << "fn,ft1,ft2,fx,fy,fz,frame_nx,frame_ny,frame_nz,freewheel_vel,drive_vel\n";
    }

    bool has_first_nonzero = false;
    Row first_nonzero{};
    Row max_force{};
    double max_abs_freewheel = 0.0;
    double max_abs_drive = 0.0;

    for (int i = 0; i < steps; ++i) {
      data->ctrl[actuator_drive] = static_cast<sim_scalar_t>(cfg.ctrl);
      sim_step(model, data);

      Row row{};
      row.time = static_cast<double>(data->time);

      bool wrote_debug_contact = false;
      for (int c = 0; c < data->ncon; ++c) {
        const sim_contact_t& con = data->contact[c];
        const int g1 = GeomA(con);
        const int g2 = GeomB(con);
        const bool is_pair = (g1 == geom_gear1 && g2 == geom_gear2) ||
                             (g1 == geom_gear2 && g2 == geom_gear1);
        if (!is_pair) {
          continue;
        }

        ++row.ncon;
        sim_scalar_t force_local[6] = {0, 0, 0, 0, 0, 0};
        sim_contactForce(model, data, c, force_local);
        row.fn_sum += static_cast<double>(force_local[0]);

        row.fx += static_cast<double>(con.frame[0] * force_local[0] +
                                      con.frame[1] * force_local[1] +
                                      con.frame[2] * force_local[2]);
        row.fy += static_cast<double>(con.frame[3] * force_local[0] +
                                      con.frame[4] * force_local[1] +
                                      con.frame[5] * force_local[2]);
        row.fz += static_cast<double>(con.frame[6] * force_local[0] +
                                      con.frame[7] * force_local[1] +
                                      con.frame[8] * force_local[2]);

        if (debug_contacts_out.is_open()) {
          const double fx = static_cast<double>(con.frame[0] * force_local[0] +
                                                con.frame[1] * force_local[1] +
                                                con.frame[2] * force_local[2]);
          const double fy = static_cast<double>(con.frame[3] * force_local[0] +
                                                con.frame[4] * force_local[1] +
                                                con.frame[5] * force_local[2]);
          const double fz = static_cast<double>(con.frame[6] * force_local[0] +
                                                con.frame[7] * force_local[1] +
                                                con.frame[8] * force_local[2]);
          debug_contacts_out << i << ','
                             << std::setprecision(17) << static_cast<double>(data->time) << ','
                             << data->ncon << ','
                             << row.ncon << ','
                             << c << ','
                             << g1 << ','
                             << g2 << ','
                             << static_cast<double>(con.dist) << ','
                             << con.efc_address << ','
                             << static_cast<double>(force_local[0]) << ','
                             << static_cast<double>(force_local[1]) << ','
                             << static_cast<double>(force_local[2]) << ','
                             << fx << ','
                             << fy << ','
                             << fz << ','
                             << static_cast<double>(con.frame[0]) << ','
                             << static_cast<double>(con.frame[1]) << ','
                             << static_cast<double>(con.frame[2]) << ','
                             << static_cast<double>(data->qvel[dof_freewheel]) << ','
                             << static_cast<double>(data->qvel[dof_drive]) << '\n';
          wrote_debug_contact = true;
        }
      }

      row.fworld_mag = std::sqrt(row.fx * row.fx + row.fy * row.fy + row.fz * row.fz);
      row.freewheel_vel = static_cast<double>(data->qvel[dof_freewheel]);
      row.drive_vel = static_cast<double>(data->qvel[dof_drive]);

      if (debug_contacts_out.is_open() && !wrote_debug_contact) {
        debug_contacts_out << i << ','
                           << std::setprecision(17) << row.time << ','
                           << data->ncon << ','
                           << row.ncon << ','
                           << -1 << ','
                           << -1 << ','
                           << -1 << ','
                           << 0 << ','
                           << -1 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << 0 << ','
                           << row.freewheel_vel << ','
                           << row.drive_vel << '\n';
      }

      WriteRow(out, row);

      if (!has_first_nonzero && row.ncon > 0 && row.fworld_mag > 0.0) {
        has_first_nonzero = true;
        first_nonzero = row;
      }
      if (row.fworld_mag > max_force.fworld_mag) {
        max_force = row;
      }

      max_abs_freewheel = std::max(max_abs_freewheel, std::abs(row.freewheel_vel));
      max_abs_drive = std::max(max_abs_drive, std::abs(row.drive_vel));
    }

    out.close();
    if (debug_contacts_out.is_open()) {
      debug_contacts_out.close();
    }

    if (has_first_nonzero) {
      std::cout << "first_nonzero,time=" << first_nonzero.time
                << ",fworld_mag=" << first_nonzero.fworld_mag
                << ",ncon=" << first_nonzero.ncon
                << ",freewheel_vel=" << first_nonzero.freewheel_vel
                << ",drive_vel=" << first_nonzero.drive_vel << "\n";
    } else {
      std::cout << "first_nonzero,none\n";
    }

    std::cout << "max_force_row,time=" << max_force.time
              << ",fworld_mag=" << max_force.fworld_mag
              << ",fn_sum=" << max_force.fn_sum
              << ",ncon=" << max_force.ncon << "\n";
    std::cout << "max_abs_freewheel_vel=" << max_abs_freewheel << "\n";
    std::cout << "max_abs_drive_vel=" << max_abs_drive << "\n";
    std::cout << "csv=" << std::filesystem::absolute(out_path).string() << "\n";

    sim_deleteData(data);
    sim_deleteModel(model);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
