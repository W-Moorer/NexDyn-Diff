#include <simcore/simcore.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>

namespace simcore::plugin::sdf {

namespace {

struct GearAttribute {
  static constexpr int kCount = 5;
  static constexpr const char* kNames[kCount] = {
      "alpha", "diameter", "teeth", "thickness", "innerdiameter"};
  static constexpr sim_scalar_t kDefaults[kCount] = {0, 2.8, 25, 0.2, -1};
};

constexpr const char* GearAttribute::kNames[GearAttribute::kCount];
constexpr sim_scalar_t GearAttribute::kDefaults[GearAttribute::kCount];

struct GearData {
  std::array<sim_scalar_t, GearAttribute::kCount> attribute{};
};

sim_scalar_t Union(sim_scalar_t a, sim_scalar_t b) {
  return sim_math_min(a, b);
}

sim_scalar_t Intersection(sim_scalar_t a, sim_scalar_t b) {
  return sim_math_max(a, b);
}

sim_scalar_t Subtraction(sim_scalar_t a, sim_scalar_t b) {
  return sim_math_max(a, -b);
}

sim_scalar_t SmoothUnion(sim_scalar_t a, sim_scalar_t b, sim_scalar_t k) {
  sim_scalar_t h = sim_math_clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
  return b * (1.0 - h) + a * h - k * h * (1.0 - h);
}

sim_scalar_t SmoothIntersection(sim_scalar_t a, sim_scalar_t b, sim_scalar_t k) {
  return Subtraction(Intersection(a, b), SmoothUnion(Subtraction(a, b), Subtraction(b, a), k));
}

sim_scalar_t Circle(sim_scalar_t rho, sim_scalar_t r) {
  return rho - r;
}

sim_scalar_t Extrusion(const sim_scalar_t p[3], sim_scalar_t sdf2d, sim_scalar_t h) {
  sim_scalar_t w[2] = {sdf2d, sim_math_abs(p[2]) - h};
  sim_scalar_t w_abs[2] = {sim_math_max(w[0], 0), sim_math_max(w[1], 0)};
  return sim_math_min(sim_math_max(w[0], w[1]), 0) + sim_math_norm(w_abs, 2);
}

sim_scalar_t Mod(sim_scalar_t x, sim_scalar_t y) {
  return x - y * sim_math_floor(x / y);
}

sim_scalar_t Distance2D(const sim_scalar_t p[3], const sim_scalar_t attr[GearAttribute::kCount]) {
  // Based on upstream MuJoCo sdf gear plugin implementation.
  sim_scalar_t D = attr[1];
  sim_scalar_t N = attr[2];
  sim_scalar_t psi = 3.096e-5 * N * N - 6.557e-3 * N + 0.551;
  sim_scalar_t alpha = attr[0];

  sim_scalar_t R = D / 2.0;
  sim_scalar_t rho = sim_math_norm(p, 2);

  sim_scalar_t Pd = N / D;
  sim_scalar_t P = SIM_PI / Pd;
  sim_scalar_t a = 1.0 / Pd;

  sim_scalar_t Do = D + 2.0 * a;
  sim_scalar_t Ro = Do / 2.0;
  sim_scalar_t h = 2.2 / Pd;

  sim_scalar_t innerR = Ro - h - 0.14 * D;
  if (attr[4] >= 0.0) {
    innerR = attr[4] / 2.0;
  }

  if (innerR - rho > 0.0) {
    return innerR - rho;
  }
  if (Ro - rho < -0.2) {
    return rho - Ro;
  }

  sim_scalar_t Db = D * sim_math_cos(psi);
  sim_scalar_t Rb = Db / 2.0;

  sim_scalar_t fi = sim_math_atan2(p[1], p[0]) + alpha;
  sim_scalar_t alpha_stride = P / R;

  sim_scalar_t inv_alpha = sim_math_acos(Rb / R);
  sim_scalar_t inv_phi = sim_math_tan(inv_alpha) - inv_alpha;
  sim_scalar_t shift = alpha_stride / 2.0 - 2.0 * inv_phi;

  sim_scalar_t fia = Mod(fi + shift / 2.0, alpha_stride) - shift / 2.0;
  sim_scalar_t fib = Mod(-fi - shift + shift / 2.0, alpha_stride) - shift / 2.0;

  sim_scalar_t dista = -1.0e6;
  sim_scalar_t distb = -1.0e6;

  if (Rb < rho) {
    sim_scalar_t acos_rb_rho = sim_math_acos(Rb / rho);
    sim_scalar_t thetaa = fia + acos_rb_rho;
    sim_scalar_t thetab = fib + acos_rb_rho;
    sim_scalar_t ta = sim_math_sqrt(rho * rho - Rb * Rb);
    dista = ta - Rb * thetaa;
    distb = ta - Rb * thetab;
  }

  sim_scalar_t gear_outer = Circle(rho, Ro);
  sim_scalar_t gear_low_base = Circle(rho, Ro - h);
  sim_scalar_t crown_base = Circle(rho, innerR);
  sim_scalar_t cogs = Intersection(dista, distb);
  sim_scalar_t base_walls =
      Intersection(fia - (alpha_stride - shift), fib - (alpha_stride - shift));

  cogs = Intersection(base_walls, cogs);
  cogs = SmoothIntersection(gear_outer, cogs, 0.0035 * D);
  cogs = SmoothUnion(gear_low_base, cogs, Rb - Ro + h);
  cogs = Subtraction(cogs, crown_base);
  return cogs;
}

sim_scalar_t Distance(const sim_scalar_t p[3], const sim_scalar_t attr[GearAttribute::kCount]) {
  return Extrusion(p, Distance2D(p, attr), attr[3] / 2.0);
}

void Gradient(sim_scalar_t grad[3], const sim_scalar_t p[3], const sim_scalar_t attr[GearAttribute::kCount]) {
  const sim_scalar_t eps = 1e-8;
  const sim_scalar_t d0 = Distance(p, attr);

  sim_scalar_t px[3] = {p[0] + eps, p[1], p[2]};
  sim_scalar_t py[3] = {p[0], p[1] + eps, p[2]};
  sim_scalar_t pz[3] = {p[0], p[1], p[2] + eps};
  grad[0] = (Distance(px, attr) - d0) / eps;
  grad[1] = (Distance(py, attr) - d0) / eps;
  grad[2] = (Distance(pz, attr) - d0) / eps;
}

bool CheckAttr(const char* name, const sim_model_t* m, int instance) {
  const char* value_raw = sim_getPluginConfig(m, instance, name);
  if (!value_raw) {
    return true;
  }
  std::string value(value_raw);
  value.erase(std::remove_if(value.begin(), value.end(),
                             [](unsigned char c) { return std::isspace(c); }),
              value.end());
  char* end = nullptr;
  std::strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

sim_scalar_t ReadAttrOrDefault(
    const sim_model_t* m, int instance, const char* name, sim_scalar_t def) {
  const char* value_raw = sim_getPluginConfig(m, instance, name);
  if (!value_raw) {
    return def;
  }
  std::string value(value_raw);
  value.erase(std::remove_if(value.begin(), value.end(),
                             [](unsigned char c) { return std::isspace(c); }),
              value.end());
  if (value.empty()) {
    return def;
  }
  char* end = nullptr;
  double parsed = std::strtod(value.c_str(), &end);
  if (end != value.data() + value.size()) {
    sim_warning("Invalid attribute value for '%s', falling back to default", name);
    return def;
  }
  return static_cast<sim_scalar_t>(parsed);
}

int GearInit(const sim_model_t* m, sim_data_t* d, int instance) {
  if (!CheckAttr("alpha", m, instance) || !CheckAttr("diameter", m, instance) ||
      !CheckAttr("teeth", m, instance) || !CheckAttr("thickness", m, instance) ||
      !CheckAttr("innerdiameter", m, instance)) {
    sim_warning("Invalid parameter specification in Gear plugin");
    return -1;
  }

  auto* data = new GearData();
  for (int i = 0; i < GearAttribute::kCount; ++i) {
    data->attribute[i] = ReadAttrOrDefault(
        m, instance, GearAttribute::kNames[i], GearAttribute::kDefaults[i]);
  }
  d->plugin_data[instance] = reinterpret_cast<uintptr_t>(data);
  return 0;
}

void GearDestroy(sim_data_t* d, int instance) {
  auto* data = reinterpret_cast<GearData*>(d->plugin_data[instance]);
  delete data;
  d->plugin_data[instance] = 0;
}

void GearReset(const sim_model_t*, sim_scalar_t*, void*, int) {}

void GearCompute(const sim_model_t*, sim_data_t*, int, int) {}

sim_scalar_t GearDistance(const sim_scalar_t p[3], const sim_data_t* d, int instance) {
  auto* data = reinterpret_cast<GearData*>(d->plugin_data[instance]);
  return Distance(p, data->attribute.data());
}

void GearGradient(sim_scalar_t grad[3], const sim_scalar_t p[3], const sim_data_t* d, int instance) {
  auto* data = reinterpret_cast<GearData*>(d->plugin_data[instance]);
  Gradient(grad, p, data->attribute.data());
}

sim_scalar_t GearStaticDistance(const sim_scalar_t p[3], const sim_scalar_t* attr) {
  return Distance(p, attr);
}

void GearAttributeFn(sim_scalar_t attr[], const char* name[], const char* value[]) {
  for (int i = 0; i < GearAttribute::kCount; ++i) {
    const char* key = name ? name[i] : GearAttribute::kNames[i];
    const char* raw = value ? value[i] : nullptr;
    if (!raw || std::string(raw).empty()) {
      attr[i] = GearAttribute::kDefaults[i];
      continue;
    }
    char* end = nullptr;
    double parsed = std::strtod(raw, &end);
    if (end != raw + std::strlen(raw)) {
      sim_warning("Invalid attribute '%s' value, using default", key ? key : "unknown");
      attr[i] = GearAttribute::kDefaults[i];
    } else {
      attr[i] = static_cast<sim_scalar_t>(parsed);
    }
  }
}

void GearAabb(sim_scalar_t aabb[6], const sim_scalar_t* attr) {
  aabb[0] = 0;
  aabb[1] = 0;
  aabb[2] = 0;
  aabb[3] = attr[1] / 2.0 * 1.25;
  aabb[4] = attr[1] / 2.0 * 1.25;
  aabb[5] = attr[3] / 2.0 * 1.1;
}

int RegisterGearPluginWithName(const char* plugin_name) {
  SIM_pPlugin plugin;
  sim_plugin_defaultPlugin(&plugin);

  plugin.name = plugin_name;
  plugin.capabilityflags |= SIM_PLUGIN_SDF;
  plugin.nattribute = GearAttribute::kCount;
  plugin.attributes = GearAttribute::kNames;

  plugin.nstate = +[](const sim_model_t*, int) { return 0; };
  plugin.init = GearInit;
  plugin.destroy = GearDestroy;
  plugin.reset = GearReset;
  plugin.compute = GearCompute;
  plugin.sdf_distance = GearDistance;
  plugin.sdf_gradient = GearGradient;
  plugin.sdf_staticdistance = GearStaticDistance;
  plugin.sdf_attribute = GearAttributeFn;
  plugin.sdf_aabb = GearAabb;

  return sim_plugin_registerPlugin(&plugin);
}

int RegisterGearPlugin() {
  // Preferred generic name for this project.
  const int primary_slot = RegisterGearPluginWithName("simcore.sdf.gear");

  // Backward-compatible alias for old XML files.
  (void)RegisterGearPluginWithName("mujoco.sdf.gear");

  return primary_slot;
}

}  // namespace

}  // namespace simcore::plugin::sdf

SIM_PLUGIN_LIB_INIT {
  (void)simcore::plugin::sdf::RegisterGearPlugin();
}
