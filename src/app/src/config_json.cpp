#include "falconguide/app/config_json.hpp"

#include <Eigen/Core>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace falconguide::app {

using json = nlohmann::json;
using namespace estimation;

// ── helpers ───────────────────────────────────────────────────────────────────

static json jpath()  { return {{"type","string"},{"x-ui-hint","path"}}; }
static json jstr()   { return {{"type","string"}}; }
static json jbool()  { return {{"type","boolean"}}; }
static json jnum(double mn) { return {{"type","number"},{"minimum",mn}}; }
static json jnum(double mn, double mx) { return {{"type","number"},{"minimum",mn},{"maximum",mx}}; }
static json jint(int mn)    { return {{"type","integer"},{"minimum",mn}}; }
static json jtabs(std::initializer_list<const char*> v) {
  return {{"type","string"},{"enum",v},{"x-ui-hint","tabs"}};
}
static json jvec3() {
  return {{"type","array"},{"items",{{"type","number"}}},{"minItems",3},{"maxItems",3}};
}

static std::optional<double> ReadOptDouble(const json& j, const std::string& key) {
  if (!j.contains(key) || j[key].is_null()) return std::nullopt;
  return j[key].get<double>();
}

static json WriteOptDouble(std::optional<double> v) {
  if (!v) return nullptr;
  return *v;
}

static Eigen::Vector3d ReadVec3(const json& j, const std::string& key, Eigen::Vector3d def) {
  if (!j.contains(key) || !j[key].is_array()) return def;
  const auto& a = j[key];
  if (a.size() < 3) return def;
  return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
}

static json WriteVec3(const Eigen::Vector3d& v) {
  return json::array({v.x(), v.y(), v.z()});
}

static json WriteMat3(const Eigen::Matrix3d& m) {
  json a = json::array();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      a.push_back(m(r, c));
  return a;
}

static Eigen::Matrix3d ReadMat3(const json& j, const std::string& key) {
  if (!j.contains(key) || !j[key].is_array() || j[key].size() < 9)
    return Eigen::Matrix3d::Identity();
  const auto& a = j[key];
  Eigen::Matrix3d m;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      m(r, c) = a[r * 3 + c].get<double>();
  return m;
}

// ── nav config from/to JSON ───────────────────────────────────────────────────

static NavigationSystemConfig NavFromJson(const json& j) {
  NavigationSystemConfig cfg;

  // backend
  if (j.contains("backend")) {
    const auto b = j["backend"].get<std::string>();
    if      (b == "UKF")   cfg.backend = EstimatorBackendChoice::Ukf;
    else if (b == "Ceres") cfg.backend = EstimatorBackendChoice::Ceres;
    else if (b == "GTSAM") cfg.backend = EstimatorBackendChoice::Gtsam;
    else                   cfg.backend = EstimatorBackendChoice::Ekf;
  }

  if (j.contains("dead_reckoning_threshold_s")) cfg.dead_reckoning_threshold_s = j["dead_reckoning_threshold_s"];
  if (j.contains("min_imu_dt_s"))               cfg.min_imu_dt_s               = j["min_imu_dt_s"];
  if (j.contains("max_imu_dt_s"))               cfg.max_imu_dt_s               = j["max_imu_dt_s"];

  // IMU
  if (j.contains("imu")) {
    const auto& ji = j["imu"];
    auto& n = cfg.imu.noise;
    if (ji.contains("accel_noise_density"))  n.accel_noise_density_mps2_per_sqrthz  = ji["accel_noise_density"];
    if (ji.contains("gyro_noise_density"))   n.gyro_noise_density_radps_per_sqrthz  = ji["gyro_noise_density"];
    if (ji.contains("accel_random_walk"))    n.accel_random_walk_mps3_per_sqrthz    = ji["accel_random_walk"];
    if (ji.contains("gyro_random_walk"))     n.gyro_random_walk_radps2_per_sqrthz   = ji["gyro_random_walk"];
  }

  // GNSS
  if (j.contains("gnss")) {
    const auto& jg = j["gnss"];
    cfg.gnss.enabled = jg.value("enabled", false);
    const auto mode  = jg.value("mode", std::string("LooselyCoupled"));
    cfg.gnss.mode = (mode == "TightlyCoupled") ? GnssIntegrationMode::TightlyCoupled
                                               : GnssIntegrationMode::LooselyCoupled;
    if (jg.contains("loosely_coupled")) {
      const auto& lc = jg["loosely_coupled"];
      cfg.gnss.loosely_coupled.position_sigma_m = ReadOptDouble(lc, "position_sigma_m");
      cfg.gnss.loosely_coupled.velocity_sigma_mps = ReadOptDouble(lc, "velocity_sigma_mps");
      if (lc.contains("position_gate")) cfg.gnss.loosely_coupled.position_gate = lc["position_gate"];
    }
    if (jg.contains("tightly_coupled")) {
      const auto& tc = jg["tightly_coupled"];
      if (tc.contains("base_pseudorange_sigma_m"))   cfg.gnss.tightly_coupled.base_pseudorange_sigma_m   = tc["base_pseudorange_sigma_m"];
      if (tc.contains("pseudorange_noise_floor_m"))  cfg.gnss.tightly_coupled.pseudorange_noise_floor_m  = tc["pseudorange_noise_floor_m"];
      if (tc.contains("base_doppler_sigma_mps"))     cfg.gnss.tightly_coupled.base_doppler_sigma_mps     = tc["base_doppler_sigma_mps"];
      if (tc.contains("innovation_gate_sigma"))      cfg.gnss.tightly_coupled.innovation_gate_sigma      = tc["innovation_gate_sigma"];
    }
  }

  // Barometer
  if (j.contains("barometer")) {
    const auto& jb = j["barometer"];
    cfg.barometer.enabled = jb.value("enabled", false);
    cfg.barometer.options.altitude_sigma_m         = ReadOptDouble(jb, "altitude_sigma_m");
    cfg.barometer.options.isa_sea_level_pressure_pa = jb.value("isa_sea_level_pressure_pa", 101325.0);
    cfg.barometer.options.innovation_gate           = jb.value("innovation_gate", 0.0);
  }

  // Magnetometer
  if (j.contains("magnetometer")) {
    const auto& jm = j["magnetometer"];
    cfg.magnetometer.enabled = jm.value("enabled", false);
    cfg.magnetometer.options.reference_field_enu_tesla =
        ReadVec3(jm, "reference_field_enu_tesla", cfg.magnetometer.options.reference_field_enu_tesla);
    cfg.magnetometer.options.sigma_tesla    = ReadOptDouble(jm, "sigma_tesla");
    cfg.magnetometer.options.innovation_gate = jm.value("innovation_gate", 0.0);
  }

  // Star tracker
  if (j.contains("star_tracker")) {
    const auto& js = j["star_tracker"];
    cfg.star_tracker.enabled = js.value("enabled", false);
    if (js.contains("sigma_yaw_rad"))   cfg.star_tracker.options.sigma_yaw_rad   = js["sigma_yaw_rad"];
    if (js.contains("sigma_pitch_rad")) cfg.star_tracker.options.sigma_pitch_rad = js["sigma_pitch_rad"];
    if (js.contains("sigma_roll_rad"))  cfg.star_tracker.options.sigma_roll_rad  = js["sigma_roll_rad"];
    if (js.contains("use_pitch_roll"))  cfg.star_tracker.options.use_pitch_roll  = js["use_pitch_roll"];
    if (js.contains("innovation_gate")) cfg.star_tracker.options.innovation_gate = js["innovation_gate"];
  }

  // Wheel odometry
  if (j.contains("wheel_odometry")) {
    const auto& jw = j["wheel_odometry"];
    cfg.wheel_odometry.enabled = jw.value("enabled", false);
    if (jw.contains("sigma_linear_mps"))   cfg.wheel_odometry.options.sigma_linear_mps   = jw["sigma_linear_mps"];
    if (jw.contains("sigma_angular_radps")) cfg.wheel_odometry.options.sigma_angular_radps = jw["sigma_angular_radps"];
    if (jw.contains("use_angular_rate"))   cfg.wheel_odometry.options.use_angular_rate   = jw["use_angular_rate"];
    if (jw.contains("innovation_gate"))    cfg.wheel_odometry.options.innovation_gate    = jw["innovation_gate"];
  }

  // Radar altimeter
  if (j.contains("radar_altimeter")) {
    const auto& jr = j["radar_altimeter"];
    cfg.radar_altimeter.enabled = jr.value("enabled", false);
    if (jr.contains("innovation_gate")) cfg.radar_altimeter.options.innovation_gate = jr["innovation_gate"];
  }

  // Range finder
  if (j.contains("range_finder")) {
    const auto& jr = j["range_finder"];
    cfg.range_finder.enabled = jr.value("enabled", false);
    if (jr.contains("expected_range_m"))    cfg.range_finder.options.expected_range_m    = jr["expected_range_m"];
    if (jr.contains("use_as_altitude"))     cfg.range_finder.options.use_as_altitude     = jr["use_as_altitude"];
    if (jr.contains("terrain_elevation_m")) cfg.range_finder.options.terrain_elevation_m = jr["terrain_elevation_m"];
    cfg.range_finder.options.sigma_m = ReadOptDouble(jr, "sigma_m");
    if (jr.contains("innovation_gate"))     cfg.range_finder.options.innovation_gate     = jr["innovation_gate"];
  }

  // Optical flow
  if (j.contains("optical_flow")) {
    const auto& jo = j["optical_flow"];
    cfg.optical_flow.enabled = jo.value("enabled", false);
    if (jo.contains("fallback_altitude_m")) cfg.optical_flow.options.fallback_altitude_m = jo["fallback_altitude_m"];
    if (jo.contains("sigma_radps"))         cfg.optical_flow.options.sigma_radps         = jo["sigma_radps"];
    if (jo.contains("innovation_gate"))     cfg.optical_flow.options.innovation_gate     = jo["innovation_gate"];
  }

  // DVL
  if (j.contains("dvl")) {
    const auto& jd = j["dvl"];
    cfg.dvl.enabled = jd.value("enabled", false);
    cfg.dvl.options.dvl_to_body_rotation = ReadMat3(jd, "dvl_to_body_rotation");
    if (jd.contains("bottom_track_only")) cfg.dvl.options.bottom_track_only = jd["bottom_track_only"];
    cfg.dvl.options.sigma_mps = ReadOptDouble(jd, "sigma_mps");
    if (jd.contains("innovation_gate"))   cfg.dvl.options.innovation_gate   = jd["innovation_gate"];
  }

  // Echo sounder
  if (j.contains("echo_sounder")) {
    const auto& je = j["echo_sounder"];
    cfg.echo_sounder.enabled = je.value("enabled", false);
    if (je.contains("sound_speed_mps"))  cfg.echo_sounder.options.sound_speed_mps  = je["sound_speed_mps"];
    if (je.contains("depth_origin_m"))   cfg.echo_sounder.options.depth_origin_m   = je["depth_origin_m"];
    cfg.echo_sounder.options.sigma_m = ReadOptDouble(je, "sigma_m");
    if (je.contains("innovation_gate"))  cfg.echo_sounder.options.innovation_gate  = je["innovation_gate"];
  }

  // Airspeed
  if (j.contains("airspeed")) {
    const auto& ja = j["airspeed"];
    cfg.airspeed.enabled = ja.value("enabled", false);
    cfg.airspeed.options.wind_enu_mps = ReadVec3(ja, "wind_enu_mps", Eigen::Vector3d::Zero());
    if (ja.contains("sigma_mps"))        cfg.airspeed.options.sigma_mps        = ja["sigma_mps"];
    if (ja.contains("innovation_gate"))  cfg.airspeed.options.innovation_gate  = ja["innovation_gate"];
  }

  // External pose
  if (j.contains("external_pose")) {
    const auto& je = j["external_pose"];
    cfg.external_pose.enabled = je.value("enabled", false);
    if (je.contains("use_position"))    cfg.external_pose.options.use_position    = je["use_position"];
    if (je.contains("use_orientation")) cfg.external_pose.options.use_orientation = je["use_orientation"];
    if (je.contains("innovation_gate")) cfg.external_pose.options.innovation_gate = je["innovation_gate"];
  }

  // External velocity
  if (j.contains("external_velocity")) {
    const auto& je = j["external_velocity"];
    cfg.external_velocity.enabled = je.value("enabled", false);
    if (je.contains("use_angular_rate")) cfg.external_velocity.options.use_angular_rate = je["use_angular_rate"];
    if (je.contains("innovation_gate"))  cfg.external_velocity.options.innovation_gate  = je["innovation_gate"];
  }

  // External odometry
  if (j.contains("external_odometry")) {
    const auto& je = j["external_odometry"];
    cfg.external_odometry.enabled = je.value("enabled", false);
    if (je.contains("use_position"))    cfg.external_odometry.options.use_position    = je["use_position"];
    if (je.contains("use_velocity"))    cfg.external_odometry.options.use_velocity    = je["use_velocity"];
    if (je.contains("use_orientation")) cfg.external_odometry.options.use_orientation = je["use_orientation"];
    cfg.external_odometry.options.position_sigma_m    = ReadOptDouble(je, "position_sigma_m");
    cfg.external_odometry.options.velocity_sigma_mps  = ReadOptDouble(je, "velocity_sigma_mps");
    cfg.external_odometry.options.orientation_sigma_rad = ReadOptDouble(je, "orientation_sigma_rad");
    if (je.contains("innovation_gate")) cfg.external_odometry.options.innovation_gate = je["innovation_gate"];
  }

  // Aiding solution
  if (j.contains("aiding_solution")) {
    const auto& ja = j["aiding_solution"];
    cfg.aiding_solution.enabled = ja.value("enabled", false);
    if (ja.contains("min_match_score"))  cfg.aiding_solution.options.min_match_score  = ja["min_match_score"];
    cfg.aiding_solution.options.position_sigma_m    = ReadOptDouble(ja, "position_sigma_m");
    cfg.aiding_solution.options.velocity_sigma_mps  = ReadOptDouble(ja, "velocity_sigma_mps");
    cfg.aiding_solution.options.attitude_sigma_rad  = ReadOptDouble(ja, "attitude_sigma_rad");
    if (ja.contains("innovation_gate")) cfg.aiding_solution.options.innovation_gate = ja["innovation_gate"];
  }

  // UKF sigma params
  if (j.contains("ukf_sigma")) {
    const auto& ju = j["ukf_sigma"];
    if (ju.contains("alpha")) cfg.ukf_sigma_params.alpha = ju["alpha"];
    if (ju.contains("beta"))  cfg.ukf_sigma_params.beta  = ju["beta"];
    if (ju.contains("kappa")) cfg.ukf_sigma_params.kappa = ju["kappa"];
  }

  return cfg;
}

static json NavToJson(const NavigationSystemConfig& cfg) {
  json j;

  switch (cfg.backend) {
    case EstimatorBackendChoice::Ukf:   j["backend"] = "UKF";   break;
    case EstimatorBackendChoice::Ceres: j["backend"] = "Ceres"; break;
    case EstimatorBackendChoice::Gtsam: j["backend"] = "GTSAM"; break;
    default:                            j["backend"] = "EKF";   break;
  }
  j["dead_reckoning_threshold_s"] = cfg.dead_reckoning_threshold_s;
  j["min_imu_dt_s"] = cfg.min_imu_dt_s;
  j["max_imu_dt_s"] = cfg.max_imu_dt_s;

  const auto& n = cfg.imu.noise;
  j["imu"] = {
    {"accel_noise_density",  n.accel_noise_density_mps2_per_sqrthz},
    {"gyro_noise_density",   n.gyro_noise_density_radps_per_sqrthz},
    {"accel_random_walk",    n.accel_random_walk_mps3_per_sqrthz},
    {"gyro_random_walk",     n.gyro_random_walk_radps2_per_sqrthz},
  };

  j["gnss"] = {
    {"enabled", cfg.gnss.enabled},
    {"mode",    cfg.gnss.mode == GnssIntegrationMode::TightlyCoupled ? "TightlyCoupled" : "LooselyCoupled"},
    {"loosely_coupled", {
      {"position_sigma_m",   WriteOptDouble(cfg.gnss.loosely_coupled.position_sigma_m)},
      {"velocity_sigma_mps", WriteOptDouble(cfg.gnss.loosely_coupled.velocity_sigma_mps)},
      {"position_gate",      cfg.gnss.loosely_coupled.position_gate},
    }},
    {"tightly_coupled", {
      {"base_pseudorange_sigma_m",  cfg.gnss.tightly_coupled.base_pseudorange_sigma_m},
      {"pseudorange_noise_floor_m", cfg.gnss.tightly_coupled.pseudorange_noise_floor_m},
      {"base_doppler_sigma_mps",    cfg.gnss.tightly_coupled.base_doppler_sigma_mps},
      {"innovation_gate_sigma",     cfg.gnss.tightly_coupled.innovation_gate_sigma},
    }},
  };

  j["barometer"] = {
    {"enabled",                    cfg.barometer.enabled},
    {"altitude_sigma_m",           WriteOptDouble(cfg.barometer.options.altitude_sigma_m)},
    {"isa_sea_level_pressure_pa",  cfg.barometer.options.isa_sea_level_pressure_pa},
    {"innovation_gate",            cfg.barometer.options.innovation_gate},
  };

  j["magnetometer"] = {
    {"enabled",                   cfg.magnetometer.enabled},
    {"reference_field_enu_tesla", WriteVec3(cfg.magnetometer.options.reference_field_enu_tesla)},
    {"sigma_tesla",               WriteOptDouble(cfg.magnetometer.options.sigma_tesla)},
    {"innovation_gate",           cfg.magnetometer.options.innovation_gate},
  };

  j["star_tracker"] = {
    {"enabled",         cfg.star_tracker.enabled},
    {"sigma_yaw_rad",   cfg.star_tracker.options.sigma_yaw_rad},
    {"sigma_pitch_rad", cfg.star_tracker.options.sigma_pitch_rad},
    {"sigma_roll_rad",  cfg.star_tracker.options.sigma_roll_rad},
    {"use_pitch_roll",  cfg.star_tracker.options.use_pitch_roll},
    {"innovation_gate", cfg.star_tracker.options.innovation_gate},
  };

  j["wheel_odometry"] = {
    {"enabled",            cfg.wheel_odometry.enabled},
    {"sigma_linear_mps",   cfg.wheel_odometry.options.sigma_linear_mps},
    {"sigma_angular_radps",cfg.wheel_odometry.options.sigma_angular_radps},
    {"use_angular_rate",   cfg.wheel_odometry.options.use_angular_rate},
    {"innovation_gate",    cfg.wheel_odometry.options.innovation_gate},
  };

  j["radar_altimeter"] = {
    {"enabled",         cfg.radar_altimeter.enabled},
    {"innovation_gate", cfg.radar_altimeter.options.innovation_gate},
  };

  j["range_finder"] = {
    {"enabled",            cfg.range_finder.enabled},
    {"expected_range_m",   cfg.range_finder.options.expected_range_m},
    {"use_as_altitude",    cfg.range_finder.options.use_as_altitude},
    {"terrain_elevation_m",cfg.range_finder.options.terrain_elevation_m},
    {"sigma_m",            WriteOptDouble(cfg.range_finder.options.sigma_m)},
    {"innovation_gate",    cfg.range_finder.options.innovation_gate},
  };

  j["optical_flow"] = {
    {"enabled",             cfg.optical_flow.enabled},
    {"fallback_altitude_m", cfg.optical_flow.options.fallback_altitude_m},
    {"sigma_radps",         cfg.optical_flow.options.sigma_radps},
    {"innovation_gate",     cfg.optical_flow.options.innovation_gate},
  };

  j["dvl"] = {
    {"enabled",             cfg.dvl.enabled},
    {"dvl_to_body_rotation",WriteMat3(cfg.dvl.options.dvl_to_body_rotation)},
    {"bottom_track_only",   cfg.dvl.options.bottom_track_only},
    {"sigma_mps",           WriteOptDouble(cfg.dvl.options.sigma_mps)},
    {"innovation_gate",     cfg.dvl.options.innovation_gate},
  };

  j["echo_sounder"] = {
    {"enabled",        cfg.echo_sounder.enabled},
    {"sound_speed_mps",cfg.echo_sounder.options.sound_speed_mps},
    {"depth_origin_m", cfg.echo_sounder.options.depth_origin_m},
    {"sigma_m",        WriteOptDouble(cfg.echo_sounder.options.sigma_m)},
    {"innovation_gate",cfg.echo_sounder.options.innovation_gate},
  };

  j["airspeed"] = {
    {"enabled",        cfg.airspeed.enabled},
    {"wind_enu_mps",   WriteVec3(cfg.airspeed.options.wind_enu_mps)},
    {"sigma_mps",      cfg.airspeed.options.sigma_mps},
    {"innovation_gate",cfg.airspeed.options.innovation_gate},
  };

  j["external_pose"] = {
    {"enabled",         cfg.external_pose.enabled},
    {"use_position",    cfg.external_pose.options.use_position},
    {"use_orientation", cfg.external_pose.options.use_orientation},
    {"innovation_gate", cfg.external_pose.options.innovation_gate},
  };

  j["external_velocity"] = {
    {"enabled",          cfg.external_velocity.enabled},
    {"use_angular_rate", cfg.external_velocity.options.use_angular_rate},
    {"innovation_gate",  cfg.external_velocity.options.innovation_gate},
  };

  j["external_odometry"] = {
    {"enabled",              cfg.external_odometry.enabled},
    {"use_position",         cfg.external_odometry.options.use_position},
    {"use_velocity",         cfg.external_odometry.options.use_velocity},
    {"use_orientation",      cfg.external_odometry.options.use_orientation},
    {"position_sigma_m",     WriteOptDouble(cfg.external_odometry.options.position_sigma_m)},
    {"velocity_sigma_mps",   WriteOptDouble(cfg.external_odometry.options.velocity_sigma_mps)},
    {"orientation_sigma_rad",WriteOptDouble(cfg.external_odometry.options.orientation_sigma_rad)},
    {"innovation_gate",      cfg.external_odometry.options.innovation_gate},
  };

  j["aiding_solution"] = {
    {"enabled",            cfg.aiding_solution.enabled},
    {"min_match_score",    cfg.aiding_solution.options.min_match_score},
    {"position_sigma_m",   WriteOptDouble(cfg.aiding_solution.options.position_sigma_m)},
    {"velocity_sigma_mps", WriteOptDouble(cfg.aiding_solution.options.velocity_sigma_mps)},
    {"attitude_sigma_rad", WriteOptDouble(cfg.aiding_solution.options.attitude_sigma_rad)},
    {"innovation_gate",    cfg.aiding_solution.options.innovation_gate},
  };

  j["ukf_sigma"] = {
    {"alpha", cfg.ukf_sigma_params.alpha},
    {"beta",  cfg.ukf_sigma_params.beta},
    {"kappa", cfg.ukf_sigma_params.kappa},
  };

  return j;
}

// ── SessionConfig ─────────────────────────────────────────────────────────────

SessionConfig SessionConfig::FromJson(const std::string& json_str) {
  SessionConfig sc;
  try {
    const auto j = json::parse(json_str);

    if (j.contains("nav")) sc.nav = NavFromJson(j["nav"]);

    if (j.contains("dataset")) {
      const auto& jd = j["dataset"];
      sc.dataset.type = jd.value("type", std::string("csv"));
      sc.dataset.path = jd.value("path", std::string{});
    }

    if (j.contains("ipc")) {
      const auto& ji = j["ipc"];
      sc.ipc.socket_path = ji.value("socket_path", std::string("/tmp/falconguide.sock"));
      sc.ipc.max_clients = ji.value("max_clients", 8);
    }

    sc.playback_speed = j.value("playback_speed", 1.0);

  } catch (const std::exception& ex) {
    throw std::invalid_argument(std::string("SessionConfig::FromJson: ") + ex.what());
  }
  return sc;
}

SessionConfig SessionConfig::FromPath(const std::string& path) {
  if (!std::filesystem::exists(path))
    throw std::invalid_argument("Config file not found: " + path);
  std::ifstream f(path);
  if (!f.is_open())
    throw std::runtime_error("Cannot open config file: " + path);
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  return FromJson(s);
}

std::string SessionConfig::ToJson() const {
  json j;
  j["nav"]            = NavToJson(nav);
  j["dataset"]        = {{"type", dataset.type}, {"path", dataset.path}};
  j["ipc"]            = {{"socket_path", ipc.socket_path}, {"max_clients", ipc.max_clients}};
  j["playback_speed"] = playback_speed;
  return j.dump(2);
}

void SessionConfig::SaveToPath(const std::string& path) const {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
  std::ofstream f(path);
  if (!f.is_open()) throw std::runtime_error("Cannot write config: " + path);
  f << ToJson();
}

nlohmann::json SessionConfig::ToSchemaJson() {
  return {
    {"playback_speed", jnum(0.0)},
    {"dataset", {{"type","object"},{"properties",{
      {"type",  jtabs({"csv"})},
      {"path",  jpath()},
    }}}},
    {"ipc", {{"type","object"},{"properties",{
      {"socket_path", jstr()},
      {"max_clients", jint(1)},
    }}}},
    {"nav", {{"type","object"},{"properties",{
      {"backend",                       jtabs({"EKF","UKF","Ceres","GTSAM"})},
      {"dead_reckoning_threshold_s",    jnum(0.0)},
      {"min_imu_dt_s",                  jnum(0.0)},
      {"max_imu_dt_s",                  jnum(0.0)},
      {"imu", {{"type","object"},{"properties",{
        {"accel_noise_density",  jnum(0.0)},
        {"gyro_noise_density",   jnum(0.0)},
        {"accel_random_walk",    jnum(0.0)},
        {"gyro_random_walk",     jnum(0.0)},
      }}}},
      {"gnss", {{"type","object"},{"properties",{
        {"enabled", jbool()},
        {"mode",    jtabs({"LooselyCoupled","TightlyCoupled"})},
        {"loosely_coupled", {{"type","object"},{"properties",{
          {"position_sigma_m",   jnum(0.0)},
          {"velocity_sigma_mps", jnum(0.0)},
          {"position_gate",      jnum(0.0)},
        }}}},
      }}}},
      {"barometer", {{"type","object"},{"properties",{
        {"enabled",                   jbool()},
        {"altitude_sigma_m",          jnum(0.0)},
        {"isa_sea_level_pressure_pa", jnum(0.0)},
        {"innovation_gate",           jnum(0.0)},
      }}}},
      {"magnetometer", {{"type","object"},{"properties",{
        {"enabled",                   jbool()},
        {"reference_field_enu_tesla", jvec3()},
        {"sigma_tesla",               jnum(0.0)},
        {"innovation_gate",           jnum(0.0)},
      }}}},
      {"wheel_odometry", {{"type","object"},{"properties",{
        {"enabled",            jbool()},
        {"sigma_linear_mps",   jnum(0.0)},
        {"sigma_angular_radps",jnum(0.0)},
        {"use_angular_rate",   jbool()},
        {"innovation_gate",    jnum(0.0)},
      }}}},
      {"optical_flow", {{"type","object"},{"properties",{
        {"enabled",             jbool()},
        {"fallback_altitude_m", jnum(0.0)},
        {"sigma_radps",         jnum(0.0)},
        {"innovation_gate",     jnum(0.0)},
      }}}},
      {"airspeed", {{"type","object"},{"properties",{
        {"enabled",        jbool()},
        {"wind_enu_mps",   jvec3()},
        {"sigma_mps",      jnum(0.0)},
        {"innovation_gate",jnum(0.0)},
      }}}},
    }}}},
  };
}

bool SessionConfig::Validate(std::string& error) const {
  if (dataset.path.empty()) { error = "dataset.path is empty"; return false; }
  if (ipc.socket_path.empty()) { error = "ipc.socket_path is empty"; return false; }
  if (ipc.max_clients < 1) { error = "ipc.max_clients must be >= 1"; return false; }
  if (playback_speed < 0.0) { error = "playback_speed must be >= 0 (0 = as fast as possible)"; return false; }
  return true;
}

}  // namespace falconguide::app
