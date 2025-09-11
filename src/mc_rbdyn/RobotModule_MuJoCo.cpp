#include <mc_rbdyn/MuJoCo.h>
#include <mc_rbdyn/RobotModule.h>
#include <locale>
#include <sstream>
#include <tinyxml2.h>

using namespace tinyxml2;

namespace
{
std::string vec3_to_str(const Eigen::Vector3d & v)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << v.x() << " " << v.y() << " " << v.z();
  return oss.str();
}
std::string vec4_to_str(const Eigen::Vector4d & v)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << v.x() << " " << v.y() << " " << v.z() << " " << v.w();
  return oss.str();
}
std::string double_to_str(double v)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << v;
  return oss.str();
}
} // namespace

namespace mc_rbdyn
{

tinyxml2::XMLElement * RobotModule::toMCJF(tinyxml2::XMLDocument & doc) const
{
  auto mujoco = doc.NewElement("mujoco");
  mujoco->SetAttribute("model", name.c_str());

  // <compiler>
  auto compiler = doc.NewElement("compiler");
  compiler->SetAttribute("angle", "radian");
  compiler->SetAttribute("meshdir", "../meshes");
  mujoco->InsertEndChild(compiler);

  // <size>
  auto size = doc.NewElement("size");
  size->SetAttribute("njmax", "2000");
  size->SetAttribute("nconmax", "500");
  mujoco->InsertEndChild(size);

  // <option>
  auto option = doc.NewElement("option");
  option->SetAttribute("timestep", "0.001");
  option->SetAttribute("impratio", "5");
  option->SetAttribute("iterations", "50");
  option->SetAttribute("tolerance", "1e-10");
  option->SetAttribute("solver", "Newton");
  option->SetAttribute("jacobian", "dense");
  option->SetAttribute("cone", "pyramidal");
  mujoco->InsertEndChild(option);

  // <default> (as before)
  auto default_elem = doc.NewElement("default");
  auto joint_default = doc.NewElement("joint");
  joint_default->SetAttribute("limited", "true");
  joint_default->SetAttribute("damping", "0.2");
  joint_default->SetAttribute("pos", "0 0 0");
  default_elem->InsertEndChild(joint_default);
  auto motor_default = doc.NewElement("motor");
  motor_default->SetAttribute("ctrllimited", "false");
  motor_default->SetAttribute("forcelimited", "false");
  default_elem->InsertEndChild(motor_default);
  auto collision_default = doc.NewElement("default");
  collision_default->SetAttribute("class", "collision");
  auto collision_geom = doc.NewElement("geom");
  collision_geom->SetAttribute("condim", "3");
  collision_geom->SetAttribute("material", "matgeom");
  collision_geom->SetAttribute("type", "mesh");
  collision_geom->SetAttribute("group", "0");
  collision_default->InsertEndChild(collision_geom);
  default_elem->InsertEndChild(collision_default);
  auto visual_default = doc.NewElement("default");
  visual_default->SetAttribute("class", "visual");
  auto visual_geom = doc.NewElement("geom");
  visual_geom->SetAttribute("condim", "3");
  visual_geom->SetAttribute("material", "matgeom");
  visual_geom->SetAttribute("type", "mesh");
  visual_geom->SetAttribute("group", "1");
  visual_geom->SetAttribute("conaffinity", "0");
  visual_geom->SetAttribute("contype", "0");
  visual_default->InsertEndChild(visual_geom);
  default_elem->InsertEndChild(visual_default);
  mujoco->InsertEndChild(default_elem);

  // <asset> (add meshes/materials as needed)
  auto asset = doc.NewElement("asset");
  // TODO: Iterate _visual/_collision and add <mesh> elements
  mujoco->InsertEndChild(asset);

  // <worldbody>
  auto worldbody = doc.NewElement("worldbody");

  // Helper: Recursively add bodies
  std::function<void(int, XMLElement *)> add_body;
  add_body = [&](int bodyIdx, XMLElement * parent)
  {
    const auto & body = mb.body(bodyIdx);
    auto body_elem = doc.NewElement("body");
    body_elem->SetAttribute("name", body.name().c_str());
    // Set position if needed (use mbc, or zero for root)
    // TODO: Set pos/quat from mbc if available

    // <inertial>
    if(body.inertia().mass() > 0)
    {
      auto inertial = doc.NewElement("inertial");
      inertial->SetAttribute("mass", double_to_str(body.inertia().mass()).c_str());
      // TODO: Set pos/quat/diaginertia from inertia
      body_elem->InsertEndChild(inertial);
    }

    // <geom> (collision)
    auto it_geom = mujocoMetadata.geom.find(body.name());
    if(it_geom != mujocoMetadata.geom.end())
    {
      const auto & meta = it_geom->second;
      auto geom = doc.NewElement("geom");
      geom->SetAttribute("name", body.name().c_str());
      geom->SetAttribute("friction", (double_to_str(meta.friction_slide) + " " + double_to_str(meta.friction_spin) + " "
                                      + double_to_str(meta.friction_roll))
                                         .c_str());
      geom->SetAttribute("margin", double_to_str(meta.margin).c_str());
      geom->SetAttribute("group", meta.group);
      geom->SetAttribute("material", meta.material.c_str());
      // TODO: Set mesh/type/size/pos as needed
      body_elem->InsertEndChild(geom);
    }

    // <site> (for sensors, etc.)
    for(const auto & [site_name, site_meta] : mujocoMetadata.site)
    {
      if(site_meta.name == body.name()) // or use a mapping
      {
        auto site = doc.NewElement("site");
        site->SetAttribute("name", site_meta.name.c_str());
        site->SetAttribute("pos", vec3_to_str(site_meta.pos).c_str());
        site->SetAttribute("quat", vec4_to_str(site_meta.quat).c_str());
        site->SetAttribute("size", vec3_to_str(site_meta.size).c_str());
        site->SetAttribute("type", site_meta.type.c_str());
        site->SetAttribute("group", site_meta.group);
        body_elem->InsertEndChild(site);
      }
    }

    // <joint>
    for(int j = 0; j < mb.nrJoints(); ++j)
    {
      const auto & joint = mb.joint(j);
      if(mb.predecessor(j) == bodyIdx)
      {
        auto joint_elem = doc.NewElement("joint");
        joint_elem->SetAttribute("name", joint.name().c_str());
        // TODO: Set type, axis, range, armature, etc.
        body_elem->InsertEndChild(joint_elem);
      }
    }

    // Recursively add children
    for(int c = 0; c < mb.nrBodies(); ++c)
    {
      if(mb.parent(c) == bodyIdx) { add_body(c, body_elem); }
    }
    parent->InsertEndChild(body_elem);
  };

  // Start recursion from root body (usually index 0)
  add_body(0, worldbody);
  mujoco->InsertEndChild(worldbody);

  // <actuator>
  auto actuator = doc.NewElement("actuator");
  for(const auto & [joint_name, actuator_meta] : mujocoMetadata.actuator)
  {
    auto motor = doc.NewElement("motor");
    motor->SetAttribute("name", (joint_name + "_motor").c_str());
    motor->SetAttribute("joint", joint_name.c_str());
    motor->SetAttribute("gear", double_to_str(actuator_meta.gear).c_str());
    motor->SetAttribute("ctrllimited", actuator_meta.ctrllimited ? "true" : "false");
    motor->SetAttribute("forcelimited", actuator_meta.forcelimited ? "true" : "false");
    std::string ctrlrange =
        double_to_str(actuator_meta.ctrlrange_min) + " " + double_to_str(actuator_meta.ctrlrange_max);
    std::string forcerange =
        double_to_str(actuator_meta.forcerange_min) + " " + double_to_str(actuator_meta.forcerange_max);
    motor->SetAttribute("ctrlrange", ctrlrange.c_str());
    motor->SetAttribute("forcerange", forcerange.c_str());
    actuator->InsertEndChild(motor);
  }
  mujoco->InsertEndChild(actuator);

  // <sensor>
  auto sensor = doc.NewElement("sensor");
  // TODO: Add force/torque/imu sensors using _forceSensors, _bodySensors, mujocoMetadata.site
  mujoco->InsertEndChild(sensor);

  return mujoco;
}

} // namespace mc_rbdyn
