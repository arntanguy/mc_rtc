#pragma once

#include <mc_rtc/gui/Visual.h>
#include <mc_rtc/visual_utils.h>
#include "../3rd-party/paramesh/mesh_generators.hpp"

namespace mc_rtc::gui
{

/** Creates a Sphere
 *
 * \tparam GetPos Callback to get the sphere position
 *
 * \tparam GetRadius A double (fixed radius) or callback to get the radius
 *
 * \tparam GetColor A color (fixed color) or callback to get the color
 *
 * If \tparam GetRadius and/or \tparam GetColor are callbacks they are invoked immediately
 */
template<typename GetPos,
         typename GetCurvePoints,
         typename GetCurveNormals,
         typename GetColor = const mc_rtc::gui::Color &>
auto ParametricSurface(const std::string & name,
                       GetCurvePoints points_fn,
                       GetCurveNormals normals_fn,
                       GetPos get_pos_fn,
                       size_t rings = 32,
                       size_t slices = 32,
                       GetColor color_fn = mc_rtc::gui::Color::Red)
{
  vector<MeshTriangle> tlist; // contains triangles for mesh
  vector<MeshVertex> vlist; // contains vertices for mesh

  auto points_fn_ = [points_fn](double theta, double phi)
  {
    Eigen::Vector3d p = points_fn(theta, phi);
    return ::vec3(p.x(), p.y(), p.z());
  };
  auto normals_fn_ = [normals_fn](double theta, double phi)
  {
    Eigen::Vector3d p = normals_fn(theta, phi);
    return ::vec3(p.x(), p.y(), p.z());
  };
  GeneratePointsAndNormals(vlist, rings, slices, points_fn_, normals_fn_, 2.0f * mc_rtc::constants::PI / (float)rings,
                           mc_rtc::constants::PI / (float)slices);
  GenerateFaces(tlist, (int)rings, (int)slices);
  // GenerateSphereVertexNormals(vlist);

  TriangleMesh mesh;
  mesh.nv = (uint32_t)vlist.size();
  mesh.nt = (uint32_t)tlist.size();
  mesh.vertexArray = (MeshVertex *)malloc(vlist.size() * sizeof(MeshVertex));
  mesh.triangleArray = (MeshTriangle *)malloc(tlist.size() * sizeof(MeshTriangle));

  copy(vlist.begin(), vlist.begin() + vlist.size(), mesh.vertexArray);
  copy(tlist.begin(), tlist.begin() + tlist.size(), mesh.triangleArray);

  auto path = fmt::format("/tmp/generated-parametric-mesh-{}.ply", name);
  ::WritePLYMesh(mesh, path);

  auto visual_mesh = mc_rtc::makeVisualMesh(path, 1.0);

  auto get_visual_fn = [=]() mutable -> const rbd::parsers::Visual & { return visual_mesh; };
  return Visual(name, get_visual_fn, get_pos_fn);
}

} // namespace mc_rtc::gui
