#include <mc_rtc/gui/Polyhedron.h>
#include <mc_rtc/utils/heatmap.h>
#include <paramesh/mesh.hpp>
#include <paramesh/mesh_generators.hpp>

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
template<typename GetPos, typename GetCurvePoints, typename GetCurveNormals, typename GetColors>
auto ParametricPolyhedron(const std::string & name,
                          GetCurvePoints points_fn,
                          GetCurveNormals normals_fn,
                          GetColors colors_fn,
                          GetPos get_pos_fn,
                          size_t rings = 32,
                          size_t slices = 32)
{
  using MeshTriangle = paramesh::MeshTriangle;
  using MeshVertex = paramesh::MeshVertex;
  std::vector<paramesh::MeshTriangle> tlist; // contains triangles for mesh
  std::vector<paramesh::MeshVertex> vlist; // contains vertices for mesh
  std::vector<glm::vec4> clist;

  // Just a stupid conversion to glm types
  auto points_fn_ = [points_fn](double theta, double phi)
  {
    Eigen::Vector3d p = points_fn(theta, phi);
    return glm::vec3(p.x(), p.y(), p.z());
  };
  auto normals_fn_ = [normals_fn](double theta, double phi)
  {
    Eigen::Vector3d p = normals_fn(theta, phi);
    return glm::vec3(p.x(), p.y(), p.z());
  };
  auto colors_fn_ = [colors_fn](double theta, double phi)
  {
    mc_rtc::gui::Color c = colors_fn(theta, phi);
    return glm::vec4(c.r, c.g, c.b, c.a);
  };

  paramesh::GeneratePointsAndNormalsAndColors(vlist, clist, rings, slices, points_fn_, normals_fn_, colors_fn_,
                                              2.0f * mc_rtc::constants::PI / (float)rings,
                                              mc_rtc::constants::PI / (float)slices);
  paramesh::GenerateFaces(tlist, (int)rings, (int)slices);
  // GenerateSphereVertexNormals(vlist);

  paramesh::TriangleMesh mesh;
  mesh.nv = (uint32_t)vlist.size();
  mesh.nt = (uint32_t)tlist.size();
  mesh.vertexArray = (MeshVertex *)malloc(vlist.size() * sizeof(MeshVertex));
  mesh.triangleArray = (MeshTriangle *)malloc(tlist.size() * sizeof(MeshTriangle));

  copy(vlist.begin(), vlist.begin() + vlist.size(), mesh.vertexArray);
  copy(tlist.begin(), tlist.begin() + tlist.size(), mesh.triangleArray);

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(vlist.size());
  for(const auto & vertex : vlist) { vertices.emplace_back(vertex.vx, vertex.vy, vertex.vz); }
  auto vertices_fn = [vertices]() { return vertices; };

  std::vector<std::array<size_t, 3>> triangles;
  triangles.resize(tlist.size());
  for(size_t i = 0; i < tlist.size(); i++)
  {
    const auto & t = tlist[i];
    triangles[i] = {t.i0, t.i1, t.i2};
  }
  auto triangles_fn = [triangles]() { return triangles; };

  // per-vertex color
  std::vector<mc_rtc::gui::Color> colors;
  colors.resize(clist.size());
  for(size_t i = 0; i < clist.size(); i++)
  {
    const auto & c = clist[i];
    colors[i] = {c.r, c.g, c.b, c.a};
  }

  auto polyhedron_colors_fn = [colors]() { return colors; };

  return Polyhedron(name, vertices_fn, triangles_fn, polyhedron_colors_fn);
}

} // namespace mc_rtc::gui
