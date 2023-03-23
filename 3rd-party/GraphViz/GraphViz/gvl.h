#ifndef GVL_H_
#define GVL_H_

#include <utility>
#include <vector>
#include <unordered_set>
#include <string_view>
#include <ostream>
#include <cstdio>

namespace gvl {

struct Property {
  std::string name;
  std::string value;
  Property(std::string_view name, std::string_view value) : name(name), value(value) {}
};

struct NodeId {
  std::string name;
  std::string port;

  NodeId(std::string_view name, std::string_view port = "") : name(name), port(port) {}
};

struct Edge {
  NodeId from;
  NodeId to;
  std::vector<Property> properties;

  Edge(const NodeId& from, const NodeId& to, std::vector<Property> properties = {})
      : from(from), to(to), properties(std::move(properties)) {}
};

struct Node {
  NodeId id;
  std::vector<Property> properties;

  explicit Node(const NodeId& id, std::vector<Property> properties = {}) : id(id), properties(std::move(properties)) {}
};

/*!
 * A class describing the structure of the desired graph for output through GraphViz.
 * All Strings should be alive when RenderDot will be called. Temporary strings are not allowed.
 */
class Graph {
 public:

  /*!
   * Create empty graph
   * @param graph_name name of created graph
   */
  explicit Graph(const std::string_view& graph_name)
      : nodes_(),
        edges_(),
        graph_name_(graph_name),
        graph_props_(),
        common_edge_props_(),
        common_node_props_() {}

  // Ban copy and move
  Graph(const Graph& other) = delete;
  Graph& operator=(const Graph& other) = delete;

  Graph(Graph&& other) = delete;
  Graph& operator=(Graph&& other) = delete;

  /*!
   * Add a new node to the list of nodes with default properties
   * @param node name and port.
   */
  void AddNode(const NodeId& node) {
    const std::vector<Property> default_node_properties = {{"label", node.name}};
    nodes_.emplace_back(node, default_node_properties);
  }

  /*!
   * Add a new edge to the list of edges with no properties.
   * @param from start of edge
   * @param to end of edge
   */
  void AddEdge(const NodeId& from, const NodeId& to, const std::vector<Property> properties = {}) {
    edges_.emplace_back(from, to, properties);
  }

  /*!
   * Add a property to graph
   * @param name property name
   * @param value property value
   */
  void AddGraphProperty(const std::string_view& name, const std::string_view& value) {
    graph_props_.emplace_back(name, value);
  }

  /*!
   * Add a common property to edges
   * @param name property name
   * @param value property value
   */
  void AddCommonEdgeProperty(const std::string_view& name, const std::string_view& value) {
    common_edge_props_.emplace_back(name, value);
  }

  /*!
   * Add a common property to nodes
   * @param name property name
   * @param value property value
   */
  void AddCommonNodeProperty(const std::string_view& name, const std::string_view& value) {
    common_node_props_.emplace_back(name, value);
  }

  /*!
   * Write graph to file in DOT language format.
   * @param out output stream to write
   */
  void RenderDot(std::ostream & out) const;

 private:
  void PrintCommonProperties(std::ostream & out) const;

 private:
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::string_view graph_name_;
  std::vector<Property> graph_props_;
  std::vector<Property> common_edge_props_;
  std::vector<Property> common_node_props_;
};


std::ostream& operator<<(std::ostream& out, const NodeId& node_id) {
  out << node_id.name;

  if (!node_id.port.empty()) {
    out << ":" << node_id.port;
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const std::vector<Property>& properties) {
  if (!properties.empty()) {
    out << "[ ";
    for (const Property& prop : properties) {
      out << prop.name << "=\"" << prop.value << "\" ";
    }
    out << "]";
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const Edge& edge) {
  out << "\"" << edge.from << "\" -> \"" << edge.to << "\"" << edge.properties << ";\n";
  return out;
}

std::ostream& operator<<(std::ostream& out, const Node& node) {
  out << node.id << node.properties << ";\n";
  return out;
}

void Graph::PrintCommonProperties(std::ostream& out) const {
  if (!this->graph_props_.empty()) {
    out << "graph " << this->graph_props_ << ";\n";
  }

  if (!this->common_edge_props_.empty()) {
    out << "edge " << this->common_edge_props_ << ";\n";
  }

  if (!this->common_node_props_.empty()) {
    out << "node " << this->common_node_props_ << ";\n";
  }
}

void Graph::RenderDot(std::ostream& out) const {
  out << "digraph " << this->graph_name_ << " {\n";

  PrintCommonProperties(out);

  for (const Node& node : this->nodes_) {
    out << node;
  }

  for (const Edge& edge : this->edges_) {
    out << edge;
  }

  out << "}";
}

} // namespace gvl


#endif // GVL_H_
