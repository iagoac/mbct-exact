#ifndef CPLEX_GRAPH_HH_
#define CPLEX_GRAPH_HH_

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <queue>
#include <functional>
#include <utility>
#include <iostream>

class Arc {

public:
  uint   _from  = 0;
  uint   _to    = 0;
  double _lower = 0.0;
  double _upper = 0.0;
  double _value = 0.0;
  bool   _valid = true;

  static struct Asc {
    bool operator () (const Arc &from, const Arc &to) const { return from.value() < to.value(); }
  } Asc;

  static struct Desc {
    bool operator () (const Arc &from, const Arc &to) const { return from.value() > to.value(); }
  } Desc;

  Arc (void) : _valid(false) {}

  Arc (uint from, uint to, double value)
    : _to(to), _lower(value), _upper(value), _value(value), _valid(true) {}

  Arc (uint to, double lower, double upper, double value)
    : _to(to), _lower(lower), _upper(upper), _value(value), _valid(true) {}

  Arc (uint from, uint to, double lower, double upper, double value, bool valid)
    : _from(from), _to(to), _lower(lower), _upper(upper), _value(value), _valid(valid) {}

  // Arc (const std::pair<const uint, int> &vertex_val)
  //   : _from(vertex_val.first), _to(vertex_val.first),
  //     _value(vertex_val.second), _valid(true) {}

  // Arc transpose (void) const { return Arc(this->to(), this->from(), this->lower(), this->upper(), this->value()); }

  void set_multiplier_value (double mul) { this->_value = this->lower() + mul * (this->upper() - this->lower()); }

  bool operator < (const Arc &other) const { return Asc(*this, other); }
  bool operator > (const Arc &other) const { return Desc(*this, other); }

  bool operator == (const Arc &arc) const {
    return (this->value() == arc.value() && this->to() == arc.to() &&
      this->lower() == arc.lower() && this->upper() == arc.upper());
  }

  const uint   &from  (void) const { return this->_from;  }
  const uint   &to    (void) const { return this->_to;    }
  const double &value (void) const { return this->_value; }
  const double &lower (void) const { return this->_lower; }
  const double &upper (void) const { return this->_upper; }

  const bool   &valid (void) const { return this->_valid; }
  operator bool (void) const { return this->valid(); }

  // friend std::ostream &operator << (std::ostream &out, const Arc &arc) {
  //   out << "Arc{ " << arc.from() << " --(" << arc.value() << ")-> " << arc.to() << " }";
  //   return out;
  // }
};

namespace std {
  template <>
  class hash<Arc> {
  public:
    size_t operator () (const Arc &e) const {
      const std::hash<int> ihasher;
      const std::hash<uint> uhasher;
      std::size_t result = 0;

      result ^= uhasher(e.value()) + 0x9e3779b9 + (result << 6) + (result >> 2);
      result ^= uhasher(e.to()) + 0x9e3779b9 + (result << 6) + (result >> 2);
      result ^= ihasher(e.lower()) + 0x9e3779b9 + (result << 6) + (result >> 2);
      result ^= ihasher(e.upper()) + 0x9e3779b9 + (result << 6) + (result >> 2);

      return result;
    }
  };
};

class Graph {

  uint _size, _arc_count = 0;
  uint _s = 0, _t;
  bool directed = false;
  bool robust = false;

public:
  std::vector<std::map<uint, Arc>> _in, _out;
  Graph (uint size = 0);
  Graph (uint size, const std::vector<Arc> &arcs);
  Graph (const std::string &fname, bool directed = false, bool robust = false);

  // Graph transpose (void) const {
  //   Graph result(this->size());
  //   result._in = this->out();
  //   result._out = this->in();
  //   result._arc_count = this->arc_count();
  //   return result;
  // };

  // template <typename T>
  // static Graph from_map (
  //   const std::unordered_map<T, std::unordered_map<T, uint>> &arcs,
  //   std::unordered_map<T, uint> &index
  // ) {
  //   index.clear();
  //
  //   for (const std::pair<T, std::unordered_map<T, uint>> &from_to : arcs) {
  //     index.emplace(from_to.first, index.size());
  //
  //     for (const std::pair<T, uint> &to_val : from_to.second) {
  //       index.emplace(to_val.first, index.size());
  //     }
  //   }
  //
  //   Graph result(index.size());
  //
  //   for (const std::pair<T, std::unordered_map<T, uint>> &from_to : arcs) {
  //     for (const std::pair<T, uint> &to_val : from_to.second) {
  //       result.set(index[from_to.first], index[to_val.first], to_val.second);
  //     }
  //   }
  //
  //   return result;
  // }

  void set (uint from, uint to, double lower, double upper, double value, bool force = false);
  void set (uint from, uint to, double value, bool force = false) { this->set(from, to, value, value, value, force); }
  void set (const Arc &arc, bool force = false) { this->set(arc.from(), arc.to(), arc.value(), arc.value(), arc.value(), force); }
  void set_robust (uint from, uint to, double lower, double upper, bool force = false) { this->set(from, to, lower, upper, lower, force); }
  void set_robust (const Arc &arc, bool force = false) { this->set(arc.from(), arc.to(), arc.lower(), arc.upper(), arc.value(), force); }

  // void set_robust (uint from, uint to, double lower, double upper, bool force = false);
  // void set_robust (const Arc &arc, bool force = false) {
  //   this->set_all_values(arc.from(), arc.to(), arc.lower(), arc.upper(), arc.value(), force);
  // }

  double solutio_cost(int *ant);
  double path_cost(int *ant, uint k);
  void set_multiplier (double mult);

  // bool get (uint from, uint to, double &value) const;
  // bool get (const Arc &arc, double &value) const { return this->get(arc.from(), arc.to(), value); }

  const double get_upper(uint i, uint j) {
    return ((this->is_directed()) ? this->_out[i][j].upper() : this->_out[std::min(i,j)][std::max(i,j)].upper());
  }

  const double get_lower(uint i, uint j) {
    return ((this->is_directed()) ? this->_out[i][j].lower() : this->_out[std::min(i,j)][std::max(i,j)].lower());
  }

  const double get_value(uint i, uint j) {
    return ((this->is_directed()) ? this->_out[i][j].value() : this->_out[std::min(i,j)][std::max(i,j)].value());
  }

  const uint get_from(uint i, uint j) {
    return ((this->is_directed()) ? this->_out[i][j].from() : this->_out[std::min(i,j)][std::max(i,j)].from());
  }

  const uint get_to(uint i, uint j) {
    return ((this->is_directed()) ? this->_out[i][j].to() : this->_out[std::min(i,j)][std::max(i,j)].to());
  }

  int get_root(void) const {
    return (this->_s);
  }

  const bool is_directed() const {
    return this->directed;
  }

  double get (uint from, uint to) const { return this->_out[from].at(to); }
  double get (const Arc &arc) const { return this->get(arc.from(), arc.to()); }

  bool exists (uint from, uint to) const { return this->_out[from].count(to); }
  bool exists (const Arc &arc) const { return this->exists(arc.from(), arc.to()); }

  // bool remove (uint from, uint to, int &value);
  // bool remove (const Arc &arc, int &value) { return this->remove(arc.from(), arc.to(), value); }
  //
  // bool remove (uint from, uint to) { int _; return this->remove(from, to, _); }
  // bool remove (const Arc &arc) { return this->remove(arc.from(), arc.to()); }
  //
  // void clear_in (uint to);
  // void clear_out (uint from);
  // void clear_arcs (uint vertex) { this->clear_in(vertex); this->clear_out(vertex); };
  // void clear_arcs (void);

  uint count_in (uint to) const { return this->_in[to].size(); }
  uint count_out (uint from) const { return this->_out[from].size(); }

  const uint &size (void) const { return this->_size; }
  const uint &arc_count (void) const { return this->_arc_count; }

  const std::vector<std::map<uint, Arc>> &out (void) const { return this->_out; }
  const std::vector<std::map<uint, Arc>> &in (void) const { return this->_in; }

  const std::map<uint, Arc> &out (uint vertex) const { return this->_out[vertex]; }
  const std::map<uint, Arc> &in (uint vertex) const { return this->_in[vertex]; }

  // void widest_path (uint from, std::vector<int> &width, std::vector<uint> &parent, const std::unordered_set<uint> &stop = {}) const;
  // uint max_flow (uint from, uint to, Graph &pure, uint limit = 0) const;
  // std::vector<std::vector<Arc>> disjoint_paths (uint from, uint to, uint limit = 0) const;
  //
  // void set_attr (const std::string &name, int value) { this->_attr[name] = value; }
  // const int &get_attr (const std::string &name) const { return this->_attr.at(name); }

  void print (void) {
      for (uint i = 0; i < this->_out.size(); i++) {
        std::cout << " i = " << i << "; size = " << this->_out[i].size() << std::endl;
        for (std::map<uint, Arc>::iterator it = this->_out[i].begin(); it != this->_out[i].end(); ++it) {
          std::cout << i << ": [" << it->second.from() << "," << it->second.to() << "] = " << it->second.value() << std::endl;
        }
      }
  }



};

#endif
