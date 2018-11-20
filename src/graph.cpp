#include <algorithm>
#include <stack>
#include <sstream>
#include "disjoint_set.hpp"
#include "graph.hpp"

Graph::Graph (uint size) : _size(size), _out(size), _in(size) {}

Graph::Graph (uint size, const std::vector<Arc> &arcs) : Graph(size) {
	for (const Arc &arc : arcs) {
		this->set(arc);
	}
}

Graph::Graph (const std::string &fname, bool directed, bool robust) {

	this->directed = directed;
	this->robust = robust;

  #ifdef DEBUG
    std::cout << "Creating the graph - " << fname << std::endl;
  #endif

	// open the graph file
	std::fstream file(fname, std::fstream::in);
	std::string line;
	std::stringstream ss;

	std::getline(file, line);
  ss.str(line);

	/* Read the number of vertices and edges */
  ss >> this->_size >> this->_arc_count;
	this->_s = 0;
	this->_t = this->_size - 1;

  #ifdef DEBUG
    std::cout << "size = " << this->_size << "; arc_count = " << this->_arc_count << std::endl;
  #endif

  // allocate space for the variables
	this->_in.resize(this->size(), {});
  this->_out.resize(this->size(), {});

  #ifdef DEBUG
    std::cout << "Resized; " << this->size() << std::endl;
  #endif

  #ifdef DEBUG
    std::cout << "Size: " << this->_size << std::endl;
    if (this->robust) {
      std::cout << "It is robust" << std::endl;
    } else {
      std::cout << "It is not robust" << std::endl;
    }
  #endif

  // read the arcs
  for (uint count = 0; count < this->_arc_count; count++) {
		uint from, to;
    double lower, upper, value;

    std::getline(file, line);
		ss.str(line);
		ss.clear();

    #ifdef DEBUG
      std::cout << "Reading line " << count << ": " << line << std::endl;
    #endif

    if (this->robust) {
			ss >> from >> to >> lower >> upper;
	    this->set_robust(from, to, lower, upper, false);
      // if (!this->directed) {
      //   this->set_robust(to, from, lower, upper, false);
      // }
		} else {
			ss >> from >> to >> value;
			this->set(from, to, value, false);
      // if (!this->directed) {
      //   this->set(to, from, value, false);
      // }
		}

    #ifdef DEBUG
      std::cout << "from: " << from << "; to: " << to << std::endl;
    #endif
  }

	file.close();
}

// std::vector<Arc> Graph::parent2path (uint from, uint to, const std::vector<uint> &parent) const {
// 	std::vector<Arc> result;
//
// 	for (uint b = to, a = parent[b]; b != from; b = a, a = parent[b]) {
// 		result.emplace_back(a, b, this->get(a, b));
// 	}
//
// 	std::reverse(result.begin(), result.end());
// 	return result;
// }

void Graph::set (uint from, uint to, double lower, double upper, double value, bool force) {
  std::cout << "Adding arc from " << from << " to " << to << ". It exists? " << this->exists(from, to) << std::endl;
  if (!this->exists(from, to)) {
    this->_out[from].emplace(to, Arc(from, to, lower, upper, value, true));
    this->_in[to].emplace(from, Arc(from, to, lower, upper, value, true));
    /* this->_arc_count++; */
  } else if (force) {
    this->_out[from][to]._lower = lower;
    this->_out[from][to]._upper = upper;
    this->_out[from][to]._value = value;
    this->_in[to][from]._lower  = lower;
    this->_in[to][from]._upper  = upper;
    this->_in[to][from]._value  = value;
  } else {
    throw "Connection exists.";
  }
}

// bool Graph::get (uint from, uint to, double &value) const {
// 	std::map<uint, Arc>::const_iterator found = this->_out[from].find(to);
//
// 	if (found != this->_out[from].end()) {
// 		value = found->second;
// 		return true;
// 	}
//
// 	value = 0;
// 	return false;
// }
//
// bool Graph::remove (uint from, uint to, int &value) {
// 	std::map<uint, Arc>::const_iterator found = this->_out[from].find(to);
//
// 	if (found != this->_out[from].end()) {
// 		value = found->second;
// 		this->_out[from].erase(found);
// 		this->_in[to].erase(from);
// 		this->_arc_count--;
//
// 		if (this->_in[to].empty()) {
// 			this->_roots.insert(to);
// 		}
//
// 		return true;
// 	}
//
// 	return false;
// }
//
// void Graph::clear_in (uint to) {
// 	for (const Arc &arc : this->arcs_in(to)) {
// 		this->remove(arc);
// 	}
// }
//
// void Graph::clear_out (uint from) {
// 	for (const Arc &arc : this->arcs_out(from)) {
// 		this->remove(arc);
// 	}
// }
//
// void Graph::clear_arcs (void) {
// 	if (!this->arc_count()) {
// 		return;
// 	}
//
// 	this->_arc_count = 0;
//
// 	for (uint i = 0; i < this->size(); ++i) {
// 		this->_out[i].clear();
// 		this->_in[i].clear();
// 		this->_roots.insert(i);
// 	}
// }
//
// std::vector<Arc> Graph::arcs (void) const {
// 	std::vector<Arc> result;
//
// 	for (uint i = 0; i < this->size(); ++i) {
// 		for (const std::pair<uint, Arc> &arc : this->_out[i]) {
// 			result.emplace_back(i, arc.first, arc.second);
// 		}
// 	}
//
// 	return result;
// }
//
// std::vector<Arc> Graph::arcs (uint vertex) const {
// 	std::vector<Arc> result;
//
// 	for (const std::pair<uint, Arc> &arc : this->in(vertex)) {
// 		result.emplace_back(arc.first, vertex, arc.second);
// 	}
//
// 	for (const std::pair<uint, Arc> &arc : this->out(vertex)) {
// 		result.emplace_back(vertex, arc.first, arc.second);
// 	}
//
// 	return result;
// }
//
// std::vector<Arc> Graph::arcs_in (uint vertex) const {
// 	std::vector<Arc> result;
// 	result.reserve(this->count_in(vertex));
//
// 	for (const std::pair<uint, Arc> &arc : this->in(vertex)) {
// 		result.emplace_back(arc.first, vertex, arc.second);
// 	}
//
// 	return result;
// }
//
// std::vector<Arc> Graph::arcs_out (uint vertex) const {
// 	std::vector<Arc> result;
// 	result.reserve(this->count_out(vertex));
//
// 	for (const std::pair<uint, Arc> &arc : this->out(vertex)) {
// 		result.emplace_back(vertex, arc.first, arc.second);
// 	}
//
// 	return result;
// }

// Graph Graph::msa (uint from, bool maximum) const {
// 	std::vector<std::vector<Arc>> cycles(this->size()), queues(this->size());
// 	DisjointSet<uint> S, W;
// 	std::vector<Arc> enter(this->size()), lambda(this->size());
// 	std::vector<uint> roots, min(this->size()), rset(1, from);
// 	std::unordered_map<Arc, std::unordered_map<Arc, uint>> Fmap;
// 	std::vector<int> change(this->size(), 0);
//
// 	for (uint i = 0; i < this->size(); ++i) {
// 		for (const Arc &arc : this->in(i)) {
// 			queues[i].emplace_back(arc.from(), i, arc.value());
// 		}
//
// 		if (maximum) {
// 			std::sort(queues[i].begin(), queues[i].end(), Arc::Asc);
// 		} else {
// 			std::sort(queues[i].begin(), queues[i].end(), Arc::Desc);
// 		}
//
// 		S.make(i);
// 		W.make(i);
//
// 		if (i != from) {
// 			roots.emplace_back(i);
// 		}
//
// 		min[i] = i;
// 	}
//
// 	while (!roots.empty()) {
// 		uint k = roots.back();
// 		roots.pop_back();
//
// 		if (queues[k].empty()) {
// 			rset.emplace_back(k);
// 			continue;
// 		}
//
// 		Arc arc = std::move(queues[k].back());
// 		queues[k].pop_back();
//
// 		if (S.find(arc.from()) == k) {
// 			roots.emplace_back(k);
// 			continue;
// 		}
//
// 		Fmap[arc] = {};
//
// 		for (const Arc &carc : cycles[k]) {
// 			Fmap[arc].emplace(carc, 1);
// 		}
//
// 		if (cycles[k].empty()) {
// 			lambda[k] = arc;
// 		}
//
// 		if (W.find(arc.from()) != W.find(arc.to())) {
// 			W.join(arc.from(), arc.to());
// 			enter[k] = arc;
// 			continue;
// 		}
//
// 		std::vector<Arc> cycle;
// 		std::unordered_set<uint> repr;
// 		const Arc *select = &arc;
//
// 		enter[k] = Arc();
//
// 		for (const Arc *xy = &arc; *xy; xy = &enter[S.find(xy->from())]) {
// 			cycle.emplace_back(*xy);
// 			repr.emplace(S.find(xy->from()));
//
// 			if ((maximum && *xy < *select) || (!maximum && *xy > *select)) {
// 				select = xy;
// 			}
// 		}
//
// 		for (const Arc &xy : cycle) {
// 			change[S.find(xy.to())] = select->value() - xy.value();
// 		}
//
// 		const uint m = min[S.find(select->to())];
//
// 		for (const uint &x : repr) {
// 			S.join(k, x);
// 		}
//
// 		k = S.find(k);
// 		min[k] = m;
// 		roots.emplace_back(k);
// 		cycles[k] = std::move(cycle);
//
// 		std::vector<Arc> qk;
//
// 		for (const uint &x : repr) {
// 			std::vector<Arc> qx, qm;
// 			qm.swap(qk);
//
// 			for (Arc &xy : queues[x]) {
// 				if (S.find(xy.from()) != k || S.find(xy.to()) != k) {
// 					qx.emplace_back(xy.from(), xy.to(), xy.value() + change[x]);
// 				}
// 			}
//
// 			queues[x].clear();
// 			qk.resize(qx.size() + qm.size());
//
// 			if (maximum) {
// 				std::merge(qx.begin(), qx.end(), qm.begin(), qm.end(), qk.begin(), Arc::Asc);
// 			} else {
// 				std::merge(qx.begin(), qx.end(), qm.begin(), qm.end(), qk.begin(), Arc::Desc);
// 			}
// 		}
//
// 		queues[k] = std::move(qk);
// 	}
//
// 	std::unordered_map<Arc, uint> index;
// 	Graph F = Graph::from_map(Fmap, index);
// 	std::vector<Arc> inv_index(index.size());
// 	std::unordered_set<uint> removed;
// 	std::stack<uint> Froots;
// 	Graph result(this->size());
//
// 	const auto remove_parents = [&] (const Arc &arc) -> void {
// 		uint vertex = index[arc];
//
// 		while (true) {
// 			for (const Arc &arc : F.arcs_out(vertex)) {
// 				Froots.push(arc.to());
// 			}
//
// 			F.clear_out(vertex);
// 			removed.insert(vertex);
//
// 			if (!F.count_in(vertex)) {
// 				break;
// 			}
//
// 			vertex = F.arcs_in(vertex)[0].from();
// 		}
// 	};
//
// 	for (const std::pair<Arc, uint> &arc_pos : index) {
// 		inv_index[arc_pos.second] = arc_pos.first;
// 	}
//
// 	for (const uint &root : F.roots()) {
// 		Froots.push(root);
// 	}
//
// 	for (const uint &v : rset) {
// 		if (lambda[min[v]]) {
// 			remove_parents(lambda[min[v]]);
// 		}
// 	}
//
// 	while (!Froots.empty()) {
// 		const uint root = Froots.top();
// 		Froots.pop();
//
// 		if (removed.count(root)) {
// 			continue;
// 		}
//
// 		const Arc &arc = inv_index[root];
// 		result.set(arc, this->get(arc));
//
// 		remove_parents(lambda[arc.to()]);
// 	}
//
// 	return result;
// }

// bool Graph::bfs (uint from, std::vector<uint> &parent, std::vector<bool> &reach, const std::unordered_set<uint> &stop) const {
// 	std::queue<uint> vertices({ from });
// 	std::set<uint> found({ from });
//
// 	parent.resize(this->size());
// 	reach.resize(this->size());
//
// 	std::fill(reach.begin(), reach.end(), false);
// 	std::fill(parent.begin(), parent.end(), 0);
//
// 	reach[from] = true;
// 	parent[from] = from;
//
// 	if (stop.count(from)) {
// 		return true;
// 	}
//
// 	while (!vertices.empty()) {
// 		const uint vertex = vertices.front();
// 		vertices.pop();
//
// 		for (const Arc &arc : this->out(vertex)) {
// 			const uint &next = arc.to();
//
// 			if (found.count(next)) {
// 				continue;
// 			}
//
// 			found.insert(next);
// 			parent[next] = vertex;
// 			reach[next] = true;
//
// 			if (stop.count(next)) {
// 				return true;
// 			}
//
// 			vertices.emplace(next);
// 		}
// 	}
//
// 	return false;
// }
//
// bool Graph::path (uint from, uint to, std::vector<Arc> &result) const {
// 	std::vector<uint> parent;
// 	std::vector<bool> reach;
//
// 	result.clear();
//
// 	if (this->bfs(from, parent, reach, { to })) {
// 		result = std::move(this->parent2path(from, to, parent));
// 		return true;
// 	}
//
// 	return false;
// }
//
// void Graph::widest_path (
// 	uint from, std::vector<int> &width, std::vector<uint> &parent,
// 	const std::unordered_set<uint> &stop
// ) const {
// 	width.resize(this->size());
// 	parent.resize(this->size());
//
// 	using width_type = std::decay<decltype(width)>::type::value_type;
//
// 	std::fill(width.begin(), width.end(), std::numeric_limits<width_type>::min());
// 	std::fill(parent.begin(), parent.end(), 0);
//
// 	width[from] = std::numeric_limits<width_type>::max();
//
// 	std::priority_queue<Arc> heap;
// 	heap.emplace(from, from, width[from]);
//
// 	while (!heap.empty()) {
// 		const Arc top = std::move(heap.top());
// 		const uint vertex = top.to();
//
// 		heap.pop();
//
// 		if (stop.count(vertex)) {
// 			break;
// 		}
//
// 		if (top.value() != width[vertex]) {
// 			continue;
// 		}
//
// 		for (const Arc &arc : this->_out[vertex]) {
// 			const double found = std::min(arc.value(), (double) width[vertex]);
//
// 			if (found > width[arc.to()]) {
// 				width[arc.to()] = found;
// 				parent[arc.to()] = vertex;
// 				heap.emplace(vertex, arc.to(), found);
// 			}
// 		}
// 	}
// }
//
// uint Graph::max_flow (uint from, uint to, Graph &pure, uint limit) const {
// 	uint i = 0;
// 	std::vector<Arc> flow;
// 	std::vector<uint> vertices(this->size(), 0);
// 	std::unordered_map<Arc, int> flows;
// 	Graph residual(*this);
//
// 	while ((limit == 0 || i++ < limit) && residual.path(from, to, flow)) {
// 		const int lesser = std::min_element(flow.begin(), flow.end())->value();
//
// 		for (const Arc &arc : flow) {
// 			const int remain = arc.value() - lesser;
// 			double value;
//
// 			vertices[arc.to()] += lesser;
//
// 			flows[Arc(arc.from(), arc.to(), 0)] += lesser;
// 			flows[Arc(arc.to(), arc.from(), 0)] -= lesser;
//
// 			if (remain == 0) {
// 				residual.remove(arc);
// 			} else {
// 				residual.set(arc, remain, true);
// 			}
//
// 			residual.get(arc.to(), arc.from(), value);
// 			residual.set(arc.to(), arc.from(), value + lesser, true);
// 		}
// 	}
//
// 	pure = Graph(this->size());
//
// 	for (const std::pair<Arc, int> &arc_flow : flows) {
// 		if (arc_flow.second > 0) {
// 			pure.set(arc_flow.first, this->get(arc_flow.first));
// 		}
// 	}
//
// 	return vertices[to];
// }
//
// std::vector<std::vector<Arc>> Graph::disjoint_paths (uint from, uint to, uint limit) const {
// 	Graph flows, copy(this->size());
// 	std::vector<std::vector<Arc>> paths;
//
// 	for (uint i = 0; i < this->size(); ++i) {
// 		for (const Arc &arc : this->out(i)) {
// 			copy.set_value(i, arc.to(), 1.0);
// 		}
// 	}
//
// 	limit = copy.max_flow(from, to, flows, limit);
// 	Graph pure(flows);
// 	paths.reserve(limit);
//
// 	while (paths.size() < limit) {
// 		std::vector<bool> open(this->size(), false);
// 		std::vector<uint> parent(flows.size());
// 		std::vector<Arc> found;
// 		uint vertex = from;
//
// 		while (vertex != to) {
// 			open[vertex] = true;
//
// 			for (const Arc &arc : flows.out(vertex)) {
// 				const uint &next = arc.to();
//
// 				if (!open[next]) {
// 					parent[next] = vertex;
// 					flows.remove(vertex, next);
// 					found.emplace_back(vertex, next, this->get(vertex, next));
// 					vertex = next;
// 					break;
// 				}
// 			}
// 		}
//
// 		paths.emplace_back(std::move(found));
// 	}
//
// 	return paths;
// }
