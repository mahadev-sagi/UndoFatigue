#include <iostream>
#include <list>
#include <map>
#include <set>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include "math.h"    
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <limits>



using namespace std;

class Graph{
    public:
        class GraphNode{
            public:
                long long id;
                double latitude;
                double longitude;
                double distance_from_point;
                GraphNode(long long p_o_i, double la, double lo): id(p_o_i), latitude(la), longitude(lo), distance_from_point(0){};
                bool operator<(const GraphNode& other) const{
                    return this->distance_from_point < other.distance_from_point;
                 }

        };
        unordered_map <long long,std::vector<pair<long long,double>>> map_of_uf;
        map <long long, pair<double, double>> id_to_coords_pairs;
        map <long long, std::string> id_to_poi;
      //map<int, pair<GraphNode, GraphNode>> distances;
        int numNodes;
        Graph(): numNodes(0){};
        
        void add_node(Graph::GraphNode da_node_1, Graph::GraphNode da_node_2, double distance){
                
                    map_of_uf[da_node_1.id].push_back({da_node_2.id, distance});
                     map_of_uf[da_node_2.id].push_back({da_node_1.id, distance});

                
              }
        void print()
    {
        // Get the number of vertices
        cout << "Adjacency Matrix for the Graph: " << endl;
        for (auto row: map_of_uf) {
            cout << row.first << "->";
            for(auto   col: row.second){
                 cout << "ID:" << col.first << "Length:" << col.second;

            }
          
            cout << endl;
        }
    }
     double haversine_formula(pair<double, double> point_1, pair<double, double> point_2){
            double earth_radius = 6371;
            
            double lat_1 = point_1.first;
            double long_1 = point_1.second;
            double lat_2 = point_2.first;
            double long_2 = point_2.second;
            double delta_cos_one = lat_1;
            double delta_cos_two = lat_2;
            double delta_sin_lat = (lat_2 - lat_1);
            double delta_sin_long = (long_2 - long_1);
            double a = pow(std::sin(delta_sin_lat/2.0), 2) + pow(std::sin(delta_sin_long/2.0), 2) * std::cos(delta_cos_one) * std::cos(delta_cos_two);
            double c = 2 * asin(sqrt(a));
            double distance = earth_radius * c;
            return distance;

        }
        void trim(std::string &s) {
    // Remove leading spaces/tabs/newlines
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
        [](unsigned char ch) { return !std::isspace(ch); }));

    // Remove trailing spaces/tabs/newlines
    s.erase(std::find_if(s.rbegin(), s.rend(),
        [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
}
    void search_nodes_for_coordinates(string nodes){
         std::ifstream nodes_file(nodes);
            std::string line_in_nodes;
            std::getline(nodes_file, line_in_nodes);
              pair<double, double> result;

            while(std::getline(nodes_file, line_in_nodes)){
                 std::stringstream actual_nodes_line(line_in_nodes);
                std::string id;
                std::string lat_node;
                std::string lon_node;
                std::getline(actual_nodes_line, id, ',');

                std::getline(actual_nodes_line, lat_node, ',');
                std::getline(actual_nodes_line, lon_node, ',');
                  trim(id);
                  trim(lat_node);
                  trim(lon_node);
                
                
                 id_to_coords_pairs[std::stoll(id)] = {std::stod(lat_node), std::stod(lon_node)};
                }
        
    }
    
    void search_places_for_places(string nodes, string places){
       
      
        double current_distance;
    std::map<std::string, pair<double, double>> locations = map_version_of_places(places);
    std::map<long long, std::pair<double, double>> pairings = id_to_coords_pairs;      
    for (auto n: pairings) {
        double min_distance = 1e18;
        std::string min_vertice = "";
        for(auto l: locations){
            current_distance = haversine_formula({M_PI/180 * l.second.first, M_PI/180* l.second.second}, {M_PI/180* n.second.first, M_PI/180 * n.second.second});
            if(min_distance > current_distance){
                min_distance = current_distance;
                min_vertice = l.first;
            }

        }
       
         id_to_poi[n.first] = min_vertice;
        //closest_place.clear();
        }
    }
    map<pair<long long, long long>, double> map_version_of_edges(std::string edges_path){
        
    std::ifstream edges_file(edges_path);
    std::string line;
    std::getline(edges_file, line); // skip header
    map<pair<long long, long long>, double> result;    
    
    while (std::getline(edges_file, line)) {
        std::stringstream ss(line);
        std::string id1_str, id2_str, key, dist_str;
        std::getline(ss, id1_str, ',');
        std::getline(ss, id2_str, ',');
        std::getline(ss, key, ',');
        std::getline(ss, dist_str, ',');

        long long id1 = std::stoll(id1_str);
        long long id2 = std::stoll(id2_str);
        double distance = std::stod(dist_str); 
        result[{id1, id2}] = distance;
    }
    return result;
}
std::map<std::string, pair<double, double>> map_version_of_places(std::string places){
        
      std::ifstream places_file(places);
    std::string line_in_places;
    std::getline(places_file, line_in_places); // skip header

    std::map<std::string, pair<double, double>> result;
    
    while (std::getline(places_file, line_in_places)) {
        if (line_in_places.empty()) continue; // skip blank lines
        std::stringstream actual_line(line_in_places);

        std::string place, lat, lon;
        std::getline(actual_line, place, ',');
        std::getline(actual_line, lat, ',');
        std::getline(actual_line, lon, ',');

        trim(place);
        trim(lat);
        trim(lon);
        result[place] = {std::stod(lat), std::stod(lon)};
    }
    return result;
}
   

    void process_csv_files_and_add_them_to_graph(Graph& graph_object, 
                                             const std::string& edges_path, 
                                             const std::string& nodes_path) {

    std::map<std::pair<long long, long long>, double> edges = map_version_of_edges(edges_path);
    search_nodes_for_coordinates(nodes_path);
    std::map<long long, std::pair<double, double>> nodes = id_to_coords_pairs;                                 
    for(auto edge: edges) {
          graph_object.add_node(GraphNode(edge.first.first, nodes[edge.first.first].first, nodes[edge.first.first].second), 
          GraphNode(edge.first.second, nodes[edge.first.second].first, nodes[edge.first.second].second), edge.second);

      
     
    // Read nodes into a map: id -> (lat, lon)
    }}

bool is_edge(int s, int v){
     for(auto row: map_of_uf){
    if(row.first == s){
      for(auto col: row.second){
        if(col.first == v){
          return true;
        }
      }
    }
  }
    return false;
}

double getWeight(int from, int to)
{
    std::map<int, int> all_weights;
    double result;
    /*
        TODO: getWeight() returns a sorted vector containing all 
        weights of the edges connecting the from and to vertex
    */
  //int result = 0;
    for(auto row: map_of_uf){
    if(row.first == from){
      for(auto col: row.second){
        if(col.first == to){
          result = col.second;
          break;
        }
      }
    }
  }
 /* for(auto i: all_weights){
    for(int rep = 0; rep < i.second; rep++){
    result.push_back(i.first);
  }*/
  
    return result;
}
/*double dijistras(unordered_map <long long,std::vector<pair<long long,double>>> adj_list, long long start, long long end){

    set<long long> vertices_already_computed;

    set<long long> vertices_we_need_to_compute;
    map<long long, double> shortest_distance;
    map<long long, int> previous;
    vector<long long> all_nodes;
    int graph_dimension_size = adj_list.size();

    for(auto pairs: adj_list){
        vertices_we_need_to_compute.insert(pairs.first);
        shortest_distance[pairs.first] = 1e18;
        previous[pairs.first] = -1;
    }
    shortest_distance[start] = 0;
    int temp_count = 0;
   for(auto v: vertices_we_need_to_compute){
        previous[v] = start;
        if(is_edge(start, v)){
            shortest_distance[v] = getWeight(start, v);
        }
        else{
            shortest_distance[v] = 1e18;
        }

    }
    map<double, long long> dist;

    while(!vertices_we_need_to_compute.empty()){
       
        int i = 0;
        double minimum = 1e18;
        long long min_vert = -1;
         for(auto u: vertices_we_need_to_compute){
            dist[shortest_distance[u]] = u;
       
         }
        min_vert = dist.begin()->second;
        vertices_we_need_to_compute.erase(min_vert);
        vertices_already_computed.insert(min_vert);
        if(min_vert == end){
            break;
        }
        vector<long long> adjacent_vertices;
          for(auto &[vertex, weight]: adj_list[min_vert]){
                
            if(shortest_distance[vertex] > dist.begin()->first + weight){
                shortest_distance[vertex] = dist.begin()->first + weight;
                previous[vertex] = min_vert;
                all_nodes.push_back(min_vert);
            }

          }
        
    }
   std::string result;
   for(auto i: vertices_already_computed){
    result += to_string(i);
   }
   return result;
   return shortest_distance[end];
}
*/
vector<long long> path_reconstruction(unordered_map<long long, long long> &previous,
                                   long long start, long long end){

    vector<long long> the_path;
    long long current = end;

    
    if(previous.find(end) == previous.end() && end != start){
        return {}; 
    }

    
    while(current != start){
        the_path.push_back(current);
        if (previous.find(current) == previous.end()) {
            return {}; 
        }
        current = previous[current];
    }
    
    return the_path;
}
                                   
pair<double, vector<long long>> dijistras(unordered_map<long long, vector<pair<long long, double>>> &adj_list,
                 long long start, long long end) {
    unordered_map<long long, double> shortest_distance;
    unordered_map<long long, long long> previous;
    unordered_set<long long> visited;

    // Initialize distances
    for (auto &p : adj_list)
        shortest_distance[p.first] = 1e18;
    shortest_distance[start] = 0;

    // Min-heap priority queue (distance, vertex)
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [dist, u] = pq.top();
        pq.pop();

        if (visited.count(u)) continue;
        visited.insert(u);

        if (u == end) break;  // early stop when goal reached

        for (auto &[v, w] : adj_list[u]) {
            if (shortest_distance[v] > dist + w) {
                shortest_distance[v] = dist + w;
                previous[v] = u;
                pq.push({shortest_distance[v], v});
            }
        }
    }

    return {shortest_distance[end], path_reconstruction(previous, start, end)};
}
pair<double, vector<long long>> a_star(unordered_map <long long,std::vector<pair<long long,double>>> adj_list, long long start, long long end, map<long long, pair<double, double>> id_to_coords){
    /*set<long long> vertices_already_computed;

    unordered_set<long long> vertices_we_need_to_compute;
    unordered_map<long long, double> shortest_distance;
    unordered_map<long long, int> previous;
    vector<long long> all_nodes;
    int graph_dimension_size = adj_list.size();

    for(auto pairs: adj_list){
        vertices_we_need_to_compute.insert(pairs.first);
         shortest_distance[pairs.first] = 1e18;
         previous[pairs.first] = -1;
    }
    
    shortest_distance[start] = 0;
    int temp_count = 0;
    /*for(auto v: vertices_we_need_to_compute){
        previous[v] = start;
        if(is_edge(start, v)){
            shortest_distance[v] = getWeight(start, v);
        }
        else{
            shortest_distance[v] = 1e18;
        }

    }        map<double, long long> dist;

    while(!vertices_we_need_to_compute.empty()){
       
        int i = 0;
        double minimum = 1e18;
        int min_vert = -1;

         for(auto u: vertices_we_need_to_compute){
            double f = shortest_distance[u] + haversine_formula(id_to_coords[u], id_to_coords[end]);
            dist[f] = u;
           
         }
         min_vert = dist.begin()->second;

        vertices_we_need_to_compute.erase(min_vert);
        if(min_vert == end){
            break;
        }
        vertices_already_computed.insert(min_vert);
        vector<int> adjacent_vertices;
          for(auto v: adj_list[min_vert]){
                
            if(shortest_distance[v.first] > getWeight(min_vert, v.first) + (shortest_distance[min_vert])){
                shortest_distance[v.first] = getWeight(min_vert, v.first) + (shortest_distance[min_vert]);
                previous[v.first] = min_vert;
            }
          }

    }*/
   unordered_map<long long, double> shortest_distance;
    unordered_map<long long, long long> previous;
    unordered_set<long long> visited;

    for (auto &p : adj_list)
        shortest_distance[p.first] = 1e18;

    // Min-heap priority queue for f = g + h
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [f_val, u] = pq.top();
        pq.pop();

        if (visited.count(u)) continue;
        visited.insert(u);

        if (u == end) break; // we found the goal

        for (auto &[v, w] : adj_list[u]) {
            double g_new = shortest_distance[u] + w;
            if (g_new < shortest_distance[v]) {
                shortest_distance[v] = g_new;
                previous[v] = u;
                double h = haversine_formula({id_to_coords[v].first * M_1_PI/180,id_to_coords[v].second * M_1_PI/180.0 }, {id_to_coords[end].first * M_1_PI/180,id_to_coords[end].second * M_1_PI/180 });
            }
        }
    }
    return {shortest_distance[end], path_reconstruction(previous, start, end)};

}
   

};


int main(){

    Graph my_graph;
    
    my_graph.process_csv_files_and_add_them_to_graph(my_graph, "uf_edges (2).csv","uf_nodes (2).csv");
    my_graph.search_nodes_for_coordinates("uf_nodes (2).csv");
    my_graph.search_places_for_places("uf_nodes (2).csv", "uf_places (1).csv" );
    my_graph.print();
    cout << "Shortest Distance(DIJISTRAS):" << my_graph.dijistras(my_graph.map_of_uf, 84729190, 10082349131).first;
    cout << "DIJSTRA'S PATH:  " << my_graph.id_to_poi[84729190] << " to " <<  my_graph.id_to_poi[10082349131] << endl;
    for(long long nodes : my_graph.dijistras(my_graph.map_of_uf, 84729190, 10082349131).second){
        cout << my_graph.id_to_poi[nodes]<< "->";
    }
    cout << endl;
    cout << "Shortest Distance(A_STAR):" << my_graph.a_star(my_graph.map_of_uf, 84729190, 10082349131, my_graph.id_to_coords_pairs).first;
    cout << "A_STAR'S PATH:" << my_graph.id_to_poi[84729190] << " to " <<  my_graph.id_to_poi[10082349131] << endl;
    for(long long nodes : my_graph.a_star(my_graph.map_of_uf, 84729190, 10082349131, my_graph.id_to_coords_pairs).second){
        cout << my_graph.id_to_poi[nodes] << "->";
    }
    
}
    