#ifdef _WIN32
    #define _WIN32_WINNT 0x0A00 
 #endif
#define _USE_MATH_DEFINES 
#include <cmath>          
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_1_PI
#define M_1_PI (1.0 / M_PI)
#endif
#include "httplib.h"
#include "json.hpp"
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <vector>
#include <fstream>
#include <sstream>   
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <limits>


using json = nlohmann::json;
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
   s = s;
}
    void search_nodes_for_coordinates(string nodes){
         std::ifstream nodes_file(nodes);
        if(!nodes_file.is_open()){
            std::cout<<"DEBUG FILE NOT OPEN"<<std::endl;
        }
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
            current_distance = haversine_formula(std::pair<double, double>(M_PI / 180.0 * l.second.first, M_PI / 180.0 * l.second.second),std::pair<double, double>(M_PI / 180.0 * n.second.first, M_PI / 180.0 * n.second.second));
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
        trim(id1_str);
        trim(id2_str);
        trim(dist_str);
        if(id1_str.empty() || id2_str.empty() || dist_str.empty()) {
            continue;}
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
    std::cout << "DEBUG: Loaded " << edges.size() << " edges from file." << std::endl;
    int edges_added_to_graph = 0;
    search_nodes_for_coordinates(nodes_path);
    std::map<long long, std::pair<double, double>> nodes = id_to_coords_pairs;  
    std::cout << "DEBUG: Copied " << nodes.size() << " nodes into local map." << std::endl;                               
    for(auto edge: edges) {
        long long id1 = edge.first.first;
        long long id2 = edge.first.second;
        if (nodes.count(id1) && nodes.count(id2)){
          graph_object.add_node(GraphNode(edge.first.first, nodes[edge.first.first].first, nodes[edge.first.first].second), 
          GraphNode(edge.first.second, nodes[edge.first.second].first, nodes[edge.first.second].second), edge.second);
        }
        edges_added_to_graph++;
     
    // Read nodes into a map: id -> (lat, lon)
    }
    std::cout << "DEBUG: Successfully added " << edges_added_to_graph << " valid edges to graph." << std::endl;
}

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
        auto it = previous.find(current);
        if (it == previous.end() || it->second == -1) {
            return {}; 
        }
        current = it->second;
    }
    the_path.push_back(start);
    std::reverse(the_path.begin(), the_path.end());
    return the_path;
}
pair<double, std::vector<long long>> dijistras(unordered_map <long long, std::vector<pair<long long,double>>> adj_list, long long start, long long end){

    unordered_map<long long, double> shortest_distance;
    unordered_map<long long, long long> previous;
    int graph_dimension_size = adj_list.size();    
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<pair<double, long long>>> p_s;


    for(auto pairs: adj_list){
        shortest_distance[pairs.first] = 1e18;
        previous[pairs.first] = -1;
    }
    shortest_distance[start] = 0;
    p_s.push({0.0, start});

    while(!p_s.empty()){
       pair<double, long long> dist = p_s.top();
       p_s.pop();
       
        if(dist.second == end){
            break;
        }
        //vector<long long> adjacent_vertices;
          for(auto &[vertex, weight]: adj_list[dist.second]){
                
            if(shortest_distance[vertex] > dist.first + weight){
                shortest_distance[vertex] = dist.first + weight;
                previous[vertex] = dist.second;
                p_s.push({shortest_distance[vertex], vertex});
               // all_nodes.push_back(min_vert);
            }

          }
        
    }
  
  // return result;
    return {shortest_distance[end], path_reconstruction(previous, start, end)};
}

                             
pair<double, vector<long long>> a_star(unordered_map <long long,std::vector<pair<long long,double>>> adj_list, long long start, long long end, map<long long, pair<double, double>> id_to_coords){
   
     unordered_map<long long, double> shortest_distance;
    unordered_map<long long, long long> previous;
    int graph_dimension_size = adj_list.size();    
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<pair<double, long long>>> p_s;


    for(auto pairs: adj_list){
        shortest_distance[pairs.first] = 1e18;
        previous[pairs.first] = -1;
    }
    shortest_distance[start] = 0;
    p_s.push({0.0, start});

   
    while(!p_s.empty()){
       pair<double, long long> dist = p_s.top();
       p_s.pop();
       
        if(dist.second == end){
            break;
        }
        //vector<long long> adjacent_vertices;
          for(auto &[vertex, weight]: adj_list[dist.second]){
            double g = shortest_distance[dist.second] + weight;
            if (g < shortest_distance[vertex]) {
                shortest_distance[vertex] = g;
                previous[vertex] = dist.second;
                long long v = vertex;
                double h = haversine_formula({id_to_coords[v].first * M_1_PI/180,id_to_coords[v].second * M_1_PI/180 }, {id_to_coords[end].first * M_1_PI/180,id_to_coords[end].second * M_1_PI/180 });
                p_s.push({g + h, v});
            }
        }
    }
    return {shortest_distance[end], path_reconstruction(previous, start, end)};

}



vector<string> getAllPlaces() {
    set<string> uniquePlaces;
    vector<string> places;
    for (auto pair : id_to_poi) {
        uniquePlaces.insert(pair.second);
    }
    for(auto name: uniquePlaces){
        places.push_back(name);
    }
    return places;
}


long long getNodeIdByPlace(const string& placeName) {
    for (auto pair : id_to_poi) {
        if (pair.second == placeName) {
            return pair.first;
        }
    }
    // If place not found return -1 (for error handling)
    return -1;
}


json generateSteps(const vector<string>& pathPlaces) { 
    json steps = json::array();
    if (pathPlaces.size() < 2){ 
        return steps;}
    
    
    steps.push_back({
        {"instruction", "Start at " + pathPlaces[0]}
    });
    
    for (auto i = 1; i < pathPlaces.size() - 1; i++) {
        steps.push_back({
            {"instruction", "Continue to " + pathPlaces[i]}
        });
    }
    
    steps.push_back({
        {"instruction", "Arrive at " + pathPlaces.back()}
    });
    
    return steps;
}


json getPathAsJson(long long start, long long end, const string algorithm) {
    json result;
    pair<double,vector<long long>> nodePath; 
    if (algorithm == "dijstra") {
        nodePath = dijistras(map_of_uf, start, end); 
    } else {
        nodePath = a_star(map_of_uf, start, end, id_to_coords_pairs); 
    }
    
    double distance = nodePath.first;
    int path_size = nodePath.second.size();
    //debug statements
    cout << "Algorithm: " << algorithm << endl;
    cout << "Distance found: " << distance << endl;
    cout << "Path node count: " << path_size << endl;
    
    json pathArray = json::array();
    json coordinatesArray = json::array();
    vector<string> pathPlaces;
    
    if (nodePath.second.empty() && start != end) {
        result["error"] = "No path found between " + id_to_poi[start] + " and " + id_to_poi[end];
        return result;
    }

    for (long long nodeId : nodePath.second) {
        pathArray.push_back(id_to_poi[nodeId]);
        pathPlaces.push_back(id_to_poi[nodeId]);
        
        json coord = json::array();
        coord.push_back(id_to_coords_pairs[nodeId].first);
        coord.push_back(id_to_coords_pairs[nodeId].second);
        coordinatesArray.push_back(coord);
    }
    
    result["path"] = pathArray;
    result["coordinates"] = coordinatesArray;
    result["steps"] = generateSteps(pathPlaces); 
    
    return result;
}
   

};


int main(){
    //testing area
    // Graph my_graph;
    
    // my_graph.process_csv_files_and_add_them_to_graph(my_graph, "uf_edges.csv","uf_nodes.csv");
    // my_graph.search_nodes_for_coordinates("uf_nodes.csv");
    // my_graph.search_places_for_places("uf_nodes.csv", "uf_places.csv" );
    // my_graph.print();
    // cout << "Shortest Distance(DIJISTRAS):" << my_graph.dijistras(my_graph.map_of_uf, 84729190, 10082349131).first;
    // cout << "DIJSTRA'S PATH:  " << my_graph.id_to_poi[84729190] << " to " <<  my_graph.id_to_poi[10082349131] << endl;
    // for(long long nodes : my_graph.dijistras(my_graph.map_of_uf, 84729190, 10082349131).second){
    //     cout << my_graph.id_to_poi[nodes]<< "->";
    // }
    // cout << endl;
    // cout << "Shortest Distance(A_STAR):" << my_graph.a_star(my_graph.map_of_uf, 84729190, 10082349131, my_graph.id_to_coords_pairs).first;
    // cout << "A_STAR'S PATH:" << my_graph.id_to_poi[84729190] << " to " <<  my_graph.id_to_poi[10082349131] << endl;
    // for(long long nodes : my_graph.a_star(my_graph.map_of_uf, 84729190, 10082349131, my_graph.id_to_coords_pairs).second){
    //     cout << my_graph.id_to_poi[nodes] << "->";
    // }

    //http lib server area
    Graph my_graph;
    my_graph.process_csv_files_and_add_them_to_graph(my_graph, "uf_edges.csv", "uf_nodes.csv");
    my_graph.search_nodes_for_coordinates("uf_nodes.csv");
    my_graph.search_places_for_places("uf_nodes.csv", "uf_places.csv");
    httplib::Server svr;
     svr.set_default_headers({
        {"Access-Control-Allow-Origin", "*"},
        {"Access-Control-Allow-Methods", "GET, POST, OPTIONS"},
        {"Access-Control-Allow-Headers", "Content-Type"}
    });
    
    
    svr.Options(".*", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("", "text/plain");
        res.status = 200;
    });
    
   
    svr.Get("/api/places", [&my_graph](const httplib::Request&, httplib::Response& res) {
        try {
            json response;
            response["places"] = my_graph.getAllPlaces();
            res.set_content(response.dump(), "application/json");
        } catch (const exception& e) {
            json error;
            error["error"] = e.what();
            res.status = 500;
            res.set_content(error.dump(), "application/json");
        }
    });
    
   
    svr.Post("/api/route", [&my_graph](const httplib::Request& req, httplib::Response& res) {
        try {
            auto body = json::parse(req.body);
            
            string startPlace = body["start"];
            string endPlace = body["end"];
            
            long long startNode = my_graph.getNodeIdByPlace(startPlace);
            long long endNode = my_graph.getNodeIdByPlace(endPlace);
            
            if (startNode == -1 || endNode == -1) {
                json error;
                error["error"] = "Invalid place name(s)";
                res.status = 400;
                res.set_content(error.dump(), "application/json");
                return;
            }
            
            json dijkstraResult = my_graph.getPathAsJson(startNode, endNode, "dijkstra");
            json aStarResult = my_graph.getPathAsJson(startNode, endNode, "astar");
            
            json response;
            response["dijkstraResult"] = dijkstraResult;
            response["aStarResult"] = aStarResult;
            
            res.set_content(response.dump(), "application/json");
        } catch (const exception& e) {
            json error;
            error["error"] = e.what();
            res.status = 500;
            res.set_content(error.dump(), "application/json");
        }
    });
    
    
    svr.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        json response;
        response["status"] = "online";
        res.set_content(response.dump(), "application/json");
    });
    // tests prompt for compilation
    cout << "Server starting on http://localhost:8080" << endl;
    cout << "Available endpoints:" << endl;
    cout << "  GET  /api/places" << endl;
    cout << "  GET  /api/status" << endl;
    cout<< "testng 3"<<endl;
    
    svr.listen("localhost", 8080);

    
}
    
