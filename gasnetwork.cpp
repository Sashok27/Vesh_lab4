#include "gasnetwork.h"
#include <iostream>
#include <stack>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

void GasNetwork::addConnection(int pipe_id, int cs_in_id, int cs_out_id) {
    connections.push_back(Connection(pipe_id, cs_in_id, cs_out_id));
}

void GasNetwork::displayConnections(const map<int, Pipe>& pipes, const map<int, CompressorStation>& stations) const {
    if (connections.empty()) {
        cout << "Нет соединений в сети." << endl;
        return;
    }
    
    cout << "Соединения в газотранспортной сети:" << endl;
    for (const auto& conn : connections) {
        auto pipe_it = pipes.find(conn.pipe_id);
        auto cs_in_it = stations.find(conn.cs_in_id);
        auto cs_out_it = stations.find(conn.cs_out_id);
        
        if (pipe_it != pipes.end() && cs_in_it != stations.end() && cs_out_it != stations.end()) {
            cout << "Труба ID " << conn.pipe_id << " (диаметр " << pipe_it->second.getDiameter() 
                 << " мм) соединяет КС ID " << conn.cs_in_id << " \"" << cs_in_it->second.getName()
                 << "\" -> КС ID " << conn.cs_out_id << " \"" << cs_out_it->second.getName() << "\"" << endl;
        }
    }
}

bool GasNetwork::isPipeConnected(int pipe_id) const {
    for (const auto& conn : connections) {
        if (conn.pipe_id == pipe_id) {
            return true;
        }
    }
    return false;
}

bool GasNetwork::isConnectionExists(int cs_in_id, int cs_out_id) const {
    for (const auto& conn : connections) {
        if (conn.cs_in_id == cs_in_id && conn.cs_out_id == cs_out_id) {
            return true;
        }
    }
    return false;
}

void GasNetwork::removeConnectionsWithPipe(int pipe_id) {
    connections.erase(
        remove_if(connections.begin(), connections.end(),
            [pipe_id](const Connection& conn) { return conn.pipe_id == pipe_id; }),
        connections.end());
}

void GasNetwork::removeConnectionsWithCS(int cs_id) {
    connections.erase(
        remove_if(connections.begin(), connections.end(),
            [cs_id](const Connection& conn) { 
                return conn.cs_in_id == cs_id || conn.cs_out_id == cs_id; 
            }),
        connections.end());
}

// Проверка на наличие циклов с помощью DFS
bool GasNetwork::dfs(int v, set<int>& visited, set<int>& recursionStack, 
                    const map<int, vector<int>>& graph) const {
    if (visited.find(v) == visited.end()) {
        visited.insert(v);
        recursionStack.insert(v);
        
        if (graph.find(v) != graph.end()) {
            for (int neighbor : graph.at(v)) {
                if (visited.find(neighbor) == visited.end() && dfs(neighbor, visited, recursionStack, graph)) {
                    return true;
                } else if (recursionStack.find(neighbor) != recursionStack.end()) {
                    return true;
                }
            }
        }
    }
    
    recursionStack.erase(v);
    return false;
}

bool GasNetwork::hasCycle() const {
    // Построение графа
    map<int, vector<int>> graph;
    for (const auto& conn : connections) {
        graph[conn.cs_in_id].push_back(conn.cs_out_id);
    }
    
    set<int> visited;
    set<int> recursionStack;
    
    for (const auto& node : graph) {
        if (visited.find(node.first) == visited.end()) {
            if (dfs(node.first, visited, recursionStack, graph)) {
                return true;
            }
        }
    }
    
    return false;
}

// Топологическая сортировка (алгоритм Кана)
vector<int> GasNetwork::topologicalSort(const map<int, CompressorStation>& stations) const {
    // Построение графа и подсчет входящих степеней
    map<int, vector<int>> graph;
    map<int, int> inDegree;
    
    // Инициализация для всех станций
    for (const auto& station : stations) {
        inDegree[station.first] = 0;
    }
    
    // Построение графа и подсчет входящих степеней
    for (const auto& conn : connections) {
        graph[conn.cs_in_id].push_back(conn.cs_out_id);
        inDegree[conn.cs_out_id]++;
    }
    
    // Очередь для вершин с нулевой входящей степенью
    queue<int> q;
    for (const auto& node : inDegree) {
        if (node.second == 0) {
            q.push(node.first);
        }
    }
    
    vector<int> result;
    
    while (!q.empty()) {
        int current = q.front();
        q.pop();
        result.push_back(current);
        
        if (graph.find(current) != graph.end()) {
            for (int neighbor : graph[current]) {
                inDegree[neighbor]--;
                if (inDegree[neighbor] == 0) {
                    q.push(neighbor);
                }
            }
        }
    }
    
    // Проверка на наличие цикла
    if (result.size() != stations.size()) {
        throw runtime_error("Обнаружен цикл в графе. Топологическая сортировка невозможна.");
    }
    
    return result;
}

// Сохранение соединений в файл
void GasNetwork::saveToFile(ostream& out) const {
    out << "CONNECTIONS " << connections.size() << endl;
    for (const auto& conn : connections) {
        out << conn.pipe_id << " " << conn.cs_in_id << " " << conn.cs_out_id << endl;
    }
}

// Загрузка соединений из файла
void GasNetwork::loadFromFile(istream& in) {
    connections.clear();
    
    string marker;
    int count;
    in >> marker >> count;
    
    for (int i = 0; i < count; i++) {
        int pipe_id, cs_in_id, cs_out_id;
        in >> pipe_id >> cs_in_id >> cs_out_id;
        connections.push_back(Connection(pipe_id, cs_in_id, cs_out_id));
    }
}


// Построение графа для расчета потока
static std::map<int, std::vector<std::pair<int, double>>> buildFlowGraph(
    const std::vector<Connection>& connections,
    const std::map<int, Pipe>& pipes) {
    
    std::map<int, std::vector<std::pair<int, double>>> graph;
    
    for (const auto& conn : connections) {
        auto pipe_it = pipes.find(conn.pipe_id);
        if (pipe_it != pipes.end()) {
            double capacity = pipe_it->second.getProductivity();
            if (capacity > 0) {  // Только если труба не в ремонте
                graph[conn.cs_in_id].push_back({conn.cs_out_id, capacity});
            }
        }
    }
    
    return graph;
}

// Построение графа для расчета пути
static std::map<int, std::vector<std::pair<int, double>>> buildWeightGraph(
    const std::vector<Connection>& connections,
    const std::map<int, Pipe>& pipes) {
    
    std::map<int, std::vector<std::pair<int, double>>> graph;
    
    for (const auto& conn : connections) {
        auto pipe_it = pipes.find(conn.pipe_id);
        if (pipe_it != pipes.end()) {
            double weight = pipe_it->second.getWeight();
            if (weight < std::numeric_limits<double>::infinity()) {  // Только если не в ремонте
                graph[conn.cs_in_id].push_back({conn.cs_out_id, weight});
            }
        }
    }
    
    return graph;
}

// Алгоритм Дейкстры для поиска кратчайшего пути
static std::pair<std::map<int, double>, std::map<int, int>> dijkstra(
    int start, 
    const std::map<int, std::vector<std::pair<int, double>>>& graph,
    const std::set<int>& nodes) {
    
    // Инициализация
    std::map<int, double> dist;
    std::map<int, int> prev;
    std::set<int> unvisited = nodes;
    
    for (int node : nodes) {
        dist[node] = std::numeric_limits<double>::infinity();
        prev[node] = -1;
    }
    dist[start] = 0;
    
    // Основной цикл алгоритма
    while (!unvisited.empty()) {
        // Находим узел с минимальным расстоянием
        int current = -1;
        double min_dist = std::numeric_limits<double>::infinity();
        
        for (int node : unvisited) {
            if (dist[node] < min_dist) {
                min_dist = dist[node];
                current = node;
            }
        }
        
        if (current == -1 || dist[current] == std::numeric_limits<double>::infinity()) {
            break;  // Оставшиеся узлы недостижимы
        }
        
        unvisited.erase(current);
        
        // Обновляем расстояния до соседей
        if (graph.find(current) != graph.end()) {
            for (const auto& neighbor : graph.at(current)) {
                int next = neighbor.first;
                double weight = neighbor.second;
                
                double new_dist = dist[current] + weight;
                if (new_dist < dist[next]) {
                    dist[next] = new_dist;
                    prev[next] = current;
                }
            }
        }
    }
    
    return {dist, prev};
}

// Расчет максимального потока (упрощенный алгоритм)
double GasNetwork::calculateMaxFlow(int source_id, int sink_id, 
                                   const std::map<int, Pipe>& pipes) const {
    
    if (source_id == sink_id) return 0.0;
    
    // Собираем все узлы
    std::set<int> nodes;
    for (const auto& conn : connections) {
        nodes.insert(conn.cs_in_id);
        nodes.insert(conn.cs_out_id);
    }
    
    // Проверяем существование узлов
    if (nodes.find(source_id) == nodes.end() || nodes.find(sink_id) == nodes.end()) {
        throw std::runtime_error("Исходный или конечный узел не найден в сети");
    }
    
    // Строим матрицу пропускных способностей
    std::map<int, std::map<int, double>> capacity;
    for (const auto& conn : connections) {
        auto pipe_it = pipes.find(conn.pipe_id);
        if (pipe_it != pipes.end()) {
            double cap = pipe_it->second.getProductivity();
            capacity[conn.cs_in_id][conn.cs_out_id] += cap;  // Суммируем, если несколько труб
        }
    }
    
    // Упрощенный алгоритм поиска максимального потока
    double max_flow = 0.0;
    std::map<int, std::map<int, double>> flow;
    
    // Пока есть увеличивающий путь
    while (true) {
        // BFS для поиска увеличивающего пути
        std::queue<int> q;
        std::map<int, int> parent;
        std::map<int, bool> visited;
        
        q.push(source_id);
        visited[source_id] = true;
        parent[source_id] = -1;
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            
            for (const auto& kv : capacity[u]) {
                int v = kv.first;
                double cap = kv.second;
                if (!visited[v] && cap - flow[u][v] > 0) {
                    q.push(v);
                    parent[v] = u;
                    visited[v] = true;
                }
            }
        }
        
        // Если путь не найден - выходим
        if (!visited[sink_id]) break;
        
        // Находим минимальную остаточную пропускную способность на пути
        double path_flow = std::numeric_limits<double>::infinity();
        for (int v = sink_id; v != source_id; v = parent[v]) {
            int u = parent[v];
            path_flow = std::min(path_flow, capacity[u][v] - flow[u][v]);
        }
        
        // Обновляем поток
        for (int v = sink_id; v != source_id; v = parent[v]) {
            int u = parent[v];
            flow[u][v] += path_flow;
            flow[v][u] -= path_flow;  // Обратный поток для остаточной сети
        }
        
        max_flow += path_flow;
    }
    
    return max_flow;
}

// Поиск кратчайшего пути
std::vector<int> GasNetwork::findShortestPath(int start_id, int end_id,
                                             const std::map<int, Pipe>& pipes) const {
    
    if (start_id == end_id) return {start_id};
    
    // Собираем все узлы
    std::set<int> nodes;
    for (const auto& conn : connections) {
        nodes.insert(conn.cs_in_id);
        nodes.insert(conn.cs_out_id);
    }
    
    // Проверяем существование узлов
    if (nodes.find(start_id) == nodes.end() || nodes.find(end_id) == nodes.end()) {
        throw std::runtime_error("Начальный или конечный узел не найден в сети");
    }
    
    // Строим граф весов
    auto graph = buildWeightGraph(connections, pipes);
    
    // Запускаем Дейкстру
    auto [distances, predecessors] = dijkstra(start_id, graph, nodes);
    
    // Проверяем, достижима ли конечная точка
    if (distances[end_id] == std::numeric_limits<double>::infinity()) {
        return {};  // Путь не найден
    }
    
    // Восстанавливаем путь
    std::vector<int> path;
    int current = end_id;
    
    while (current != -1) {
        path.push_back(current);
        current = predecessors[current];
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}