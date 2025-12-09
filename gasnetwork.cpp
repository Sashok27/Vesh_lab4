#include "gasnetwork.h"
#include <iostream>
#include <stack>
#include <queue>
#include <algorithm>

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