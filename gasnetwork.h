#ifndef GASNETWORK_H
#define GASNETWORK_H

#include "pipe.h"
#include "cs.h"
#include <vector>
#include <map>
#include <set>
#include <algorithm> // Добавляем для std::remove_if

struct Connection {
    int pipe_id;
    int cs_in_id;   // ID станции входа
    int cs_out_id;  // ID станции выхода
    
    // Конструктор для удобства
    Connection(int pipe, int in, int out) : pipe_id(pipe), cs_in_id(in), cs_out_id(out) {}
};

class GasNetwork {
private:
    std::vector<Connection> connections;
    
public:
    void addConnection(int pipe_id, int cs_in_id, int cs_out_id);
    void displayConnections(const std::map<int, Pipe>& pipes, const std::map<int, CompressorStation>& stations) const;
    bool isPipeConnected(int pipe_id) const;
    bool isConnectionExists(int cs_in_id, int cs_out_id) const;
    bool hasCycle() const;
    std::vector<int> topologicalSort(const std::map<int, CompressorStation>& stations) const;
    void removeConnectionsWithPipe(int pipe_id);
    void removeConnectionsWithCS(int cs_id);
    
    // Методы для сохранения и загрузки
    void saveToFile(std::ostream& out) const;
    void loadFromFile(std::istream& in);
    const std::vector<Connection>& getConnections() const { return connections; }
    
    // Расчет максимального потока между двумя КС
    double calculateMaxFlow(int source_id, int sink_id, 
                           const std::map<int, Pipe>& pipes) const;
    
    // Поиск кратчайшего пути по весу (длине)
    std::vector<int> findShortestPath(int start_id, int end_id,
                                     const std::map<int, Pipe>& pipes) const;
    
   


private:
    bool dfs(int v, std::set<int>& visited, std::set<int>& recursionStack, 
             const std::map<int, std::vector<int>>& graph) const;
};

#endif