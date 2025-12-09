#ifndef MANAGER_H
#define MANAGER_H

#include "pipe.h"
#include "cs.h"
#include "gasnetwork.h"
#include "shablones.h"
#include <map>
#include <string>

class Manager {
private:
    std::map<int, Pipe> pipes;
    std::map<int, CompressorStation> stations;
    GasNetwork network;
    int next_pipe_id = 0;
    int next_station_id = 0;

    void displayMenu() const;
    void addPipe();
    void addStation();
    void displayAllObjects() const;
    void batchEditObjects();
    void saveToFile() const;
    void loadFromFile();
    void logAction(const std::string& action) const;
    void connectStations();
    void displayNetwork() const;
    void topologicalSort();

    int getNextPipeId() { return next_pipe_id++; }
    int getNextStationId() { return next_station_id++; }

public:
    void run();
};

#endif