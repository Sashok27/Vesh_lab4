#ifndef CS_H
#define CS_H

#include <iostream>
#include <string>

class CompressorStation {
public:
    int id;  
    
    //конструкторы
    CompressorStation() : id(0), name(""), total_workshops(0), working_workshops(0), station_class("") {}
    CompressorStation(int station_id) : id(station_id), name(""), total_workshops(0), working_workshops(0), station_class("") {}
    
    
    void input();
    void display() const;
    void startWorkshop() { if (working_workshops < total_workshops) working_workshops++; }
    void stopWorkshop() { if (working_workshops > 0) working_workshops--; }
    
    
    std::string getName() const { return name; }
    int getTotalWorkshops() const { return total_workshops; }
    int getWorkingWorkshops() const { return working_workshops; }
    std::string getClass() const { return station_class; }
    double getUnusedPercentage() const { 
        return total_workshops > 0 ? (total_workshops - working_workshops) * 100.0 / total_workshops : 0; 
    }

    friend std::ostream& operator<<(std::ostream& os, const CompressorStation& cs);
    friend std::istream& operator>>(std::istream& is, CompressorStation& cs);

private:
    std::string name;
    int total_workshops;
    int working_workshops;
    std::string station_class;
};

std::ostream& operator<<(std::ostream& os, const CompressorStation& cs);
std::istream& operator>>(std::istream& is, CompressorStation& cs);

#endif