#include "cs.h"
#include "shablones.h"
#include <iostream>

using namespace std;

void CompressorStation::input() {
    cin.ignore(10000, '\n');
    cout << "Укажите название КС: ";
    getline(cin, name);
    
    total_workshops = getValidInput<int>("Укажите всего цехов: ", 
        [](int x) { return x > 0; });
    
    working_workshops = getValidInput<int>("Укажите работающие цехи: ",
        [this](int x) { return x >= 0 && x <= total_workshops; });
    
    cout << "Укажите класс: ";
    cin.ignore();
    getline(cin, station_class);
}

void CompressorStation::display() const {
    cout << "ID: " << id << "\nНазвание: " << name 
         << "\nВсего цехов: " << total_workshops
         << "\nРаботающих: " << working_workshops 
         << "\nКласс: " << station_class
         << "\nНезадействовано: " << getUnusedPercentage() << "%" << endl;
}

ostream& operator<<(ostream& os, const CompressorStation& cs) {
    os << cs.id << endl << cs.name << endl << cs.total_workshops << endl 
       << cs.working_workshops << endl << cs.station_class;
    return os;
}

istream& operator>>(istream& is, CompressorStation& cs) {
    is >> cs.id;
    is.ignore();
    getline(is, cs.name);
    is >> cs.total_workshops >> cs.working_workshops;
    is.ignore();
    getline(is, cs.station_class);
    return is;
}