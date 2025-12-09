#include "pipe.h"
#include "shablones.h"
#include <iostream>
#include <cmath>      // для sqrt pow
#include <limits>     // для numeric_limits
using namespace std; 

void Pipe::input() {
    cin.ignore(10000, '\n');
    cout << "Укажите название трубы: ";
    getline(cin, name);
    
    length = getValidInput<float>("Укажите длину (км): ", [](float x) { return x > 0; });
    
    // Ограничение диаметров
    diameter = getValidInput<int>("Укажите диаметр (500, 700, 1000, 1400 мм): ", 
        [](int x) { return x == 500 || x == 700 || x == 1000 || x == 1400; });
    
    repair = false;
}

void Pipe::display() const {
    cout << "ID: " << id << "\nНазвание: " << name << "\nДлина: " << length 
         << " км\nДиаметр: " << diameter << " мм\nВ ремонте: " << (repair ? "Да" : "Нет") 
         << "\nПропускная способность: " << getProductivity() 
         << " усл. ед.\nВес для пути: " 
         << (getWeight() == std::numeric_limits<double>::infinity() ? "бесконечность" : to_string(getWeight())) 
         << endl;
}

std::ostream& operator<<(std::ostream& os, const Pipe& pipe) {
    os << pipe.id << endl << pipe.name << endl << pipe.length << endl 
       << pipe.diameter << endl << pipe.repair;
    return os;
}

std::istream& operator>>(std::istream& is, Pipe& pipe) {
    is >> pipe.id;
    is.ignore();
    getline(is, pipe.name);
    is >> pipe.length >> pipe.diameter >> pipe.repair;
    return is;
}


double Pipe::getProductivity() const {
    if (repair) return 0.0;  // Если в ремонте - пропускная способность 0
    
    //диаметр в мм, переводим в метры
    double d_m = diameter / 1000.0;
    //длина в км, переводим в метры
    double l_m = length * 1000.0;
    
    //проверка деления на ноль
    if (l_m <= 0) return 0.0;
    
    // c * sqrt(d^5 / l), c = 0.0384
    double productivity = 0.0384 * sqrt(pow(d_m, 5) / l_m);
    
    //поправочный коэффициент для удобства
    productivity *= 1000;
    
    return productivity;
}

double Pipe::getWeight() const {
    if (repair) return std::numeric_limits<double>::infinity();  // Бесконечный вес при ремонте
    return length;  // Вес = длина в км
}