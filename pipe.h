#ifndef PIPE_H
#define PIPE_H

#include <iostream>
#include <string>

class Pipe {
public:
    int id;  
    
    
    Pipe() : id(0), name(""), length(0), diameter(0), repair(false) {}
    Pipe(int pipe_id) : id(pipe_id), name(""), length(0), diameter(0), repair(false) {}
    
    
    void input();
    void display() const;
    
    
    std::string getName() const { return name; }
    float getLength() const { return length; }
    int getDiameter() const { return diameter; }
    bool getRepair() const { return repair; }
    void setRepair(bool status) { repair = status; }
    void toggleRepair() { repair = !repair; }

    friend std::ostream& operator<<(std::ostream& os, const Pipe& pipe);
    friend std::istream& operator>>(std::istream& is, Pipe& pipe);

private:
    std::string name;
    float length;
    int diameter;
    bool repair;
};

//перегрузка операторов
std::ostream& operator<<(std::ostream& os, const Pipe& pipe);
std::istream& operator>>(std::istream& is, Pipe& pipe);

#endif