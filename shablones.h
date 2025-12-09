#ifndef SHABLONES_H
#define SHABLONES_H

#include <iostream>
#include <functional>


template<typename T>
T getValidInput(const std::string& prompt) {
    T value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value) {
            return value;
        } else {
            std::cout << "Ошибка ввода! Попробуйте снова: ";
            std::cin.clear();
            std::cin.ignore(10000, '\n');
        }
    }
}

template<typename T>
T getValidInput(const std::string& prompt, std::function<bool(T)> condition) {
    T value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && condition(value)) {
            return value;
        } else {
            std::cout << "Ошибка ввода! Попробуйте снова: ";
            std::cin.clear();
            std::cin.ignore(10000, '\n');
        }
    }
}

#endif