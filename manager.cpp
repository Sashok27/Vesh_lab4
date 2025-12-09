#include "manager.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

using namespace std;

void Manager::run() {
    while (true) {
        displayMenu();
        int option = getValidInput<int>("");
        
        switch (option) {
            case 1: addPipe(); break;
            case 2: addStation(); break;
            case 3: displayAllObjects(); break;
            case 4: batchEditObjects(); break;
            case 5: saveToFile(); break;
            case 6: loadFromFile(); break;
            case 7: connectStations(); break;
            case 8: displayNetwork(); break;
            case 9: topologicalSort(); break;
            case 0: cout << "Выход из программы" << endl; return;
            default: cout << "Неверный выбор!" << endl;
        }
    }
}

void Manager::displayMenu() const {
    cout << "\nВыберите действие:\n"
         << "1. Добавить трубу\n2. Добавить КС\n3. Просмотреть все объекты\n"
         << "4. Пакетное редактирование объектов\n5. Сохранить\n6. Загрузить\n"
         << "7. Соединить станции\n8. Показать сеть\n9. Топологическая сортировка\n"
         << "0. Выход\nВаш выбор: ";
}

void Manager::addPipe() {
    Pipe pipe(getNextPipeId());
    pipe.input();
    pipes[pipe.id] = pipe;
    cout << "Труба добавлена с ID: " << pipe.id << endl;
    logAction("Добавлена труба ID " + to_string(pipe.id));
}

void Manager::addStation() {
    CompressorStation station(getNextStationId());
    station.input();
    stations[station.id] = station;
    cout << "КС добавлена с ID: " << station.id << endl;
    logAction("Добавлена КС ID " + to_string(station.id));
}

void Manager::displayAllObjects() const {
    cout << "\nТрубы (" << pipes.size() << "):" << endl;
    if (pipes.empty()) cout << "Нет труб" << endl;
    else for (const auto& p : pipes) p.second.display();
    
    cout << "\nКомпрессорные станции (" << stations.size() << "):" << endl;
    if (stations.empty()) cout << "Нет КС" << endl;
    else for (const auto& s : stations) s.second.display();
    
    displayNetwork();
}

void Manager::batchEditObjects() {
    displayAllObjects();
    
    cout << "\nВыберите тип объектов:\n"
         << "0 - Трубы\n1 - КС\nВаш выбор: ";
    
    int object_type = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
    
    if (object_type == 0) { // Работа с трубами
        if (pipes.empty()) {
            cout << "Нет доступных труб" << endl;
            return;
        }
        
        cout << "Выберите способ выбора труб:\n0 - Выбрать конкретные\n1 - Все трубы\nВаш выбор: ";
        int selection_mode = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
        
        map<int, Pipe> selected_pipes;
        
        if (selection_mode == 1) { 
            selected_pipes = pipes;
        } else { //выбрать конкретные
            cout << "Выберите способ поиска:\n0 - Ввести ID\n1 - Поиск по фильтру\nВаш выбор: ";
            int search_mode = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
            
            if (search_mode == 0) {
                cout << "Введите ID труб через пробел: ";
                cin.ignore();
                string input;
                getline(cin, input);
                
                stringstream ss(input);
                int id;
                while (ss >> id) {
                    if (pipes.find(id) != pipes.end()) {
                        selected_pipes[id] = pipes[id];
                    } else {
                        cout << "Труба с ID " << id << " не найдена" << endl;
                    }
                }
            } else { //фильтр
                cout << "Поиск труб:\n0 - По названию\n1 - По признаку 'в ремонте'\nВаш выбор: ";
                int filter_choice = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
                
                if (filter_choice == 0) { 
                    string name;
                    cout << "Введите название для поиска: ";
                    cin.ignore();
                    getline(cin, name);
                    for (const auto& p : pipes) {
                        if (p.second.getName().find(name) != string::npos) {
                            selected_pipes[p.first] = p.second;
                        }
                    }
                } else { 
                    bool repair_status = getValidInput<int>("Искать трубы в ремонте? (1-Да, 0-Нет): ") == 1;
                    for (const auto& p : pipes) {
                        if (p.second.getRepair() == repair_status) {
                            selected_pipes[p.first] = p.second;
                        }
                    }
                }
                
                if (selected_pipes.empty()) {
                    cout << "Трубы не найдены" << endl;
                    return;
                }
                
                cout << "Найдено труб: " << selected_pipes.size() << endl;
                for (const auto& p : selected_pipes) {
                    p.second.display();
                }
                
                cout << "Выберите:\n0 - Выбрать конкретные\n1 - Все найденные\nВаш выбор: ";
                int choose_mode = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
                
                if (choose_mode == 0) { 
                    map<int, Pipe> temp_pipes = selected_pipes;
                    selected_pipes.clear();
                    
                    cout << "Введите ID найденных труб через пробел: ";
                    cin.ignore();
                    string input;
                    getline(cin, input);
                    
                    stringstream ss(input);
                    int id;
                    while (ss >> id) {
                        if (temp_pipes.find(id) != temp_pipes.end()) {
                            selected_pipes[id] = temp_pipes[id];
                        } else {
                            cout << "Труба с ID " << id << " не найдена в результатах поиска" << endl;
                        }
                    }
                }
            }
        }
        
        if (selected_pipes.empty()) {
            cout << "Не выбрано ни одной трубы" << endl;
            return;
        }
        
        cout << "Выбрано труб: " << selected_pipes.size() << endl;
        cout << "Выберите действие:\n0 - Удалить\n1 - Изменить\nВаш выбор: ";
        int action_choice = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
        
        if (action_choice == 0) { 
            for (const auto& p : selected_pipes) {
                network.removeConnectionsWithPipe(p.first); // Удаляем соединения с этой трубой
                pipes.erase(p.first);
                cout << "Труба ID " << p.first << " удалена" << endl;
                logAction("Удалена труба ID " + to_string(p.first));
            }
        } else { 
            cout << "Установить статус 'в ремонте'? (1-Да, 0-Нет): ";
            bool new_status = getValidInput<int>("") == 1;
            
            for (const auto& p : selected_pipes) {
                pipes[p.first].setRepair(new_status);
                cout << "Труба ID " << p.first << " обновлена" << endl;
                logAction("Пакетное редактирование трубы ID " + to_string(p.first));
            }
        }
        
    } else { // Работа с КС
        if (stations.empty()) {
            cout << "Нет доступных КС" << endl;
            return;
        }
        
        cout << "Выберите способ выбора КС:\n0 - Выбрать конкретные\n1 - Все КС\nВаш выбор: ";
        int selection_mode = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
        
        map<int, CompressorStation> selected_stations;
        
        if (selection_mode == 1) { 
            selected_stations = stations;
        } else { 
            cout << "Выберите способ поиска:\n0 - Ввести ID\n1 - Поиск по фильтру\nВаш выбор: ";
            int search_mode = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
            
            if (search_mode == 0) { 
                cout << "Введите ID КС через пробел: ";
                cin.ignore();
                string input;
                getline(cin, input);
                
                stringstream ss(input);
                int id;
                while (ss >> id) {
                    if (stations.find(id) != stations.end()) {
                        selected_stations[id] = stations[id];
                    } else {
                        cout << "КС с ID " << id << " не найдена" << endl;
                    }
                }
            } else { 
                cout << "Поиск КС:\n0 - По названию\n1 - По проценту незадействованных цехов\nВаш выбор: ";
                int filter_choice = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
                
                if (filter_choice == 0) { 
                    string name;
                    cout << "Введите название для поиска: ";
                    cin.ignore();
                    getline(cin, name);
                    for (const auto& s : stations) {
                        if (s.second.getName().find(name) != string::npos) {
                            selected_stations[s.first] = s.second;
                        }
                    }
                } else { //процент простоя
                    double min_percent = getValidInput<double>("Минимальный процент незадействованных цехов: ");
                    for (const auto& s : stations) {
                        if (s.second.getUnusedPercentage() >= min_percent) {
                            selected_stations[s.first] = s.second;
                        }
                    }
                }
                
                if (selected_stations.empty()) {
                    cout << "КС не найдены" << endl;
                    return;
                }
                
                cout << "Найдено КС: " << selected_stations.size() << endl;
                for (const auto& s : selected_stations) {
                    s.second.display();
                }
                
                cout << "Выберите:\n0 - Выбрать конкретные\n1 - Все найденные\nВаш выбор: ";
                int choose_mode = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
                
                if (choose_mode == 0) { 
                    map<int, CompressorStation> temp_stations = selected_stations;
                    selected_stations.clear();
                    
                    cout << "Введите ID найденных КС через пробел: ";
                    cin.ignore();
                    string input;
                    getline(cin, input);
                    
                    stringstream ss(input);
                    int id;
                    while (ss >> id) {
                        if (temp_stations.find(id) != temp_stations.end()) {
                            selected_stations[id] = temp_stations[id];
                        } else {
                            cout << "КС с ID " << id << " не найдена в результатах поиска" << endl;
                        }
                    }
                }
            }
        }
        
        if (selected_stations.empty()) {
            cout << "Не выбрано ни одной КС" << endl;
            return;
        }
        
        cout << "Выбрано КС: " << selected_stations.size() << endl;
        cout << "Выберите действие:\n0 - Удалить\n1 - Изменить\nВаш выбор: ";
        int action_choice = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
        
        if (action_choice == 0) { 
            for (const auto& s : selected_stations) {
                network.removeConnectionsWithCS(s.first); // Удаляем соединения с этой КС
                stations.erase(s.first);
                cout << "КС ID " << s.first << " удалена" << endl;
                logAction("Удалена КС ID " + to_string(s.first));
            }
        } else { 
            cout << "Выберите действие с цехами:\n0 - Запустить цех\n1 - Остановить цех\nВаш выбор: ";
            int workshop_choice = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
            
            for (const auto& s : selected_stations) {
                auto& station = stations[s.first];
                if (workshop_choice == 0) {
                    station.startWorkshop();
                } else {
                    station.stopWorkshop();
                }
                cout << "КС ID " << s.first << " обновлена. Работает цехов: " 
                     << station.getWorkingWorkshops() << endl;
                logAction("Пакетное редактирование КС ID " + to_string(s.first));
            }
        }
    }
}

void Manager::saveToFile() const {
    string filename;
    cout << "Введите имя файла: ";
    cin.ignore();
    getline(cin, filename);
    
    ofstream file(filename);
    if (!file) {
        cout << "Ошибка создания файла" << endl;
        return;
    }
    
    // Сохраняем трубы
    file << "PIPES " << pipes.size() << endl;
    for (const auto& p : pipes) {
        file << p.second << endl;
    }
    
    // Сохраняем станции
    file << "STATIONS " << stations.size() << endl;
    for (const auto& s : stations) {
        file << s.second << endl;
    }
    
    // Сохраняем соединения
    network.saveToFile(file);
    
    file.close();
    cout << "Данные сохранены в файл: " << filename << endl;
    logAction("Сохранение в файл: " + filename);
}

void Manager::loadFromFile() {
    string filename;
    cout << "Введите имя файла: ";
    cin.ignore();
    getline(cin, filename);
    
    ifstream file(filename);
    if (!file) {
        cout << "Ошибка открытия файла" << endl;
        return;
    }
    
    pipes.clear();
    stations.clear();
    
    string marker;
    int count;
    
    // Загружаем трубы
    file >> marker >> count;
    for (int i = 0; i < count; i++) {
        Pipe pipe;
        file >> pipe;
        pipes[pipe.id] = pipe;
        if (pipe.id >= next_pipe_id) next_pipe_id = pipe.id + 1;
    }
    
    // Загружаем станции
    file >> marker >> count;
    for (int i = 0; i < count; i++) {
        CompressorStation station;
        file >> station;
        stations[station.id] = station;
        if (station.id >= next_station_id) next_station_id = station.id + 1;
    }
    
    // Загружаем соединения
    network.loadFromFile(file);
    
    file.close();
    cout << "Данные загружены из файла: " << filename << endl;
    logAction("Загрузка из файла: " + filename);
}

void Manager::connectStations() {
    if (stations.size() < 2) {
        cout << "Для соединения нужно как минимум 2 КС." << endl;
        return;
    }
    
    cout << "Доступные диаметры труб: 500, 700, 1000, 1400 мм" << endl;
    int diameter = getValidInput<int>("Введите диаметр трубы: ", 
        [](int x) { return x == 500 || x == 700 || x == 1000 || x == 1400; });
    
    // Поиск свободной трубы с заданным диаметром
    int selected_pipe_id = -1;
    for (const auto& pipe : pipes) {
        if (pipe.second.getDiameter() == diameter && !network.isPipeConnected(pipe.first)) {
            selected_pipe_id = pipe.first;
            cout << "Найдена свободная труба ID " << pipe.first << " с диаметром " << diameter << " мм" << endl;
            break;
        }
    }
    
    // Если труба не найдена, предлагаем создать новую
    if (selected_pipe_id == -1) {
        cout << "Свободной трубы с диаметром " << diameter << " мм не найдено." << endl;
        cout << "Хотите создать новую трубу? (1 - Да, 0 - Нет): ";
        int choice = getValidInput<int>("", [](int x) { return x == 0 || x == 1; });
        
        if (choice == 1) {
            addPipe();
            selected_pipe_id = next_pipe_id - 1; // ID новой трубы
            
            // Проверяем, что диаметр новой трубы соответствует требуемому
            if (pipes[selected_pipe_id].getDiameter() != diameter) {
                cout << "Диаметр новой трубы не соответствует требуемому. Соединение не создано." << endl;
                return;
            }
        } else {
            return;
        }
    }
    
    // Выбор станций для соединения
    cout << "\nДоступные КС:" << endl;
    for (const auto& station : stations) {
        cout << "ID: " << station.first << " - " << station.second.getName() << endl;
    }
    
    int cs_in_id = getValidInput<int>("Введите ID входной КС: ", 
        [this](int x) { return stations.find(x) != stations.end(); });
    
    int cs_out_id = getValidInput<int>("Введите ID выходной КС: ", 
        [this, cs_in_id](int x) { 
            return stations.find(x) != stations.end() && x != cs_in_id; 
        });
    
    // Проверки на корректность соединения
    if (network.isConnectionExists(cs_in_id, cs_out_id)) {
        cout << "Ошибка: такое соединение уже существует." << endl;
        return;
    }
    
    // Проверка на обратное соединение
    if (network.isConnectionExists(cs_out_id, cs_in_id)) {
        cout << "Предупреждение: существует обратное соединение КС " << cs_out_id 
             << " -> КС " << cs_in_id << endl;
    }
    
    // Проверка на создание цикла
    GasNetwork temp_network = network; // Временная копия для проверки
    temp_network.addConnection(selected_pipe_id, cs_in_id, cs_out_id);
    if (temp_network.hasCycle()) {
        cout << "Ошибка: создание этого соединения приведет к циклу в графе." << endl;
        return;
    }
    
    // Добавление соединения
    network.addConnection(selected_pipe_id, cs_in_id, cs_out_id);
    cout << "Соединение создано: КС " << cs_in_id << " -> КС " << cs_out_id 
         << " (труба ID " << selected_pipe_id << ")" << endl;
    
    logAction("Создано соединение: КС " + to_string(cs_in_id) + " -> КС " + 
              to_string(cs_out_id) + " (труба ID " + to_string(selected_pipe_id) + ")");
}

void Manager::displayNetwork() const {
    network.displayConnections(pipes, stations);
}

void Manager::topologicalSort() {
    if (stations.empty()) {
        cout << "Нет КС для сортировки." << endl;
        return;
    }
    
    if (network.getConnections().empty()) {
        cout << "Нет соединений между КС." << endl;
        return;
    }
    
    try {
        vector<int> sorted = network.topologicalSort(stations);
        cout << "Топологический порядок КС:" << endl;
        for (size_t i = 0; i < sorted.size(); i++) {
            int id = sorted[i];
            cout << i + 1 << ". КС ID: " << id << " - \"" << stations.at(id).getName() << "\"" << endl;
        }
        logAction("Выполнена топологическая сортировка");
    } catch (const exception& e) {
        cout << "Ошибка: " << e.what() << endl;
    }
}

void Manager::logAction(const string& action) const {
    ofstream log("actions.log", ios::app);
    auto now = chrono::system_clock::now();
    time_t time = chrono::system_clock::to_time_t(now);
    char time_str[100];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&time));
    log << "[" << time_str << "] " << action << endl;
}