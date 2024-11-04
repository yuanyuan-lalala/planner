#include"iostream"
#include"chrono"
#include"functional"
#include "stdexcept"
#include"iomanip"
#define TIME_IT(operation)                                                                              \     
    do{                                                                                                 \               
        auto start_time =  std::chrono::high_resolution_clock::now();                                   \
        operation;                                                                                      \
        auto end_time = std::chrono::high_resolution_clock::now();                                      \ 
        std::chrono::duration<double> duration = end_time - start_time;                                 \
        std::cout<<std::setprecision(2)<<"Operation took"<<duration.count()*1000<<" ms."<<std::endl;    \
    }while(0)                                                                                           \

inline void time_it(const std::function<void()>& operation) {
    auto start_time = std::chrono::high_resolution_clock::now();
    operation();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << std::setprecision(2) << "Operation took " << duration.count() * 1000 << " ms." << std::endl;
}