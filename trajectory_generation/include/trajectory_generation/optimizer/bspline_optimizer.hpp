#pragma once
#include "optimizer/optimizer_algorithm.hpp"

class BsplineOptimizer:public OptimizerAlgorithm{
    public:
    explicit BsplineOptimizer(std::shared_ptr<GlobalMap> global_map);
    void optimize() override;



};