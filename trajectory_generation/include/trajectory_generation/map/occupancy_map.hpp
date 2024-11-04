#pragma once
#include"base_map.hpp"
class OccupancyMap : public BaseMap{

    OccupancyNodePtr **OccupancyMap; /// 2D地图用于进行raycast

};