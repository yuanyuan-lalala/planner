#pragma once
#include "Eigen/Core"
#include "vector"
#include "memory"
class GraphNode
{
public:
    enum NODE_TYPE{
        Guard = 1,
        Connector = 2
    };
    enum NODE_STATE{
        NEW = 1,
        CLOSE = 2,
        OPEN = 3
    };

    GraphNode() {}
    GraphNode(Eigen::Vector3d position, NODE_TYPE type, int id) {
        pos = position;
        type_ = type;
        state_ = NEW;
        m_id = id;
    }
    ~GraphNode() {}

    std::vector<std::shared_ptr<GraphNode>> neighbors;
    std::vector<std::shared_ptr<GraphNode>> neighbors_but_noconnected;
    NODE_TYPE type_;
    NODE_STATE state_;

    Eigen::Vector3d pos;
    int m_id;
    typedef std::shared_ptr<GraphNode> Ptr;
};
