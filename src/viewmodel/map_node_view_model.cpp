//
// Created by antique on 24. 2. 20.
//

#include "viewmodel/map_node_view_model.h"

MapNodeViewModel::MapNodeViewModel() : m_mapNodes_ptr(std::make_shared<Subject<std::map<std::string, Position>>>()) {
}

MAP_NODES MapNodeViewModel::mapNodes() {
    return m_mapNodes_ptr;
}

void MapNodeViewModel::add_map_node(std::string nodeId, Position position) {
    std::map<std::string, Position> value = m_mapNodes_ptr->value();
    value.insert(std::make_pair(nodeId, position));

    m_mapNodes_ptr->notify(value);
}

void MapNodeViewModel::edit_map_node(std::string nodeId, Position position) {
    std::map<std::string, Position> value = m_mapNodes_ptr->value();
    value[nodeId] = position;

    m_mapNodes_ptr->notify(value);
}
