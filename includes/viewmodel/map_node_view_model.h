//
// Created by antique on 24. 2. 20.
//

#ifndef NODE_EDITOR_MAP_NODE_VIEW_MODEL_H
#define NODE_EDITOR_MAP_NODE_VIEW_MODEL_H


#include "model/MapNode.h"
#include "utils/patterns/observer/subject.h"
#include <iostream>

using MAP_NODES = std::shared_ptr<Subject<std::map<std::string, Position>>>;
class MapNodeViewModel : public Singleton<MapNodeViewModel> {
    friend class Singleton<MapNodeViewModel>;

public:
    MapNodeViewModel();

    MAP_NODES mapNodes();

    void update_map_nodes(const std::vector<MapNode>& nodes);

    void add_map_node(const std::string& nodeId, Position position);

    void remove_map_node(const std::string& nodeId);

    void edit_map_node(const std::string& nodeId, Position position);
private:
    MAP_NODES m_mapNodes_ptr;
};


#endif //NODE_EDITOR_MAP_NODE_VIEW_MODEL_H
