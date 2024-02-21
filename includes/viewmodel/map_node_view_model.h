//
// Created by antique on 24. 2. 20.
//

#ifndef NODE_EDITOR_MAP_NODE_VIEW_MODEL_H
#define NODE_EDITOR_MAP_NODE_VIEW_MODEL_H


#include "model/MapNode.h"
#include "patterns/observer/subject.h"

class MapNodeViewModel : public Subject<MapNode>, public Singleton<MapNodeViewModel> {
    friend class Singleton<MapNodeViewModel>;

};


#endif //NODE_EDITOR_MAP_NODE_VIEW_MODEL_H
