//
// Created by antique on 24. 2. 20.
//

#ifndef NODE_EDITOR_CONCRETEOBSERVER_H
#define NODE_EDITOR_CONCRETEOBSERVER_H

#include "observer.h"
#include "model/MapNode.h"

class ConcreteObserver : public Observer<MapNode> {
public:
    void update(MapNode node) override {
        observerState = node;

        nlohmann::json node_json = node;
        std::cout << node_json.dump(4) << "\n";

/*        std::cout << "Node_ID: " << node.nodeId << "Position: " << observerState.position.latitude << ", "
                  << observerState.position.longitude << "\n";*/
    }

private:
    MapNode observerState;
};

#endif //NODE_EDITOR_CONCRETEOBSERVER_H
