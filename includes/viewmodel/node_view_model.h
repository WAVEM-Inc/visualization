//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODE_VIEW_MODEL_H
#define NODE_EDITOR_NODE_VIEW_MODEL_H


#include <utility>

#include "model/Node.h"

#include "utils/patterns/singleton/singleton.h"
#include "utils/patterns/observer/subject.h"

using NODES = std::shared_ptr<Subject<std::map<std::string, Node>>>;
class NodeViewModel : public Singleton<NodeViewModel>{
    friend class Singleton<NodeViewModel>;

public:
    NodeViewModel();

    virtual ~NodeViewModel();

    void addNode(Node node);

    void updateNode(Node node);

    void removeNode(std::string nodeId);

    void attachToNodes(std::shared_ptr<Observer<std::map<std::string, Node>>> observer) {
        m_nodes_ptr->attach(std::move(observer));
    }

    void detachFromNodes(std::shared_ptr<Observer<std::map<std::string, Node>>> observer) {
        m_nodes_ptr->detach(std::move(observer));
    }

private:
    NODES m_nodes_ptr;
};


#endif //NODE_EDITOR_NODE_VIEW_MODEL_H
