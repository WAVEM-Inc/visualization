//
// Created by antique on 24. 2. 27.
//

#include "viewmodel/node_view_model.h"

NodeViewModel::NodeViewModel() : m_nodes_ptr(std::make_shared<Subject<std::map<std::string, Node>>>()){

}

NodeViewModel::~NodeViewModel() {

}

void NodeViewModel::addNode(Node node) {

}

void NodeViewModel::updateNode(Node node) {

}

void NodeViewModel::removeNode(std::string nodeId) {

}
