//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_PATH_H
#define NODE_EDITOR_PATH_H

#include <iostream>
#include "Node.h"
#include "vector"

struct Path {
    std::string id;
    std::string name;
    std::vector<Node> nodelist;
};

#endif //NODE_EDITOR_PATH_H
