//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_NODETYPE_H
#define NODE_EDITOR_NODETYPE_H

#include <string>

enum NodeType {
    PATH_NODE,
    WORK_NODE,
    WORKPLACE_NODE
};

std::string getTypeName(NodeType type);

NodeType getTypeFromName(const std::string& text);

std::string getTypeKorName(NodeType type);

NodeType getTypeFromKorName(const std::string &text);

#endif //NODE_EDITOR_NODETYPE_H
