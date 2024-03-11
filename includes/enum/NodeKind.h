//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_NODEKIND_H
#define NODE_EDITOR_NODEKIND_H

#include <string>

enum NodeKind {
    INTERSECTION,
    CONNECTING,
    ENDPOINT,
    WAITING
};

std::string getKindName(NodeKind kind);

std::string getKindKorName(NodeKind kind);

#endif //NODE_EDITOR_NODEKIND_H
