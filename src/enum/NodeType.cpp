//
// Created by antique on 24. 2. 29.
//

#include <stdexcept>
#include "enum/NodeType.h"

std::string getTypeName(NodeType type) {
    switch (type) {
        case PATH_NODE:
            return "path_node";
        case WORK_NODE:
            return "work_node";
        case WORKPLACE_NODE:
            return "workplace_node";
    }
}

NodeType getTypeFromName(const std::string& text) {
    if (text == "path_node") {
        return PATH_NODE;
    } else if (text == "work_node") {
        return WORK_NODE;
    } else if (text == "workplace_node") {
        return WORKPLACE_NODE;
    }

    // 기본값 또는 에러 처리
    throw std::invalid_argument("Unknown node type: " + text);
}

std::string getTypeKorName(NodeType type) {
    switch (type) {
        case PATH_NODE:
            return "경로";
        case WORK_NODE:
            return "작업";
        case WORKPLACE_NODE:
            return "대기";
    }
}

NodeType getTypeFromKorName(const std::string &text) {
    if (text == "경로") {
        return PATH_NODE;
    } else if (text == "작업") {
        return WORK_NODE;
    } else if (text == "대기") {
        return WORKPLACE_NODE;
    }

    // 한국어 이름에 해당하는 NodeType이 없는 경우 에러 처리
    throw std::invalid_argument("Unknown node type: " + text);
}