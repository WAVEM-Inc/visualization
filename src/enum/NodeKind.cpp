//
// Created by antique on 24. 3. 5.
//

#include "enum/NodeKind.h"

std::string getKindName(NodeKind kind) {
    switch (kind) {
        case NodeKind::INTERSECTION:
            return "intersection";
        case NodeKind::CONNECTING:
            return "connecting";
        case NodeKind::ENDPOINT:
            return "endpoint";
        case NodeKind::WAITING:
            return "waiting";
        default:
            return "";
    }
}

std::string getKindKorName(NodeKind kind) {
    switch (kind) {
        case NodeKind::INTERSECTION:
            return "교차로";
        case NodeKind::CONNECTING:
            return "연결";
        case NodeKind::ENDPOINT:
            return "종점";
        case NodeKind::WAITING:
            return "일시정지";
        default:
            return "";
    }
}