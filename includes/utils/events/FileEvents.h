//
// Created by antique on 24. 2. 23.
//

#ifndef NODE_EDITOR_FILEEVENTS_H
#define NODE_EDITOR_FILEEVENTS_H

#include "struct/RouteFile.h"

namespace event {

    struct FILE_CHANGE {
        RouteFile file;
    };

    struct FILE_SAVABLE {
        bool isSavable;
    };
}

#endif //NODE_EDITOR_FILEEVENTS_H
