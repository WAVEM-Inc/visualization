//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_CODE_FILE_READER_H
#define NODE_EDITOR_CODE_FILE_READER_H


#include "struct/NodeCode.h"

class CodeFileReader {
public:
    std::vector<NodeCode> loadFile();
};


#endif //NODE_EDITOR_CODE_FILE_READER_H
