//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_CODE_FILE_READER_H
#define NODE_EDITOR_CODE_FILE_READER_H


#include "struct/NodeCode.h"
#include "struct/CodeGroup.h"

class CodeFileReader {
public:
    std::vector<CodeGroup> loadFile();
};


#endif //NODE_EDITOR_CODE_FILE_READER_H
