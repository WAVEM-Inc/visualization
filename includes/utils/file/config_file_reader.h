//
// Created by antique on 24. 3. 12.
//

#ifndef ROUTE_EDITOR_CONFIG_FILE_READER_H
#define ROUTE_EDITOR_CONFIG_FILE_READER_H


#include "struct/ConfigFile.h"

class ConfigFileReader {
public:
    explicit ConfigFileReader();

    virtual ~ConfigFileReader();

public:
    ConfigFile loadFile();
};


#endif //ROUTE_EDITOR_CONFIG_FILE_READER_H
