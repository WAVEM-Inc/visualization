//
// Created by antique on 24. 3. 12.
//

#ifndef ROUTE_EDITOR_CONFIG_FILE_WRITER_H
#define ROUTE_EDITOR_CONFIG_FILE_WRITER_H


class ConfigFileWriter {
public:
    virtual ~ConfigFileWriter();

    bool saveFile();

public:
    explicit ConfigFileWriter();
};


#endif //ROUTE_EDITOR_CONFIG_FILE_WRITER_H
