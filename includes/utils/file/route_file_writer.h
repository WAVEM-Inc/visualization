//
// Created by antique on 24. 2. 26.
//

#ifndef NODE_EDITOR_ROUTE_FILE_WRITER_H
#define NODE_EDITOR_ROUTE_FILE_WRITER_H


#include <QString>
#include "struct/RouteFile.h"

class RouteFileWriter : QObject {
Q_OBJECT
public:
    explicit RouteFileWriter(QObject *parent = nullptr);

    bool saveFile(const QString &filePath, const RouteFile &routeFileData);
};

#endif //NODE_EDITOR_ROUTE_FILE_WRITER_H
