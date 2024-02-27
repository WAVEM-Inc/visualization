//
// Created by antique on 24. 2. 26.
//

#ifndef NODE_EDITOR_ROUTE_FILE_READER_H
#define NODE_EDITOR_ROUTE_FILE_READER_H


#include <QObject>

class RouteFileReader : public QObject {
Q_OBJECT
public:
    explicit RouteFileReader(QObject *parent = nullptr);

    bool loadFile(const QString &filePath);
};


#endif //NODE_EDITOR_ROUTE_FILE_READER_H
