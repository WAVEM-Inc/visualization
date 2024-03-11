//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_PATH_INFO_MODEL_H
#define NODE_EDITOR_PATH_INFO_MODEL_H


#include <QObject>
#include <qmap.h>
#include "struct/Path.h"

class PathInfoModel : public QObject {
Q_OBJECT

public:
    static PathInfoModel &getInstance() {
        static PathInfoModel instance;
        return instance;
    }

    PathInfoModel(const PathInfoModel &) = delete;

    PathInfoModel &operator=(const PathInfoModel &) = delete;

    void setPathInfoMap(const QMap<std::string, std::string>& pathInfoMap);

    void setPathInfoMap(const std::vector<Path> &paths);

    QMap<std::string, std::string> getPathInfoMap() const;

    void addPathInfo(const std::string& id, const std::string& name);

    bool updatePathInfo(const std::string& id, const std::string& name);

    void selectCurrentPathId(const std::string &pathId);

    std::string getCurrentPathId();

    std::string getCurrentPathName() const;

signals:
    void pathInfoMapChanged(const QMap<std::string, std::string> &pathInfoMap);

    void currentPathIdChanged(const std::string &pathId);

private:
    explicit PathInfoModel(QObject *parent = nullptr);

    void clearPathInfoMap();

private:
    QMap<std::string, std::string> _pathInfoMap; // key: pathId, value: pathName
    std::string _currentPathId;
};


#endif //NODE_EDITOR_PATH_INFO_MODEL_H
