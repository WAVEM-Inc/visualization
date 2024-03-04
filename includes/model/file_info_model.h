//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_FILE_INFO_MODEL_H
#define NODE_EDITOR_FILE_INFO_MODEL_H


#include <QObject>
#include "struct/FileInfo.h"

class FileInfoModel : public QObject {
Q_OBJECT

public:
    static FileInfoModel &getInstance() {
        static FileInfoModel instance;
        return instance;
    }

    FileInfoModel(const FileInfoModel&) = delete;
    FileInfoModel& operator=(const FileInfoModel&) = delete;

    void updateFileInfo(const FileInfo &info);

    [[nodiscard]] FileInfo getFileInfo() const;

    void updateFileSavable(bool savable);

    [[nodiscard]] bool getFileSavable() const;

signals:
    void fileInfoChanged(const FileInfo &info);

    void fileSavableChanged(bool savable);

private:
    explicit FileInfoModel(QObject *parent = nullptr);

private:
    FileInfo _fileInfo;
    bool _fileSavable;
};


#endif //NODE_EDITOR_FILE_INFO_MODEL_H
