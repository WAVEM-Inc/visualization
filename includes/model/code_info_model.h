//
// Created by antique on 24. 3. 13.
//

#ifndef ROUTE_EDITOR_CODE_INFO_MODEL_H
#define ROUTE_EDITOR_CODE_INFO_MODEL_H


#include <QObject>
#include "struct/CodeGroup.h"

class CodeInfoModel : public QObject {
Q_OBJECT
public:
    static CodeInfoModel &getInstance() {
        static CodeInfoModel instance;
        return instance;
    }

    CodeInfoModel(const CodeInfoModel&) = delete;
    CodeInfoModel& operator=(const CodeInfoModel&) = delete;

public:
    void setCodeMap(const std::vector<CodeGroup>& codes);

    std::map<std::string, CodeGroup> getCodeMap();

    std::vector<Code> getCodesByCategory(const std::string& category);

    std::string getNameByCode(const std::string &codeType, const std::string &code);

signals:
    void codeMapChanged(const std::map<std::string, CodeGroup> &codeMap);

private:
    explicit CodeInfoModel(QObject *parent = nullptr);

private:
    std::map<std::string, CodeGroup> _codeMap;
};

#endif //ROUTE_EDITOR_CODE_INFO_MODEL_H
