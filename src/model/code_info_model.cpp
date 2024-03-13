//
// Created by antique on 24. 3. 13.
//

#include <iostream>
#include "model/code_info_model.h"

CodeInfoModel::CodeInfoModel(QObject *parent) : QObject(parent) {

}

void CodeInfoModel::setCodeMap(const std::vector<CodeGroup>& codes) {
    std::map<std::string, CodeGroup> codeMap;
    for (const CodeGroup &group : codes) {
        codeMap.insert(std::pair<std::string, CodeGroup>(group.category, group));
    }

    _codeMap = codeMap;
    emit codeMapChanged(_codeMap);
}

std::map<std::string, CodeGroup> CodeInfoModel::getCodeMap() {
    return _codeMap;
}

std::vector<Code> CodeInfoModel::getCodesByCategory(const std::string& category) {
    nlohmann::json json = _codeMap[category];
    std::cout << json.dump(4) << "\n";

    return _codeMap[category].codes;
}

std::string CodeInfoModel::getNameByCode(const std::string &codeType, const std::string &code) {
    std::vector<Code> codes = getCodesByCategory(codeType);
    for (const Code &c : codes) {
        if (c.code == code) {
            return c.name;
        }
    }
    return "";
}
