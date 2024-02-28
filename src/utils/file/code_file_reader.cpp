//
// Created by antique on 24. 2. 27.
//

#include <string>
#include <fstream>
#include <iostream>
#include "utils/file/code_file_reader.h"

std::vector<NodeCode> CodeFileReader::loadFile() {
    std::string filePath(RESOURCE_DIR);
    filePath += "/code/code.json";
    std::ifstream file(filePath);
    std::string fileContents;

    if (file) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        fileContents = buffer.str();
    }

    nlohmann::json json = nlohmann::json::parse(fileContents);
    std::vector<NodeCode> codes = json.get<std::vector<NodeCode>>();

    file.close();

    return codes;
}
