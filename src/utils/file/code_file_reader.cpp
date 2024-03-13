//
// Created by antique on 24. 2. 27.
//

#include <string>
#include <fstream>
#include "utils/file/code_file_reader.h"

std::vector<CodeGroup> CodeFileReader::loadFile() {
    std::string filePath(RESOURCE_DIR);
    filePath += "/code/codes.json";
    std::ifstream file(filePath);
    std::string fileContents;

    if (file) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        fileContents = buffer.str();
    }

    nlohmann::json json = nlohmann::json::parse(fileContents);
    std::vector<CodeGroup> codes = json.get<std::vector<CodeGroup>>();

    file.close();

    return codes;
}
