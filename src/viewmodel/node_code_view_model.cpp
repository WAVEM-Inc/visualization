//
// Created by antique on 24. 2. 27.
//

#include "viewmodel/node_code_view_model.h"
#include "utils/file/code_file_reader.h"

NodeCodeViewModel::NodeCodeViewModel() : m_codes_ptr(std::make_shared<Subject<std::map<int, std::string>>>()){
    CodeFileReader reader;
    std::vector<NodeCode> codes = reader.loadFile();
}

void NodeCodeViewModel::updateCodes(const std::vector<NodeCode>& codes) {
    std::map<int, std::string> codesMap;
    for (const NodeCode &code : codes) {
        codesMap.insert({code.code, code.name});
    }

    m_codes_ptr->notify(codesMap);
}




