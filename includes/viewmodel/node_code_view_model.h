//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODE_CODE_VIEW_MODEL_H
#define NODE_EDITOR_NODE_CODE_VIEW_MODEL_H


#include <map>
#include "utils/patterns/singleton/singleton.h"
#include "utils/patterns/observer/subject.h"
#include "model/NodeCode.h"

class NodeCodeViewModel : public Singleton<NodeCodeViewModel>{
    friend class Singleton<NodeCodeViewModel>;

public:
    NodeCodeViewModel();

    void updateCodes(const std::vector<NodeCode>& codes);

private:
    std::shared_ptr<Subject<std::map<int, std::string>>> m_codes_ptr;
};


#endif //NODE_EDITOR_NODE_CODE_VIEW_MODEL_H
