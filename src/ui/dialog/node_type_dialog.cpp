//
// Created by antique on 24. 2. 29.
//

#include <QVBoxLayout>
#include "ui/dialog/node_type_dialog.h"
#include "model/code_info_model.h"
#include "enum/CodeType.h"

NodeTypeDialog::NodeTypeDialog(QWidget *parent) : QDialog(parent) {
    this->setWindowTitle("노드 타입 선택");

    resize(200, 100);

    _comboBox_ptr = new QComboBox;
    std::vector<Code> codes = CodeInfoModel::getInstance().getCodesByCategory(CodeType::NODE_TYPE);
    for (const Code &code: codes) {
        _comboBox_ptr->addItem(
                QString::fromStdString(code.name),
                QVariant::fromValue(QString::fromStdString(code.code))
        );
    }

    _okButton_ptr = new QPushButton(tr("OK"));
    connect(_okButton_ptr, &QPushButton::clicked, this, &NodeTypeDialog::accept);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(_comboBox_ptr);
    layout->addWidget(_okButton_ptr);
    setLayout(layout);
}

NodeTypeDialog::~NodeTypeDialog() {

}

std::string NodeTypeDialog::selectedOption() const {
    // 현재 선택된 항목의 사용자 데이터를 가져옵니다.
    QVariant data = _comboBox_ptr->currentData();
    if (data.isValid()) {
        // 사용자 데이터를 std::string으로 변환하여 반환합니다.
        return data.toString().toStdString();
    } else {
        // 유효하지 않은 경우, 빈 문자열 반환
        return "";
    }
}
