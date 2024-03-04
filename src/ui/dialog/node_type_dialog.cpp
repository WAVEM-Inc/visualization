//
// Created by antique on 24. 2. 29.
//

#include <QVBoxLayout>
#include "ui/dialog/node_type_dialog.h"
#include "enum/NodeType.h"

NodeTypeDialog::NodeTypeDialog(QWidget *parent) : QDialog(parent) {
    this->setWindowTitle("노드 타입 선택");

    resize(200, 100);

    _comboBox_ptr =  new QComboBox;
    _comboBox_ptr->addItem(QString(getTypeKorName(NodeType::PATH_NODE).c_str()));
    _comboBox_ptr->addItem(QString(getTypeKorName(NodeType::WORK_NODE).c_str()));
    _comboBox_ptr->addItem(QString(getTypeKorName(NodeType::WORKPLACE_NODE).c_str()));

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
    return _comboBox_ptr->currentText().toStdString();
}
