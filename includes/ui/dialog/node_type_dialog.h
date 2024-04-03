//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_NODE_TYPE_DIALOG_H
#define NODE_EDITOR_NODE_TYPE_DIALOG_H


#include <QDialog>
#include <QComboBox>
#include <QPushButton>

class NodeTypeDialog : public QDialog {
Q_OBJECT

public:
    explicit NodeTypeDialog(QWidget *parent = nullptr);

    virtual ~NodeTypeDialog();

    std::string selectedOption() const;

private:
    QComboBox *_comboBox_ptr;
    QPushButton *_okButton_ptr;
};


#endif //NODE_EDITOR_NODE_TYPE_DIALOG_H
