//
// Created by antique on 24. 3. 11.
//

#include <QLabel>
#include <QGridLayout>
#include "ui/editor/detection_range_view.h"
#include "model/code_info_model.h"
#include "enum/CodeType.h"

DetectionRangeView::DetectionRangeView(QWidget *parent, int num) : QWidget(parent) {
    _index = num;

    QGridLayout *layout = new QGridLayout(this);

    _numLb_ptr = new QLabel(QString::number(_index) + QString("번 감지범위"));
    _deleteBtn_ptr = new QPushButton("X");
    _deleteBtn_ptr->setFixedSize(30, 30);
    layout->addWidget(_numLb_ptr, 0, 0);
    layout->addWidget(_deleteBtn_ptr, 0, 1, Qt::AlignRight);

    QLabel *widthLabel = new QLabel("감지 폭");
    _width_ptr = new QLineEdit();
    _width_ptr->setValidator(new QIntValidator(_width_ptr));
    layout->addWidget(widthLabel, 1, 0);
    layout->addWidget(_width_ptr, 1, 1);

    QLabel *heightLabel = new QLabel("감지 길이");
    _height_ptr = new QLineEdit();
    _height_ptr->setValidator(new QIntValidator(_height_ptr));
    layout->addWidget(heightLabel, 2, 0);
    layout->addWidget(_height_ptr, 2, 1);

    QLabel *actionLabel = new QLabel("감지 후 처리 코드");
    _actionCode_ptr = new QComboBox();
    layout->addWidget(actionLabel, 3, 0);
    layout->addWidget(_actionCode_ptr, 3, 1);

    QLabel *latLabel = new QLabel("영역 거리");
    _offset_ptr = new QLineEdit();
    _offset_ptr->setValidator(new QDoubleValidator(_offset_ptr));
    layout->addWidget(latLabel, 4, 0);
    layout->addWidget(_offset_ptr, 4, 1);

    std::vector<Code> actionCodes = CodeInfoModel::getInstance().getCodesByCategory(CodeType::ACTION_CODE);
    for (const Code &code: actionCodes) {
        _actionCode_ptr->addItem(
                QString::fromStdString(code.name),
                QVariant::fromValue(QString::fromStdString(code.code))
        );
    }

    connect(_deleteBtn_ptr, &QPushButton::clicked, [this]() {
        emit deleteButtonClicked(_index);
    });
}

DetectionRangeView::~DetectionRangeView() = default;


void DetectionRangeView::setDetectionRange(DetectionRange detectionRange) {
    _offset_ptr->setText(QString::number(detectionRange.offset, 'f', 2));
    _width_ptr->setText(QString::number(detectionRange.width));
    _height_ptr->setText(QString::number(detectionRange.height));
    _actionCode_ptr->setCurrentText(CodeInfoModel::getInstance().getNameByCode(CodeType::ACTION_CODE, detectionRange.actionCode).c_str());
}

DetectionRange DetectionRangeView::getDetectionRange() {
    DetectionRange range;
    range.offset = _offset_ptr->text().toDouble();
    range.width = _width_ptr->text().toDouble();
    range.height = _height_ptr->text().toDouble();
    range.actionCode = _actionCode_ptr->currentData().toString().toStdString();

    return range;
}

void DetectionRangeView::updateIndex(int num) {
    _index = num;
    _numLb_ptr->setText(QString::number(_index) + QString("번 감지범위"));
}
