//
// Created by antique on 24. 3. 11.
//

#include <QLabel>
#include <QGridLayout>
#include "ui/editor/detection_range_view.h"

DetectionRangeView::DetectionRangeView(QWidget *parent, int num) :
        QWidget(parent) {
    QGridLayout *layout = new QGridLayout(this);

    QLabel *numLabel = new QLabel(QString::number(num));
    layout->addWidget(numLabel, 0, 0);

    QLabel *lngLabel = new QLabel("경도");
    _longitude_ptr = new QLineEdit();
    layout->addWidget(lngLabel, 1, 0);
    layout->addWidget(_longitude_ptr, 1, 1);

    QLabel *latLabel = new QLabel("위도");
    _latitude_ptr = new QLineEdit();
    layout->addWidget(latLabel, 2, 0);
    layout->addWidget(_latitude_ptr, 2, 1);

    QLabel *widthLabel = new QLabel("감지 폭");
    _width_ptr = new QLineEdit();
    layout->addWidget(widthLabel, 3, 0);
    layout->addWidget(_width_ptr, 3, 1);

    QLabel *heightLabel = new QLabel("감지 길이");
    _height_ptr = new QLineEdit();
    layout->addWidget(heightLabel, 4, 0);
    layout->addWidget(_height_ptr, 4, 1);

    QLabel *actionLabel = new QLabel("감지 후 처리 코드");
    _actionCode_ptr = new QComboBox();
    layout->addWidget(actionLabel, 5, 0);
    layout->addWidget(_actionCode_ptr, 5, 1);
}

DetectionRangeView::~DetectionRangeView() = default;


void DetectionRangeView::setDetectionRange(DetectionRange detectionRange) {
    _latitude_ptr->setText(QString::number(detectionRange.position.latitude));
    _longitude_ptr->setText(QString::number(detectionRange.position.longitude));
    _width_ptr->setText(QString::number(detectionRange.width));
    _height_ptr->setText(QString::number(detectionRange.height));
    _actionCode_ptr->setCurrentText(detectionRange.processingCode.c_str());
}