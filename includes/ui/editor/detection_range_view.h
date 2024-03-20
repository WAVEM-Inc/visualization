//
// Created by antique on 24. 3. 11.
//

#ifndef NODE_EDITOR_DETECTION_RANGE_VIEW_H
#define NODE_EDITOR_DETECTION_RANGE_VIEW_H


#include <QWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include "struct/DetectionRange.h"

class DetectionRangeView : public QWidget {
Q_OBJECT
public:
    explicit DetectionRangeView(QWidget *parent = nullptr, int num = 0);

    ~DetectionRangeView() override;

    void setDetectionRange(DetectionRange detectionRange);

    DetectionRange getDetectionRange();

private:
    QLineEdit *_offset_ptr;
    QLineEdit *_width_ptr;
    QLineEdit *_height_ptr;
    QComboBox *_actionCode_ptr;
    QPushButton *_mapPosBtn_ptr;
    QPushButton *_vehiclePosBtn_ptr;
};


#endif //NODE_EDITOR_DETECTION_RANGE_VIEW_H
