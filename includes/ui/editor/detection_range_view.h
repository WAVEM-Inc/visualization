//
// Created by antique on 24. 3. 11.
//

#ifndef NODE_EDITOR_DETECTION_RANGE_VIEW_H
#define NODE_EDITOR_DETECTION_RANGE_VIEW_H


#include <QWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include "struct/DetectionRange.h"

class DetectionRangeView : public QWidget {
Q_OBJECT
public:
    explicit DetectionRangeView(QWidget *parent = nullptr, int num = 0);

    ~DetectionRangeView() override;

    void setDetectionRange(DetectionRange detectionRange);

    DetectionRange getDetectionRange();

    void updateIndex(int num);

signals:
    void deleteButtonClicked(int index);

private:
    int _index;
    QLabel *_numLb_ptr;
    QLineEdit *_offset_ptr;
    QLineEdit *_widthLeft_ptr;
    QLineEdit *_widthRight_ptr;
    QLineEdit *_height_ptr;
    QComboBox *_actionCode_ptr;
    QPushButton *_deleteBtn_ptr;
};


#endif //NODE_EDITOR_DETECTION_RANGE_VIEW_H
