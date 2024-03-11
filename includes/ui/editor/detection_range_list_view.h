//
// Created by antique on 24. 3. 11.
//

#ifndef NODE_EDITOR_DETECTION_RANGE_LIST_VIEW_H
#define NODE_EDITOR_DETECTION_RANGE_LIST_VIEW_H


#include <QWidget>
#include <QVBoxLayout>
#include "detection_range_view.h"

class DetectionRangeListView : public QWidget {
Q_OBJECT
public:
    explicit DetectionRangeListView(QWidget *parent = nullptr);

    ~DetectionRangeListView() override;

    void setDetectionRanges(const std::vector<DetectionRange> &ranges);

    void addDetectionRange(const DetectionRange &range);

    void clear();

private:
    QList<DetectionRangeView *> _views_ptr;

    QVBoxLayout *_layout_ptr;
};


#endif //NODE_EDITOR_DETECTION_RANGE_LIST_VIEW_H
