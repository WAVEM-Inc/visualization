//
// Created by antique on 24. 3. 11.
//

#include "ui/editor/detection_range_list_view.h"

DetectionRangeListView::DetectionRangeListView(QWidget *parent) {
    _layout_ptr = new QVBoxLayout();
    setLayout(_layout_ptr);
}

void DetectionRangeListView::setDetectionRanges(const std::vector<DetectionRange> &ranges) {
    clear();

    for (const DetectionRange &range : ranges) {
        addDetectionRange(range);
    }
}

void DetectionRangeListView::addDetectionRange(const DetectionRange &range) {
    DetectionRangeView *view = new DetectionRangeView();
    view->setDetectionRange(range);

    _layout_ptr->addWidget(view);
    _views_ptr.push_back(view);
}

void DetectionRangeListView::clear() {
    if (_layout_ptr != NULL) {
        QLayoutItem *item;
        while ((item = _layout_ptr->takeAt(0)) != NULL) {
            delete item->widget();
            delete item;
        }
    }
    _views_ptr.clear();
}

DetectionRangeListView::~DetectionRangeListView() = default;
