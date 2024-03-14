//
// Created by antique on 24. 3. 11.
//

#include "ui/editor/detection_range_list_view.h"

DetectionRangeListView::DetectionRangeListView(QWidget *parent) : QWidget(parent) {
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
    auto *view = new DetectionRangeView(this, _views_ptr.size() + 1);
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

std::vector<DetectionRange> DetectionRangeListView::getDetectionRanges() {
    std::vector<DetectionRange> ranges;
    for (DetectionRangeView *view: _views_ptr)
        ranges.push_back(view->getDetectionRange());

    return ranges;
}

DetectionRangeListView::~DetectionRangeListView() = default;
