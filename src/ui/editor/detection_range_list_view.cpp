//
// Created by antique on 24. 3. 11.
//

#include <QMessageBox>
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
    if (_views_ptr.size() < 3) {
        auto *view = new DetectionRangeView(this, _views_ptr.size() + 1);
        view->setDetectionRange(range);

        _layout_ptr->addWidget(view);
        _views_ptr.push_back(view);

        connect(view, &DetectionRangeView::deleteButtonClicked, this, [this, view]() {
            // view를 _views_ptr에서 찾아 인덱스를 얻음
            auto it = std::find(_views_ptr.begin(), _views_ptr.end(), view);
            if (it != _views_ptr.end()) {
                int index = std::distance(_views_ptr.begin(), it);

                // 해당 DetectionRangeView 삭제
                _layout_ptr->removeWidget(view);
                _views_ptr.erase(it);
                delete view;

                // 나머지 view들의 인덱스 업데이트
                updateIndexes();

                // 선택적: 변경사항 UI에 반영
                this->update();
            }
        });
    } else {
        QMessageBox::information(nullptr, "감지 범위 개수 제한", "감지 범위는 최대 3개까지 추가가 가능 합니다.");
    }
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

void DetectionRangeListView::updateIndexes() {
    int newIndex = 1;
    for (auto *view : _views_ptr) {
        // 각 view의 새로운 인덱스 설정
        view->updateIndex(newIndex++);
    }
}

DetectionRangeListView::~DetectionRangeListView() = default;
