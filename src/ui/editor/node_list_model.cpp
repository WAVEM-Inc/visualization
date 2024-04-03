//
// Created by antique on 24. 2. 27.
//

#include "ui/editor/node_list_model.h"
#include "model/node_info_model.h"
#include <QMimeData>

NodeListModel::NodeListModel(QObject *parent) : QStandardItemModel(parent) {

}

QMimeData *NodeListModel::mimeData(const QModelIndexList &indexes) const {
    QMimeData *mimeData = QStandardItemModel::mimeData(indexes);
    if (indexes.count() > 0) {
        // 첫 번째 선택된 항목의 행 번호를 인코딩하여 저장합니다.
        int sourceRow = indexes.first().row();
        QByteArray encodedData = QByteArray::number(sourceRow);
        mimeData->setData("application/x-row", encodedData);
    }
    return mimeData;
}

bool NodeListModel::dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column,
                                 const QModelIndex &parent) {
    Q_UNUSED(action);
    Q_UNUSED(column);

    int maxRow = rowCount(QModelIndex()) - 1;
    int beginRow;

    if (row != -1) {
        beginRow = row;
    } else if (parent.isValid()) {
        beginRow = parent.row();
    } else {
        beginRow = maxRow;
    }

    // 드랍 지점이 모델의 행 수를 초과하지 않도록 조정
    if (beginRow > maxRow) {
        beginRow = maxRow;
    }

    // MIME 데이터에서 드래그 시작 지점의 행 번호를 추출합니다.
    QByteArray encodedData = data->data("application/x-row");
    int sourceRow = encodedData.toInt();

    // 드래그 시작 지점과 수정된 드롭 지점의 행 번호를 출력합니다.
    std::cout << "Dragged from: " << sourceRow << " to: " << beginRow << std::endl;
    NodeInfoModel::getInstance().changeIndexes(sourceRow, beginRow);

    // 실제 데이터 변경이나 항목 이동 없이 성공적으로 처리되었음을 나타내기 위해 true를 반환합니다.
    return true;
}

NodeListModel::~NodeListModel() = default;
