import "./BlueSpaceRequestComponent.css";

interface BlueSpaceRequestComponentProps {
    onStraightClick: () => void;
    onStraightHorizonTestClick: () => void;
    onStraightVerticalTestClick: () => void;
    onRotationTestClick: () => void;
    onCallClick: () => void;
    onDeliveryClick: () => void;
    onWaitingClick: () => void;
    onMissionClick: () => void;
    onControlMoveToDestClick: () => void;
    onControlMsCompleteClick: () => void;
    onControlGraphSyncClick: () => void;
}

const BlueSpaceRequestComponent: React.FC<BlueSpaceRequestComponentProps> = ({
    onStraightClick,
    onStraightHorizonTestClick,
    onStraightVerticalTestClick,
    onRotationTestClick,
    onCallClick,
    onDeliveryClick,
    onWaitingClick,
    onMissionClick,
    onControlMoveToDestClick,
    onControlMsCompleteClick,
    onControlGraphSyncClick
}: BlueSpaceRequestComponentProps) => {
    return (
        <div className="">
            <div className={"bluespace_request_btn_container"}>
                <button className={"bluespace_btn_request"} onClick={onStraightClick}>직진</button>
                <button className={"bluespace_btn_request"} onClick={onStraightHorizonTestClick}>직진(가로) 테스트</button>
                <button className={"bluespace_btn_request"} onClick={onStraightVerticalTestClick}>직진(세로) 테스트</button>
                <button className={"bluespace_btn_request"} onClick={onRotationTestClick}>회전 테스트</button>
                <button className={"bluespace_btn_request"} onClick={onCallClick}>호출</button>
                <button className={"bluespace_btn_request"} onClick={onDeliveryClick}>배송</button>
                <button className={"bluespace_btn_request"} onClick={onWaitingClick}>대기</button>
                <button className={"bluespace_btn_request"} onClick={onMissionClick}>임무 할당</button>
                <button className={"bluespace_btn_request"} onClick={onControlMoveToDestClick}>상차지 이동 제어</button>
                <button className={"bluespace_btn_request"} onClick={onControlMsCompleteClick}>복귀 이동 제어</button>
                <button className={"bluespace_btn_request"} onClick={onControlGraphSyncClick}>그래프 제어</button>
            </div>
        </div>
    );
};

export default BlueSpaceRequestComponent;