import "./BlueSpaceRequestComponent.css";

interface BlueSpaceRequestComponentProps {
    onStraightClick: () => void;
    onStraightHorizonTestClick?: () => void;
    onStraightVerticalTestClick?: () => void;
    onRotationTestClick?: () => void;
    onCallClick: () => void;
    onDeliveryClick: () => void;
    onWaitingClick: () => void;
    onMissionClick: () => void;
    onControlMoveToDestClick: () => void;
    onControlMsCompleteReturnClick: () => void;
    onControlMsCompleteNoReturnClick: () => void;
    onControlGraphSyncClick?: () => void;
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
    onControlMsCompleteReturnClick,
    onControlMsCompleteNoReturnClick,
    onControlGraphSyncClick
}: BlueSpaceRequestComponentProps) => {
    return (
        <div className="">
            <div className={"bluespace_request_btn_container"}>
                <button className={"bluespace_btn_request"} onClick={onStraightClick}>직진</button>
                <button className={"bluespace_btn_request"} onClick={onCallClick}>호출</button>
                <button className={"bluespace_btn_request"} onClick={onDeliveryClick}>배송</button>
                <button className={"bluespace_btn_request"} onClick={onWaitingClick}>대기</button>
                <button className={"bluespace_btn_request"} onClick={onMissionClick}>임무 할당</button>
                <button className={"bluespace_btn_request"} onClick={onControlMoveToDestClick}>상차지 이동 제어</button>
                <button className={"bluespace_btn_request"} onClick={onControlMsCompleteReturnClick}>대기 장소 복귀 제어</button>
                <button className={"bluespace_btn_request"} onClick={onControlMsCompleteNoReturnClick}>대기 장소 미복귀 제어</button>
            </div>
        </div>
    );
};

export default BlueSpaceRequestComponent;