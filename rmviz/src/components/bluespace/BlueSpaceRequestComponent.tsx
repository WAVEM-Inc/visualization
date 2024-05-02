import "./BlueSpaceRequestComponent.css";

interface BlueSpaceRequestComponentProps {
    onStraightClick: () => void;
    onStraightTestClick: () => void;
    onCallClick: () => void;
    onDeliveryClick: () => void;
    onWaitingClick: () => void;
}

const BlueSpaceRequestComponent: React.FC<BlueSpaceRequestComponentProps> = ({
    onStraightClick,
    onStraightTestClick,
    onCallClick,
    onDeliveryClick,
    onWaitingClick
}: BlueSpaceRequestComponentProps) => {
    return (
        <div className="">
            <div className={"bluespace_request_btn_container"}>
                <button className={"bluespace_btn_request"} onClick={onStraightClick}>직진</button>
                <button className={"bluespace_btn_request"} onClick={onStraightTestClick}>직진 테스트</button>
                <button className={"bluespace_btn_request"} onClick={onCallClick}>호출</button>
                <button className={"bluespace_btn_request"} onClick={onDeliveryClick}>배송</button>
                <button className={"bluespace_btn_request"} onClick={onWaitingClick}>대기</button>
            </div>
        </div>
    );
};

export default BlueSpaceRequestComponent;