import "./KECRequestComponent.css";

interface KECRequestComponentProps {
    onCall01Click: () => void;
    onDelivery02NCall02Click: () => void;
    onWaiting03Click: () => void;
    onCall04Click: () => void;
    onDelivery05NCall05Click: () => void;
    onWaiting06Click: () => void;
    onTemp01Click: () => void;
    onTemp02Click: () => void;
    onTemp03Click: () => void;
    onDeliveringMissionClick: () => void;
    onReturningMissionClick: () => void;
    onControlMoveToDestClick: () => void;
    onControlMsCompleteReturnClick: () => void;
    onControlMsCompleteNoReturnClick: () => void;
}

const KECRequestComponent: React.FC<KECRequestComponentProps> = ({
    onCall01Click,
    onDelivery02NCall02Click,
    onWaiting03Click,
    onCall04Click,
    onDelivery05NCall05Click,
    onWaiting06Click,
    onTemp01Click,
    onTemp02Click,
    onTemp03Click,
    onDeliveringMissionClick,
    onReturningMissionClick,
    onControlMoveToDestClick,
    onControlMsCompleteReturnClick,
    onControlMsCompleteNoReturnClick
}: KECRequestComponentProps) => {
    return (
        <div className="">
            <div className={"kec_request_btn_container"}>
                <button className={"kec_btn_request"} onClick={onCall01Click}>호출 01{"(대기 → A)"}</button>
                <button className={"kec_btn_request"} onClick={onDelivery02NCall02Click}>배송02, 호출 02{"(A → B)"}</button>
                <button className={"kec_btn_request"} onClick={onWaiting03Click}>대기 03{"(B → 대기R)"}</button>
                <button className={"kec_btn_request"} onClick={onCall04Click}>호출 04{"(대기R → B)"}</button>
                <button className={"kec_btn_request"} onClick={onDelivery05NCall05Click}>배송 05, 호출 05{"(B → A)"}</button>
                <button className={"kec_btn_request"} onClick={onWaiting06Click}>대기 06{"(A → 대기R)"}</button>
                <button className={"kec_btn_request"} onClick={onTemp01Click}>임시 경로 - 1</button>
                <button className={"kec_btn_request"} onClick={onTemp02Click}>임시 경로 - 2</button>
                <button className={"kec_btn_request"} onClick={onTemp03Click}>임시 경로 - 3</button>
                <button className={"kec_btn_request"} onClick={onDeliveringMissionClick}>배송 임무 할당</button>
                <button className={"kec_btn_request"} onClick={onReturningMissionClick}>복귀 임무 할당</button>
                <button className={"kec_btn_request"} onClick={onControlMoveToDestClick}>하차지 이동 제어</button>
                <button className={"kec_btn_request"} onClick={onControlMsCompleteReturnClick}>대기 장소 복귀 제어</button>
                <button className={"kec_btn_request"} onClick={onControlMsCompleteNoReturnClick}>대기 장소 미복귀 제어</button>
            </div>
        </div>
    );
};

export default KECRequestComponent;
