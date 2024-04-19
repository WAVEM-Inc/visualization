import "./BlueSpaceRequestComponent.css";


interface BlueSpaceRequestComponentProps {
    onStraightClick: () => void;
    onIntersection1Click: () => void;
    onIntersection2Click: () => void;
    onLoadingClick: () => void;
    onLanding1Click: () => void;
    onLanding2Click: () => void;
    onGPSShadow1Click: () => void;
    onGPSShadow2Click: () => void;
    onEmergencyStopClick: () => void;
    onEmergencyResumeClick: () => void;
}

const BlueSpaceRequestComponent: React.FC<BlueSpaceRequestComponentProps> = ({
    onStraightClick,
    onIntersection1Click,
    onIntersection2Click,
    onLoadingClick,
    onLanding1Click,
    onLanding2Click,
    onGPSShadow1Click,
    onGPSShadow2Click,
    onEmergencyStopClick,
    onEmergencyResumeClick
}: BlueSpaceRequestComponentProps) => {
    return (
        <div className={"bluespace_request_btn_container"}>
            <button className={"bluespace_btn_request"} onClick={onStraightClick}>직진 이탈</button>
            <button className={"bluespace_btn_request"} onClick={onIntersection1Click}>교차로 - 1</button>
            <button className={"bluespace_btn_request"} onClick={onIntersection2Click}>교차로 - 2</button>
            <button className={"bluespace_btn_request"} onClick={onLoadingClick}>상차 U턴</button>
            <button className={"bluespace_btn_request"} onClick={onLanding1Click}>하차 - 1</button>
            <button className={"bluespace_btn_request"} onClick={onLanding2Click}>하차 - 2</button>
            <button className={"bluespace_btn_request"} onClick={onGPSShadow1Click}>GPS 음영 지역 - 1</button>
            <button className={"bluespace_btn_request"} onClick={onGPSShadow2Click}>GPS 음영 지역 - 2</button>
            <button className={"bluespace_btn_request bluespace_btn_emergency_resume"} onClick={onEmergencyStopClick}>비상 정지</button>
            <button className={"bluespace_btn_request bluespace_btn_emergency_stop"} onClick={onEmergencyResumeClick}>재개</button>
        </div>
    );
};

export default BlueSpaceRequestComponent;