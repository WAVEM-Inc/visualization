import { Link } from "react-router-dom";
import "./EntryPage.css";

export default function EntryPage(): React.ReactElement<any, any> | null {

    return (
        <div className="dash_board_container">
            <Link to={"/bluespace/dashboard"}>
                용인 블루스페이스
            </Link>
            <Link to={"/kec"}>
                구미 KEC
            </Link>
            <Link to={"/atc"}>
                부산 원광밸브
            </Link>
        </div>
    );
}