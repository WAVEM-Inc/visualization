import React from "react";
import { Link } from "react-router-dom";
import "./TopDropdownComponent.css";

const TopDropDownComponent: React.FC = () => {
    return (
        <ul>
            <li className="top_dropdown_menu">
                <Link className="top_dropdown_" to={"/data"}>
                    데이터 보드
                </Link>
            </li>
        </ul>
    )
}

export default TopDropDownComponent;