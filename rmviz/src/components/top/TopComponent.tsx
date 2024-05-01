import React, { useState } from 'react';
import "./TopComponent.css";
import TopDropDownComponent from './TopDropdownComponent';
import { Link } from 'react-router-dom';


const TopComponents: React.FC = () => {
    const [isDropdownView, setDropdownView] = useState(false);
    const [menuIconRotation, setMenuIconRotation] = useState(0);

    const handleClickContainer = () => {
        setDropdownView(!isDropdownView);
        setMenuIconRotation((prevRotation) => prevRotation === 0 ? 90 : 0);
    }

    return (
        <div className="top_components">
            <label className="top_dropdown_toggle_label" onClick={handleClickContainer} style={{ transform: `rotate(${menuIconRotation}deg)`}}>
                ☰
            </label>
            {isDropdownView && <TopDropDownComponent />}
            <div className="top_logo">
                <p className="top_logo_title">
                    <Link className="top_logo_title_link" to={"/bluespace"}>
                        RMViZ
                    </Link>
                </p>
            </div>
            <div className="top_heartbeat">
                
            </div>
        </div>
    )
}

export default TopComponents;
