import React, { useState } from 'react';
import "./TopComponent.css";
import TopDropDownComponent from './TopDropdownComponent';


const TopComponents: React.FC = () => {
    const [isDropdownView, setDropdownView] = useState(false);
    const [menuIconRotation, setMenuIconRotation] = useState(0);

    const handleClickContainer = () => {
        setDropdownView(!isDropdownView);
        setMenuIconRotation((prevRotation) => prevRotation === 0 ? 90 : 0);
    }

    return (
        <div className="top_components">
            <div className="top_logo">
                <p className="top_logo_title">
                    RMViZ
                </p>
            </div>
            <label className="top_dropdown_toggle_label" onClick={handleClickContainer} style={{ transform: `rotate(${menuIconRotation}deg)`}}>
                ☰
            </label>
            {isDropdownView && <TopDropDownComponent />}
        </div>
    )
}

export default TopComponents;
