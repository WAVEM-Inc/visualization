import { useEffect, useState } from 'react';
import "./TopComponent.css";
import "./TopComponent.scss";


const TopComponents = () => {
    const [isMobile, setIsMobile] = useState(window.innerWidth <= 768);
    const [menuOpen, setMenuOpen] = useState(false);
    const [menuIconRotation, setMenuIconRotation] = useState(0);

    useEffect(() => {
        const handleResize = () => {
            setIsMobile(window.innerWidth <= 768);
        };

        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, []);

    const handleMenuToggle = () => {
        setMenuOpen((prevMenuOpen) => !prevMenuOpen);
        setMenuIconRotation((prevRotation) => prevRotation === 0 ? 90 : 0);
    };

    return (
        <header className="header">
            <nav className={`desktop-nav ${menuOpen ? 'menu-open' : ''}`}>
                <div className="logo">RMViZ</div>
                <div className="hamburger-menu" onClick={handleMenuToggle} style={{ transform: `rotate(${menuIconRotation}deg)` }}>
                    ☰
                </div>
                {menuOpen && (
                    <div className="mobile-menu">
                        <ul>
                            <li>다크모드</li>
                            <li>About</li>
                            <li>Contact</li>
                        </ul>
                    </div>
                )}
            </nav>
        </header>
    );
}

export default TopComponents;
