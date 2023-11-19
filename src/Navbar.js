import React from 'react';
import { Link } from 'react-scroll';

const Navbar = () => {
    return (
        <nav>
            <ul>
                <li><Link to="home" smooth={true} duration={1000}>Home</Link></li>
                <li><Link to="portfolio" smooth={true} duration={1000}>Portfolio</Link></li>
                <li><Link to="contact" smooth={true} duration={1000}>Contact</Link></li>
            </ul>
        </nav>
    );
};

export default Navbar;