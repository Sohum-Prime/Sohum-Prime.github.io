import React from 'react';
import Navbar from './Navbar';
import Home from './Home';
import Portfolio from './Portfolio';
import Contact from './Contact';
import './App.css'; // Your CSS file

function App() {
  return (
    <div className="App">
      <Navbar />
      <Home />
      <Portfolio />
      <Contact />
    </div>
  );
}

export default App;