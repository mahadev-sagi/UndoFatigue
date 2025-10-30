import React from 'react';
import { mockApiResponse } from '../mockdata'; 

function SearchScreen({ onFindRoutes }) {
  
  const handleFindRouteClick = () => {
    console.log("Simulating API call...");
    onFindRoutes(mockApiResponse);
  };

  return (
    <main className="main-page">
      <h1 className="main-title">UndoFatigue</h1>
      <div className="input-container">
        <input placeholder="Your Location" />
        <input placeholder="Where to?" />
        <button onClick={handleFindRouteClick}>Find Route</button>
      </div>
    </main>
  );
}

export default SearchScreen;