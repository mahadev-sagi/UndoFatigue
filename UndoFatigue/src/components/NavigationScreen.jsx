import React from 'react';

// Your component function
function NavigationScreen({ route, onEndTrip }) {
  return (
    <div className="screen">
      <h1>Navigating...</h1>
      <p>Follow the path:</p>

      <h2>{route ? route.join(' -> ') : 'No route selected'}</h2>

    
      <canvas id="navigation-map"></canvas>

      <button onClick={onEndTrip}>End Trip</button>
    </div>
  );
}

export default NavigationScreen;