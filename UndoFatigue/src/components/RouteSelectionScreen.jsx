// src/components/RouteSelectionScreen.js
import React from 'react';

function RouteSelectionScreen({ routeData, onSelectRoute }) {
  if (!routeData) return <div>Loading...</div>;

  const { dijkstraResult, aStarResult } = routeData;

  return (
    <div className="screen">
      <h1>Select a Route</h1>     
      <div 
        className="route-card" 
        onClick={() => onSelectRoute(dijkstraResult.path)}
      >
        <h2>Shortest Route (Dijkstra's)</h2>
        <p>Distance: {dijkstraResult.distance} km</p>
        <p>Path: {dijkstraResult.path.join(' -> ')}</p>
      </div>
      <div 
        className="route-card" 
        onClick={() => onSelectRoute(aStarResult.path)}
      >
        <h2>Easiest Route (A*)</h2>
        <p>Distance: {aStarResult.distance} km</p>
        <p>Path: {aStarResult.path.join(' -> ')}</p>
      </div>
    </div>
  );
}

export default RouteSelectionScreen;