
import React, { useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet';
import { mockApiResponse } from '../mockdata';

function SearchScreen({ onFindRoutes }) {
  const [startLocation, setStartLocation] = useState('');
  const [endLocation, setEndLocation] = useState('');

  const handleFindRouteClick = () => {
    console.log(`Simulating API call for route from ${startLocation} to ${endLocation}`);
    onFindRoutes(mockApiResponse);
  };
  
  const handleNearbyClick = (location) => {
    setEndLocation(location);
  };
  
 
  const ufPosition = [29.6483, -82.3494];

  return (
    <div className="search-screen-wrapper">
      
     
      <MapContainer
        center={ufPosition}
        zoom={15}
        className="map-background" 
        scrollWheelZoom={true} 
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        
        <Marker position={[29.6489, -82.3413]}>
          <Popup>Marston Science Library</Popup>
        </Marker>
        <Marker position={[29.6480, -82.3458]}>
          <Popup>The Hub</Popup>
        </Marker>
      </MapContainer>

      <div className="search-ui-overlay">
        <h1 className="main-title">UndoFatigue</h1>

        <div className="search-card">
          <div className="input-container">
            <input
              placeholder="Your Location"
              value={startLocation}
              onChange={(e) => setStartLocation(e.target.value)}
            />
            <input
              placeholder="Where to?"
              value={endLocation}
              onChange={(e) => setEndLocation(e.target.value)}
            />
            <button onClick={handleFindRouteClick}>Find Routes</button>
          </div>
        </div>
        <div className="locations-nearby-card">
          <h3>Locations near me</h3>
          <ul>
            <li onClick={() => handleNearbyClick('The Hub')}>The Hub</li>
            <li onClick={() => handleNearbyClick('Marston Science Library')}>Marston Science Library</li>
            <li onClick={() => handleNearbyClick('Reitz Union')}>Reitz Union</li>
          </ul>
        </div>
      </div>
    </div>
  );
}

export default SearchScreen;