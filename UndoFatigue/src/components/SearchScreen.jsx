import React, { useState } from 'react';
import { MapContainer, TileLayer } from 'react-leaflet';
//import { getMockRoutes } from '../mockdata'; 
import {getRoutes} from '../api';

const ufPosition = [29.6483, -82.3494];

function SearchScreen({ onFindRoutes }) {
  const [startLocation, setStartLocation] = useState('');
  const [endLocation, setEndLocation] = useState('');
  const [error, setError] = useState(null); 
  const [isLoading, setIsLoading] = useState(false);

  const handleFindRouteClick = async () => {
    setError(null);
    setIsLoading(true);

    const start = startLocation.trim();
    const end = endLocation.trim();

    if (start === '' && end === '') {
      setError('Your Location and Destination are required');
      setIsLoading(false);
      return; 
    } else if (start === '') {
      setError('Your location is required');
      setIsLoading(false);
      return; 
    } else if (end === '') {
      setError('Destination is required');
      setIsLoading(false);
      return; 
    }
    
    try{
      const routeData = await getRoutes(start,end); 
      onFindRoutes(routeData, start, end);
    } catch (err) {
      setError(err.message || 'Failed to find routes. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleStartChange = (e) => {
    setStartLocation(e.target.value);
    if (error) setError(null);
  }
  
  const handleEndChange = (e) => {
    setEndLocation(e.target.value);
    if (error) setError(null);
  }

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
      </MapContainer>
      
      <div className="search-ui-overlay">
        <h1 className="main-title">UndoFatigue</h1>
        <div className="search-card">
          <div className="input-container">
            <input
              placeholder="Your Location"
              value={startLocation}
              onChange={handleStartChange} 
              className={error ? 'input-error' : ''}
              disabled={isLoading}
            />
            <input
              placeholder="Where to?"
              value={endLocation}
              onChange={handleEndChange} 
              className={error ? 'input-error' : ''}
              disabled={isLoading}
            />
            
            {error && <div className="error-message">{error}</div>}
            
            <button onClick={handleFindRouteClick} disabled={isLoading}>
              {isLoading ? 'Finding Routes...' : 'Find Routes'}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

export default SearchScreen;