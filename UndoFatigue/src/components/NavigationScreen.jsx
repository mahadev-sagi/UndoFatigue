import React from 'react';
import { MapContainer, TileLayer, Polyline, Marker, Popup } from 'react-leaflet';




function BackButton({ onClick }) {
  return (
    <button className="back-button" onClick={onClick}>
       ←
    </button>
  );
}

function NavigationScreen({ route, onEndTrip, onBack }) {
  if (!route) {
    return (
      <div className="map-screen-container">
        No route selected. <button onClick={onEndTrip}>Go Back</button>
      </div>
    );
  }
  
  const { path, coordinates, eta, distance, steps } = route;
  const pathColor = { color: 'var(--primary-color)', weight: 6 };

  const firstStep = steps?.[0]?.instruction || "Start your route";
  const secondStep = steps?.[1]?.instruction || `Arrive at ${path[path.length - 1]}`;
  const mapCenter = coordinates?.[0] || [29.6483, -82.3494];
  const startMarker = coordinates?.[0];
  const endMarker = coordinates?.[coordinates.length - 1];

  return (
    <div className="map-screen-container">
      
      <MapContainer
        center={mapCenter}
        zoom={17}
        className="map-background"
        scrollWheelZoom={true}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <Polyline pathOptions={pathColor} positions={coordinates} />
        {startMarker && (
          <Marker position={startMarker}>
            <Popup>Start: {path[0]}</Popup>
          </Marker>
        )}
        {endMarker && (
          <Marker position={endMarker}>
            <Popup>End: {path[path.length - 1]}</Popup>
          </Marker>
        )}
      </MapContainer>
       <BackButton onClick={onBack} />

     
      
      <div className="nav-steps-card">
        <h3>Directions</h3>
        <ul className="steps-list">
          {steps && steps.map((step, index) => (
            <li key={index} className="step-item">
              <span className="step-instruction">{step.instruction}</span>
            </li>
          ))}
        </ul>
        <div className="nav-footer">
          <button onClick={onEndTrip}>End Trip</button>
        </div>
      </div>
    </div>
  );
}

export default NavigationScreen;