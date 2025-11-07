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
  

  const path = route.path;
  const coordinates = route.coordinates;
  const steps = route.steps;

  const pathColor = { color: 'var(--primary-color)', weight: 6 };

  let firstStep = "Start your route";
  let secondStep = "";
  if (steps && steps.length > 0 && steps[0].instruction) {
    firstStep = steps[0].instruction;
  }
  if (steps && steps.length > 1 && steps[1].instruction) {
    secondStep = steps[1].instruction;
  } else if (path && path.length > 0) {
    secondStep = "Arrive at " + path[path.length - 1];
  }

  let mapCenter = [29.6483, -82.3494];
  let startMarker = null;
  let endMarker = null;

  if (coordinates && coordinates.length > 0) {
    mapCenter = coordinates[0];
    startMarker = coordinates[0];
    endMarker = coordinates[coordinates.length - 1];
  }

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