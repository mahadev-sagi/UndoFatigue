import React from 'react';
import { MapContainer, TileLayer, Polyline, Marker } from 'react-leaflet';




function BackButton({ onClick }) {
  return (
    <button className="back-button" onClick={onClick}>
      ←
    </button>
  );
}


function RouteSelectionScreen({ routeData, onSelectRoute, startLocation, endLocation, onBack }) {
  if (!routeData) {
    return <div className="map-screen-container"></div>;
  }


  const { dijkstraResult, aStarResult } = routeData;

  const formatPath = (path) => path.join(' → ');
  
  const easiestRouteColor = { color: 'var(--primary-color)', weight: 6 };
  const shortestRouteColor = { color: 'var(--accent-color)', weight: 4};
  let mapCenter = [29.6483, -82.3494];
  let startMarker = null;
  let endMarker = null; 

  if(aStarResult && aStarResult.coordinates && aStarResult.coordinates.length > 0){
    mapCenter = aStarResult.coordinates[0];
    startMarker = aStarResult.coordinates[0];
    endMarker = aStarResult.coordinates[aStarResult.coordinates.length -1];
  }
  if(dijkstraResult && dijkstraResult.coordinates && dijkstraResult.coordinates.length > 0){
    mapCenter = dijkstraResult.coordinates[0];
    startMarker = dijkstraResult.coordinates[0];
    endMarker = dijkstraResult.coordinates[dijkstraResult.coordinates.length -1];
  }


  return (
    <div className="map-screen-container">
      <BackButton onClick={onBack} />
      
      <MapContainer
        center={mapCenter}
        zoom={16}
        className="map-background"
        scrollWheelZoom={true}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        
        
        {dijkstraResult && dijkstraResult.coordinates && (
          <Polyline pathOptions={shortestRouteColor} positions={dijkstraResult.coordinates} />
        )}
        {aStarResult && aStarResult.coordinates && (
          <Polyline pathOptions={easiestRouteColor} positions={aStarResult.coordinates} />
        )}
        
        {startMarker && <Marker position={startMarker} />}
        {endMarker && <Marker position={endMarker} />}
      </MapContainer>
     
     
    <div className="route-summary-bubble">
      <span className="label">From:</span>
      <input value={startLocation || 'Your Location'} readOnly />
      <span className="label">To:</span>
      <input value={endLocation || 'Hume Hall'} readOnly />
    </div>


      
      <div className="route-options-card">
        <h2 className="screen-title" style={{textAlign: 'center', marginTop: 0, marginBottom: '16px'}}>Select a Route</h2>
        
       
        <div
          className="route-card recommended"
        >
          <div className="route-card-header">
            <h2>Easiest Route</h2>
            <span className="badge">Recommended</span>
          </div>
          <p className="route-path">{formatPath(aStarResult.path)}</p>
          <div className="route-stats">
          </div>
          <button
            className="go-button"
            onClick={(e) => {
              e.stopPropagation(); 
              onSelectRoute(aStarResult);
            }}
          >
            GO
          </button>
        </div>
        
       
        <div
          className="route-card"
          
        >
          <div className="route-card-header">
            <h2>Shortest Route</h2>
            <span className="badge badge-secondary">Shortest</span>
          </div>
          <p className="route-path">{formatPath(dijkstraResult.path)}</p>
          <div className="route-stats">
          </div>
          <button
            className="go-button"
            onClick={(e) => {
              e.stopPropagation(); 
              onSelectRoute(dijkstraResult);
            }}
          >
            GO
          </button>
        </div>
      </div>
    </div>
  );
}

export default RouteSelectionScreen;