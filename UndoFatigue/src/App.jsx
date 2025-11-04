import { useState } from 'react';
import 'leaflet/dist/leaflet.css';
import './App.css';
import SearchScreen from './components/SearchScreen';
import RouteSelectionScreen from './components/RouteSelectionScreen';
import NavigationScreen from './components/NavigationScreen';

function App() {
  const [routeData, setRouteData] = useState(null);
  const [selectedRoute, setSelectedRoute] = useState(null);
  const [startLocation, setStartLocation] = useState('');
  const [endLocation, setEndLocation] = useState('');

  const handleFindRoutes = (data, start, end) => {
    setRouteData(data);
    setStartLocation(start);
    setEndLocation(end);
    setSelectedRoute(null); 
  };

  const handleSelectRoute = (route) => {
    setSelectedRoute(route);
  };

  const handleEndTrip = () => {
    setRouteData(null);
    setSelectedRoute(null);
  };
  
  const handleBackToSearch = () => {
    setRouteData(null);
  };
  
  const handleBackToRouteSelect = () => {
    setSelectedRoute(null);
  };

  if (selectedRoute) {
    
    return (
      <NavigationScreen 
        route={selectedRoute} 
        onEndTrip={handleEndTrip} 
        onBack={handleBackToRouteSelect}
      />
    );
  }

  if (routeData) {
    
    return (
      <RouteSelectionScreen
        routeData={routeData}
        onSelectRoute={handleSelectRoute}
        startLocation={startLocation}
        endLocation={endLocation}
        onBack={handleBackToSearch}
      />
    );
  }

  
  return <SearchScreen onFindRoutes={handleFindRoutes} />;
}

export default App;