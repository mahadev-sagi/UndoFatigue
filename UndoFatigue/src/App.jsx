import { useState } from 'react'
import './App.css'
import SearchScreen from './components/SearchScreen'
import RouteSelectionScreen from './components/RouteSelectionScreen'
import NavigationScreen from './components/NavigationScreen'
import 'leaflet/dist/leaflet.css';

function App() {
  const [routeData, setRouteData] = useState(null);
  const [selectedRoute, setSelectedRoute] = useState(null);

  const handleFindRoutes = (data) => {
    setRouteData(data);
  };

  const handleSelectRoute = (route) => {
    setSelectedRoute(route);
  };

  const handleEndTrip = () => {
    setRouteData(null);
    setSelectedRoute(null);
  };

  if (selectedRoute) {
    return <NavigationScreen route={selectedRoute} onEndTrip={handleEndTrip} />;
  }

  if (routeData) {
    return <RouteSelectionScreen routeData={routeData} onSelectRoute={handleSelectRoute} />;
  }

  return (
    <SearchScreen onFindRoutes={handleFindRoutes} />
  )
}

export default App
