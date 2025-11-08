
// this was used for testing before real api integration. code not needed. 


const calculateETA = (distanceMeters) => {
  const speedMetersPerSecond = 1.4;
  const seconds = distanceMeters / speedMetersPerSecond;
  return Math.ceil(seconds / 60);
};

const mockApiDatabase = {
  // test route
  "Rawlings Hall_Hume Hall": {
    
    dijkstraResult: {
      distance: 820,
      path: ["Rawlings Hall", "Museum Road", "Hume Hall"],
      coordinates: [
        [29.64631, -82.34341], // Rawlings
        [29.6455, -82.3495],
        [29.64481, -82.35201]  // Hume
      ],
      eta: calculateETA(820),
      steps: [
        { instruction: "Start at Rawlings Hall", distance: "50m" },
        { instruction: "Head west on Museum Road", distance: "720m" },
        { instruction: "Arrive at Hume Hall", distance: "50m" }
      ]
    },
  
    aStarResult: {
      distance: 1050, 
      path: ["Rawlings Hall", "Turlington Hall", "Library West", "Hume Hall"],
      coordinates: [
        [29.64631, -82.34341], // Rawlings
        [29.64811, -82.34350],
        [29.6498, -82.3490],
        [29.64481, -82.35201]  // Hume
      ],
      eta: calculateETA(1050),
      steps: [
        { instruction: "Start at Rawlings Hall", distance: "100m" },
        { instruction: "Head north to Turlington Plaza", distance: "300m" },
        { instruction: "Turn left, walk past Library West", distance: "450m" },
        { instruction: "Continue south to Hume Hall", distance: "200m" },
        { instruction: "You have arrived at Hume Hall", distance: "" }
      ]
    }
  }
  
  
};

export const getMockRoutes = (startName, endName) => {
  const key = `${startName.trim()}_${endName.trim()}`;
  
  return mockApiDatabase[key] || null; 
};