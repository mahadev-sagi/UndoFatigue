# Undo Fatigue
* **Team Members:** Pratyush Kulkarni, Mahadev Sagi
## Overview

Undo Fatigue is a web-based navigation tool designed to help students navigate the University of Florida campus more efficiently. The core problem this project solves is student fatigue from walking long, non-optimal routes that may include uphill terrain or construction zones.

This application provides two distinct route options between any two locations on campus:
1.  **The Shortest Route:** Calculated using **Dijkstra's Algorithm** to find the path with the minimum total distance.
2.  **The Easiest Route:** Calculated using the **A\* Search Algorithm**, which factors in heuristics to find a more optimal path, balancing distance and (in future iterations) factors like elevation.

The application features an interactive map interface where users can input their start and destination, view both route options overlaid on the map, and then select one to receive step-by-step navigation instructions.


## 🛠️ Technology Stack

* **Backend:**
    * **Language:** C++17
    * **Server:** `httplib.h` (A C++ single-header HTTP server library)
    * **JSON:** `nlohmann/json.hpp` (A C++ JSON library)
    * **Algorithms:** Dijkstra's, A* Search
* **Frontend:**
    * **Framework:** React.js
    * **Build Tool:** Vite
    * **Mapping:** React-Leaflet, Leaflet.js
    * **Styling:** CSS

---

## 🚀 Getting Started & Setup

This project is split into two parts that must be run concurrently: the C++ backend server and the React frontend application. The header libraries (`httplib.h`, `json.hpp`) are included in this repository for easy setup.

### 1. Backend Setup (C++ Server)

The backend is a C++ executable that loads the UF campus graph data and serves route calculations via a local HTTP server.

1.  **Navigate to the Backend Directory**
    * This directory should contain `Nearest_Path_Finder.cpp`, your data files (`uf_edges.csv`, `uf_nodes.csv`, `uf_places.csv`), and the `include/` folder containing the header libraries.

2.  **Verify Include Paths**
    * Make sure `Nearest_Path_Finder.cpp` is using the correct paths for the included libraries (e.g., `#include "include/httplib.h"`).

3.  **Important: Update File Paths (If Needed)**
    * Your `Nearest_Path_Finder.cpp` file has relative paths to the CSV files. By default, it looks for them in a `../backend/` directory.
    * You should update these paths in the `main()` function to load the CSVs from the *same* directory as the executable.

    **Change these lines:**
    ```cpp
    // From:
    my_graph.process_csv_files_and_add_them_to_graph(my_graph, "../backend/uf_edges.csv", "../backend/uf_nodes.csv");
    my_graph.search_nodes_for_coordinates("../backend/uf_nodes.csv");
    my_graph.search_places_for_places("../backend/uf_nodes.csv", "../backend/uf_places.csv");

    // To:
    my_graph.process_csv_files_and_add_them_to_graph(my_graph, "uf_edges.csv", "uf_nodes.csv");
    my_graph.search_nodes_for_coordinates("uf_nodes.csv");
    my_graph.search_places_for_places("uf_nodes.csv", "uf_places.csv");
    ```

4.  **Compile the Server**
    * Open a terminal in the backend directory.
    * **On Windows (with g++):**
        ```bash
        g++ -std=c++17 Nearest_Path_Finder.cpp -pthread -lws2_32 -o server.exe
        ```

5.  **Run the Server**
    * Once compiled, run the executable:
    * **On Windows:**
        ```bash
        ./server.exe
        ```

    * The server will start and print `Server starting on http://localhost:8080`.

### 2. Frontend Setup (React App)

The frontend is a React application built with Vite. It communicates with the C++ backend.

1.  **Navigate to the Frontend Directory**
    * This directory should contain your `.jsx` files, `api.js`, and `package.json`.

2.  **Install Dependencies**
    * Install the necessary npm packages:

    ```bash
    npm install
    ```

3.  **Run the Application**
    * Start the Vite development server:

    ```bash
    npm run dev
    ```

4.  **View the App**
    * Your terminal will show a message, typically `Local: http://localhost:5173/`.
    * Open this URL in your web browser.
    * As long as the C++ backend is also running, you can now use the application to find routes.

---

