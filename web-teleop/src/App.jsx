import React from "react";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { ROSProvider } from "./context/ROSContext";
import TopNav from "./components/TopNav";
import TeleopPage from "./pages/TeleopPage";
import HealthPage from "./pages/HealthPage";
import GPSMissionPlannerPage from "./pages/GPSMissionPlannerPage";

export default function App() {
  return (
    <BrowserRouter>
      <ROSProvider>
        <div className="app-container">
          <TopNav />
          <Routes>
            <Route path="/" element={<TeleopPage />} />
            <Route path="/health" element={<HealthPage />} />
            <Route path="/gps-mission" element={<GPSMissionPlannerPage />} />
          </Routes>
        </div>
      </ROSProvider>
    </BrowserRouter>
  );
}
