import React from "react";
import { Link, useLocation } from "react-router-dom";

export default function TopNav() {
  const loc = useLocation();
  const active = (path) => ({
    padding: "8px 12px",
    borderRadius: 10,
    textDecoration: "none",
    color: "inherit",
    background: loc.pathname === path ? "rgba(0,0,0,0.08)" : "transparent",
    border: "1px solid rgba(0,0,0,0.12)",
  });

  return (
    <div style={{ display: "flex", gap: 10, padding: 12, borderBottom: "1px solid rgba(0,0,0,0.12)" }}>
      <Link to="/" style={active("/")}>Teleop</Link>
      <Link to="/health" style={active("/health")}>Health</Link>
      <Link to="/gps-mission" style={active("/gps-mission")}>GPS Mission</Link>
    </div>
  );
}
