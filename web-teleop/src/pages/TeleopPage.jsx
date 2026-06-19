import React, { useEffect, useMemo, useRef, useState } from "react";
import nipplejs from "nipplejs";
import { useROS } from "../context/ROSContext";
import * as ROSLIB from "roslib";

const clamp = (v, min, max) => Math.min(max, Math.max(min, v));

const BG = "#04090f";
const SURFACE = "#07111d";
const SRF2 = "#0b1929";
const BORDER = "#0f2236";
const BORDER2 = "#162d46";
const TEXT = "#c8dde8";
const TEXT2 = "#4a7a96";
const TEXT3 = "#1e3a52";
const ACCENT = "#0ea5e9";
const MONO = "'JetBrains Mono','Fira Code',monospace";

const btnStyle = (color, bg, border) => ({
  padding: "0.42rem 0.75rem", background: bg, border,
  borderRadius: 5, color, cursor: "pointer",
  fontWeight: 700, fontSize: "0.65rem", fontFamily: MONO,
  transition: "all 0.15s", whiteSpace: "nowrap",
});
const inpStyle = {
  width: "100%", padding: "0.42rem 0.55rem",
  background: "#03070e", border: `1px solid ${BORDER}`,
  borderRadius: 4, color: TEXT, fontSize: "0.68rem",
  outline: "none", fontFamily: MONO, boxSizing: "border-box",
};
const lblStyle = {
  fontSize: "0.53rem", color: TEXT3, letterSpacing: "0.1em",
  marginBottom: "0.3rem", textTransform: "uppercase",
};

const commitFloat = (str, fallback, onValid, onInvalid) => {
  const p = parseFloat(str);
  (!isNaN(p) && isFinite(p) && p >= 0) ? onValid(p) : onInvalid(String(fallback));
};

const ageLabel = (ts, key) => {
  if (!ts) return { text: "Veri yok", color: TEXT3 };
  const ageSec = (Date.now() - ts) / 1000;
  const stale = key === "battery" ? 300 : 10;
  const warn = key === "battery" ? 120 : 5;
  const color = ageSec > stale ? "#ef4444" : ageSec > warn ? "#f59e0b" : "#10b981";
  return { text: `${ageSec.toFixed(1)}s ago`, color };
};

export default function TeleopPage() {
  const {
    ros, isConnected, status: globalStatus, errorText: globalErrorText, reconnect,
    // ── Cihazlar arası senkron + sayfa değişiminde kaybolmaz ──
    estop, setEstop,
    speedMode, setSpeedMode,
    linearMax, setLinearMax,
    angularMax, setAngularMax,
  } = useROS();

  const [telemetry, setTelemetry] = useState({
    battery: { enabled: true, topic: "/battery_state", messageType: "sensor_msgs/BatteryState", valuePath: "percentage", auxPath: "voltage", scale: 1, unit: "%", auxUnit: "V", json: false },
    temp: { enabled: true, topic: "/motor_state", messageType: "std_msgs/Float32MultiArray", valuePath: "data.0", scale: 1, unit: "°C", json: false },
    fan: { enabled: false, topic: "/fan_rpm", messageType: "std_msgs/Int32", valuePath: "data", scale: 1, unit: "rpm", json: false },
  });

  // Input buffer state'leri context değerinden başlatılır
  const [linearMaxStr, setLinearMaxStr] = useState(() => String(linearMax));
  const [angularMaxStr, setAngularMaxStr] = useState(() => String(angularMax));
  // Context değeri başka cihazdan veya localStorage'dan değişirse input'u güncelle
  useEffect(() => { setLinearMaxStr(String(linearMax)); }, [linearMax]);
  useEffect(() => { setAngularMaxStr(String(angularMax)); }, [angularMax]);

  const [scaleStr, setScaleStr] = useState({ battery: "1", temp: "1", fan: "1" });
  const [telErr, setTelErr] = useState("");
  const [telVals, setTelVals] = useState({
    battery: { value: null, aux: null, ts: 0 },
    temp: { value: null, aux: null, ts: 0 },
    fan: { value: null, aux: null, ts: 0 },
  });

  const telSubsRef = useRef({ battery: null, temp: null, fan: null });

  // Cihaza özgü config (publish hedefleri) — bunları senkronlamıyoruz
  const [topicName, setTopicName] = useState("/cmd_vel_nav");
  const [emergencyTopic, setEmergencyTopic] = useState("/emergency/active");
  const [emergencyMsgType, setEmergencyMsgType] = useState("std_msgs/Bool");
  const [showSettings, setShowSettings] = useState(false);
  const [controlMode, setControlMode] = useState("joystick");
  const [tick, setTick] = useState(0);

  // estop / speedMode / linearMax / angularMax → ROSContext (cihazlar arası senkron)
  const [speedBusy, setSpeedBusy] = useState(false);

  const cmdVelRef = useRef(null);
  // emergencyRef / emergencySubRef artık ROSContext'te yönetiliyor
  const joystickZoneRef = useRef(null);
  const joystickRef = useRef(null);
  const axesRef = useRef({ x: 0, y: 0 });
  const timerRef = useRef(null);
  const lastSentRef = useRef({ lin: 0, ang: 0, zeroCount: 0 });
  const twist0 = useMemo(() => ({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }), []);

  useEffect(() => {
    const id = setInterval(() => setTick(t => t + 1), 1000);
    return () => clearInterval(id);
  }, []);

  const publishTwist = (linX, angZ) => {
    cmdVelRef.current?.publish({ ...twist0, linear: { x: linX, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angZ } });
  };

  // Speed mode → context setSpeedMode çağırıyor. Context hem state'i günceller
  // hem /adaptive_velocity topic'ine latched publish eder → tüm cihazlar senkron.
  const publishSpeedMode = (mode) => {
    if (!isConnected) return;
    setSpeedBusy(true);
    setSpeedMode(mode);
    setTimeout(() => setSpeedBusy(false), 300);
  };

  const safeStop = () => {
    axesRef.current = { x: 0, y: 0 };
    publishTwist(0, 0);
    setTimeout(() => publishTwist(0, 0), 80);
    setTimeout(() => publishTwist(0, 0), 160);
  };

  // cmd_vel publisher — topic adı değiştiğinde yeniden kur
  useEffect(() => {
    if (!ros || !isConnected) { cmdVelRef.current = null; return; }
    cmdVelRef.current = new ROSLIB.Topic({ ros, name: topicName, messageType: "geometry_msgs/Twist", queue_length: 1 });
    return () => { cmdVelRef.current = null; };
  }, [ros, isConnected, topicName]);

  // ── estop dışarıdan (başka cihaz, STM veya context sub) aktifleşirse:
  //    güvenlik için hemen üç kez sıfır bas ──
  const estopPrevRef = useRef(false);
  useEffect(() => {
    if (estop && !estopPrevRef.current) {
      publishTwist(0, 0);
      setTimeout(() => publishTwist(0, 0), 80);
      setTimeout(() => publishTwist(0, 0), 160);
    }
    estopPrevRef.current = estop;
    // publishTwist bir closure — cmdVelRef güncel olduğu sürece sorun yok
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [estop]);

  useEffect(() => {
    const zone = joystickZoneRef.current;
    if (!zone || controlMode !== "joystick") return;
    const create = () => {
      joystickRef.current?.destroy(); joystickRef.current = null;
      const rect = zone.getBoundingClientRect();
      const size = Math.min(Math.max(1, rect.width), Math.max(1, rect.height)) * 0.65;
      const mgr = nipplejs.create({ zone, mode: "static", position: { left: "50%", top: "50%" }, color: ACCENT, size, restOpacity: 0.7, dynamicPage: true });
      joystickRef.current = mgr;
      mgr.on("move", (_, d) => { axesRef.current = { x: -clamp(d.vector.x, -1, 1), y: clamp(d.vector.y, -1, 1) }; });
      mgr.on("end", () => { axesRef.current = { x: 0, y: 0 }; publishTwist(0, 0); });
    };
    const raf = requestAnimationFrame(create);
    const onResize = () => requestAnimationFrame(create);
    window.addEventListener("resize", onResize);
    return () => { cancelAnimationFrame(raf); window.removeEventListener("resize", onResize); joystickRef.current?.destroy(); joystickRef.current = null; };
  }, [controlMode]);

  const getByPath = (obj, path) => {
    if (!obj || !path) return undefined;
    return String(path).split(".").map(s => s.trim()).filter(Boolean).reduce((c, p) => c == null ? undefined : c[p], obj);
  };
  const parseMaybeJson = (msg, cfg) => {
    if (!cfg?.json) return msg;
    const raw = msg?.data;
    if (typeof raw !== "string") return msg;
    try { return JSON.parse(raw); } catch { return msg; }
  };
  const cleanupTel = () => {
    const s = telSubsRef.current;
    ["battery", "temp", "fan"].forEach(k => { try { s[k]?.unsubscribe(); } catch { } s[k] = null; });
  };

  useEffect(() => {
    if (!isConnected || !ros) { cleanupTel(); return; }
    setTelErr(""); cleanupTel();
    const subs = telSubsRef.current;
    const makeSub = key => {
      const cfg = telemetry[key];
      if (!cfg?.enabled) return;
      try {
        const t = new ROSLIB.Topic({ ros, name: cfg.topic, messageType: cfg.messageType });
        t.subscribe(msg => {
          const m = parseMaybeJson(msg, cfg);
          const rv = getByPath(m, cfg.valuePath);
          const ra = cfg.auxPath ? getByPath(m, cfg.auxPath) : undefined;
          const val = typeof rv === "number" ? rv : rv != null ? Number(rv) : null;
          const aux = typeof ra === "number" ? ra : ra != null ? Number(ra) : null;
          setTelVals(p => ({
            ...p, [key]: {
              value: val == null || !isFinite(val) ? null : val * (cfg.scale ?? 1),
              aux: aux == null || !isFinite(aux) ? null : aux,
              ts: Date.now(),
            }
          }));
        });
        subs[key] = t;
      } catch (e) { setTelErr(p => p || `${key}: ${e?.message || e}`); }
    };
    ["battery", "temp", "fan"].forEach(makeSub);
    return cleanupTel;
  }, [isConnected, ros, telemetry]);

  // ── estop aktifken sürekli sıfır bas, nav2'yi bastır ──
  useEffect(() => {
    if (timerRef.current) clearInterval(timerRef.current);
    timerRef.current = setInterval(() => {
      if (!isConnected) return;

      if (estop) {
        publishTwist(0, 0);
        return;
      }

      const { x, y } = axesRef.current;
      const DZ = 0.05;
      const cx = Math.abs(x) < DZ ? 0 : x, cy = Math.abs(y) < DZ ? 0 : y;
      const lin = clamp(cy * linearMax, -linearMax, linearMax);
      const ang = clamp(cx * angularMax, -angularMax, angularMax);
      const isZ = lin === 0 && ang === 0;
      const last = lastSentRef.current;
      if (isZ) { if (last.zeroCount < 3) { publishTwist(0, 0); last.zeroCount++; } last.lin = 0; last.ang = 0; return; }
      last.zeroCount = 0; last.lin = lin; last.ang = ang;
      publishTwist(lin, ang);
    }, 50);
    return () => { clearInterval(timerRef.current); timerRef.current = null; };
  }, [isConnected, estop, linearMax, angularMax]);

  const telCards = [
    { k: "battery", label: "BATTERY", fmt: v => v == null ? "—" : `${v.toFixed(0)}`, barColor: v => v == null ? TEXT3 : v > 50 ? "#10b981" : v > 20 ? "#f59e0b" : "#ef4444" },
    { k: "temp", label: "TEMP", fmt: v => v == null ? "—" : `${v.toFixed(1)}`, barColor: v => v == null ? TEXT3 : v < 60 ? "#10b981" : v < 80 ? "#f59e0b" : "#ef4444" },
    { k: "fan", label: "FAN RPM", fmt: v => v == null ? "—" : `${v.toFixed(0)}`, barColor: () => ACCENT },
  ];

  const dirBtns = [
    { label: "↖", x: -0.7, y: 1, diag: true },
    { label: "↑", x: 0, y: 1 },
    { label: "↗", x: 0.7, y: 1, diag: true },
    { label: "←", x: -1, y: 0 },
    { stop: true },
    { label: "→", x: 1, y: 0 },
    { label: "↙", x: -0.7, y: -1, diag: true },
    { label: "↓", x: 0, y: -1 },
    { label: "↘", x: 0.7, y: -1, diag: true },
  ];

  return (
    <div style={{
      minHeight: "calc(100vh - 56px)", width: "100vw",
      background: BG,
      backgroundImage: "radial-gradient(rgba(14,165,233,0.06) 1px, transparent 1px)",
      backgroundSize: "24px 24px",
      color: TEXT, padding: "0.65rem",
      fontFamily: MONO, overflow: "hidden", boxSizing: "border-box",
    }}>
      <div style={{ maxWidth: 1400, margin: "0 auto", height: "calc(100vh - 80px)", display: "flex", flexDirection: "column", gap: "0.5rem" }}>

        {/* ── HEADER ── */}
        <div style={{ flexShrink: 0, display: "flex", alignItems: "center", justifyContent: "space-between" }}>
          <div style={{ display: "flex", alignItems: "center", gap: "0.65rem" }}>
            <div style={{
              width: 32, height: 32, background: "rgba(14,165,233,0.1)",
              border: `1px solid rgba(14,165,233,0.35)`, borderRadius: 6,
              display: "flex", alignItems: "center", justifyContent: "center",
              fontSize: "1rem", boxShadow: "0 0 12px rgba(14,165,233,0.15)",
            }}>◆</div>
            <div>
              <div style={{ fontSize: "0.82rem", fontWeight: 800, letterSpacing: "0.14em", color: ACCENT }}>SIMSOFT ATOH</div>
              <div style={{ fontSize: "0.55rem", color: TEXT2, letterSpacing: "0.1em", marginTop: 1 }}>TELEOP CONTROL · cmd_vel_nav</div>
            </div>
          </div>
          <button
            onClick={() => setShowSettings(v => !v)}
            style={btnStyle(showSettings ? ACCENT : TEXT2, showSettings ? "rgba(14,165,233,0.1)" : "transparent", `1px solid ${showSettings ? "rgba(14,165,233,0.4)" : BORDER2}`)}>
            ⚙ {showSettings ? "Gizle" : "Ayarlar"}
          </button>
        </div>

        {/* ── STATUS BAR ── */}
        <div style={{
          flexShrink: 0, background: SURFACE, borderRadius: 5,
          padding: "0.45rem 0.85rem",
          border: `1px solid ${isConnected ? "rgba(16,185,129,0.2)" : "rgba(239,68,68,0.2)"}`,
          display: "flex", alignItems: "center", gap: "0.6rem", flexWrap: "wrap",
        }}>
          <div style={{
            width: 7, height: 7, borderRadius: "50%", flexShrink: 0,
            background: isConnected ? "#10b981" : "#ef4444",
            boxShadow: `0 0 8px ${isConnected ? "#10b981" : "#ef4444"}`,
          }} />
          <span style={{ fontSize: "0.68rem", color: TEXT2, flex: 1 }}>{globalStatus || "—"}</span>
          {globalErrorText && (
            <span style={{ fontSize: "0.62rem", color: "#f87171", background: "rgba(239,68,68,0.08)", padding: "0.15rem 0.4rem", borderRadius: 3 }}>
              ⚠ {globalErrorText}
            </span>
          )}
          {!isConnected
            ? <button onClick={reconnect} style={btnStyle(ACCENT, "rgba(14,165,233,0.12)", `1px solid ${ACCENT}`)}>↻ Bağlan</button>
            : <span style={{ fontSize: "0.62rem", color: "#10b981", fontWeight: 600 }}>✓ CONNECTED</span>
          }
        </div>

        {/* ── TELEMETRY BAR ── */}
        <div style={{ display: "grid", gridTemplateColumns: "repeat(3,1fr)", gap: "0.5rem", flexShrink: 0 }}>
          {telCards.map(({ k, label, fmt, barColor }) => {
            const cfg = telemetry[k];
            const { value, aux, ts } = telVals[k];
            void tick;
            const age = ageLabel(ts, k);
            const col = barColor(value);
            return (
              <div key={k} style={{ background: SURFACE, borderRadius: 5, padding: "0.65rem 0.75rem", border: `1px solid ${BORDER2}`, opacity: cfg.enabled ? 1 : 0.4 }}>
                <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "0.2rem" }}>
                  <div style={lblStyle}>{label}</div>
                  <div style={{ fontSize: "0.53rem", color: TEXT3 }}>{cfg.topic}</div>
                </div>
                <div style={{ display: "flex", alignItems: "baseline", gap: "0.25rem" }}>
                  <span style={{ fontFamily: MONO, fontSize: "1.2rem", fontWeight: 900, color: col, lineHeight: 1 }}>
                    {fmt(value)}
                  </span>
                  {value != null && (
                    <span style={{ fontSize: "0.6rem", color: TEXT2 }}>{cfg.unit}</span>
                  )}
                </div>
                {k === "battery" && aux != null && (
                  <div style={{ fontSize: "0.6rem", color: TEXT2, marginTop: "0.15rem" }}>{aux.toFixed(2)} {cfg.auxUnit}</div>
                )}
                {k === "battery" && value != null && (
                  <div style={{ marginTop: "0.35rem", height: 3, background: BORDER, borderRadius: 2, overflow: "hidden" }}>
                    <div style={{ height: "100%", width: `${Math.min(100, Math.max(0, value))}%`, background: col, borderRadius: 2, transition: "width 0.5s ease" }} />
                  </div>
                )}
                <div style={{ fontSize: "0.53rem", color: age.color, marginTop: "0.28rem" }}>
                  {telErr ? "⚠ ERR" : !cfg.enabled ? "DISABLED" : age.text}
                </div>
              </div>
            );
          })}
        </div>

        {/* ── SPEED MODE (adaptive_velocity) ── */}
        <div style={{
          flexShrink: 0, background: SURFACE, borderRadius: 5,
          border: `1px solid ${BORDER2}`,
          padding: "0.5rem 0.75rem",
          display: "flex", alignItems: "center", gap: "0.6rem", flexWrap: "wrap",
        }}>
          <div style={{ display: "flex", flexDirection: "column", gap: "0.15rem" }}>
            <div style={lblStyle}>HIZ MODU</div>
            <div style={{ fontSize: "0.55rem", color: TEXT3 }}>/adaptive_velocity · std_msgs/Int32</div>
          </div>
          <div style={{ display: "flex", gap: "0.35rem", marginLeft: "auto", flexWrap: "wrap" }}>
            {[
              { id: 1, label: "1 · HASSAS", color: "#10b981" },
              { id: 2, label: "2 · NORMAL", color: "#0ea5e9" },
              { id: 3, label: "3 · HIZLI", color: "#f59e0b" },
              { id: 4, label: "4 · MAX", color: "#ef4444" },
            ].map(({ id, label, color }) => {
              const active = speedMode === id;
              const disabled = !isConnected || speedBusy;
              return (
                <button
                  key={id}
                  onClick={() => publishSpeedMode(id)}
                  disabled={disabled}
                  title={!isConnected ? "ROS bağlı değil" : `Modu ${id} olarak yayınla`}
                  style={{
                    ...btnStyle(
                      active ? color : TEXT2,
                      active ? `${color}22` : "transparent",
                      `1px solid ${active ? color : BORDER2}`
                    ),
                    minWidth: 98,
                    opacity: disabled ? 0.55 : 1,
                    cursor: disabled ? "not-allowed" : "pointer",
                    boxShadow: active ? `0 0 10px ${color}33` : "none",
                  }}
                >
                  {label}
                </button>
              );
            })}
          </div>
        </div>

        {/* ── SETTINGS ── */}
        {showSettings && (
          <div style={{ flexShrink: 0, background: SURFACE, borderRadius: 6, padding: "0.85rem", border: `1px solid ${BORDER2}`, maxHeight: "52vh", overflowY: "auto" }}>
            <div style={{ fontSize: "0.68rem", fontWeight: 700, color: TEXT, letterSpacing: "0.1em", marginBottom: "0.65rem" }}>CONFIGURATION</div>

            <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(155px,1fr))", gap: "0.55rem", marginBottom: "0.75rem" }}>
              {[
                { label: "CMD_VEL TOPIC", value: topicName, set: setTopicName },
                { label: "EMERGENCY TOPIC", value: emergencyTopic, set: setEmergencyTopic },
                { label: "EMERGENCY MSG TYPE", value: emergencyMsgType, set: setEmergencyMsgType },
              ].map(({ label, value, set }) => (
                <div key={label}>
                  <div style={lblStyle}>{label}</div>
                  <input type="text" value={value} onChange={e => set(e.target.value)} style={inpStyle} />
                </div>
              ))}
              <div>
                <div style={lblStyle}>LINEAR MAX (m/s)</div>
                <input type="text" inputMode="decimal" value={linearMaxStr}
                  onChange={e => setLinearMaxStr(e.target.value)}
                  onBlur={() => commitFloat(linearMaxStr, linearMax, v => { setLinearMax(v); setLinearMaxStr(String(v)); }, setLinearMaxStr)}
                  style={inpStyle} />
              </div>
              <div>
                <div style={lblStyle}>ANGULAR MAX (rad/s)</div>
                <input type="text" inputMode="decimal" value={angularMaxStr}
                  onChange={e => setAngularMaxStr(e.target.value)}
                  onBlur={() => commitFloat(angularMaxStr, angularMax, v => { setAngularMax(v); setAngularMaxStr(String(v)); }, setAngularMaxStr)}
                  style={inpStyle} />
              </div>
            </div>

            {/* Telemetry sources */}
            <div style={{ fontSize: "0.6rem", fontWeight: 700, color: TEXT2, letterSpacing: "0.08em", marginBottom: "0.4rem" }}>TELEMETRY SOURCES</div>
            <div style={{ display: "flex", flexDirection: "column", gap: "0.4rem", marginBottom: "0.75rem" }}>
              {["battery", "temp", "fan"].map(k => {
                const cfg = telemetry[k];
                const names = { battery: "BATTERY", temp: "TEMPERATURE", fan: "FAN" };
                return (
                  <div key={k} style={{ background: SRF2, border: `1px solid ${BORDER2}`, borderRadius: 5, padding: "0.55rem 0.65rem" }}>
                    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "0.4rem" }}>
                      <span style={{ fontSize: "0.65rem", fontWeight: 700, color: TEXT }}>{names[k]}</span>
                      <label style={{ fontSize: "0.6rem", color: TEXT2, display: "flex", gap: "0.35rem", alignItems: "center", cursor: "pointer" }}>
                        <input type="checkbox" checked={!!cfg.enabled} onChange={e => setTelemetry(p => ({ ...p, [k]: { ...p[k], enabled: e.target.checked } }))} style={{ accentColor: ACCENT }} />
                        ACTIVE
                      </label>
                    </div>
                    <div style={{ display: "grid", gridTemplateColumns: "2fr 2fr 1fr 1fr", gap: "0.35rem" }}>
                      {[
                        { lbl: "TOPIC", val: cfg.topic, set: v => setTelemetry(p => ({ ...p, [k]: { ...p[k], topic: v } })) },
                        { lbl: "MSG TYPE", val: cfg.messageType, set: v => setTelemetry(p => ({ ...p, [k]: { ...p[k], messageType: v } })) },
                        { lbl: "VALUE PATH", val: cfg.valuePath, set: v => setTelemetry(p => ({ ...p, [k]: { ...p[k], valuePath: v } })) },
                      ].map(({ lbl, val, set }) => (
                        <div key={lbl}>
                          <div style={lblStyle}>{lbl}</div>
                          <input value={val} onChange={e => set(e.target.value)} style={{ ...inpStyle, fontSize: "0.63rem", padding: "0.35rem 0.45rem" }} />
                        </div>
                      ))}
                      <div>
                        <div style={lblStyle}>SCALE</div>
                        <input type="text" inputMode="decimal" value={scaleStr[k]}
                          onChange={e => setScaleStr(p => ({ ...p, [k]: e.target.value }))}
                          onBlur={() => {
                            const p = parseFloat(scaleStr[k]);
                            if (!isNaN(p) && isFinite(p)) { setTelemetry(pr => ({ ...pr, [k]: { ...pr[k], scale: p } })); setScaleStr(pr => ({ ...pr, [k]: String(p) })); }
                            else setScaleStr(pr => ({ ...pr, [k]: String(cfg.scale) }));
                          }}
                          style={{ ...inpStyle, fontSize: "0.63rem", padding: "0.35rem 0.45rem" }} />
                      </div>
                    </div>
                    {k === "battery" && (
                      <div style={{ marginTop: "0.4rem", display: "grid", gridTemplateColumns: "2fr 1fr 1fr", gap: "0.35rem" }}>
                        {[
                          { lbl: "AUX PATH", val: cfg.auxPath || "", set: v => setTelemetry(p => ({ ...p, battery: { ...p.battery, auxPath: v } })) },
                          { lbl: "UNIT", val: cfg.unit || "", set: v => setTelemetry(p => ({ ...p, battery: { ...p.battery, unit: v } })) },
                          { lbl: "AUX UNIT", val: cfg.auxUnit || "", set: v => setTelemetry(p => ({ ...p, battery: { ...p.battery, auxUnit: v } })) },
                        ].map(({ lbl, val, set }) => (
                          <div key={lbl}>
                            <div style={lblStyle}>{lbl}</div>
                            <input value={val} onChange={e => set(e.target.value)} style={{ ...inpStyle, fontSize: "0.63rem", padding: "0.35rem 0.45rem" }} />
                          </div>
                        ))}
                      </div>
                    )}
                    <label style={{ marginTop: "0.35rem", fontSize: "0.58rem", color: TEXT2, display: "flex", gap: "0.35rem", alignItems: "center", cursor: "pointer" }}>
                      <input type="checkbox" checked={!!cfg.json} onChange={e => setTelemetry(p => ({ ...p, [k]: { ...p[k], json: e.target.checked } }))} style={{ accentColor: ACCENT }} />
                      JSON STRING PARSE
                    </label>
                  </div>
                );
              })}
            </div>

            {/* Control mode */}
            <div style={{ fontSize: "0.6rem", fontWeight: 700, color: TEXT2, letterSpacing: "0.08em", marginBottom: "0.4rem" }}>CONTROL MODE</div>
            <div style={{ display: "flex", gap: "0.4rem", marginBottom: "0.75rem" }}>
              {[{ id: "joystick", label: "⊙ JOYSTICK" }, { id: "buttons", label: "⊞ BUTTONS" }].map(({ id, label }) => (
                <button key={id}
                  onClick={() => { setControlMode(id); if (id === "buttons") safeStop(); }}
                  style={{ ...btnStyle(id === controlMode ? ACCENT : TEXT2, id === controlMode ? "rgba(14,165,233,0.1)" : "transparent", `1px solid ${id === controlMode ? "rgba(14,165,233,0.4)" : BORDER2}`), flex: 1 }}>
                  {label}
                </button>
              ))}
            </div>

            {/* cmd_vel info */}
            <div style={{ background: SRF2, border: `1px solid ${BORDER}`, borderRadius: 4, padding: "0.55rem 0.65rem" }}>
              <div style={lblStyle}>CMD_VEL FORMAT</div>
              <div style={{ fontSize: "0.6rem", color: TEXT2, lineHeight: 2.1, fontFamily: MONO }}>
                <div>↑ İleri &nbsp; linear.x = <span style={{ color: ACCENT }}>+{linearMax}</span> m/s</div>
                <div>↓ Geri &nbsp; linear.x = <span style={{ color: ACCENT }}>-{linearMax}</span> m/s</div>
                <div>← Sola &nbsp; angular.z = <span style={{ color: ACCENT }}>+{angularMax}</span> rad/s</div>
                <div>→ Sağa &nbsp; angular.z = <span style={{ color: ACCENT }}>-{angularMax}</span> rad/s</div>
              </div>
            </div>
          </div>
        )}

        {/* ── MAIN CONTROL ── */}
        <div style={{ flex: 1, display: "grid", gridTemplateColumns: window.innerWidth < 768 ? "1fr" : "repeat(2,1fr)", gap: "0.6rem", minHeight: 0, overflow: "auto" }}>

          {/* JOYSTICK */}
          {controlMode === "joystick" && (
            <div style={{ background: SURFACE, borderRadius: 6, padding: "0.85rem", border: `1px solid ${BORDER2}`, display: "flex", flexDirection: "column" }}>
              <div style={{ fontSize: "0.68rem", fontWeight: 700, color: TEXT, letterSpacing: "0.1em", marginBottom: "0.65rem" }}>JOYSTICK CONTROL</div>
              <div
                ref={joystickZoneRef}
                style={{
                  flex: 1, minHeight: 240, maxHeight: 460, aspectRatio: "1",
                  borderRadius: 8,
                  background: "radial-gradient(circle at center, rgba(14,165,233,0.05) 0%, rgba(2,6,9,0.8) 70%)",
                  border: `1px solid ${BORDER2}`,
                  position: "relative", overflow: "hidden",
                  touchAction: "none", userSelect: "none",
                }}
              >
                {/* Crosshair */}
                <div style={{ position: "absolute", inset: 0, pointerEvents: "none" }}>
                  <div style={{ position: "absolute", top: "50%", left: 0, right: 0, height: 1, background: BORDER, transform: "translateY(-50%)" }} />
                  <div style={{ position: "absolute", left: "50%", top: 0, bottom: 0, width: 1, background: BORDER, transform: "translateX(-50%)" }} />
                  <div style={{ position: "absolute", top: "50%", left: "50%", width: 64, height: 64, border: `1px solid ${BORDER2}`, borderRadius: "50%", transform: "translate(-50%,-50%)" }} />
                  <div style={{ position: "absolute", top: "50%", left: "50%", width: 130, height: 130, border: `1px solid ${BORDER}`, borderRadius: "50%", transform: "translate(-50%,-50%)", opacity: 0.6 }} />
                </div>
                {/* E-stop overlay — joystick'i komple kilitle */}
                {estop && (
                  <div style={{
                    position: "absolute", inset: 0, borderRadius: 8,
                    background: "rgba(239,68,68,0.08)",
                    border: "2px solid rgba(239,68,68,0.4)",
                    display: "flex", alignItems: "center", justifyContent: "center",
                    pointerEvents: "none", zIndex: 10,
                  }}>
                    <div style={{ fontSize: "0.7rem", color: "#ef4444", fontWeight: 800, letterSpacing: "0.12em", textAlign: "center" }}>
                      ⬡<br />ACİL DURDURMA<br />AKTİF
                    </div>
                  </div>
                )}
                <div style={{ position: "absolute", bottom: "0.6rem", left: 0, right: 0, textAlign: "center", fontSize: "0.53rem", color: TEXT3, pointerEvents: "none" }}>
                  DRAG TO MOVE · RELEASE TO STOP
                </div>
              </div>
              <div style={{ marginTop: "0.45rem", display: "flex", justifyContent: "space-between", fontSize: "0.6rem", color: TEXT2 }}>
                <span>LIN ±{linearMax} m/s</span>
                <span>ANG ±{angularMax} rad/s</span>
              </div>
            </div>
          )}

          {/* BUTTONS */}
          {controlMode === "buttons" && (
            <div style={{ background: SURFACE, borderRadius: 6, padding: "0.85rem", border: `1px solid ${BORDER2}`, display: "flex", flexDirection: "column" }}>
              <div style={{ fontSize: "0.68rem", fontWeight: 700, color: TEXT, letterSpacing: "0.1em", marginBottom: "0.65rem" }}>DIRECTIONAL CONTROL</div>
              <div style={{ flex: 1, display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: "0.35rem", minHeight: 240, maxHeight: 460, aspectRatio: "1" }}>
                {dirBtns.map((b, i) =>
                  b.stop ? (
                    <button key="stop"
                      onClick={() => { axesRef.current = { x: 0, y: 0 }; publishTwist(0, 0); }}
                      disabled={estop}
                      style={{
                        background: "rgba(239,68,68,0.1)", border: "1px solid rgba(239,68,68,0.4)",
                        borderRadius: 6, color: "#ef4444",
                        cursor: estop ? "not-allowed" : "pointer", opacity: estop ? 0.3 : 1,
                        fontFamily: MONO, fontSize: "0.6rem", fontWeight: 700,
                        display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", gap: "0.2rem",
                      }}
                    >
                      <span style={{ fontSize: "1rem" }}>⬡</span>STOP
                    </button>
                  ) : (
                    <button key={i}
                      onPointerDown={() => { axesRef.current = { x: b.x, y: b.y }; }}
                      onPointerUp={() => { axesRef.current = { x: 0, y: 0 }; publishTwist(0, 0); }}
                      onPointerLeave={() => { axesRef.current = { x: 0, y: 0 }; publishTwist(0, 0); }}
                      disabled={estop}
                      style={{
                        background: b.diag ? "transparent" : "rgba(14,165,233,0.08)",
                        border: `1px solid ${b.diag ? BORDER : "rgba(14,165,233,0.3)"}`,
                        borderRadius: 6, color: b.diag ? TEXT2 : ACCENT,
                        fontSize: "1.2rem", cursor: estop ? "not-allowed" : "pointer",
                        opacity: estop ? 0.3 : 1, transition: "all 0.1s", userSelect: "none",
                      }}
                    >{b.label}</button>
                  )
                )}
              </div>
              <div style={{ marginTop: "0.4rem", fontSize: "0.53rem", color: TEXT2, textAlign: "center" }}>
                HOLD = MOVE · RELEASE = STOP
              </div>
            </div>
          )}

          {/* E-STOP */}
          <div style={{
            background: SURFACE, borderRadius: 6, padding: "0.85rem",
            border: `1px solid ${estop ? "rgba(239,68,68,0.45)" : BORDER2}`,
            display: "flex", flexDirection: "column",
            boxShadow: estop ? "0 0 24px rgba(239,68,68,0.12), inset 0 0 30px rgba(239,68,68,0.04)" : "none",
            transition: "all 0.25s",
          }}>
            <div style={{ fontSize: "0.68rem", fontWeight: 700, color: estop ? "#ef4444" : TEXT, letterSpacing: "0.1em", marginBottom: "0.65rem" }}>
              EMERGENCY STOP
            </div>

            <div style={{ flex: 1, display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem" }}>
              <button
                onClick={() => {
                  setEstop(true);                            // context publish eder + tüm cihazlar senkron
                  // Güvenlik: hemen üç kez sıfır bas
                  publishTwist(0, 0);
                  setTimeout(() => publishTwist(0, 0), 80);
                  setTimeout(() => publishTwist(0, 0), 160);
                }}
                style={{
                  background: "rgba(239,68,68,0.1)", border: "2px solid #ef4444", borderRadius: 8,
                  color: "#ef4444", fontFamily: MONO, fontSize: "0.7rem", fontWeight: 800,
                  letterSpacing: "0.06em", cursor: "pointer",
                  display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", gap: "0.45rem",
                  padding: "1.1rem 0.75rem",
                  boxShadow: "0 0 14px rgba(239,68,68,0.18)", transition: "all 0.15s",
                }}
                onMouseEnter={e => { e.currentTarget.style.background = "rgba(239,68,68,0.22)"; e.currentTarget.style.boxShadow = "0 0 28px rgba(239,68,68,0.4)"; }}
                onMouseLeave={e => { e.currentTarget.style.background = "rgba(239,68,68,0.1)"; e.currentTarget.style.boxShadow = "0 0 14px rgba(239,68,68,0.18)"; }}
              >
                <div style={{ width: 38, height: 38, borderRadius: "50%", border: "2px solid #ef4444", display: "flex", alignItems: "center", justifyContent: "center", fontSize: "1.2rem", boxShadow: "0 0 10px #ef4444" }}>⬡</div>
                ACİL<br />DURDUR
              </button>

              <button
                onClick={() => setEstop(false)}
                style={{
                  background: estop ? "rgba(16,185,129,0.1)" : "transparent",
                  border: `2px solid ${estop ? "#10b981" : BORDER2}`,
                  borderRadius: 8, color: estop ? "#10b981" : TEXT2,
                  fontFamily: MONO, fontSize: "0.7rem", fontWeight: 800,
                  letterSpacing: "0.06em", cursor: "pointer",
                  display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", gap: "0.45rem",
                  padding: "1.1rem 0.75rem",
                  boxShadow: estop ? "0 0 14px rgba(16,185,129,0.2)" : "none",
                  transition: "all 0.2s",
                }}
                onMouseEnter={e => { if (estop) { e.currentTarget.style.background = "rgba(16,185,129,0.2)"; e.currentTarget.style.boxShadow = "0 0 28px rgba(16,185,129,0.35)"; } }}
                onMouseLeave={e => { if (estop) { e.currentTarget.style.background = "rgba(16,185,129,0.1)"; e.currentTarget.style.boxShadow = "0 0 14px rgba(16,185,129,0.2)"; } }}
              >
                <div style={{ width: 38, height: 38, borderRadius: "50%", border: `2px solid ${estop ? "#10b981" : BORDER2}`, display: "flex", alignItems: "center", justifyContent: "center", fontSize: "1.2rem", boxShadow: estop ? "0 0 10px #10b981" : "none" }}>◉</div>
                E-STOP<br />ÇÖZ
              </button>
            </div>

            {estop && (
              <div style={{ marginTop: "0.6rem", padding: "0.45rem 0.6rem", background: "rgba(239,68,68,0.07)", border: "1px solid rgba(239,68,68,0.3)", borderRadius: 5, textAlign: "center" }}>
                <div style={{ fontSize: "0.63rem", color: "#ef4444", fontWeight: 700, letterSpacing: "0.1em" }}>⬡ ACİL DURDURMA AKTİF</div>
                <div style={{ fontSize: "0.55rem", color: TEXT2, marginTop: 2 }}>{emergencyTopic} → true</div>
                <div style={{ fontSize: "0.55rem", color: TEXT2, marginTop: 1 }}>{topicName} → 0,0 @ 20Hz</div>
              </div>
            )}

            <div style={{ marginTop: "0.6rem", display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.4rem" }}>
              {[
                { label: "LINEAR MAX", value: `${linearMax.toFixed(2)} m/s` },
                { label: "ANGULAR MAX", value: `${angularMax.toFixed(2)} r/s` },
              ].map(({ label, value }) => (
                <div key={label} style={{ background: SRF2, border: `1px solid ${BORDER}`, borderRadius: 4, padding: "0.38rem 0.5rem" }}>
                  <div style={lblStyle}>{label}</div>
                  <div style={{ fontFamily: MONO, fontSize: "0.72rem", color: ACCENT, fontWeight: 600 }}>{value}</div>
                </div>
              ))}
            </div>

            {/* Routing bilgi kartı — robotta mux YOK, teleop ve nav2 aynı topic'e yazıyor */}
            <div style={{ marginTop: "0.5rem", background: SRF2, border: `1px solid ${BORDER}`, borderRadius: 4, padding: "0.38rem 0.6rem" }}>
              <div style={lblStyle}>⚠ ROUTING (MUX YOK)</div>
              <div style={{ fontSize: "0.58rem", color: TEXT2, lineHeight: 1.9, fontFamily: MONO }}>
                <div><span style={{ color: ACCENT }}>teleop</span> → {topicName} → ros2_to_twist → /twist (canbus)</div>
                <div><span style={{ color: "#f59e0b" }}>nav2</span> &nbsp;→ {topicName} <span style={{ color: TEXT2 }}>(aynı topic — nav2 modundayken teleop kullanma)</span></div>
              </div>
            </div>
          </div>
        </div>

        {/* FOOTER */}
        <div style={{ flexShrink: 0, textAlign: "center", fontSize: "0.55rem", color: TEXT3 }}>
          REAL-TIME ROS TELEOP ·&nbsp;
          <span style={{ color: ACCENT }}>{topicName}</span>
          &nbsp;·&nbsp;
          <span style={{ color: estop ? "#ef4444" : TEXT3 }}>
            {estop ? "⬡ E-STOP ACTIVE" : "● NOMINAL"}
          </span>
        </div>

      </div>
    </div>
  );
}
