import React, { useEffect, useMemo, useState, useCallback, useRef } from "react";
import { useROS } from "../context/ROSContext";
import * as ROSLIB from "roslib";

const BG      = "#04090f";
const SURFACE = "#07111d";
const SRF2    = "#0b1929";
const BORDER  = "#0f2236";
const BORDER2 = "#162d46";
const TEXT    = "#c8dde8";
const TEXT2   = "#4a7a96";
const TEXT3   = "#1e3a52";
const ACCENT  = "#0ea5e9";
const MONO    = "'JetBrains Mono','Fira Code',monospace";

const LEVEL = {
  0: { label: "OK",    color: "#10b981", dim: "rgba(16,185,129,0.1)",  border: "rgba(16,185,129,0.3)"  },
  1: { label: "WARN",  color: "#f59e0b", dim: "rgba(245,158,11,0.1)",  border: "rgba(245,158,11,0.3)"  },
  2: { label: "ERROR", color: "#ef4444", dim: "rgba(239,68,68,0.1)",   border: "rgba(239,68,68,0.3)"   },
  3: { label: "STALE", color: "#4a7a96", dim: "rgba(74,122,150,0.1)",  border: "rgba(74,122,150,0.3)"  },
};

function levelNum(lvl) {
  if (typeof lvl === "number") return lvl;
  if (typeof lvl === "string") { const n = Number(lvl); return !Number.isNaN(n) ? n : lvl.length ? lvl.charCodeAt(0) : 0; }
  return 0;
}
function pickWorst(arr) {
  let w = 0;
  for (const s of arr || []) { const n = levelNum(s.level); if (n > w) w = n; }
  return w;
}
function stampToMs(stamp) {
  if (!stamp || typeof stamp.sec !== "number") return null;
  return stamp.sec * 1000 + Math.floor((stamp.nanosec || 0) / 1e6);
}
function formatAgo(stamp) {
  const ms = stampToMs(stamp);
  if (ms == null) return "—";
  const diff = Math.max(0, (Date.now() - ms) / 1000);
  if (diff < 1)    return "NOW";
  if (diff < 60)   return `${diff.toFixed(1)}s ago`;
  if (diff < 3600) return `${Math.floor(diff/60)}m ago`;
  return `${Math.floor(diff/3600)}h ago`;
}

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
const cardStyle = (bg = SRF2, border = BORDER2) => ({
  background: bg, borderRadius: 5, padding: "0.75rem 0.85rem",
  border: `1px solid ${border}`,
});

// ── Default monitored topics ────────────────────────────────────────────────
// /usb_cam/image_raw default KAPALI — CompressedImage 30fps × 100KB → rosbridge
// JSON serialization'ı CPU yer. Kullanıcı isterse manuel açabilir.
const DEFAULT_HZ_TOPICS = [
  { topic: "/odom",          messageType: "nav_msgs/Odometry",     label: "ODOM",     enabled: true  },
  { topic: "/rtk/odom",      messageType: "nav_msgs/Odometry",     label: "RTK ODOM", enabled: true  },
  { topic: "/imu/data",      messageType: "sensor_msgs/Imu",       label: "IMU",      enabled: true  },
  { topic: "/scan_filtered", messageType: "sensor_msgs/LaserScan", label: "LIDAR",    enabled: true  },
  { topic: "/camera1",       messageType: "sensor_msgs/Image",     label: "CAMERA",   enabled: false },
  { topic: "/cmd_vel_nav",   messageType: "geometry_msgs/Twist",   label: "CMD_VEL",  enabled: true  },
];

// ── Topic Hz Monitor ────────────────────────────────────────────────────────
const WINDOW_SEC = 3;           // sliding window for Hz calc
const TICK_MS    = 500;         // UI refresh interval

const TopicHzMonitor = ({ ros, connected }) => {
  const [topics, setTopics]       = useState(() => DEFAULT_HZ_TOPICS.map(t => ({ ...t, enabled: t.enabled !== false })));
  const [hzData, setHzData]       = useState({});   // { topic: { hz, count, lastTs, jitter } }
  const [showAdd, setShowAdd]     = useState(false);
  const [newTopic, setNewTopic]   = useState("");
  const [newType, setNewType]     = useState("");
  const [newLabel, setNewLabel]   = useState("");

  const subsRef      = useRef({});   // { topic: ROSLIB.Topic }
  const stampBufRef  = useRef({});   // { topic: number[] }  — timestamps ring buffer

  // subscribe / unsubscribe
  useEffect(() => {
    if (!ros || !connected) {
      // cleanup all
      Object.values(subsRef.current).forEach(t => { try { t.unsubscribe(); } catch {} });
      subsRef.current  = {};
      stampBufRef.current = {};
      setHzData({});
      return;
    }

    const active = topics.filter(t => t.enabled);
    const activeSet = new Set(active.map(t => t.topic));

    // remove stale subs
    Object.keys(subsRef.current).forEach(tp => {
      if (!activeSet.has(tp)) {
        try { subsRef.current[tp].unsubscribe(); } catch {}
        delete subsRef.current[tp];
        delete stampBufRef.current[tp];
      }
    });

    // add new subs
    active.forEach(cfg => {
      if (subsRef.current[cfg.topic]) return;   // already subscribed
      if (!stampBufRef.current[cfg.topic]) stampBufRef.current[cfg.topic] = [];
      try {
        // CompressedImage / Image / PointCloud2 / LaserScan: byte-array ağırlıklı
        // mesajlar için cbor-raw kullan (rosbridge JSON serialization'ı atlar).
        // Diğer mesajlar küçük ve sık olduğu için JSON varsayılan yeterli — cbor
        // moduna güvenmiyoruz çünkü roslibjs/rosbridge kombinasyonu bazı mesaj
        // tiplerinde subscribe callback'ini çağırmıyor (Odometry, Imu, Twist).
        const isBinaryHeavy = /Image|PointCloud2|LaserScan/.test(cfg.messageType);
        const opts = {
          ros, name: cfg.topic, messageType: cfg.messageType,
          queue_length: 1,
        };
        if (isBinaryHeavy) opts.compression = "cbor-raw";
        const t = new ROSLIB.Topic(opts);
        t.subscribe(() => {
          const buf = stampBufRef.current[cfg.topic];
          if (buf) buf.push(Date.now());
        });
        subsRef.current[cfg.topic] = t;
      } catch {}
    });

    return () => {
      Object.values(subsRef.current).forEach(t => { try { t.unsubscribe(); } catch {} });
      subsRef.current  = {};
      stampBufRef.current = {};
    };
  }, [ros, connected, topics]);

  // periodic Hz calculation
  useEffect(() => {
    const id = setInterval(() => {
      const now = Date.now();
      const cutoff = now - WINDOW_SEC * 1000;
      const next = {};

      topics.forEach(cfg => {
        const buf = stampBufRef.current[cfg.topic];
        if (!buf) { next[cfg.topic] = { hz: null, count: 0, lastTs: null, jitter: null }; return; }

        // trim old entries
        while (buf.length > 0 && buf[0] < cutoff) buf.shift();

        const count = buf.length;
        if (count < 2) {
          next[cfg.topic] = { hz: count > 0 ? 0 : null, count, lastTs: buf[buf.length - 1] || null, jitter: null };
          return;
        }

        const span = (buf[buf.length - 1] - buf[0]) / 1000;
        const hz   = span > 0 ? (count - 1) / span : 0;

        // jitter: std-dev of intervals
        const intervals = [];
        for (let i = 1; i < buf.length; i++) intervals.push(buf[i] - buf[i - 1]);
        const mean = intervals.reduce((a, b) => a + b, 0) / intervals.length;
        const variance = intervals.reduce((a, v) => a + (v - mean) ** 2, 0) / intervals.length;
        const jitter = Math.sqrt(variance);

        next[cfg.topic] = { hz, count, lastTs: buf[buf.length - 1], jitter };
      });

      setHzData(next);
    }, TICK_MS);
    return () => clearInterval(id);
  }, [topics]);

  const hzColor = (hz) => {
    if (hz == null) return TEXT3;
    if (hz === 0)   return "#ef4444";
    if (hz < 5)     return "#f59e0b";
    return "#10b981";
  };

  const hzBarWidth = (hz, expected) => {
    if (hz == null || hz === 0) return 0;
    return Math.min(100, (hz / expected) * 100);
  };

  const addTopic = () => {
    const t = newTopic.trim();
    const m = newType.trim() || "std_msgs/Empty";
    const l = newLabel.trim() || t.replace(/^\//, "").split("/").pop().toUpperCase();
    if (!t) return;
    if (topics.find(x => x.topic === t)) return;
    setTopics(p => [...p, { topic: t, messageType: m, label: l, enabled: true }]);
    setNewTopic(""); setNewType(""); setNewLabel(""); setShowAdd(false);
  };

  const removeTopic = (tp) => {
    setTopics(p => p.filter(x => x.topic !== tp));
  };

  const toggleTopic = (tp) => {
    setTopics(p => p.map(x => x.topic === tp ? { ...x, enabled: !x.enabled } : x));
  };

  return (
    <div style={cardStyle()}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "0.75rem" }}>
        <div>
          <div style={{ fontSize: "0.68rem", fontWeight: 700, color: TEXT, letterSpacing: "0.1em", marginBottom: 2 }}>
            TOPIC HZ MONITOR
          </div>
          <div style={{ fontSize: "0.58rem", color: TEXT2 }}>
            Canlı yayın frekansı · {WINDOW_SEC}s pencere
          </div>
        </div>
        <button onClick={() => setShowAdd(v => !v)} style={btnStyle(showAdd ? ACCENT : TEXT2, showAdd ? "rgba(14,165,233,0.1)" : "transparent", `1px solid ${showAdd ? "rgba(14,165,233,0.4)" : BORDER2}`)}>
          + Topic Ekle
        </button>
      </div>

      {/* Add topic form */}
      {showAdd && (
        <div style={{ background: SURFACE, border: `1px solid ${BORDER2}`, borderRadius: 5, padding: "0.6rem 0.7rem", marginBottom: "0.65rem", display: "grid", gridTemplateColumns: "2fr 2fr 1fr auto", gap: "0.4rem", alignItems: "end" }}>
          <div>
            <div style={lblStyle}>TOPIC</div>
            <input value={newTopic} onChange={e => setNewTopic(e.target.value)} placeholder="/odom" style={inpStyle} />
          </div>
          <div>
            <div style={lblStyle}>MSG TYPE</div>
            <input value={newType} onChange={e => setNewType(e.target.value)} placeholder="nav_msgs/Odometry" style={inpStyle} />
          </div>
          <div>
            <div style={lblStyle}>LABEL</div>
            <input value={newLabel} onChange={e => setNewLabel(e.target.value)} placeholder="ODOM" style={inpStyle} />
          </div>
          <button onClick={addTopic} style={{ ...btnStyle(ACCENT, "rgba(14,165,233,0.12)", `1px solid ${ACCENT}`), height: "fit-content" }}>
            Ekle
          </button>
        </div>
      )}

      {/* Hz grid */}
      <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(200px,1fr))", gap: "0.5rem" }}>
        {topics.map(cfg => {
          const d  = hzData[cfg.topic] || { hz: null, count: 0, lastTs: null, jitter: null };
          const hz = d.hz;
          const col = hzColor(hz);
          const alive = hz != null && hz > 0;
          const ageMs = d.lastTs ? Date.now() - d.lastTs : null;
          const stale = ageMs != null && ageMs > 3000;

          // expected Hz heuristic
          const expected =
            cfg.topic.includes("scan") || cfg.topic.includes("lidar") ? 15 :
            cfg.topic.includes("imu")  ? 100 :
            cfg.topic.includes("odom") ? 50 :
            cfg.topic.includes("camera") || cfg.topic.includes("image") ? 30 :
            cfg.topic.includes("cmd_vel") ? 20 : 30;

          return (
            <div key={cfg.topic} style={{
              background: SURFACE,
              borderRadius: 5,
              border: `1px solid ${!cfg.enabled ? BORDER : alive ? `${col}44` : stale ? "rgba(239,68,68,0.3)" : BORDER2}`,
              overflow: "hidden",
              opacity: cfg.enabled ? 1 : 0.45,
              transition: "all 0.2s",
            }}>
              {/* header */}
              <div style={{
                padding: "0.5rem 0.65rem 0.35rem",
                display: "flex", justifyContent: "space-between", alignItems: "center",
              }}>
                <div style={{ display: "flex", alignItems: "center", gap: "0.4rem" }}>
                  <div style={{
                    width: 6, height: 6, borderRadius: "50%",
                    background: !cfg.enabled ? TEXT3 : alive ? col : stale ? "#ef4444" : TEXT3,
                    boxShadow: alive ? `0 0 6px ${col}` : "none",
                    transition: "all 0.3s",
                  }} />
                  <span style={{ fontFamily: MONO, fontSize: "0.7rem", fontWeight: 700, color: TEXT }}>{cfg.label}</span>
                </div>
                <div style={{ display: "flex", gap: "0.25rem" }}>
                  <button
                    onClick={() => toggleTopic(cfg.topic)}
                    title={cfg.enabled ? "Devre dışı bırak" : "Etkinleştir"}
                    style={{ background: "none", border: "none", cursor: "pointer", fontSize: "0.6rem", color: cfg.enabled ? TEXT2 : "#ef4444", padding: "0.15rem" }}
                  >
                    {cfg.enabled ? "●" : "○"}
                  </button>
                  {!DEFAULT_HZ_TOPICS.find(x => x.topic === cfg.topic) && (
                    <button
                      onClick={() => removeTopic(cfg.topic)}
                      title="Kaldır"
                      style={{ background: "none", border: "none", cursor: "pointer", fontSize: "0.6rem", color: "#ef4444", padding: "0.15rem" }}
                    >
                      ✕
                    </button>
                  )}
                </div>
              </div>

              {/* Hz value */}
              <div style={{ padding: "0 0.65rem 0.25rem" }}>
                <div style={{ display: "flex", alignItems: "baseline", gap: "0.3rem" }}>
                  <span style={{ fontFamily: MONO, fontSize: "1.35rem", fontWeight: 900, color: col, lineHeight: 1 }}>
                    {hz == null ? "—" : hz < 1 ? hz.toFixed(2) : hz.toFixed(1)}
                  </span>
                  {hz != null && <span style={{ fontSize: "0.6rem", color: TEXT2, fontWeight: 600 }}>Hz</span>}
                </div>
              </div>

              {/* Hz bar */}
              <div style={{ margin: "0 0.65rem 0.45rem", height: 3, background: BORDER, borderRadius: 2, overflow: "hidden" }}>
                <div style={{
                  height: "100%",
                  width: `${hzBarWidth(hz, expected)}%`,
                  background: col,
                  borderRadius: 2,
                  transition: "width 0.4s ease",
                }} />
              </div>

              {/* Meta */}
              <div style={{
                padding: "0.35rem 0.65rem 0.45rem",
                borderTop: `1px solid ${BORDER}`,
                display: "flex", justifyContent: "space-between", alignItems: "center",
              }}>
                <span style={{ fontFamily: MONO, fontSize: "0.53rem", color: TEXT3, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap", maxWidth: "55%" }}>
                  {cfg.topic}
                </span>
                <span style={{ fontSize: "0.53rem", color: TEXT3 }}>
                  {d.jitter != null ? `±${d.jitter.toFixed(1)}ms` : "—"}
                </span>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};


// ── TF Tree ─────────────────────────────────────────────────────────────────
const TFTreeVisualizer = ({ ros, connected }) => {
  const [frames,        setFrames]   = useState([]);
  const [selected,      setSelected] = useState(null);
  const [loading,       setLoading]  = useState(false);
  const [error,         setError]    = useState("");

  const fetchFrames = useCallback(() => {
    if (!ros || !connected) return;
    setLoading(true); setError("");
    const all = {};
    const process = msg => {
      (msg.transforms || []).forEach(t => {
        all[t.child_frame_id] = { child: t.child_frame_id, parent: t.header.frame_id };
      });
    };
    // 1.5 saniye dinleyip frame'leri topluyoruz — throttle koymak gerekli, aksi
    // halde /tf saniyede ~100 kez yayınlandığı için 150 mesaj serileştirilir.
    const tf       = new ROSLIB.Topic({ ros, name: "/tf",        messageType: "tf2_msgs/TFMessage", throttle_rate: 200, queue_length: 1 });
    const tfStatic = new ROSLIB.Topic({ ros, name: "/tf_static", messageType: "tf2_msgs/TFMessage", queue_length: 1 });
    tf.subscribe(process); tfStatic.subscribe(process);

    setTimeout(() => {
      try { tf.unsubscribe(); tfStatic.unsubscribe(); } catch {}
      const arr = Object.values(all);
      if (arr.length > 0) setFrames(arr);
      else setError("Frame bulunamadı — /tf veya /tf_static aktif değil");
      setLoading(false);
    }, 1500);

    setTimeout(() => {
      setLoading(p => { if (p) { setError("Timeout — frame verisi alınamadı"); return false; } return p; });
    }, 5000);
  }, [ros, connected]);

  useEffect(() => { if (connected) fetchFrames(); }, [connected, fetchFrames]);

  const tree = useMemo(() => {
    if (!frames.length) return { roots: [], frameMap: {}, parentMap: {} };
    const frameMap = {}, parentMap = {};
    frames.forEach(f => {
      frameMap[f.child] = f.parent;
      if (!parentMap[f.parent]) parentMap[f.parent] = [];
      parentMap[f.parent].push(f.child);
    });
    const children = new Set(Object.keys(frameMap));
    const roots    = [...new Set(Object.values(frameMap).filter(p => !children.has(p)))];
    return { roots, frameMap, parentMap };
  }, [frames]);

  const KEY_FRAMES = new Set(["map", "odom", "base_link", "base_footprint"]);

  const renderNode = (frame, level = 0) => {
    const kids  = tree.parentMap[frame] || [];
    const isKey = KEY_FRAMES.has(frame);
    const isSel = selected === frame;

    return (
      <div key={`${frame}-${level}`}>
        <div
          onClick={() => setSelected(frame)}
          style={{
            display: "flex", alignItems: "center", gap: "0.5rem",
            marginLeft: `${level * 1.1}rem`,
            marginBottom: "0.2rem",
            padding: "0.42rem 0.65rem",
            borderRadius: 4,
            border: `1px solid ${isSel ? "rgba(14,165,233,0.4)" : isKey ? "rgba(245,158,11,0.25)" : "transparent"}`,
            background: isSel ? "rgba(14,165,233,0.08)" : isKey ? "rgba(245,158,11,0.05)" : "transparent",
            cursor: "pointer", transition: "all 0.12s",
          }}
          onMouseEnter={e => { if (!isSel) e.currentTarget.style.background = "rgba(14,165,233,0.05)"; }}
          onMouseLeave={e => { if (!isSel) e.currentTarget.style.background = isKey ? "rgba(245,158,11,0.05)" : "transparent"; }}
        >
          <span style={{ fontSize: "0.65rem", color: kids.length ? ACCENT : TEXT3 }}>
            {kids.length ? "▸" : "·"}
          </span>
          <span style={{ fontFamily: MONO, fontSize: "0.72rem", flex: 1, color: isSel ? ACCENT : isKey ? "#f59e0b" : TEXT }}>
            {frame}
          </span>
          {isKey && (
            <span style={{ fontSize: "0.53rem", background: "rgba(245,158,11,0.12)", color: "#f59e0b", padding: "0.1rem 0.35rem", borderRadius: 3, letterSpacing: "0.06em" }}>
              KEY
            </span>
          )}
          {kids.length > 0 && (
            <span style={{ fontSize: "0.58rem", color: TEXT2 }}>{kids.length}▾</span>
          )}
        </div>
        {kids.length > 0 && (
          <div style={{ borderLeft: `1px solid ${BORDER2}`, marginLeft: `${level * 1.1 + 0.65}rem`, paddingLeft: "0.4rem", marginBottom: "0.1rem" }}>
            {kids.map(c => renderNode(c, level + 1))}
          </div>
        )}
      </div>
    );
  };

  return (
    <div style={cardStyle()}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: "0.75rem" }}>
        <div>
          <div style={{ fontSize: "0.68rem", fontWeight: 700, color: TEXT, letterSpacing: "0.1em", marginBottom: 2 }}>
            TRANSFORM TREE
          </div>
          <div style={{ fontSize: "0.58rem", color: TEXT2 }}>
            {loading ? "Yükleniyor..." : `${frames.length} frame bulundu`}
          </div>
        </div>
        <button onClick={fetchFrames} style={btnStyle(TEXT2, "transparent", `1px solid ${BORDER2}`)}>
          ↻ Yenile
        </button>
      </div>

      {error && (
        <div style={{ background: "rgba(239,68,68,0.08)", border: "1px solid rgba(239,68,68,0.25)", borderRadius: 4, padding: "0.5rem 0.65rem", fontSize: "0.65rem", color: "#f87171", marginBottom: "0.65rem" }}>
          ⚠ {error}
        </div>
      )}

      {loading && (
        <div style={{ textAlign: "center", padding: "2rem", color: TEXT3 }}>
          <div style={{ fontSize: "0.68rem" }}>⏳ Frame'ler taranıyor...</div>
        </div>
      )}

      {!loading && frames.length > 0 && (
        <div style={{ maxHeight: 420, overflowY: "auto", paddingRight: 4 }}>
          {tree.roots.length > 0
            ? tree.roots.map(r => renderNode(r))
            : <div style={{ fontSize: "0.65rem", color: TEXT3 }}>Ağaç yapısı oluşturulamadı</div>
          }
        </div>
      )}

      {selected && (
        <div style={{ marginTop: "0.75rem", padding: "0.55rem 0.7rem", background: "rgba(14,165,233,0.07)", border: `1px solid rgba(14,165,233,0.25)`, borderRadius: 4 }}>
          <div style={lblStyle}>Seçili Frame</div>
          <div style={{ fontFamily: MONO, fontSize: "0.82rem", color: ACCENT, fontWeight: 700 }}>{selected}</div>
          {tree.frameMap[selected] && (
            <div style={{ fontSize: "0.6rem", color: TEXT2, marginTop: 3 }}>
              parent → <span style={{ color: "#f59e0b" }}>{tree.frameMap[selected]}</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

// ── Main ─────────────────────────────────────────────────────────────────────
export default function HealthPage() {
  const { ros, isConnected, status: globalStatus, errorText: globalErrorText, rosbridgeUrl, reconnect } = useROS();

  const [diag,      setDiag]      = useState(null);
  const [lastStamp, setLastStamp] = useState(null);

  useEffect(() => {
    if (!ros || !isConnected) return;
    const topic = new ROSLIB.Topic({
      ros, name: "/system/diagnostics",
      messageType: "diagnostic_msgs/msg/DiagnosticArray",
      queue_length: 1,
      throttle_rate: 1000,   // 1Hz UI için fazlasıyla yeter
    });
    topic.subscribe(msg => { setDiag(msg); if (msg?.header?.stamp) setLastStamp(msg.header.stamp); });
    return () => { try { topic.unsubscribe(); } catch {} };
  }, [ros, isConnected]);

  const statuses   = diag?.status || [];
  const summary    = statuses.find(s => s.name === "system/summary" || s.name === "summary");
  const topicSt    = statuses.filter(s => s.name && s.name !== "system/summary" && s.name !== "summary");
  const worst      = pickWorst(statuses);
  const worstLevel = LEVEL[worst] || LEVEL[0];

  return (
    <div style={{
      minHeight: "calc(100vh - 56px)",
      background: BG,
      backgroundImage: "radial-gradient(rgba(14,165,233,0.06) 1px, transparent 1px)",
      backgroundSize: "24px 24px",
      color: TEXT, padding: "0.65rem",
      fontFamily: MONO, overflow: "auto", boxSizing: "border-box",
    }}>
      <div style={{ maxWidth: 1400, margin: "0 auto", display: "flex", flexDirection: "column", gap: "0.6rem" }}>

        {/* HEADER */}
        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between" }}>
          <div style={{ display: "flex", alignItems: "center", gap: "0.65rem" }}>
            <div style={{
              width: 32, height: 32, background: "rgba(14,165,233,0.1)",
              border: `1px solid rgba(14,165,233,0.35)`, borderRadius: 6,
              display: "flex", alignItems: "center", justifyContent: "center",
              fontSize: "1rem", boxShadow: "0 0 12px rgba(14,165,233,0.15)",
            }}>🏥</div>
            <div>
              <div style={{ fontSize: "0.82rem", fontWeight: 800, letterSpacing: "0.14em", color: ACCENT }}>SYSTEM HEALTH</div>
              <div style={{ fontSize: "0.55rem", color: TEXT2, letterSpacing: "0.1em", marginTop: 1 }}>DIAGNOSTICS · TF TREE · TOPIC MONITOR · HZ</div>
            </div>
          </div>
          <div style={{
            fontSize: "0.62rem", fontWeight: 700, color: worstLevel.color,
            background: worstLevel.dim, border: `1px solid ${worstLevel.border}`,
            borderRadius: 5, padding: "0.35rem 0.7rem", letterSpacing: "0.08em",
          }}>
            {worst === 0 ? "✓" : worst === 1 ? "!" : worst === 2 ? "✕" : "⏸"} {worstLevel.label}
          </div>
        </div>

        {/* STATUS BAR */}
        <div style={{
          background: SURFACE, borderRadius: 5,
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
          <span style={{ fontSize: "0.62rem", color: TEXT2, marginLeft: "auto" }}>
            TOPICS: <b style={{ color: TEXT }}>{topicSt.length}</b>
          </span>
          {!isConnected
            ? <button onClick={reconnect} style={btnStyle(ACCENT, "rgba(14,165,233,0.12)", `1px solid ${ACCENT}`)}>⚡ Bağlan</button>
            : <span style={{ fontSize: "0.62rem", color: "#10b981", fontWeight: 600 }}>● CONNECTED</span>
          }
        </div>

        {/* SUMMARY CARDS */}
        <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(220px,1fr))", gap: "0.6rem" }}>
          {/* ROSBridge */}
          <div style={cardStyle()}>
            <div style={lblStyle}>ROSBRIDGE</div>
            <div style={{ display: "flex", alignItems: "center", gap: "0.5rem", marginBottom: "0.35rem" }}>
              <div style={{
                width: 7, height: 7, borderRadius: "50%",
                background: isConnected ? "#10b981" : "#ef4444",
                boxShadow: `0 0 6px ${isConnected ? "#10b981" : "#ef4444"}`,
              }} />
              <span style={{ fontFamily: MONO, fontSize: "0.75rem", fontWeight: 700, color: isConnected ? "#10b981" : "#ef4444" }}>
                {isConnected ? "CONNECTED" : "OFFLINE"}
              </span>
            </div>
            <div style={{ fontSize: "0.6rem", color: ACCENT, wordBreak: "break-all" }}>{rosbridgeUrl}</div>
          </div>

          {/* Overall */}
          <div style={cardStyle()}>
            <div style={lblStyle}>OVERALL STATUS</div>
            <div style={{ fontSize: "1.2rem", fontWeight: 900, color: worstLevel.color, marginBottom: "0.3rem" }}>
              {worstLevel.label}
            </div>
            <div style={{ fontSize: "0.65rem", color: TEXT2 }}>{summary?.message || "Veri bekleniyor..."}</div>
          </div>

          {/* Last update */}
          <div style={cardStyle()}>
            <div style={lblStyle}>SON GÜNCELLEME</div>
            <div style={{ fontSize: "0.88rem", fontWeight: 700, color: TEXT, marginBottom: "0.3rem" }}>
              {lastStamp ? formatAgo(lastStamp) : "—"}
            </div>
            <div style={{ fontSize: "0.6rem", color: TEXT2 }}>/system/diagnostics</div>
          </div>
        </div>

        {/* ── TOPIC HZ MONITOR ── */}
        {isConnected && ros && <TopicHzMonitor ros={ros} connected={isConnected} />}

        {/* TF TREE */}
        {isConnected && ros && <TFTreeVisualizer ros={ros} connected={isConnected} />}

        {/* TOPICS */}
        {topicSt.length > 0 && (
          <div>
            <div style={{ fontSize: "0.68rem", fontWeight: 700, color: TEXT, letterSpacing: "0.1em", marginBottom: "0.5rem" }}>
              DIAGNOSTICS MONITOR &nbsp;
              <span style={{ color: TEXT2, fontWeight: 400 }}>({topicSt.length})</span>
            </div>
            <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(280px,1fr))", gap: "0.6rem" }}>
              {topicSt.map(s => {
                const lvl = LEVEL[levelNum(s.level)] || LEVEL[0];
                return (
                  <div key={s.name} style={{ background: SURFACE, borderRadius: 5, border: `1px solid ${lvl.border}`, overflow: "hidden" }}>
                    {/* Topic header */}
                    <div style={{ background: lvl.dim, padding: "0.55rem 0.75rem", borderBottom: `1px solid ${lvl.border}`, display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                      <div style={{ fontFamily: MONO, fontSize: "0.72rem", fontWeight: 700, color: TEXT }}>
                        {s.name.replace(/^topic\//, "")}
                      </div>
                      <span style={{ fontSize: "0.6rem", fontWeight: 700, color: lvl.color, background: `${lvl.color}18`, border: `1px solid ${lvl.border}`, padding: "0.1rem 0.4rem", borderRadius: 3 }}>
                        {lvl.label}
                      </span>
                    </div>
                    <div style={{ padding: "0.65rem 0.75rem" }}>
                      {s.message && (
                        <div style={{ fontFamily: MONO, fontSize: "0.65rem", color: TEXT2, marginBottom: "0.55rem", borderLeft: `2px solid ${lvl.color}`, paddingLeft: "0.45rem" }}>
                          {s.message}
                        </div>
                      )}
                      {Array.isArray(s.values) && s.values.length > 0 && (
                        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.35rem" }}>
                          {s.values.filter(kv => kv?.key && kv?.value !== "").slice(0, 6).map(kv => (
                            <div key={kv.key} style={{ background: "#03070e", border: `1px solid ${BORDER}`, borderRadius: 3, padding: "0.38rem 0.45rem" }}>
                              <div style={{ fontSize: "0.55rem", color: TEXT3, letterSpacing: "0.06em", marginBottom: "0.1rem" }}>{kv.key}</div>
                              <div style={{ fontFamily: MONO, fontSize: "0.68rem", fontWeight: 600, color: TEXT, wordBreak: "break-all" }}>{kv.value}</div>
                            </div>
                          ))}
                        </div>
                      )}
                      {s.values?.length > 6 && (
                        <div style={{ fontSize: "0.55rem", color: TEXT3, marginTop: "0.35rem" }}>+{s.values.length - 6} daha fazla</div>
                      )}
                    </div>
                  </div>
                );
              })}
            </div>
          </div>
        )}

        {/* FOOTER */}
        <div style={{ textAlign: "center", fontSize: "0.55rem", color: TEXT3, paddingBottom: "0.5rem" }}>
          SYSTEM DIAGNOSTICS · ROS HEALTH MONITOR ·&nbsp;
          <span style={{ color: ACCENT }}>/system/diagnostics</span>
        </div>

      </div>
    </div>
  );
}
