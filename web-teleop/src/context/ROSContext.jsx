import React, {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
  useCallback,
} from "react";

import * as ROSLIB from "roslib";

const ROSContext = createContext(null);

// Sayfa PC'de localhost üzerinden açılırsa (örn. `npm run dev`), rosbridge
// robotta çalıştığı için doğrudan robotun IP'sine bağlan. Sayfa bizzat
// robot üzerinden (veya başka bir host adıyla) açılırsa o adresi kullan.
const ROBOT_IP = "10.10.10.117";
const hostname = window.location.hostname || "localhost";
const isLocalDev = hostname === "localhost" || hostname === "127.0.0.1";
const DEFAULT_URL = `ws://${isLocalDev ? ROBOT_IP : hostname}:9090`;

function loadUrl() {
  return DEFAULT_URL;
}

function saveUrl(url) {
  try { localStorage.setItem("rosbridge_url_v1", url); } catch {}
}

// ──────────────────────────────────────────────────────────────────────────
// SENKRONİZE EDİLEN TOPIC'LER
// ──────────────────────────────────────────────────────────────────────────
const SYNC_TOPICS = {
  humanFollow: { name: "/human_follow/enable", type: "std_msgs/Bool"    },
  emergency:   { name: "/emergency/active",    type: "std_msgs/Bool"    },
  speedMode:   { name: "/adaptive_velocity",   type: "std_msgs/Int32"   },
  linearMax:   { name: "/ui/linear_max",       type: "std_msgs/Float32" },
  angularMax:  { name: "/ui/angular_max",      type: "std_msgs/Float32" },
};

export function ROSProvider({ children }) {
  const [rosbridgeUrl, setRosbridgeUrl] = useState(loadUrl);
  const [status, setStatus] = useState("Bağlanmadı");
  const [errorText, setErrorText] = useState("");
  const [isConnected, setIsConnected] = useState(false);

  const rosRef = useRef(null);
  const [rosInstance, setRosInstance] = useState(null);

  const reconnectTimer = useRef(null);
  const mountedRef = useRef(true);
  const connectingRef = useRef(false);

  // ──────────────────────────────────────────────────────────────────────────
  // PAYLAŞILAN STATE
  // ──────────────────────────────────────────────────────────────────────────
  const [operationMode, setOperationModeState] = useState("manual");
  const [humanFollowEnabled, setHumanFollowEnabledState] = useState(false);
  const [cameraEnabled, setCameraEnabled] = useState(false); // Yeni Kamera State
  const [estop, setEstopState] = useState(false);
  const [speedMode, setSpeedModeState] = useState(() => {
    const v = parseInt(localStorage.getItem("adaptive_velocity_mode") || "2", 10);
    return [1, 2, 3, 4].includes(v) ? v : 2;
  });
  const [linearMax, setLinearMaxState] = useState(() => {
    const v = parseFloat(localStorage.getItem("ui_linear_max") || "0.6");
    return isFinite(v) && v > 0 ? v : 0.6;
  });
  const [angularMax, setAngularMaxState] = useState(() => {
    const v = parseFloat(localStorage.getItem("ui_angular_max") || "1.2");
    return isFinite(v) && v > 0 ? v : 1.2;
  });

  const pubsRef = useRef({}); 
  const subsRef = useRef({}); 

  const ensurePublisher = useCallback((key) => {
    const ros = rosRef.current;
    if (!ros) return null;
    if (pubsRef.current[key]) return pubsRef.current[key];
    const cfg = SYNC_TOPICS[key];
    const topic = new ROSLIB.Topic({
      ros,
      name: cfg.name,
      messageType: cfg.type,
      latch: true,
      queue_size: 1,
    });
    try { topic.advertise(); } catch {}
    pubsRef.current[key] = topic;
    return topic;
  }, []);

  const publishSync = useCallback((key, value) => {
    const p = ensurePublisher(key);
    if (!p) return;
    try { p.publish({ data: value }); }
    catch (e) { console.warn(`[ROSContext] publish ${key} error:`, e); }
  }, [ensurePublisher]);

  // ──────────────────────────────────────────────────────────────────────────
  // SUBSCRIBE
  // ──────────────────────────────────────────────────────────────────────────
  useEffect(() => {
    const ros = rosRef.current;
    if (!ros || !isConnected) return;

    Object.values(subsRef.current).forEach(s => { try { s.unsubscribe(); } catch {} });
    subsRef.current = {};
    Object.values(pubsRef.current).forEach(p => { try { p.unadvertise(); } catch {} });
    pubsRef.current = {};

    try {
      const modSub = new ROSLIB.Topic({
        ros, name: "/mod",
        messageType: "std_msgs/msg/String",
        throttle_rate: 200, queue_length: 1,
      });
      modSub.subscribe((msg) => {
        const val = (msg.data || "").toLowerCase().trim();
        if (["manual", "autonomous", "task"].includes(val)) {
          setOperationModeState(val);
        }
      });
      subsRef.current.mod = modSub;
    } catch (e) { console.warn("[ROSContext] /mod sub error:", e); }

    try {
      const hfSub = new ROSLIB.Topic({
        ros, name: SYNC_TOPICS.humanFollow.name,
        messageType: SYNC_TOPICS.humanFollow.type,
        queue_length: 1,
      });
      hfSub.subscribe((msg) => {
        setHumanFollowEnabledState(!!msg.data);
      });
      subsRef.current.humanFollow = hfSub;
    } catch (e) { console.warn("[ROSContext] humanFollow sub error:", e); }

    // Yeni Kamera State Dinleyicisi
    try {
      const camSub = new ROSLIB.Topic({
        ros, name: "/camera/state",
        messageType: "std_msgs/msg/Bool",
        queue_length: 1,
      });
      camSub.subscribe((msg) => {
        setCameraEnabled(!!msg.data);
      });
      subsRef.current.cameraState = camSub;
    } catch (e) { console.warn("[ROSContext] cameraState sub error:", e); }

    try {
      const esSub = new ROSLIB.Topic({
        ros, name: SYNC_TOPICS.emergency.name,
        messageType: SYNC_TOPICS.emergency.type,
        queue_length: 1,
      });
      esSub.subscribe((msg) => {
        const active = msg.data === true || msg.data === 1 || msg.data === "1";
        setEstopState(active);
      });
      subsRef.current.estop = esSub;
    } catch (e) { console.warn("[ROSContext] estop sub error:", e); }

    try {
      const smSub = new ROSLIB.Topic({
        ros, name: SYNC_TOPICS.speedMode.name,
        messageType: SYNC_TOPICS.speedMode.type,
        queue_length: 1,
      });
      smSub.subscribe((msg) => {
        const v = parseInt(msg.data, 10);
        if ([1, 2, 3, 4].includes(v)) {
          setSpeedModeState(v);
          try { localStorage.setItem("adaptive_velocity_mode", String(v)); } catch {}
        }
      });
      subsRef.current.speedMode = smSub;
    } catch (e) { console.warn("[ROSContext] speedMode sub error:", e); }

    try {
      const lmSub = new ROSLIB.Topic({
        ros, name: SYNC_TOPICS.linearMax.name,
        messageType: SYNC_TOPICS.linearMax.type,
        queue_length: 1,
      });
      lmSub.subscribe((msg) => {
        const v = Number(msg.data);
        if (isFinite(v) && v > 0) {
          setLinearMaxState(v);
          try { localStorage.setItem("ui_linear_max", String(v)); } catch {}
        }
      });
      subsRef.current.linearMax = lmSub;
    } catch (e) { console.warn("[ROSContext] linearMax sub error:", e); }

    try {
      const amSub = new ROSLIB.Topic({
        ros, name: SYNC_TOPICS.angularMax.name,
        messageType: SYNC_TOPICS.angularMax.type,
        queue_length: 1,
      });
      amSub.subscribe((msg) => {
        const v = Number(msg.data);
        if (isFinite(v) && v > 0) {
          setAngularMaxState(v);
          try { localStorage.setItem("ui_angular_max", String(v)); } catch {}
        }
      });
      subsRef.current.angularMax = amSub;
    } catch (e) { console.warn("[ROSContext] angularMax sub error:", e); }

    return () => {
      Object.values(subsRef.current).forEach(s => { try { s.unsubscribe(); } catch {} });
      subsRef.current = {};
    };
  }, [rosInstance, isConnected]);

  // ──────────────────────────────────────────────────────────────────────────
  // PUBLIC SETTER'LAR
  // ──────────────────────────────────────────────────────────────────────────

  const cancelNav2Goal = useCallback((ros) => {
    if (!ros) return;
    try {
      const cancelTopic = new ROSLIB.Topic({
        ros, name: "/navigate_to_pose/_action/cancel",
        messageType: "action_msgs/msg/GoalID",
        queue_size: 1,
      });
      cancelTopic.publish({});
      setTimeout(() => { try { cancelTopic.unadvertise(); } catch {} }, 500);
    } catch (e) { console.warn("[ROSContext] Nav2 cancel error:", e); }

    try {
      const cmdTopic = new ROSLIB.Topic({
        ros, name: "/cmd_vel",
        messageType: "geometry_msgs/msg/Twist",
        queue_size: 1,
      });
      cmdTopic.publish({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });
      setTimeout(() => { try { cmdTopic.unadvertise(); } catch {} }, 500);
    } catch {}
  }, []);

  const setOperationMode = useCallback((newMode) => {
    const ros = rosRef.current;
    const prevMode = operationMode;
    setOperationModeState(newMode);

    if (ros && isConnected) {
      try {
        const modTopic = new ROSLIB.Topic({
          ros, name: "/mod",
          messageType: "std_msgs/msg/String",
          queue_size: 1,
          latch: true,
        });
        modTopic.advertise();
        modTopic.publish({ data: newMode });
        setTimeout(() => { try { modTopic.unadvertise(); } catch {} }, 800);
      } catch {}

      if (newMode === "manual" && (prevMode === "autonomous" || prevMode === "task")) {
        cancelNav2Goal(ros);
      }
    }
    console.log(`[ROSContext] Mode: ${prevMode} → ${newMode}`);
  }, [isConnected, operationMode, cancelNav2Goal]);

  const toggleHumanFollow = useCallback(() => {
    const next = !humanFollowEnabled;
    setHumanFollowEnabledState(next);
    if (isConnected) publishSync("humanFollow", next);
    console.log(`[ROSContext] human_follow/enable → ${next}`);
    return next;
  }, [humanFollowEnabled, isConnected, publishSync]);

  // Yeni Kamera Toggle Fonksiyonu
  const toggleCamera = useCallback(() => {
    const ros = rosRef.current;
    const next = !cameraEnabled;
    setCameraEnabled(next);
    
    if (ros && isConnected) {
      try {
        const t = new ROSLIB.Topic({
          ros, name: "/camera/enable",
          messageType: "std_msgs/Bool", queue_size: 1,
        });
        t.publish({ data: next });
        setTimeout(() => { try { t.unadvertise(); } catch {} }, 500);
        
        // Kamera kapatılırken follow da kapansın (UI tarafında da güvence)
        if (!next && humanFollowEnabled) {
          const f = new ROSLIB.Topic({
            ros, name: "/human_follow/enable",
            messageType: "std_msgs/Bool", queue_size: 1,
          });
          f.publish({ data: false });
          setHumanFollowEnabledState(false);
          setTimeout(() => { try { f.unadvertise(); } catch {} }, 500);
        }
      } catch (e) { console.warn("[ROSContext] toggleCamera error:", e); }
    }
    return next;
  }, [cameraEnabled, humanFollowEnabled, isConnected]);

  const setEstop = useCallback((active) => {
    const v = !!active;
    setEstopState(v);
    if (isConnected) publishSync("emergency", v);
  }, [isConnected, publishSync]);

  const setSpeedMode = useCallback((mode) => {
    const m = parseInt(mode, 10);
    if (![1, 2, 3, 4].includes(m)) return;
    setSpeedModeState(m);
    try { localStorage.setItem("adaptive_velocity_mode", String(m)); } catch {}
    if (isConnected) publishSync("speedMode", m);
  }, [isConnected, publishSync]);

  const setLinearMax = useCallback((v) => {
    const num = Number(v);
    if (!isFinite(num) || num <= 0) return;
    setLinearMaxState(num);
    try { localStorage.setItem("ui_linear_max", String(num)); } catch {}
    if (isConnected) publishSync("linearMax", num);
  }, [isConnected, publishSync]);

  const setAngularMax = useCallback((v) => {
    const num = Number(v);
    if (!isFinite(num) || num <= 0) return;
    setAngularMaxState(num);
    try { localStorage.setItem("ui_angular_max", String(num)); } catch {}
    if (isConnected) publishSync("angularMax", num);
  }, [isConnected, publishSync]);

  // ──────────────────────────────────────────────────────────────────────────
  // BAĞLANTI YÖNETİMİ
  // ──────────────────────────────────────────────────────────────────────────
  useEffect(() => { saveUrl(rosbridgeUrl); }, [rosbridgeUrl]);

  const connect = useCallback((url) => {
    if (connectingRef.current) return;

    if (rosRef.current) {
      try { rosRef.current.removeAllListeners(); rosRef.current.close(); } catch {}
      rosRef.current = null;
      setRosInstance((prev) => prev ? null : prev);
      setIsConnected((prev) => prev ? false : prev);
    }

    connectingRef.current = true;
    setStatus((prev) => prev === "Bağlanıyor..." ? prev : "Bağlanıyor...");
    setErrorText((prev) => prev ? "" : prev);

    const ros = new ROSLIB.Ros({ url });

    ros.on("connection", () => {
      if (!mountedRef.current) return;
      console.log("[ROSContext] 🟢 Bağlandı!");
      connectingRef.current = false;
      rosRef.current = ros;
      setRosInstance(ros);
      setIsConnected(true);
      setStatus("Bağlandı");
      setErrorText("");
      if (reconnectTimer.current) { clearTimeout(reconnectTimer.current); reconnectTimer.current = null; }
    });

    ros.on("close", () => {
      if (!mountedRef.current) return;
      connectingRef.current = false;
      rosRef.current = null;
      setRosInstance((prev) => prev ? null : prev);
      setIsConnected((prev) => {
        if (prev) { console.log("[ROSContext] 🔴 Bağlantı koptu"); setStatus("Bağlantı koptu"); }
        return false;
      });
      if (!reconnectTimer.current) {
        reconnectTimer.current = setTimeout(() => {
          reconnectTimer.current = null;
          if (mountedRef.current) connect(url);
        }, 5000);
      }
    });

    ros.on("error", (e) => {
      if (!mountedRef.current) return;
      const msg = e?.message || (e?.type === "error" ? "ROSBridge bağlantısı kurulamadı" : String(e));
      setStatus((prev) => prev === "Bağlantı hatası" ? prev : "Bağlantı hatası");
      setErrorText((prev) => prev === msg ? prev : msg);
    });

    rosRef.current = ros;
  }, []);

  useEffect(() => {
    mountedRef.current = true;
    connect(rosbridgeUrl);
    return () => {
      mountedRef.current = false;
      connectingRef.current = false;
      if (reconnectTimer.current) { clearTimeout(reconnectTimer.current); reconnectTimer.current = null; }
    };
  }, [rosbridgeUrl, connect]);

  useEffect(() => {
    return () => {
      console.log("[ROSContext] Provider unmount");
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);

      Object.values(pubsRef.current).forEach(p => { try { p.unadvertise(); } catch {} });
      pubsRef.current = {};
      Object.values(subsRef.current).forEach(s => { try { s.unsubscribe(); } catch {} });
      subsRef.current = {};

      if (rosRef.current) {
        try { rosRef.current.removeAllListeners(); rosRef.current.close(); } catch {}
      }
    };
  }, []);

  const reconnect = useCallback(() => {
    console.log("[ROSContext] Manuel reconnect");
    if (reconnectTimer.current) { clearTimeout(reconnectTimer.current); reconnectTimer.current = null; }
    if (rosRef.current) {
      try { rosRef.current.removeAllListeners(); rosRef.current.close(); } catch {}
      rosRef.current = null;
      setRosInstance(null);
      setIsConnected(false);
    }
    connectingRef.current = false;
    setTimeout(() => connect(rosbridgeUrl), 300);
  }, [rosbridgeUrl, connect]);

  const value = {
    ros: rosInstance,
    isConnected,
    status,
    errorText,
    rosbridgeUrl,
    setRosbridgeUrl,
    reconnect,

    operationMode,
    setOperationMode,

    humanFollowEnabled,
    toggleHumanFollow,

    cameraEnabled,
    toggleCamera,

    estop,
    setEstop,

    speedMode,
    setSpeedMode,

    linearMax,
    setLinearMax,

    angularMax,
    setAngularMax,
  };

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
}

export function useROS() {
  const ctx = useContext(ROSContext);
  if (!ctx) throw new Error("useROS must be used within ROSProvider");
  return ctx;
}
