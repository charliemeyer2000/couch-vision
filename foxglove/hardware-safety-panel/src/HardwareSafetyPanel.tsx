import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useCallback, useRef } from "react";
import { createRoot } from "react-dom/client";

const PUBLISH_RATE_MS = 100;
const HEARTBEAT_RATE_MS = 500;
const DEFAULT_MAX_LINEAR = 0.5;
const DEFAULT_MAX_ANGULAR = 2.0;
const VEL_BAR_MAX_LINEAR = 2.0;
const VEL_BAR_MAX_ANGULAR = 4.0;
const GAMEPAD_DEADZONE = 0.15;

type ControlMode = "gamepad" | "wasd" | "nav2";
type ActiveSource = "keyboard" | "joystick" | "gamepad" | "none";

const VALID_MODES: ControlMode[] = ["gamepad", "wasd", "nav2"];

interface PanelState {
  eStopped: boolean;
  maxLinearVel: number;
  maxAngularVel: number;
  teleopCollapsed: boolean;
  motorCollapsed: boolean;
  motorMode: ControlMode;
  maxRpm: number;
  stopRpm: number;
  rampUpRpmPerSec: number;
  rampDownRpmPerSec: number;
  brakeCurrent: number;
  pfLinearSpeed: number;
  pfLookahead: number;
  pfGoalTolerance: number;
  bleRelayEnabled: boolean;
}

interface TwistMsg {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

interface BatteryStateMsg {
  percentage: number;
}

interface VescBatteryMsg {
  voltage: number;
  current: number;
  temperature: number;
  present: boolean;
}

interface NavStatus {
  route_resolved: boolean;
  ekf_initialized: boolean;
  route_points: number;
  e_stopped?: boolean;
}

interface PathFollowerStatus {
  active: boolean;
  nav2_mode: boolean;
  has_path: boolean;
  has_pose: boolean;
  goal_reached: boolean;
  e_stopped: boolean;
  path_points: number;
  dist_to_goal?: number;
  linear_speed: number;
  max_angular_vel: number;
}

interface MotorStatus {
  connected: boolean;
  mode: string;
  e_stopped: boolean;
  rpm: number;
  erpm: number;
  temp_fet: number;
  temp_motor: number;
  current_motor: number;
  current_input: number;
  voltage_input: number;
  duty_cycle: number;
  fault_code: number;
  fault_name: string;
  target_rpm: number;
  max_rpm: number;
  commanded_rpm: number;
  commanded_erpm: number;
  errors: number;
}

const inputStyle: React.CSSProperties = {
  width: "100%",
  padding: "4px 6px",
  fontSize: "12px",
  background: "#1a1a2e",
  color: "#e0e0e0",
  border: "1px solid #333",
  borderRadius: "3px",
  boxSizing: "border-box",
};

const labelStyle: React.CSSProperties = {
  fontSize: "11px",
  color: "#999",
  marginBottom: "2px",
  display: "block",
};

const sectionStyle: React.CSSProperties = { marginBottom: "6px" };

function zeroTwist(): TwistMsg {
  return { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
}

function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}

function VelocityBar({
  label,
  value,
  maxScale,
  unit,
  centered,
}: {
  label: string;
  value: number;
  maxScale: number;
  unit: string;
  centered?: boolean;
}): ReactElement {
  const pct = Math.min(Math.abs(value) / maxScale, 1.0) * 100;
  const positive = value >= 0;

  const barOuter: React.CSSProperties = {
    width: "100%",
    height: "14px",
    background: "#1a1a2e",
    border: "1px solid #333",
    borderRadius: "3px",
    position: "relative",
    overflow: "hidden",
  };

  const barFill: React.CSSProperties = centered
    ? {
        position: "absolute",
        top: 0,
        height: "100%",
        width: `${pct / 2}%`,
        left: positive ? "50%" : `${50 - pct / 2}%`,
        background: positive ? "#22c55e" : "#3b82f6",
        transition: "width 0.05s, left 0.05s",
      }
    : {
        position: "absolute",
        top: 0,
        left: 0,
        height: "100%",
        width: `${pct}%`,
        background: positive ? "#22c55e" : "#ef4444",
        transition: "width 0.05s",
      };

  const centerLine: React.CSSProperties = {
    position: "absolute",
    left: "50%",
    top: 0,
    bottom: 0,
    width: "1px",
    background: "#555",
  };

  return (
    <div style={{ marginBottom: "4px" }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
        <span style={{ ...labelStyle, marginBottom: 0 }}>{label}</span>
        <span style={{ fontSize: "11px", color: "#e0e0e0", fontFamily: "monospace" }}>
          {value.toFixed(2)} {unit}
        </span>
      </div>
      <div style={barOuter}>
        {centered && <div style={centerLine} />}
        <div style={barFill} />
      </div>
    </div>
  );
}

function Joystick({
  disabled,
  onMove,
  onRelease,
}: {
  disabled: boolean;
  onMove: (x: number, y: number) => void;
  onRelease: () => void;
}): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState({ x: 0, y: 0 });
  const dragging = useRef(false);

  const computeNormalized = useCallback((clientX: number, clientY: number) => {
    const el = containerRef.current;
    if (!el) return { nx: 0, ny: 0 };
    const rect = el.getBoundingClientRect();
    const nx = clamp(((clientX - rect.left) / rect.width) * 2 - 1, -1, 1);
    const ny = clamp(-(((clientY - rect.top) / rect.height) * 2 - 1), -1, 1);
    return { nx, ny };
  }, []);

  useEffect(() => {
    if (disabled) return;

    const handleMouseMove = (e: MouseEvent) => {
      if (!dragging.current) return;
      const { nx, ny } = computeNormalized(e.clientX, e.clientY);
      setPos({ x: nx, y: ny });
      onMove(nx, ny);
    };

    const handleMouseUp = () => {
      if (!dragging.current) return;
      dragging.current = false;
      setPos({ x: 0, y: 0 });
      onRelease();
    };

    window.addEventListener("mousemove", handleMouseMove);
    window.addEventListener("mouseup", handleMouseUp);
    return () => {
      window.removeEventListener("mousemove", handleMouseMove);
      window.removeEventListener("mouseup", handleMouseUp);
    };
  }, [disabled, computeNormalized, onMove, onRelease]);

  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      if (disabled) return;
      dragging.current = true;
      const { nx, ny } = computeNormalized(e.clientX, e.clientY);
      setPos({ x: nx, y: ny });
      onMove(nx, ny);
    },
    [disabled, computeNormalized, onMove],
  );

  const dotX = ((pos.x + 1) / 2) * 100;
  const dotY = ((1 - pos.y) / 2) * 100;

  return (
    <div
      ref={containerRef}
      onMouseDown={handleMouseDown}
      style={{
        width: "100%",
        aspectRatio: "1",
        maxWidth: "140px",
        margin: "4px auto",
        background: disabled ? "#111" : "#1a1a2e",
        border: "1px solid #333",
        borderRadius: "8px",
        position: "relative",
        cursor: disabled ? "not-allowed" : "crosshair",
        opacity: disabled ? 0.4 : 1,
      }}
    >
      <div
        style={{
          position: "absolute",
          left: "50%",
          top: "8px",
          bottom: "8px",
          width: "1px",
          background: "#333",
        }}
      />
      <div
        style={{
          position: "absolute",
          top: "50%",
          left: "8px",
          right: "8px",
          height: "1px",
          background: "#333",
        }}
      />
      <div
        style={{
          position: "absolute",
          width: "16px",
          height: "16px",
          borderRadius: "50%",
          background: disabled ? "#555" : "#22c55e",
          border: "2px solid #fff",
          boxShadow: "0 1px 3px rgba(0,0,0,0.4)",
          left: `calc(${dotX}% - 8px)`,
          top: `calc(${dotY}% - 8px)`,
          transition: dragging.current ? "none" : "left 0.1s, top 0.1s",
          pointerEvents: "none",
        }}
      />
      <span
        style={{
          position: "absolute",
          top: "2px",
          left: "50%",
          transform: "translateX(-50%)",
          fontSize: "8px",
          color: "#555",
        }}
      >
        FWD
      </span>
      <span
        style={{
          position: "absolute",
          bottom: "2px",
          left: "50%",
          transform: "translateX(-50%)",
          fontSize: "8px",
          color: "#555",
        }}
      >
        REV
      </span>
    </div>
  );
}

function HardwareSafetyPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>(() => {
    const s = context.initialState as Partial<PanelState> | undefined;
    const savedMode = s?.motorMode as string | undefined;
    const validMode = VALID_MODES.includes(savedMode as ControlMode)
      ? (savedMode as ControlMode)
      : "gamepad";
    return {
      eStopped: s?.eStopped ?? true,
      maxLinearVel: s?.maxLinearVel ?? DEFAULT_MAX_LINEAR,
      maxAngularVel: s?.maxAngularVel ?? DEFAULT_MAX_ANGULAR,
      teleopCollapsed: s?.teleopCollapsed ?? false,
      motorCollapsed: s?.motorCollapsed ?? false,
      motorMode: validMode,
      maxRpm: s?.maxRpm ?? 500,
      stopRpm: s?.stopRpm ?? 50,
      rampUpRpmPerSec: s?.rampUpRpmPerSec ?? 500,
      rampDownRpmPerSec: s?.rampDownRpmPerSec ?? 500,
      brakeCurrent: s?.brakeCurrent ?? 0.0,
      pfLinearSpeed: s?.pfLinearSpeed ?? 0.3,
      pfLookahead: s?.pfLookahead ?? 1.5,
      pfGoalTolerance: s?.pfGoalTolerance ?? 0.5,
      bleRelayEnabled: s?.bleRelayEnabled ?? true,
    };
  });

  const [activeSource, setActiveSourceState] = useState<ActiveSource>("none");
  const activeSourceRef = useRef<ActiveSource>("none");
  const setActiveSource = useCallback((src: ActiveSource) => {
    activeSourceRef.current = src;
    setActiveSourceState(src);
  }, []);

  const [currentLinear, setCurrentLinear] = useState(0);
  const [currentAngular, setCurrentAngular] = useState(0);
  const [batteryPct, setBatteryPct] = useState<number | null>(null);
  const [thermalState, setThermalState] = useState<number | null>(null);
  const [navStatus, setNavStatus] = useState<NavStatus | null>(null);
  const [motorStatus, setMotorStatus] = useState<MotorStatus | null>(null);
  const [vescBattery, setVescBattery] = useState<VescBatteryMsg | null>(null);
  const [pathFollowerStatus, setPathFollowerStatus] = useState<PathFollowerStatus | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const [gamepadAxes, setGamepadAxes] = useState<number[]>([]);
  const [gamepadMapping, setGamepadMapping] = useState<string>("");
  const [bleConnected, setBleConnected] = useState(false);
  const bleFetchFails = useRef(0);
  const [coastFactor, setCoastFactor] = useState(0.0);
  const prevCoastRef = useRef(0.0);

  const BLE_RELAY_URL = "http://127.0.0.1:4200";

  const keysPressed = useRef(new Set<string>());
  const teleopInterval = useRef<ReturnType<typeof setInterval> | null>(null);
  const joystickInterval = useRef<ReturnType<typeof setInterval> | null>(null);
  const joystickVel = useRef({ linear: 0, angular: 0 });
  const gamepadButtons = useRef<boolean[]>([]);

  useEffect(() => {
    context.saveState(state);
  }, [context, state]);

  // BLE health poll — detect recovery after failover
  useEffect(() => {
    if (!state.bleRelayEnabled) {
      setBleConnected(false);
      return;
    }
    const checkHealth = async () => {
      try {
        const res = await fetch(`${BLE_RELAY_URL}/status`);
        const data = await res.json();
        const wasConnected = bleConnected;
        setBleConnected(data.connected);
        if (data.connected) bleFetchFails.current = 0;
        if (data.connected && !wasConnected) {
          console.log("[BLE] Relay connected — switching to BLE fast path");
        }
      } catch {
        setBleConnected(false);
      }
    };
    checkHealth();
    const interval = setInterval(checkHealth, 2000);
    return () => clearInterval(interval);
  }, [state.bleRelayEnabled, bleConnected, BLE_RELAY_URL]);

  // Exclusive-mode cmd_vel publisher
  const bleActive = state.bleRelayEnabled && bleConnected;
  const publishCmdVel = useCallback(
    (twist: TwistMsg) => {
      if (bleActive) {
        // BLE exclusive mode — update velocity bars locally
        setCurrentLinear(twist.linear.x);
        setCurrentAngular(twist.angular.z);
        fetch(`${BLE_RELAY_URL}/cmd_vel`, {
          method: "POST",
          body: JSON.stringify({ lx: twist.linear.x, az: twist.angular.z }),
          headers: { "Content-Type": "application/json" },
        })
          .then(() => {
            bleFetchFails.current = 0;
          })
          .catch(() => {
            bleFetchFails.current += 1;
            if (bleFetchFails.current >= 3) {
              console.warn("[BLE] 3 consecutive failures — falling back to WS");
              setBleConnected(false);
            }
          });
      } else {
        // WS mode — normal Foxglove publish
        context.publish?.("/cmd_vel", twist);
      }
    },
    [bleActive, context, BLE_RELAY_URL],
  );

  // E-stop: always send on both paths for safety
  const publishEStop = useCallback(
    (stop: boolean) => {
      context.publish?.("/e_stop", { data: stop });
      if (state.bleRelayEnabled) {
        fetch(`${BLE_RELAY_URL}/e_stop`, {
          method: "POST",
          body: JSON.stringify({ stop }),
          headers: { "Content-Type": "application/json" },
        }).catch(() => {});
      }
    },
    [context, state.bleRelayEnabled, BLE_RELAY_URL],
  );

  useLayoutEffect(() => {
    context.advertise?.("/cmd_vel", "geometry_msgs/Twist");
    context.advertise?.("/e_stop", "std_msgs/Bool");
    context.advertise?.("/motor/config", "std_msgs/String");
    context.advertise?.("/teleop/status", "std_msgs/String");
    context.advertise?.("/nav/path_follower/config", "std_msgs/String");
    return () => {
      context.unadvertise?.("/cmd_vel");
      context.unadvertise?.("/e_stop");
      context.unadvertise?.("/motor/config");
      context.unadvertise?.("/teleop/status");
      context.unadvertise?.("/nav/path_follower/config");
    };
  }, [context]);

  const initialPublished = useRef(false);
  useEffect(() => {
    if (!initialPublished.current) {
      initialPublished.current = true;
      context.publish?.("/e_stop", { data: state.eStopped });
    }
  }, [context, state.eStopped]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.currentFrame) {
        for (const event of renderState.currentFrame) {
          const ev = event as MessageEvent;
          if (ev.topic === "/cmd_vel") {
            const msg = ev.message as TwistMsg;
            setCurrentLinear(msg.linear.x);
            setCurrentAngular(msg.angular.z);
          } else if (ev.topic === "/iphone/battery") {
            const msg = ev.message as BatteryStateMsg;
            setBatteryPct(msg.percentage);
          } else if (ev.topic === "/iphone/thermal") {
            const msg = ev.message as { data: number };
            setThermalState(msg.data);
          } else if (ev.topic === "/nav/status") {
            try {
              const data = JSON.parse((ev.message as { data: string }).data) as NavStatus;
              setNavStatus(data);
            } catch {
              /* ignore */
            }
          } else if (ev.topic === "/motor/status") {
            try {
              const data = JSON.parse((ev.message as { data: string }).data) as MotorStatus;
              setMotorStatus(data);
            } catch {
              /* ignore */
            }
          } else if (ev.topic === "/motor/battery") {
            setVescBattery(ev.message as VescBatteryMsg);
          } else if (ev.topic === "/nav/path_follower/status") {
            try {
              const data = JSON.parse((ev.message as { data: string }).data) as PathFollowerStatus;
              setPathFollowerStatus(data);
            } catch {
              /* ignore */
            }
          }
        }
      }
    };
    context.watch("currentFrame");
    context.subscribe([
      { topic: "/cmd_vel" },
      { topic: "/iphone/battery" },
      { topic: "/iphone/thermal" },
      { topic: "/nav/status" },
      { topic: "/motor/status" },
      { topic: "/motor/battery" },
      { topic: "/nav/path_follower/status" },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Publish motor config — always mode: "nav2", no target_rpm
  const publishMotorConfig = useCallback(
    (cf?: number) => {
      context.publish?.("/motor/config", {
        data: JSON.stringify({
          mode: "nav2",
          max_rpm: state.maxRpm,
          stop_rpm: state.stopRpm,
          ramp_up_rpm_s: state.rampUpRpmPerSec,
          ramp_down_rpm_s: state.rampDownRpmPerSec,
          brake_current: state.brakeCurrent,
          max_linear_vel: state.maxLinearVel,
          max_angular_vel: state.maxAngularVel,
          coast_factor: cf ?? prevCoastRef.current,
        }),
      });
    },
    [context, state.maxRpm, state.stopRpm, state.rampUpRpmPerSec, state.rampDownRpmPerSec, state.brakeCurrent, state.maxLinearVel, state.maxAngularVel],
  );
  useEffect(() => {
    publishMotorConfig();
  }, [publishMotorConfig]);

  // Publish path follower config
  useEffect(() => {
    context.publish?.("/nav/path_follower/config", {
      data: JSON.stringify({
        linear_speed: state.pfLinearSpeed,
        lookahead: state.pfLookahead,
        goal_tolerance: state.pfGoalTolerance,
        max_angular_vel: state.maxAngularVel,
      }),
    });
  }, [context, state.pfLinearSpeed, state.pfLookahead, state.pfGoalTolerance, state.maxAngularVel]);

  // E-stop deadman: send zeros at 10Hz while stopped
  useEffect(() => {
    if (!state.eStopped) return;
    const interval = setInterval(() => {
      publishCmdVel(zeroTwist());
    }, PUBLISH_RATE_MS);
    return () => clearInterval(interval);
  }, [state.eStopped, publishCmdVel]);

  // Heartbeat: publish /teleop/status at 2Hz
  useEffect(() => {
    const interval = setInterval(() => {
      context.publish?.("/teleop/status", {
        data: JSON.stringify({
          mode: state.motorMode,
          active_source: activeSourceRef.current,
          gamepad_connected: gamepadConnected,
          e_stopped: state.eStopped,
        }),
      });
    }, HEARTBEAT_RATE_MS);
    return () => clearInterval(interval);
  }, [context, state.motorMode, gamepadConnected, state.eStopped]);

  const handleEStop = useCallback(() => {
    setState((s) => ({ ...s, eStopped: true }));
    publishEStop(true);
    publishCmdVel(zeroTwist());
  }, [publishEStop, publishCmdVel]);

  const handleArm = useCallback(() => {
    setState((s) => ({ ...s, eStopped: false }));
    publishEStop(false);
  }, [publishEStop]);

  const handleModeChange = useCallback(
    (newMode: ControlMode) => {
      publishCmdVel(zeroTwist());
      setActiveSource("none");
      setState((s) => ({ ...s, motorMode: newMode }));
    },
    [publishCmdVel, setActiveSource],
  );

  // WASD keyboard controls — only in WASD mode
  useEffect(() => {
    const active = !state.eStopped && !state.teleopCollapsed && state.motorMode === "wasd";
    if (!active) {
      if (teleopInterval.current) {
        clearInterval(teleopInterval.current);
        teleopInterval.current = null;
      }
      keysPressed.current.clear();
      return;
    }

    const computeVelocity = (): TwistMsg => {
      let linear = 0;
      let angular = 0;
      const keys = keysPressed.current;
      if (keys.has("w")) linear += state.maxLinearVel;
      if (keys.has("s")) linear -= state.maxLinearVel;
      if (keys.has("a")) angular += state.maxAngularVel;
      if (keys.has("d")) angular -= state.maxAngularVel;
      return { linear: { x: linear, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angular } };
    };

    const startPublishing = () => {
      if (teleopInterval.current) return;
      teleopInterval.current = setInterval(() => {
        publishCmdVel(computeVelocity());
      }, PUBLISH_RATE_MS);
    };

    const stopPublishing = () => {
      if (teleopInterval.current) {
        clearInterval(teleopInterval.current);
        teleopInterval.current = null;
      }
      publishCmdVel(zeroTwist());
    };

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        e.stopPropagation();
        keysPressed.current.add(key);
        setActiveSource("keyboard");
        startPublishing();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        keysPressed.current.delete(key);
        if (keysPressed.current.size === 0) {
          setActiveSource("none");
          stopPublishing();
        }
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      stopPublishing();
    };
  }, [
    publishCmdVel,
    state.eStopped,
    state.teleopCollapsed,
    state.motorMode,
    state.maxLinearVel,
    state.maxAngularVel,
    setActiveSource,
  ]);

  const handleJoystickMove = useCallback(
    (nx: number, ny: number) => {
      setActiveSource("joystick");
      joystickVel.current = {
        linear: ny * state.maxLinearVel,
        angular: -nx * state.maxAngularVel,
      };
      if (!joystickInterval.current) {
        joystickInterval.current = setInterval(() => {
          publishCmdVel({
            linear: { x: joystickVel.current.linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: joystickVel.current.angular },
          });
        }, PUBLISH_RATE_MS);
      }
    },
    [publishCmdVel, state.maxLinearVel, state.maxAngularVel, setActiveSource],
  );

  const handleJoystickRelease = useCallback(() => {
    if (joystickInterval.current) {
      clearInterval(joystickInterval.current);
      joystickInterval.current = null;
    }
    joystickVel.current = { linear: 0, angular: 0 };
    publishCmdVel(zeroTwist());
    setActiveSource("none");
  }, [publishCmdVel, setActiveSource]);

  // Gamepad connection tracking
  useEffect(() => {
    const handleConnect = () => setGamepadConnected(true);
    const handleDisconnect = () => {
      setGamepadConnected(false);
      gamepadButtons.current = [];
    };
    window.addEventListener("gamepadconnected", handleConnect);
    window.addEventListener("gamepaddisconnected", handleDisconnect);

    const gamepads = navigator.getGamepads?.();
    if (gamepads) {
      for (const gp of gamepads) {
        if (gp) {
          setGamepadConnected(true);
          break;
        }
      }
    }

    return () => {
      window.removeEventListener("gamepadconnected", handleConnect);
      window.removeEventListener("gamepaddisconnected", handleDisconnect);
    };
  }, []);

  // Gamepad polling — e-stop/arm always active, sticks only in gamepad/wasd mode
  useEffect(() => {
    const stickActive =
      !state.eStopped &&
      !state.teleopCollapsed &&
      (state.motorMode === "gamepad" || state.motorMode === "wasd");

    const applyDeadzone = (v: number): number => {
      if (Math.abs(v) < GAMEPAD_DEADZONE) return 0;
      const sign = v > 0 ? 1 : -1;
      return sign * ((Math.abs(v) - GAMEPAD_DEADZONE) / (1 - GAMEPAD_DEADZONE));
    };

    const interval = setInterval(() => {
      const gps = navigator.getGamepads?.();
      if (!gps) return;

      let gp: Gamepad | null = null;
      for (const g of gps) {
        if (g) {
          gp = g;
          break;
        }
      }
      if (!gp) return;

      // E-stop/arm buttons — always active regardless of mode
      const curr = Array.from(gp.buttons).map((b) => b.pressed);
      const prev = gamepadButtons.current;
      if (curr[1] && !prev[1]) handleEStop();
      if (curr[0] && !prev[0]) handleArm();
      gamepadButtons.current = curr;

      // Always capture axes for debug display
      setGamepadAxes(Array.from(gp.axes));
      setGamepadMapping(gp.mapping);

      // Coast trigger — read LT/RT, take max (either trigger activates coast)
      let lt: number, rt: number;
      if (gp.mapping === "standard") {
        lt = gp.buttons[6]?.value ?? 0;
        rt = gp.buttons[7]?.value ?? 0;
      } else {
        lt = gp.axes.length > 2 ? (gp.axes[2]! + 1) / 2 : 0;
        rt = gp.axes.length > 5 ? (gp.axes[5]! + 1) / 2 : 0;
      }
      const newCoast = Math.max(lt, rt);
      if (Math.abs(newCoast - prevCoastRef.current) > 0.01) {
        prevCoastRef.current = newCoast;
        setCoastFactor(newCoast);
        publishMotorConfig(newCoast);
      }

      if (!stickActive) return;

      const leftY = applyDeadzone(gp.axes[1] ?? 0);
      // Standard mapping: axes[2] = right stick X
      // Non-standard (Linux evdev): axes[3] = right stick X (axes[2] = L2 trigger)
      const rightXIndex = gp.mapping === "standard" ? 2 : 3;
      const rightX =
        gp.axes.length > rightXIndex
          ? applyDeadzone(gp.axes[rightXIndex]!)
          : applyDeadzone(gp.axes[0] ?? 0);

      const hasInput = leftY !== 0 || rightX !== 0;
      if (hasInput) {
        setActiveSource("gamepad");
        publishCmdVel({
          linear: { x: -leftY * state.maxLinearVel, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: -rightX * state.maxAngularVel },
        });
      } else if (activeSourceRef.current === "gamepad") {
        setActiveSource("none");
        publishCmdVel(zeroTwist());
      }
    }, PUBLISH_RATE_MS);

    return () => clearInterval(interval);
  }, [
    publishCmdVel,
    publishMotorConfig,
    state.eStopped,
    state.teleopCollapsed,
    state.motorMode,
    state.maxLinearVel,
    state.maxAngularVel,
    handleEStop,
    handleArm,
    setActiveSource,
  ]);

  const thermalLabel = (val: number | null): { text: string; color: string } => {
    switch (val) {
      case 0:
        return { text: "Nominal", color: "#22c55e" };
      case 1:
        return { text: "Fair", color: "#f59e0b" };
      case 2:
        return { text: "Serious", color: "#f97316" };
      case 3:
        return { text: "Critical", color: "#ef4444" };
      default:
        return { text: "--", color: "#666" };
    }
  };

  const batteryColor = (pct: number | null): string => {
    if (pct == null) return "#666";
    if (pct > 50) return "#22c55e";
    if (pct > 20) return "#f59e0b";
    return "#ef4444";
  };

  const thermal = thermalLabel(thermalState);

  const modeLabel = (mode: ControlMode): string => {
    switch (mode) {
      case "gamepad":
        return "Gamepad";
      case "wasd":
        return "WASD";
      case "nav2":
        return "Nav2";
    }
  };

  const sourceBadge =
    activeSource === "keyboard"
      ? "KB"
      : activeSource === "joystick"
        ? "JOY"
        : activeSource === "gamepad"
          ? "PAD"
          : "idle";

  return (
    <div
      style={{
        height: "100%",
        padding: "8px",
        fontFamily: "system-ui, sans-serif",
        fontSize: "12px",
        color: "#e0e0e0",
        background: "#0d0d1a",
        overflow: "auto",
        display: "flex",
        flexDirection: "column",
      }}
    >
      {/* E-Stop */}
      <div
        style={{
          position: "sticky",
          top: -8,
          zIndex: 10,
          padding: "8px",
          margin: "-8px -8px 6px -8px",
          background: state.eStopped ? "#3b0000" : "#002200",
          borderBottom: `2px solid ${state.eStopped ? "#ef4444" : "#22c55e"}`,
          transition: "background 0.3s",
        }}
      >
        <button
          onClick={handleEStop}
          style={{
            width: "100%",
            padding: "10px",
            fontSize: "14px",
            fontWeight: "bold",
            background: state.eStopped ? "#991b1b" : "#ef4444",
            color: "#fff",
            border: state.eStopped ? "2px solid #ef4444" : "2px solid #dc2626",
            borderRadius: "4px",
            cursor: "pointer",
            letterSpacing: "1px",
          }}
        >
          EMERGENCY STOP
        </button>
        <div style={{ display: "flex", gap: "6px", marginTop: "6px", alignItems: "center" }}>
          <button
            onClick={handleArm}
            disabled={!state.eStopped}
            style={{
              flex: 1,
              padding: "6px",
              fontSize: "11px",
              fontWeight: "bold",
              background: state.eStopped ? "#166534" : "#1a1a2e",
              color: state.eStopped ? "#86efac" : "#555",
              border: "1px solid #333",
              borderRadius: "3px",
              cursor: state.eStopped ? "pointer" : "default",
              opacity: state.eStopped ? 1 : 0.5,
            }}
          >
            ARM MOTORS
          </button>
          <div style={{ display: "flex", alignItems: "center", gap: "4px", fontSize: "11px" }}>
            <div
              style={{
                width: "8px",
                height: "8px",
                borderRadius: "50%",
                background: state.eStopped ? "#ef4444" : "#22c55e",
                boxShadow: `0 0 4px ${state.eStopped ? "#ef4444" : "#22c55e"}`,
              }}
            />
            <span style={{ color: state.eStopped ? "#ef4444" : "#22c55e" }}>
              {state.eStopped ? "STOPPED" : "ARMED"}
            </span>
          </div>
        </div>
      </div>

      {/* Teleop */}
      <div style={sectionStyle}>
        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            cursor: "pointer",
          }}
          onClick={() => setState((s) => ({ ...s, teleopCollapsed: !s.teleopCollapsed }))}
        >
          <label style={{ ...labelStyle, marginBottom: 0, cursor: "pointer" }}>
            Teleop Controls
          </label>
          <span style={{ fontSize: "10px", color: "#666" }}>
            {state.teleopCollapsed ? "\u25b6" : "\u25bc"}
          </span>
        </div>

        {!state.teleopCollapsed && (
          <div style={{ position: "relative", marginTop: "4px" }}>
            {/* Mode selector */}
            <div style={{ display: "flex", gap: "4px", marginBottom: "6px" }}>
              {(["gamepad", "wasd", "nav2"] as const).map((mode) => (
                <button
                  key={mode}
                  onClick={() => handleModeChange(mode)}
                  style={{
                    flex: 1,
                    padding: "5px",
                    fontSize: "11px",
                    fontWeight: "bold",
                    background: state.motorMode === mode ? "#166534" : "#1a1a2e",
                    color: state.motorMode === mode ? "#86efac" : "#888",
                    border: `1px solid ${state.motorMode === mode ? "#22c55e" : "#333"}`,
                    borderRadius: "3px",
                    cursor: "pointer",
                  }}
                >
                  {modeLabel(mode)}
                </button>
              ))}
            </div>

            {/* Active source badge — teleop modes only */}
            {state.motorMode !== "nav2" && (
              <div
                style={{
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  gap: "4px",
                  marginBottom: "4px",
                }}
              >
                <div
                  style={{
                    width: "6px",
                    height: "6px",
                    borderRadius: "50%",
                    background: activeSource === "none" ? "#555" : "#22c55e",
                    boxShadow: activeSource !== "none" ? "0 0 3px #22c55e" : "none",
                  }}
                />
                <span
                  style={{
                    fontSize: "9px",
                    color: activeSource === "none" ? "#555" : "#22c55e",
                  }}
                >
                  {sourceBadge}
                </span>
              </div>
            )}

            {/* Gamepad mode: warning if no gamepad */}
            {state.motorMode === "gamepad" && !gamepadConnected && (
              <div
                style={{
                  padding: "8px",
                  background: "#332b00",
                  border: "1px solid #f59e0b",
                  borderRadius: "3px",
                  marginBottom: "6px",
                  textAlign: "center",
                  fontSize: "11px",
                  color: "#f59e0b",
                }}
              >
                Connect gamepad to drive
              </div>
            )}

            {/* Nav2 mode: path follower status */}
            {state.motorMode === "nav2" && (
              <div
                style={{
                  padding: "8px",
                  fontSize: "11px",
                  color: "#999",
                }}
              >
                {pathFollowerStatus ? (
                  <>
                    <div
                      style={{
                        fontWeight: "bold",
                        color: pathFollowerStatus.active
                          ? "#22c55e"
                          : pathFollowerStatus.goal_reached
                            ? "#3b82f6"
                            : "#f59e0b",
                        marginBottom: "4px",
                      }}
                    >
                      {pathFollowerStatus.active
                        ? "Following path"
                        : pathFollowerStatus.goal_reached
                          ? "Goal reached"
                          : "Waiting for path"}
                    </div>
                    {pathFollowerStatus.dist_to_goal != null && (
                      <div>Goal: {pathFollowerStatus.dist_to_goal.toFixed(1)}m</div>
                    )}
                    <div style={{ marginBottom: "6px" }}>
                      Path: {pathFollowerStatus.path_points} pts ·{" "}
                      Pose: {pathFollowerStatus.has_pose ? "OK" : "NO"}
                    </div>
                  </>
                ) : (
                  <div style={{ textAlign: "center", color: "#555", marginBottom: "6px" }}>
                    Path follower not connected
                  </div>
                )}
                {/* Path follower tuning */}
                <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: "4px" }}>
                  <div>
                    <label style={labelStyle}>Speed (m/s)</label>
                    <input
                      type="number"
                      style={inputStyle}
                      value={state.pfLinearSpeed}
                      step={0.05}
                      min={0.05}
                      onFocus={(e) => e.target.select()}
                      onChange={(e) => {
                        const v = Math.max(0.05, parseFloat(e.target.value) || 0.05);
                        setState((s) => ({ ...s, pfLinearSpeed: v }));
                      }}
                    />
                  </div>
                  <div>
                    <label style={labelStyle}>Lookahead (m)</label>
                    <input
                      type="number"
                      style={inputStyle}
                      value={state.pfLookahead}
                      step={0.1}
                      min={0.3}
                      onFocus={(e) => e.target.select()}
                      onChange={(e) => {
                        const v = Math.max(0.3, parseFloat(e.target.value) || 0.3);
                        setState((s) => ({ ...s, pfLookahead: v }));
                      }}
                    />
                  </div>
                  <div>
                    <label style={labelStyle}>Goal tol (m)</label>
                    <input
                      type="number"
                      style={inputStyle}
                      value={state.pfGoalTolerance}
                      step={0.1}
                      min={0.1}
                      onFocus={(e) => e.target.select()}
                      onChange={(e) => {
                        const v = Math.max(0.1, parseFloat(e.target.value) || 0.1);
                        setState((s) => ({ ...s, pfGoalTolerance: v }));
                      }}
                    />
                  </div>
                </div>
              </div>
            )}

            {/* Velocity inputs + controls — shown in gamepad and wasd modes */}
            {state.motorMode !== "nav2" && (
              <>
                {/* Joystick — only in WASD mode */}
                {state.motorMode === "wasd" && (
                  <Joystick
                    disabled={state.eStopped}
                    onMove={handleJoystickMove}
                    onRelease={handleJoystickRelease}
                  />
                )}

                <div
                  style={{
                    fontSize: "9px",
                    color: "#555",
                    textAlign: "center",
                    marginTop: "2px",
                  }}
                >
                  {state.motorMode === "wasd"
                    ? "WASD keys \u00b7 drag joystick \u00b7 gamepad sticks"
                    : "Use sticks to drive"}
                </div>

                {gamepadConnected && (
                  <div style={{ marginTop: "3px" }}>
                    <div
                      style={{
                        display: "flex",
                        alignItems: "center",
                        justifyContent: "center",
                        gap: "4px",
                      }}
                    >
                      <div
                        style={{
                          width: "6px",
                          height: "6px",
                          borderRadius: "50%",
                          background: "#22c55e",
                          boxShadow: "0 0 3px #22c55e",
                        }}
                      />
                      <span style={{ fontSize: "9px", color: "#22c55e" }}>
                        Gamepad ({gamepadMapping || "non-standard"})
                      </span>
                    </div>
                    {gamepadAxes.length > 0 && (
                      <div
                        style={{
                          fontSize: "9px",
                          color: "#666",
                          fontFamily: "monospace",
                          textAlign: "center",
                          marginTop: "2px",
                          lineHeight: "1.4",
                        }}
                      >
                        {gamepadAxes.slice(0, 6).map((v, i) => {
                          const rightXIndex = gamepadMapping === "standard" ? 2 : 3;
                          const isActive = i === 1 || i === rightXIndex;
                          return (
                            <span
                              key={i}
                              style={{
                                color: isActive ? "#22c55e" : "#555",
                                fontWeight: isActive ? "bold" : "normal",
                              }}
                            >
                              {i > 0 ? " " : ""}
                              {i}:{v.toFixed(2)}
                            </span>
                          );
                        })}
                      </div>
                    )}
                  </div>
                )}
              </>
            )}

            {state.eStopped && (
              <div
                style={{
                  position: "absolute",
                  inset: 0,
                  background: "rgba(239,68,68,0.08)",
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  pointerEvents: "none",
                  borderRadius: "3px",
                }}
              >
                <span style={{ color: "#ef4444", fontSize: "11px", fontWeight: "bold" }}>
                  E-STOP ACTIVE
                </span>
              </div>
            )}
          </div>
        )}
      </div>

      {/* Velocity Monitor */}
      <div style={sectionStyle}>
        <label style={labelStyle}>Velocity Monitor</label>
        <VelocityBar
          label="Linear"
          value={currentLinear}
          maxScale={Math.max(state.maxLinearVel, 0.1)}
          unit="m/s"
        />
        <VelocityBar
          label="Angular"
          value={currentAngular}
          maxScale={Math.max(state.maxAngularVel, 0.1)}
          unit="rad/s"
          centered
        />
        <VelocityBar label="Coast" value={coastFactor} maxScale={1.0} unit="" />
      </div>

      {/* Motor Config */}
      <div style={sectionStyle}>
        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            cursor: "pointer",
          }}
          onClick={() => setState((s) => ({ ...s, motorCollapsed: !s.motorCollapsed }))}
        >
          <label style={{ ...labelStyle, marginBottom: 0, cursor: "pointer" }}>Motor Config</label>
          <span style={{ fontSize: "10px", color: "#666" }}>
            {state.motorCollapsed ? "\u25b6" : "\u25bc"}
          </span>
        </div>

        {!state.motorCollapsed && (
          <div style={{ marginTop: "4px" }}>
            <div style={{ display: "flex", gap: "6px", marginBottom: "6px" }}>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Max RPM</label>
                <input
                  type="number"
                  step="50"
                  min="0"
                  value={state.maxRpm}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      maxRpm: Math.max(0, parseInt(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Ramp ↑ (RPM/s)</label>
                <input
                  type="number"
                  step="50"
                  min="0"
                  value={state.rampUpRpmPerSec}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      rampUpRpmPerSec: Math.max(0, parseInt(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Ramp ↓ (RPM/s)</label>
                <input
                  type="number"
                  step="50"
                  min="0"
                  value={state.rampDownRpmPerSec}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      rampDownRpmPerSec: Math.max(0, parseInt(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
            </div>
            <div style={{ display: "flex", gap: "6px", marginBottom: "4px" }}>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Stop RPM</label>
                <input
                  type="number"
                  step="10"
                  min="0"
                  value={state.stopRpm}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      stopRpm: Math.max(0, parseInt(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Max Linear (m/s)</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  value={state.maxLinearVel}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      maxLinearVel: Math.max(0, parseFloat(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Brake Current (A)</label>
                <input
                  type="number"
                  step="1.0"
                  min="0"
                  value={state.brakeCurrent || 0}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      brakeCurrent: Math.max(0, parseFloat(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
            </div>
            <div style={{ display: "flex", gap: "6px", marginBottom: "6px" }}>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Max Angular (rad/s)</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  value={state.maxAngularVel}
                  onFocus={(e) => e.target.select()}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      maxAngularVel: Math.max(0, parseFloat(e.target.value) || 0),
                    }))
                  }
                  style={inputStyle}
                />
              </div>
              <div style={{ flex: 1 }} />
            </div>

            {/* BLE Fast Path toggle */}
            <div
              style={{
                display: "flex",
                alignItems: "center",
                justifyContent: "space-between",
                padding: "4px 6px",
                background: "#1a1a2e",
                border: `1px solid ${state.bleRelayEnabled ? (bleConnected ? "#22c55e" : "#f59e0b") : "#333"}`,
                borderRadius: "3px",
                marginBottom: "6px",
              }}
            >
              <div style={{ display: "flex", alignItems: "center", gap: "6px" }}>
                <div
                  style={{
                    width: "6px",
                    height: "6px",
                    borderRadius: "50%",
                    background: !state.bleRelayEnabled
                      ? "#555"
                      : bleConnected
                        ? "#22c55e"
                        : "#f59e0b",
                    boxShadow: state.bleRelayEnabled && bleConnected ? "0 0 3px #22c55e" : "none",
                  }}
                />
                <span style={{ fontSize: "11px", color: "#e0e0e0" }}>BLE Fast Path</span>
                {state.bleRelayEnabled && (
                  <span
                    style={{
                      fontSize: "9px",
                      color: bleConnected ? "#22c55e" : "#f59e0b",
                    }}
                  >
                    {bleConnected ? "BLE" : "WS fallback"}
                  </span>
                )}
              </div>
              <button
                onClick={() => setState((s) => ({ ...s, bleRelayEnabled: !s.bleRelayEnabled }))}
                style={{
                  padding: "2px 8px",
                  fontSize: "10px",
                  fontWeight: "bold",
                  background: state.bleRelayEnabled ? "#166534" : "#1a1a2e",
                  color: state.bleRelayEnabled ? "#86efac" : "#888",
                  border: `1px solid ${state.bleRelayEnabled ? "#22c55e" : "#555"}`,
                  borderRadius: "3px",
                  cursor: "pointer",
                }}
              >
                {state.bleRelayEnabled ? "ON" : "OFF"}
              </button>
            </div>

            {/* Motor telemetry */}
            {motorStatus && (
              <div style={{ marginTop: "4px" }}>
                <div
                  style={{
                    display: "flex",
                    alignItems: "center",
                    gap: "4px",
                    marginBottom: "4px",
                  }}
                >
                  <div
                    style={{
                      width: "6px",
                      height: "6px",
                      borderRadius: "50%",
                      background: motorStatus.connected ? "#22c55e" : "#ef4444",
                      boxShadow: `0 0 3px ${motorStatus.connected ? "#22c55e" : "#ef4444"}`,
                    }}
                  />
                  <span style={{ fontSize: "10px", color: "#999" }}>
                    {motorStatus.connected ? "Connected" : "Dry Run"}
                  </span>
                </div>
                {motorStatus.connected ? (
                  <VelocityBar
                    label="RPM"
                    value={motorStatus.rpm}
                    maxScale={state.maxRpm || 500}
                    unit="RPM"
                  />
                ) : (
                  <VelocityBar
                    label="Cmd RPM"
                    value={motorStatus.commanded_rpm}
                    maxScale={state.maxRpm || 500}
                    unit="RPM"
                  />
                )}
                {motorStatus.connected && (
                  <>
                    {vescBattery && (
                      <div
                        style={{
                          display: "flex",
                          justifyContent: "space-between",
                          padding: "4px 6px",
                          background: "#1a1a2e",
                          border: "1px solid #333",
                          borderRadius: "3px",
                          marginTop: "4px",
                          fontFamily: "monospace",
                          fontSize: "11px",
                        }}
                      >
                        <div>
                          <span style={{ color: "#999" }}>V </span>
                          <span style={{ color: "#e0e0e0" }}>
                            {vescBattery.voltage.toFixed(1)}V
                          </span>
                        </div>
                        <div>
                          <span style={{ color: "#999" }}>I </span>
                          <span
                            style={{
                              color:
                                vescBattery.current > 20
                                  ? "#ef4444"
                                  : vescBattery.current > 10
                                    ? "#f59e0b"
                                    : "#22c55e",
                              fontWeight: "bold",
                            }}
                          >
                            {vescBattery.current.toFixed(1)}A
                          </span>
                        </div>
                        <div>
                          <span style={{ color: "#999" }}>P </span>
                          <span style={{ color: "#e0e0e0" }}>
                            {(vescBattery.voltage * vescBattery.current).toFixed(0)}W
                          </span>
                        </div>
                      </div>
                    )}
                    <div
                      style={{
                        display: "flex",
                        gap: "8px",
                        fontSize: "10px",
                        fontFamily: "monospace",
                        color: "#999",
                        marginTop: "2px",
                      }}
                    >
                      <span>
                        FET:{" "}
                        <span style={{ color: motorStatus.temp_fet > 80 ? "#f97316" : "#e0e0e0" }}>
                          {motorStatus.temp_fet.toFixed(1)}C
                        </span>
                      </span>
                      <span>
                        Motor:{" "}
                        <span style={{ color: motorStatus.temp_motor > 100 ? "#ef4444" : "#e0e0e0" }}>
                          {motorStatus.temp_motor.toFixed(1)}C
                        </span>
                      </span>
                      {!vescBattery && (
                        <>
                          <span>{motorStatus.voltage_input.toFixed(1)}V</span>
                          <span>{motorStatus.current_input.toFixed(1)}A</span>
                        </>
                      )}
                    </div>
                  </>
                )}
                {motorStatus.fault_code !== 0 && (
                  <div style={{ fontSize: "10px", color: "#ef4444", marginTop: "2px" }}>
                    FAULT: {motorStatus.fault_name}
                  </div>
                )}
              </div>
            )}
          </div>
        )}
      </div>

      {/* Status */}
      <div style={{ fontSize: "10px", color: "#666", lineHeight: "1.5" }}>
        <div>
          <span>Battery: </span>
          <span style={{ color: batteryColor(batteryPct) }}>
            {batteryPct != null ? `${Math.round(batteryPct * 100)}%` : "--%"}
          </span>
          <span> · Thermal: </span>
          <span style={{ color: thermal.color }}>{thermal.text}</span>
          {vescBattery && (
            <>
              <span> · ESC: </span>
              <span style={{ color: "#e0e0e0" }}>
                {vescBattery.voltage.toFixed(1)}V {vescBattery.current.toFixed(1)}A
              </span>
            </>
          )}
        </div>
        <div>
          {navStatus ? (
            <>
              {navStatus.route_resolved
                ? `Route: ${navStatus.route_points} pts`
                : "Route: not resolved"}
              {" \u00b7 "}
              {navStatus.ekf_initialized ? "EKF: init" : "EKF: waiting"}
              {navStatus.e_stopped != null && (
                <>
                  {" \u00b7 "}
                  <span style={{ color: navStatus.e_stopped ? "#ef4444" : "#22c55e" }}>
                    {navStatus.e_stopped ? "Planner: paused" : "Planner: active"}
                  </span>
                </>
              )}
            </>
          ) : (
            "Waiting for nav status..."
          )}
        </div>
      </div>
    </div>
  );
}

export function initHardwareSafetyPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HardwareSafetyPanel context={context} />);
  return () => {
    root.unmount();
  };
}
