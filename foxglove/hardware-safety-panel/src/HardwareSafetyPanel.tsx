import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useCallback, useRef } from "react";
import { createRoot } from "react-dom/client";

const PUBLISH_RATE_MS = 100;
const DEFAULT_MAX_LINEAR = 0.5;
const DEFAULT_MAX_ANGULAR = 0.5;
const VEL_BAR_MAX_LINEAR = 2.0;
const VEL_BAR_MAX_ANGULAR = 2.0;

interface PanelState {
  eStopped: boolean;
  maxLinearVel: number;
  maxAngularVel: number;
  teleopCollapsed: boolean;
}

interface TwistMsg {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

interface BatteryStateMsg {
  percentage: number;
}

interface NavStatus {
  route_resolved: boolean;
  ekf_initialized: boolean;
  route_points: number;
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
    return {
      eStopped: s?.eStopped ?? true,
      maxLinearVel: s?.maxLinearVel ?? DEFAULT_MAX_LINEAR,
      maxAngularVel: s?.maxAngularVel ?? DEFAULT_MAX_ANGULAR,
      teleopCollapsed: s?.teleopCollapsed ?? false,
    };
  });

  const [currentLinear, setCurrentLinear] = useState(0);
  const [currentAngular, setCurrentAngular] = useState(0);
  const [batteryPct, setBatteryPct] = useState<number | null>(null);
  const [thermalState, setThermalState] = useState<number | null>(null);
  const [navStatus, setNavStatus] = useState<NavStatus | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const keysPressed = useRef(new Set<string>());
  const teleopInterval = useRef<ReturnType<typeof setInterval> | null>(null);
  const joystickInterval = useRef<ReturnType<typeof setInterval> | null>(null);
  const joystickVel = useRef({ linear: 0, angular: 0 });

  useEffect(() => {
    context.saveState(state);
  }, [context, state]);

  useLayoutEffect(() => {
    context.advertise?.("/cmd_vel", "geometry_msgs/Twist");
    context.advertise?.("/e_stop", "std_msgs/Bool");
    return () => {
      context.unadvertise?.("/cmd_vel");
      context.unadvertise?.("/e_stop");
    };
  }, [context]);

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
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // E-stop deadman: send zeros at 10Hz while stopped
  useEffect(() => {
    if (!state.eStopped) return;
    const interval = setInterval(() => {
      context.publish?.("/cmd_vel", zeroTwist());
    }, PUBLISH_RATE_MS);
    return () => clearInterval(interval);
  }, [context, state.eStopped]);

  const handleEStop = useCallback(() => {
    setState((s) => ({ ...s, eStopped: true }));
    context.publish?.("/e_stop", { data: true });
    context.publish?.("/cmd_vel", zeroTwist());
  }, [context]);

  const handleArm = useCallback(() => {
    setState((s) => ({ ...s, eStopped: false }));
    context.publish?.("/e_stop", { data: false });
  }, [context]);

  useEffect(() => {
    const teleopActive = !state.eStopped && !state.teleopCollapsed;
    if (!teleopActive) {
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
        context.publish?.("/cmd_vel", computeVelocity());
      }, PUBLISH_RATE_MS);
    };

    const stopPublishing = () => {
      if (teleopInterval.current) {
        clearInterval(teleopInterval.current);
        teleopInterval.current = null;
      }
      context.publish?.("/cmd_vel", zeroTwist());
    };

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        e.stopPropagation();
        keysPressed.current.add(key);
        startPublishing();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        keysPressed.current.delete(key);
        if (keysPressed.current.size === 0) {
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
  }, [context, state.eStopped, state.teleopCollapsed, state.maxLinearVel, state.maxAngularVel]);

  const handleJoystickMove = useCallback(
    (nx: number, ny: number) => {
      joystickVel.current = {
        linear: ny * state.maxLinearVel,
        angular: -nx * state.maxAngularVel,
      };
      if (!joystickInterval.current) {
        joystickInterval.current = setInterval(() => {
          context.publish?.("/cmd_vel", {
            linear: { x: joystickVel.current.linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: joystickVel.current.angular },
          });
        }, PUBLISH_RATE_MS);
      }
    },
    [context, state.maxLinearVel, state.maxAngularVel],
  );

  const handleJoystickRelease = useCallback(() => {
    if (joystickInterval.current) {
      clearInterval(joystickInterval.current);
      joystickInterval.current = null;
    }
    joystickVel.current = { linear: 0, angular: 0 };
    context.publish?.("/cmd_vel", zeroTwist());
  }, [context]);

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
            <div style={{ display: "flex", gap: "6px", marginBottom: "4px" }}>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Max Linear (m/s)</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="2.0"
                  value={state.maxLinearVel}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      maxLinearVel: clamp(parseFloat(e.target.value) || 0, 0, 2.0),
                    }))
                  }
                  style={inputStyle}
                  disabled={state.eStopped}
                />
              </div>
              <div style={{ flex: 1 }}>
                <label style={labelStyle}>Max Angular (rad/s)</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="2.0"
                  value={state.maxAngularVel}
                  onChange={(e) =>
                    setState((s) => ({
                      ...s,
                      maxAngularVel: clamp(parseFloat(e.target.value) || 0, 0, 2.0),
                    }))
                  }
                  style={inputStyle}
                  disabled={state.eStopped}
                />
              </div>
            </div>

            <Joystick
              disabled={state.eStopped}
              onMove={handleJoystickMove}
              onRelease={handleJoystickRelease}
            />

            <div style={{ fontSize: "9px", color: "#555", textAlign: "center", marginTop: "2px" }}>
              WASD keys or drag joystick
            </div>

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
          maxScale={VEL_BAR_MAX_LINEAR}
          unit="m/s"
        />
        <VelocityBar
          label="Angular"
          value={currentAngular}
          maxScale={VEL_BAR_MAX_ANGULAR}
          unit="rad/s"
          centered
        />
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
        </div>
        <div>
          {navStatus ? (
            <>
              {navStatus.route_resolved
                ? `Route: ${navStatus.route_points} pts`
                : "Route: not resolved"}
              {" · "}
              {navStatus.ekf_initialized ? "EKF: init" : "EKF: waiting"}
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
