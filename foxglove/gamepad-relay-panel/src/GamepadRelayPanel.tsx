import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useRef, useState, useCallback } from "react";
import { createRoot } from "react-dom/client";

const POLL_MS = 50;
const RELAY_URL = "http://127.0.0.1:4201";

interface RelayState {
  controller: string | null;
  connected: boolean;
  axes: number[];
  buttons: boolean[];
  button_names: string[];
  cmd_vel: { lx: number; az: number };
  max_linear: number;
  max_angular: number;
  deadzone: number;
  ble_connected: boolean | null;
  ble_rtt_ms: number | null;
  ble_writes_per_sec: number | null;
  post_failures: number;
}

const labelStyle: React.CSSProperties = {
  fontSize: "10px",
  color: "#8a93a0",
  letterSpacing: ".04em",
  textTransform: "uppercase",
  marginBottom: "6px",
};

function StickCanvas({
  x,
  y,
  size,
  label,
}: {
  x: number;
  y: number;
  size: number;
  label: string;
}): ReactElement {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    // circle outline
    ctx.strokeStyle = "#2a3038";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(w / 2, h / 2, w / 2 - 2, 0, Math.PI * 2);
    ctx.stroke();
    // crosshairs
    ctx.beginPath();
    ctx.moveTo(w / 2, 4);
    ctx.lineTo(w / 2, h - 4);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(4, h / 2);
    ctx.lineTo(w - 4, h / 2);
    ctx.stroke();
    // dot
    const cx = w / 2 + x * (w / 2 - 8);
    const cy = h / 2 + y * (h / 2 - 8);
    ctx.fillStyle = "#3b82f6";
    ctx.beginPath();
    ctx.arc(cx, cy, 6, 0, Math.PI * 2);
    ctx.fill();
  }, [x, y]);

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: "2px" }}>
      <canvas
        ref={canvasRef}
        width={size}
        height={size}
        style={{ background: "#1b1f26", borderRadius: "50%" }}
      />
      <div style={{ fontSize: "9px", color: "#8a93a0" }}>{label}</div>
      <div style={{ fontSize: "9px", fontFamily: "monospace", color: "#c4cad3" }}>
        {x.toFixed(2)}, {y.toFixed(2)}
      </div>
    </div>
  );
}

function GamepadRelayPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<RelayState | null>(null);
  const [error, setError] = useState<string | null>(null);
  const mounted = useRef(true);

  // Signal Foxglove we have no subscriptions but need to render
  useEffect(() => {
    context.onRender = (_, done) => done();
    context.watch("currentFrame");
  }, [context]);

  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);

  const poll = useCallback(async () => {
    try {
      const res = await fetch(`${RELAY_URL}/state`);
      const data = (await res.json()) as RelayState;
      if (mounted.current) {
        setState(data);
        setError(null);
      }
    } catch {
      if (mounted.current) {
        setError("relay offline");
      }
    }
  }, []);

  useEffect(() => {
    poll();
    const interval = setInterval(poll, POLL_MS);
    return () => clearInterval(interval);
  }, [poll]);

  const axes = state?.axes ?? [];
  const buttons = state?.buttons ?? [];
  const names = state?.button_names ?? [];

  const bleColor =
    state?.ble_connected === true ? "#22c55e" : state?.ble_connected === false ? "#ef4444" : "#555";
  const bleText =
    state?.ble_connected === true
      ? `${state.ble_rtt_ms != null ? `${state.ble_rtt_ms.toFixed(0)}ms` : "connected"}`
      : state?.ble_connected === false
        ? "disconnected"
        : "--";

  return (
    <div
      style={{
        height: "100%",
        padding: "8px",
        fontFamily: "-apple-system, system-ui, sans-serif",
        fontSize: "12px",
        color: "#e6e8eb",
        background: "#0b0d10",
        overflow: "auto",
      }}
    >
      {/* Header */}
      <div
        style={{
          fontSize: "10px",
          color: error ? "#ef4444" : state?.connected ? "#22c55e" : "#f59e0b",
          marginBottom: "8px",
        }}
      >
        {error
          ? "Relay offline — run `make teleop` on Mac"
          : state?.connected
            ? state.controller
            : "No controller detected"}
      </div>

      {/* Sticks row */}
      <div
        style={{
          display: "flex",
          justifyContent: "center",
          gap: "16px",
          marginBottom: "8px",
        }}
      >
        <StickCanvas x={axes[0] ?? 0} y={axes[1] ?? 0} size={80} label="LEFT" />
        <StickCanvas x={axes[2] ?? 0} y={axes[3] ?? 0} size={80} label="RIGHT" />
      </div>

      {/* Triggers */}
      <div style={{ display: "flex", gap: "8px", marginBottom: "8px" }}>
        {["L2", "R2"].map((label, i) => {
          const raw = axes[4 + i] ?? -1;
          const pct = Math.max(0, (raw + 1) / 2) * 100;
          return (
            <div key={label} style={{ flex: 1 }}>
              <div style={{ fontSize: "9px", color: "#8a93a0", marginBottom: "2px" }}>{label}</div>
              <div
                style={{
                  height: "6px",
                  background: "#1b1f26",
                  borderRadius: "3px",
                  overflow: "hidden",
                }}
              >
                <div
                  style={{
                    height: "100%",
                    width: `${pct}%`,
                    background: "#3b82f6",
                    transition: "width 0.03s",
                  }}
                />
              </div>
            </div>
          );
        })}
      </div>

      {/* Buttons grid */}
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "repeat(5, 1fr)",
          gap: "3px",
          marginBottom: "8px",
        }}
      >
        {names.map((name, i) => (
          <div
            key={i}
            style={{
              background: buttons[i] ? "#2563eb" : "#1b1f26",
              border: `1px solid ${buttons[i] ? "#3b82f6" : "#2a3038"}`,
              borderRadius: "4px",
              padding: "3px 2px",
              textAlign: "center",
              fontSize: "9px",
              color: buttons[i] ? "#fff" : "#8a93a0",
              transition: "background 0.05s",
            }}
          >
            {name}
          </div>
        ))}
      </div>

      {/* cmd_vel + BLE */}
      <div
        style={{
          background: "#14171c",
          border: "1px solid #232831",
          borderRadius: "6px",
          padding: "8px",
        }}
      >
        <div style={labelStyle}>Output</div>
        <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "2px" }}>
          <span style={{ fontSize: "11px", color: "#8a93a0" }}>linear.x</span>
          <span style={{ fontSize: "11px", fontFamily: "monospace", color: "#c4cad3" }}>
            {(state?.cmd_vel?.lx ?? 0).toFixed(3)} m/s
          </span>
        </div>
        <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "6px" }}>
          <span style={{ fontSize: "11px", color: "#8a93a0" }}>angular.z</span>
          <span style={{ fontSize: "11px", fontFamily: "monospace", color: "#c4cad3" }}>
            {(state?.cmd_vel?.az ?? 0).toFixed(3)} rad/s
          </span>
        </div>
        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            borderTop: "1px solid #232831",
            paddingTop: "4px",
          }}
        >
          <div style={{ display: "flex", alignItems: "center", gap: "4px" }}>
            <div
              style={{
                width: "6px",
                height: "6px",
                borderRadius: "50%",
                background: bleColor,
                boxShadow: state?.ble_connected ? `0 0 3px ${bleColor}` : "none",
              }}
            />
            <span style={{ fontSize: "10px", color: bleColor }}>BLE {bleText}</span>
          </div>
          {state?.ble_writes_per_sec != null && (
            <span style={{ fontSize: "9px", color: "#8a93a0" }}>
              {state.ble_writes_per_sec.toFixed(0)} w/s
            </span>
          )}
          {(state?.post_failures ?? 0) > 0 && (
            <span style={{ fontSize: "9px", color: "#ef4444" }}>{state!.post_failures} fails</span>
          )}
        </div>
      </div>
    </div>
  );
}

export function initGamepadRelayPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GamepadRelayPanel context={context} />);
  return () => {
    root.unmount();
  };
}
