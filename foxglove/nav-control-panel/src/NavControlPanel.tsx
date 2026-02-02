import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import * as L from "leaflet";
import { ReactElement, useEffect, useLayoutEffect, useState, useCallback, useRef } from "react";
import { createRoot } from "react-dom/client";

const DEFAULT_DEST_LAT = 38.03683;
const DEFAULT_DEST_LON = -78.503577;
const DEFAULT_LOOKAHEAD = 8.0;
const API_KEY_STORAGE_KEY = "couch_vision_gmaps_api_key";

const LEAFLET_CSS = `
.leaflet-pane,.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-tile-container,
.leaflet-pane>svg,.leaflet-pane>canvas,.leaflet-zoom-box,.leaflet-image-layer,.leaflet-layer{position:absolute;left:0;top:0}
.leaflet-container{overflow:hidden}
.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow{user-select:none;-webkit-user-select:none}
.leaflet-tile{filter:inherit;visibility:hidden}
.leaflet-tile-loaded{visibility:inherit}
.leaflet-zoom-animated{transform-origin:0 0}
.leaflet-tile-container{pointer-events:none}
.leaflet-control{position:relative;z-index:800;pointer-events:auto}
.leaflet-top,.leaflet-bottom{position:absolute;z-index:1000;pointer-events:none}
.leaflet-top{top:0}.leaflet-right{right:0}.leaflet-bottom{bottom:0}.leaflet-left{left:0}
.leaflet-control{float:left;clear:both}
.leaflet-right .leaflet-control{float:right}
.leaflet-top .leaflet-control{margin-top:10px}
.leaflet-bottom .leaflet-control{margin-bottom:10px}
.leaflet-left .leaflet-control{margin-left:10px}
.leaflet-right .leaflet-control{margin-right:10px}
.leaflet-control-zoom-in,.leaflet-control-zoom-out{display:block;width:30px;height:30px;line-height:30px;text-align:center;text-decoration:none;color:#000;background:#fff;border-bottom:1px solid #ccc;font:bold 18px/30px 'Lucida Console',Monaco,monospace;cursor:pointer}
.leaflet-control-zoom-in:hover,.leaflet-control-zoom-out:hover{background:#f4f4f4}
.leaflet-control-zoom{border-radius:5px;box-shadow:0 1px 5px rgba(0,0,0,.4)}
.leaflet-control-zoom-in{border-radius:5px 5px 0 0}
.leaflet-control-zoom-out{border-radius:0 0 5px 5px}
.leaflet-grab{cursor:grab}.leaflet-dragging .leaflet-grab{cursor:grabbing}
.leaflet-pane{z-index:400}.leaflet-tile-pane{z-index:200}.leaflet-overlay-pane{z-index:400}
.leaflet-shadow-pane{z-index:500}.leaflet-marker-pane{z-index:600}
.leaflet-tooltip-pane{z-index:650}.leaflet-popup-pane{z-index:700}
`;

interface NavStatus {
  api_key_configured: boolean;
  route_resolved: boolean;
  current_dest_lat: number;
  current_dest_lon: number;
  route_points: number;
  ekf_initialized: boolean;
}

interface GpsMsg {
  latitude: number;
  longitude: number;
}

interface PanelState {
  destLat: number;
  destLon: number;
  startLat: number;
  startLon: number;
  lookahead: number;
  useGpsStart: boolean;
}

function NavControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>(() => {
    const s = context.initialState as Partial<PanelState> | undefined;
    return {
      destLat: s?.destLat ?? DEFAULT_DEST_LAT,
      destLon: s?.destLon ?? DEFAULT_DEST_LON,
      startLat: s?.startLat ?? DEFAULT_DEST_LAT,
      startLon: s?.startLon ?? DEFAULT_DEST_LON,
      lookahead: s?.lookahead ?? DEFAULT_LOOKAHEAD,
      useGpsStart: s?.useGpsStart ?? true,
    };
  });

  const [apiKey, setApiKey] = useState(() => {
    try {
      return localStorage.getItem(API_KEY_STORAGE_KEY) ?? "";
    } catch {
      return "";
    }
  });
  const [showApiKey, setShowApiKey] = useState(false);
  const [gpsLat, setGpsLat] = useState<number | null>(null);
  const [gpsLon, setGpsLon] = useState<number | null>(null);
  const [navStatus, setNavStatus] = useState<NavStatus | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [goFlash, setGoFlash] = useState(false);

  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const startMarkerRef = useRef<L.Marker | null>(null);
  const destMarkerRef = useRef<L.Marker | null>(null);
  const leafletReady = useRef(false);

  // API key status: backend has it OR panel has it
  const backendHasKey = navStatus?.api_key_configured ?? false;
  const hasApiKey = Boolean(apiKey) || backendHasKey;

  useEffect(() => {
    context.saveState(state);
  }, [context, state]);

  useEffect(() => {
    try {
      localStorage.setItem(API_KEY_STORAGE_KEY, apiKey);
    } catch {
      /* ignore */
    }
  }, [apiKey]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.currentFrame) {
        for (const event of renderState.currentFrame) {
          const ev = event as MessageEvent;
          if (ev.topic === "/gps/fix") {
            const msg = ev.message as GpsMsg;
            setGpsLat(msg.latitude);
            setGpsLon(msg.longitude);
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
    context.subscribe([{ topic: "/gps/fix" }, { topic: "/nav/status" }]);
  }, [context]);

  useLayoutEffect(() => {
    context.advertise?.("/nav/set_destination", "std_msgs/String");
    return () => {
      context.unadvertise?.("/nav/set_destination");
    };
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    if (state.useGpsStart && gpsLat != null && gpsLon != null) {
      setState((s) => ({ ...s, startLat: gpsLat, startLon: gpsLon }));
    }
  }, [state.useGpsStart, gpsLat, gpsLon]);

  // Inject Leaflet CSS
  useEffect(() => {
    const id = "leaflet-inline-css";
    if (!document.getElementById(id)) {
      const style = document.createElement("style");
      style.id = id;
      style.textContent = LEAFLET_CSS;
      document.head.appendChild(style);
    }
  }, []);

  // Initialize Leaflet map (bundled, no CDN)
  useEffect(() => {
    if (!mapContainerRef.current || leafletReady.current) return;
    leafletReady.current = true;

    const center: L.LatLngTuple = [state.startLat, state.startLon];
    const map = L.map(mapContainerRef.current, {
      center,
      zoom: 16,
      zoomControl: true,
      attributionControl: false,
    });

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19,
    }).addTo(map);

    const greenIcon = L.divIcon({
      html: '<div style="width:14px;height:14px;background:#22c55e;border:2px solid #fff;border-radius:50%;box-shadow:0 1px 3px rgba(0,0,0,.4)"></div>',
      iconSize: [14, 14],
      iconAnchor: [7, 7],
      className: "",
    });
    const redIcon = L.divIcon({
      html: '<div style="width:14px;height:14px;background:#ef4444;border:2px solid #fff;border-radius:50%;box-shadow:0 1px 3px rgba(0,0,0,.4)"></div>',
      iconSize: [14, 14],
      iconAnchor: [7, 7],
      className: "",
    });

    const startMarker = L.marker(center, { draggable: true, icon: greenIcon }).addTo(map);
    const destMarker = L.marker([state.destLat, state.destLon], {
      draggable: true,
      icon: redIcon,
    }).addTo(map);

    startMarker.on("dragend", () => {
      const pos = startMarker.getLatLng();
      setState((s) => ({ ...s, startLat: pos.lat, startLon: pos.lng, useGpsStart: false }));
    });
    destMarker.on("dragend", () => {
      const pos = destMarker.getLatLng();
      setState((s) => ({ ...s, destLat: pos.lat, destLon: pos.lng }));
    });

    map.on("click", (e: L.LeafletMouseEvent) => {
      const { lat, lng } = e.latlng;
      if (e.originalEvent.shiftKey) {
        setState((s) => ({ ...s, startLat: lat, startLon: lng, useGpsStart: false }));
        startMarker.setLatLng([lat, lng]);
      } else {
        setState((s) => ({ ...s, destLat: lat, destLon: lng }));
        destMarker.setLatLng([lat, lng]);
      }
    });

    map.on("contextmenu", (e: L.LeafletMouseEvent) => {
      e.originalEvent.preventDefault();
      const { lat, lng } = e.latlng;
      setState((s) => ({ ...s, startLat: lat, startLon: lng, useGpsStart: false }));
      startMarker.setLatLng([lat, lng]);
    });

    mapRef.current = map;
    startMarkerRef.current = startMarker;
    destMarkerRef.current = destMarker;

    const bounds = L.latLngBounds([center, [state.destLat, state.destLon]]);
    map.fitBounds(bounds.pad(0.3));
  }, []);

  // Sync markers
  useEffect(() => {
    startMarkerRef.current?.setLatLng([state.startLat, state.startLon]);
  }, [state.startLat, state.startLon]);
  useEffect(() => {
    destMarkerRef.current?.setLatLng([state.destLat, state.destLon]);
  }, [state.destLat, state.destLon]);

  // Center map on first GPS
  const hasCentered = useRef(false);
  useEffect(() => {
    if (gpsLat != null && gpsLon != null && mapRef.current && !hasCentered.current) {
      hasCentered.current = true;
      const bounds = L.latLngBounds([gpsLat, gpsLon], [state.destLat, state.destLon]);
      mapRef.current.fitBounds(bounds.pad(0.3));
    }
  }, [gpsLat, gpsLon, state.destLat, state.destLon]);

  const handleGo = useCallback(() => {
    const payload: Record<string, unknown> = {
      dest_lat: state.destLat,
      dest_lon: state.destLon,
      lookahead_m: state.lookahead,
    };
    if (!state.useGpsStart) {
      payload.start_lat = state.startLat;
      payload.start_lon = state.startLon;
    }
    if (apiKey) {
      payload.api_key = apiKey;
    }
    context.publish?.("/nav/set_destination", { data: JSON.stringify(payload) });
    setGoFlash(true);
    setTimeout(() => setGoFlash(false), 1500);
  }, [context, state, apiKey]);

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
      {/* API Key — optional override */}
      <div style={sectionStyle}>
        <div style={{ display: "flex", gap: "4px", alignItems: "center" }}>
          <input
            type={showApiKey ? "text" : "password"}
            value={apiKey}
            onChange={(e) => setApiKey(e.target.value)}
            placeholder={backendHasKey ? "Using backend key" : "Google Maps API key..."}
            style={{ ...inputStyle, flex: 1, opacity: backendHasKey && !apiKey ? 0.5 : 1 }}
          />
          <button
            onClick={() => setShowApiKey(!showApiKey)}
            style={{
              padding: "4px 6px",
              fontSize: "10px",
              background: "#2a2a3e",
              color: "#ccc",
              border: "1px solid #333",
              borderRadius: "3px",
              cursor: "pointer",
            }}
          >
            {showApiKey ? "Hide" : "Show"}
          </button>
        </div>
        {!hasApiKey && (
          <div style={{ color: "#f59e0b", fontSize: "11px", marginTop: "2px" }}>
            API key required for Google Maps routing
          </div>
        )}
      </div>

      {/* Start */}
      <div style={sectionStyle}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <label style={{ ...labelStyle, marginBottom: 0 }}>Start</label>
          <button
            onClick={() => {
              if (gpsLat != null && gpsLon != null)
                setState((s) => ({ ...s, startLat: gpsLat, startLon: gpsLon, useGpsStart: true }));
            }}
            style={{
              padding: "2px 6px",
              fontSize: "10px",
              background: state.useGpsStart ? "#166534" : "#2a2a3e",
              color: state.useGpsStart ? "#86efac" : "#999",
              border: "1px solid #333",
              borderRadius: "3px",
              cursor: "pointer",
            }}
          >
            {state.useGpsStart ? "Using GPS" : "Use GPS"}
          </button>
        </div>
        <div style={{ display: "flex", gap: "4px", marginTop: "2px" }}>
          <input
            type="number"
            step="0.000001"
            value={state.startLat}
            onChange={(e) =>
              setState((s) => ({
                ...s,
                startLat: parseFloat(e.target.value) || 0,
                useGpsStart: false,
              }))
            }
            style={inputStyle}
            placeholder="Latitude"
          />
          <input
            type="number"
            step="0.000001"
            value={state.startLon}
            onChange={(e) =>
              setState((s) => ({
                ...s,
                startLon: parseFloat(e.target.value) || 0,
                useGpsStart: false,
              }))
            }
            style={inputStyle}
            placeholder="Longitude"
          />
        </div>
      </div>

      {/* Destination */}
      <div style={sectionStyle}>
        <label style={labelStyle}>Destination</label>
        <div style={{ display: "flex", gap: "4px" }}>
          <input
            type="number"
            step="0.000001"
            value={state.destLat}
            onChange={(e) => setState((s) => ({ ...s, destLat: parseFloat(e.target.value) || 0 }))}
            style={inputStyle}
            placeholder="Latitude"
          />
          <input
            type="number"
            step="0.000001"
            value={state.destLon}
            onChange={(e) => setState((s) => ({ ...s, destLon: parseFloat(e.target.value) || 0 }))}
            style={inputStyle}
            placeholder="Longitude"
          />
        </div>
      </div>

      {/* Map */}
      <div
        style={{
          flex: 1,
          minHeight: "180px",
          marginBottom: "6px",
          display: "flex",
          flexDirection: "column",
        }}
      >
        <div
          ref={mapContainerRef}
          style={{
            width: "100%",
            flex: 1,
            minHeight: "160px",
            borderRadius: "4px",
            border: "1px solid #333",
          }}
        />
        <div style={{ fontSize: "9px", color: "#555", marginTop: "2px", textAlign: "center" }}>
          Click = set dest · Shift+click or right-click = set start · Drag markers to adjust
        </div>
      </div>

      {/* Lookahead + GO */}
      <div style={{ display: "flex", gap: "6px", alignItems: "stretch", marginBottom: "6px" }}>
        <div style={{ width: "80px" }}>
          <label style={labelStyle}>Lookahead</label>
          <input
            type="number"
            step="0.5"
            min="1"
            max="50"
            value={state.lookahead}
            onChange={(e) =>
              setState((s) => ({
                ...s,
                lookahead: parseFloat(e.target.value) || DEFAULT_LOOKAHEAD,
              }))
            }
            style={inputStyle}
          />
        </div>
        <button
          onClick={handleGo}
          style={{
            flex: 1,
            fontSize: "14px",
            fontWeight: "bold",
            background: goFlash ? "#22c55e" : "#166534",
            color: "#fff",
            border: "none",
            borderRadius: "4px",
            cursor: "pointer",
            marginTop: "14px",
            transition: "background 0.3s",
          }}
          title="Send destination to nav2_planner"
        >
          {goFlash ? "Sent!" : "GO"}
        </button>
      </div>

      {/* Status */}
      <div style={{ fontSize: "10px", color: "#666", lineHeight: "1.5" }}>
        {navStatus ? (
          <>
            <div>
              {navStatus.route_resolved
                ? `Route: ${navStatus.route_points} pts`
                : "Route: not resolved"}
              {" · "}
              {navStatus.ekf_initialized ? "EKF: init" : "EKF: waiting"}
              {" · "}
              {navStatus.api_key_configured ? "API: ok" : "API: missing"}
            </div>
            <div>
              Backend dest: {navStatus.current_dest_lat.toFixed(6)},{" "}
              {navStatus.current_dest_lon.toFixed(6)}
              {Math.abs(navStatus.current_dest_lat - state.destLat) > 0.00001 ||
              Math.abs(navStatus.current_dest_lon - state.destLon) > 0.00001 ? (
                <span style={{ color: "#f59e0b" }}> (differs from panel — press GO)</span>
              ) : (
                <span style={{ color: "#22c55e" }}> (in sync)</span>
              )}
            </div>
          </>
        ) : (
          "Waiting for nav2_planner..."
        )}
        {gpsLat != null && (
          <div>
            GPS: {gpsLat.toFixed(6)}, {gpsLon?.toFixed(6)}
          </div>
        )}
      </div>
    </div>
  );
}

export function initNavControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<NavControlPanel context={context} />);
  return () => {
    root.unmount();
  };
}
