import { useCallback, useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import { BatteryPanel } from "./components/BatteryPanel";
import { MotorHeatPanel } from "./components/MotorHeatPanel";
import { CameraPanel } from "./components/CameraPanel";
import { Charts } from "./components/Charts";
import { EventLog } from "./components/EventLog";
import { LapLog } from "./components/LapLog";
import { LidarCharts } from "./components/LidarCharts";
import { LidarPolar } from "./components/LidarPolar";
import { MapPanel } from "./components/MapPanel";
import { PathInspectorPanel } from "./components/PathInspectorPanel";
import { RawJson } from "./components/RawJson";
import { SpeedPanel } from "./components/SpeedPanel";
import { StatusTiles } from "./components/StatusTiles";
import type {
    BatterySample,
    BatteryState,
    CompressedImage,
    LaserScan,
    LidarSample,
    MotorTemps,
    Odometry,
    OccupancyGrid,
    RacerTelemetry,
    RobotPose,
    RosLogMsg,
    RosPath,
    Sample,
    SpeedSample,
} from "./telemetry";
import {
    BATTERY_BUFFER_LEN,
    BUFFER_LEN,
    EVENT_BUFFER_LEN,
    LIDAR_BUFFER_LEN,
    SPEED_BUFFER_LEN,
    toSample,
} from "./telemetry";
import "./App.css";

const DEFAULT_ROS_URL = "ws://100.86.204.127:9090";
const RECONNECT_MS = 2000;
const TELEMETRY_STALE_MS = 500;

function resolveRosUrl(): string {
    // Priority: ?ros=ws://... in URL > VITE_ROS_URL env > hardcoded fallback.
    // The query param makes it trivial to point a browser tab at a different
    // robot without rebuilding; the env var covers deployments.
    try {
        const fromQuery = new URLSearchParams(window.location.search).get(
            "ros",
        );
        if (fromQuery) return fromQuery;
    } catch {
        /* non-browser env */
    }
    const fromEnv = (import.meta.env.VITE_ROS_URL as string | undefined) ?? "";
    return fromEnv || DEFAULT_ROS_URL;
}

function hasFlag(name: string): boolean {
    try {
        return new URLSearchParams(window.location.search).has(name);
    } catch {
        return false;
    }
}

function App() {
    const [rosUrl] = useState(resolveRosUrl);
    // Bisection toggles: ?nocam, ?noscan, ?notel disable individual subscriptions
    // without a rebuild. Use these to isolate a memory/perf source.
    const [disableCamera] = useState(() => hasFlag("nocam"));
    const [disableScan] = useState(() => hasFlag("noscan"));
    const [disableTelemetry] = useState(() => hasFlag("notel"));
    const [disableLineDebug] = useState(() => hasFlag("nolinedbg"));
    const [connected, setConnected] = useState(false);
    const [latest, setLatest] = useState<RacerTelemetry | null>(null);
    const [samples, setSamples] = useState<Sample[]>([]);
    const [imageArrivedAt, setImageArrivedAt] = useState(0);
    const [imageFormat, setImageFormat] = useState<string | null>(null);
    const [lineDebugArrivedAt, setLineDebugArrivedAt] = useState(0);
    const [lineDebugFormat, setLineDebugFormat] = useState<string | null>(null);
    const [scan, setScan] = useState<LaserScan | null>(null);
    const [scanArrivedAt, setScanArrivedAt] = useState(0);
    const [map, setMap] = useState<OccupancyGrid | null>(null);
    const [mapArrivedAt, setMapArrivedAt] = useState(0);
    const [plannedPath, setPlannedPath] = useState<RosPath | null>(null);
    const [recordedPath, setRecordedPath] = useState<RosPath | null>(null);
    const [inflatedMap, setInflatedMap] = useState<OccupancyGrid | null>(null);
    const [robotPose, setRobotPose] = useState<RobotPose | null>(null);
    const [telemetryArrivedAt, setTelemetryArrivedAt] = useState(0);
    const [now, setNow] = useState(() => Date.now());
    const [lapStartMs, setLapStartMs] = useState<number | null>(null);
    const [lapElapsedMs, setLapElapsedMs] = useState(0);
    const [lapStopped, setLapStopped] = useState(false);
    const [lapTimes, setLapTimes] = useState<number[]>([]);
    const lapStartMsRef = useRef<number | null>(null);
    lapStartMsRef.current = lapStartMs;
    const lapElapsedRef = useRef(0);
    const lapStoppedRef = useRef(false);
    lapStoppedRef.current = lapStopped;
    const motionCountRef = useRef(0);
    const isArmedRef = useRef(false);
    const [battery, setBattery] = useState<BatteryState | null>(null);
    const [batteryArrivedAt, setBatteryArrivedAt] = useState(0);
    const [motorTemps, setMotorTemps] = useState<MotorTemps | null>(null);
    const [motorTempsArrivedAt, setMotorTempsArrivedAt] = useState(0);
    const bufferRef = useRef<Sample[]>([]);
    const batteryBufferRef = useRef<BatterySample[]>([]);
    const [batterySamples, setBatterySamples] = useState<BatterySample[]>([]);
    const lidarBufferRef = useRef<LidarSample[]>([]);
    const [lidarSamples, setLidarSamples] = useState<LidarSample[]>([]);
    const [events, setEvents] = useState<RosLogMsg[]>([]);
    const latestSpeedRef = useRef(0);
    const latestSpeedTargetRef = useRef(0);
    const [speedMeasured, setSpeedMeasured] = useState<number | null>(null);
    const [speedTarget, setSpeedTarget] = useState<number | null>(null);
    const [speedArrivedAt, setSpeedArrivedAt] = useState(0);
    const speedBufferRef = useRef<SpeedSample[]>([]);
    const [speedSamples, setSpeedSamples] = useState<SpeedSample[]>([]);
    const [slamState, setSlamState] = useState<string | null>(null);
    const rosRef = useRef<ROSLIB.Ros | null>(null);
    // Camera frames bypass React state: at 30 Hz, routing ~20 KB Uint8Arrays
    // through setState ramps Chrome's tab memory by tens of MB/s (off-heap,
    // invisible to JS heap snapshots — likely React retaining state snapshots
    // across renders). CameraPanel registers a paint fn here on mount and we
    // call it synchronously from the subscribe callback.
    const cameraPaintRef = useRef<((raw: CompressedImage) => void) | null>(
        null,
    );
    const lineDebugPaintRef = useRef<((raw: CompressedImage) => void) | null>(
        null,
    );

    useEffect(() => {
        const ros = new ROSLIB.Ros({ url: rosUrl });
        rosRef.current = ros;
        let reconnectTimer: number | null = null;
        let disposed = false;

        const scheduleReconnect = () => {
            if (disposed || reconnectTimer !== null) return;
            reconnectTimer = window.setTimeout(() => {
                reconnectTimer = null;
                if (!disposed) ros.connect(rosUrl);
            }, RECONNECT_MS);
        };

        ros.on("connection", () => setConnected(true));
        ros.on("close", () => {
            setConnected(false);
            scheduleReconnect();
        });
        ros.on("error", () => {
            setConnected(false);
            scheduleReconnect();
        });

        const telemetryTopic = disableTelemetry
            ? null
            : new ROSLIB.Topic({
                  ros,
                  name: "/telemetry/racer",
                  messageType: "racer_msgs/msg/RacerTelemetry",
              });
        telemetryTopic?.subscribe((raw) => {
            const msg = raw as unknown as RacerTelemetry;
            if (msg.armed) isArmedRef.current = true;
            setLatest(msg);
            setTelemetryArrivedAt(Date.now());
            const next = bufferRef.current.slice(-(BUFFER_LEN - 1));
            next.push(toSample(msg));
            bufferRef.current = next;
            setSamples(next);
        });

        // Camera on its own topic. Use `cbor` (not `cbor-raw`): we get the full
        // message structure with `data` delivered as a Uint8Array — no base64,
        // no atob, and format/header fields preserved. `cbor-raw` would only
        // ship the raw bytes of a single uint8[] field and drop the envelope.
        const cameraTopic = disableCamera
            ? null
            : new ROSLIB.Topic({
                  ros,
                  name: "/telemetry/camera",
                  messageType: "sensor_msgs/msg/CompressedImage",
                  compression: "cbor",
                  queue_length: 1,
                  throttle_rate: 0,
              });
        cameraTopic?.subscribe((raw) => {
            const msg = raw as unknown as CompressedImage;
            cameraPaintRef.current?.(msg);
            setImageArrivedAt(Date.now());
            setImageFormat((prev) => (prev === msg.format ? prev : msg.format));
        });

        const lineDebugTopic = disableLineDebug
            ? null
            : new ROSLIB.Topic({
                  ros,
                  name: "/telemetry/line_debug",
                  messageType: "sensor_msgs/msg/CompressedImage",
                  compression: "cbor",
                  queue_length: 1,
                  throttle_rate: 0,
              });
        lineDebugTopic?.subscribe((raw) => {
            const msg = raw as unknown as CompressedImage;
            lineDebugPaintRef.current?.(msg);
            setLineDebugArrivedAt(Date.now());
            setLineDebugFormat((prev) =>
                prev === msg.format ? prev : msg.format,
            );
        });

        const scanTopic = disableScan
            ? null
            : new ROSLIB.Topic({
                  ros,
                  name: "/scan",
                  messageType: "sensor_msgs/msg/LaserScan",
                  compression: "cbor",
                  queue_length: 1,
                  throttle_rate: 200,
              });
        scanTopic?.subscribe((raw) => {
            const msg = raw as unknown as LaserScan;
            setScan(msg);
            setScanArrivedAt(Date.now());
            // Compute per-scan stats for time-series charts
            const { ranges, angle_min, angle_increment, range_min, range_max } =
                msg;
            const FWD_HALF = Math.PI / 6; // ±30°
            let totalValid = 0,
                globalMin = Infinity;
            let fwdSum = 0,
                fwdCount = 0;
            for (let i = 0; i < ranges.length; i++) {
                const r = ranges[i];
                if (!Number.isFinite(r) || r < range_min || r > range_max)
                    continue;
                totalValid++;
                if (r < globalMin) globalMin = r;
                const a = angle_min + i * angle_increment;
                if (Math.abs(a) <= FWD_HALF) {
                    fwdSum += r;
                    fwdCount++;
                }
            }
            const lidarNext = lidarBufferRef.current.slice(
                -(LIDAR_BUFFER_LEN - 1),
            );
            lidarNext.push({
                t: Date.now() / 1000,
                minRange: globalMin === Infinity ? 0 : globalMin,
                fwdMean: fwdCount > 0 ? fwdSum / fwdCount : 0,
                validPct: ranges.length > 0 ? totalValid / ranges.length : 0,
            });
            lidarBufferRef.current = lidarNext;
            setLidarSamples(lidarNext);
        });

        const mapTopic = new ROSLIB.Topic({
            ros,
            name: "/map",
            messageType: "nav_msgs/msg/OccupancyGrid",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 1000,
        });
        mapTopic.subscribe((raw) => {
            setMap(raw as unknown as OccupancyGrid);
            setMapArrivedAt(Date.now());
        });

        const plannedPathTopic = new ROSLIB.Topic({
            ros,
            name: "/planned_path",
            messageType: "nav_msgs/msg/Path",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 2000,
        });
        plannedPathTopic.subscribe((raw) => {
            setPlannedPath(raw as unknown as RosPath);
        });

        const recordedPathTopic = new ROSLIB.Topic({
            ros,
            name: "/recorded_path",
            messageType: "nav_msgs/msg/Path",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 1000,
        });
        recordedPathTopic.subscribe((raw) => {
            setRecordedPath(raw as unknown as RosPath);
        });

        const inflatedMapTopic = new ROSLIB.Topic({
            ros,
            name: "/inflated_map",
            messageType: "nav_msgs/msg/OccupancyGrid",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 5000,
        });
        inflatedMapTopic.subscribe((raw) => {
            setInflatedMap(raw as unknown as OccupancyGrid);
        });

        const robotPoseTopic = new ROSLIB.Topic({
            ros,
            name: "/robot_pose_map",
            messageType: "geometry_msgs/msg/PoseStamped",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 100,
        });
        robotPoseTopic.subscribe((raw) => {
            setRobotPose(raw as unknown as RobotPose);
        });

        const odomTopic = new ROSLIB.Topic({
            ros,
            name: "/odom_rf2o",
            messageType: "nav_msgs/msg/Odometry",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 100,
        });
        odomTopic.subscribe((raw) => {
            const msg = raw as unknown as Odometry;
            const vx = msg.twist?.twist?.linear?.x ?? 0;
            // Only start timer once armed + 3 consecutive readings > 0.1 m/s
            if (isArmedRef.current && Math.abs(vx) > 0.1) {
                motionCountRef.current += 1;
                if (
                    motionCountRef.current >= 3 &&
                    lapStartMsRef.current === null
                ) {
                    setLapStartMs(Date.now());
                }
            } else {
                motionCountRef.current = 0;
            }
        });

        const batteryTopic = new ROSLIB.Topic({
            ros,
            name: "/battery_state",
            messageType: "sensor_msgs/msg/BatteryState",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 500,
        });
        batteryTopic.subscribe((raw) => {
            const msg = raw as unknown as BatteryState;
            setBattery(msg);
            setBatteryArrivedAt(Date.now());
            const v =
                Number.isFinite(msg.voltage) && msg.voltage > 0
                    ? msg.voltage
                    : 0;
            const i =
                Number.isFinite(msg.current) && msg.current >= 0
                    ? msg.current
                    : 0;
            const next = batteryBufferRef.current.slice(
                -(BATTERY_BUFFER_LEN - 1),
            );
            next.push({
                t: Date.now() / 1000,
                voltage: v,
                current: i,
                power: v * i,
            });
            batteryBufferRef.current = next;
            setBatterySamples(next);
        });

        const motorTempsTopic = new ROSLIB.Topic({
            ros,
            name: "/motor_temps",
            messageType: "std_msgs/msg/Float32MultiArray",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 500,
        });
        motorTempsTopic.subscribe((raw) => {
            setMotorTemps(raw as unknown as MotorTemps);
            setMotorTempsArrivedAt(Date.now());
        });

        const internalStateTopic = new ROSLIB.Topic({
            ros,
            name: "/telemetry/internal_state",
            messageType: "std_msgs/msg/String",
        });
        internalStateTopic.subscribe((raw) => {
            setSlamState((raw as unknown as { data: string }).data);
        });

        const eventsTopic = new ROSLIB.Topic({
            ros,
            name: "/mavlink_events",
            messageType: "rcl_interfaces/msg/Log",
            compression: "cbor",
            queue_length: 10,
        });
        eventsTopic.subscribe((raw) => {
            const msg = raw as unknown as RosLogMsg;
            setEvents((prev) => {
                const next =
                    prev.length >= EVENT_BUFFER_LEN ? prev.slice(1) : prev;
                return [...next, msg];
            });
        });

        // rover/speed and rover/speed_target — published by rover_node's velocity PI loop.
        // Together they show whether the motor is actually achieving the commanded speed,
        // which is the clearest indicator of battery sag or motor issues.
        const speedTopic = new ROSLIB.Topic({
            ros,
            name: "/rover/speed",
            messageType: "std_msgs/msg/Float32",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 100,
        });
        speedTopic.subscribe((raw) => {
            const v = (raw as unknown as { data: number }).data;
            latestSpeedRef.current = v;
            setSpeedMeasured(v);
            setSpeedArrivedAt(Date.now());
            const next = speedBufferRef.current.slice(-(SPEED_BUFFER_LEN - 1));
            next.push({
                t: Date.now() / 1000,
                measured: v,
                setpoint: latestSpeedTargetRef.current,
            });
            speedBufferRef.current = next;
            setSpeedSamples(next);
        });

        const speedTargetTopic = new ROSLIB.Topic({
            ros,
            name: "/rover/speed_target",
            messageType: "std_msgs/msg/Float32",
            compression: "cbor",
            queue_length: 1,
            throttle_rate: 100,
        });
        speedTargetTopic.subscribe((raw) => {
            const v = (raw as unknown as { data: number }).data;
            latestSpeedTargetRef.current = v;
            setSpeedTarget(v);
        });

        return () => {
            disposed = true;
            rosRef.current = null;
            if (reconnectTimer !== null) window.clearTimeout(reconnectTimer);
            internalStateTopic.unsubscribe();
            telemetryTopic?.unsubscribe();
            cameraTopic?.unsubscribe();
            lineDebugTopic?.unsubscribe();
            scanTopic?.unsubscribe();
            mapTopic.unsubscribe();
            plannedPathTopic.unsubscribe();
            inflatedMapTopic.unsubscribe();
            odomTopic.unsubscribe();
            batteryTopic.unsubscribe();
            motorTempsTopic.unsubscribe();
            eventsTopic.unsubscribe();
            speedTopic.unsubscribe();
            speedTargetTopic.unsubscribe();
            ros.close();
        };
    }, [
        rosUrl,
        disableCamera,
        disableLineDebug,
        disableScan,
        disableTelemetry,
    ]);

    useEffect(() => {
        const id = window.setInterval(() => setNow(Date.now()), 500);
        return () => window.clearInterval(id);
    }, []);

    useEffect(() => {
        if (lapStartMs === null || lapStopped) return;
        const id = window.setInterval(() => {
            const elapsed = Date.now() - lapStartMs;
            lapElapsedRef.current = elapsed;
            setLapElapsedMs(elapsed);
        }, 100);
        return () => window.clearInterval(id);
    }, [lapStartMs, lapStopped]);

    const handleLapToggle = () => {
        if (lapStartMsRef.current === null) return;
        if (lapStoppedRef.current) {
            // Resume: shift start time so elapsed continues from frozen value
            setLapStartMs(Date.now() - lapElapsedRef.current);
            setLapStopped(false);
        } else {
            // Stop: freeze elapsed and record as a split
            const elapsed = Date.now() - lapStartMsRef.current;
            lapElapsedRef.current = elapsed;
            setLapElapsedMs(elapsed);
            setLapStopped(true);
            setLapTimes((prev) => [...prev, elapsed]);
        }
    };

    const handleLapReset = () => {
        setLapStartMs(null);
        setLapElapsedMs(0);
        lapElapsedRef.current = 0;
        setLapStopped(false);
        setLapTimes([]);
    };

    const sendPathToPlanner = useCallback((poses: Array<{ x: number; y: number; qz: number; qw: number }>) => {
        const ros = rosRef.current;
        if (!ros) return;
        const topic = new ROSLIB.Topic({ ros, name: "/dashboard/path_override", messageType: "nav_msgs/msg/Path" });
        topic.publish(new ROSLIB.Message({
            header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
            poses: poses.map((p) => ({
                header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
                pose: {
                    position: { x: p.x, y: p.y, z: 0 },
                    orientation: { x: 0, y: 0, z: p.qz, w: p.qw },
                },
            })),
        }));
    }, []);

    const confirmRace = useCallback(() => {
        const ros = rosRef.current;
        if (!ros) return;
        const topic = new ROSLIB.Topic({ ros, name: "/slam_coordinator/confirm", messageType: "std_msgs/msg/Empty" });
        topic.publish(new ROSLIB.Message({}));
    }, []);

    const slamDisplay = !slamState ? null
        : slamState.startsWith("READY") ? "Ready to race"
        : slamState.startsWith("SAVING") ? "Saving…"
        : slamState === "RACING" ? "Racing"
        : slamState === "mapping" ? null   // normal, don't clutter header
        : slamState;

    const telemetryStale =
        telemetryArrivedAt === 0 ||
        now - telemetryArrivedAt > TELEMETRY_STALE_MS;
    const telemetryAge = telemetryArrivedAt
        ? ((now - telemetryArrivedAt) / 1000).toFixed(1)
        : null;

    return (
        <div className="dashboard">
            <header className="dashboard-header">
                <h1>Racer Telemetry</h1>
                <div className="header-status">
                    {connected && telemetryAge !== null && (
                        <span
                            className={`badge ${telemetryStale ? "badge-stale" : "badge-live"}`}
                        >
                            telemetry · {telemetryAge}s ago
                        </span>
                    )}
                    {slamDisplay && (
                        <span className={`badge ${slamDisplay === "Racing" ? "badge-live" : slamDisplay === "Ready to race" ? "badge-warn" : ""}`}>
                            {slamDisplay}
                        </span>
                    )}
                    {slamState?.startsWith("READY") && (
                        <button className="confirm-race-btn" onClick={confirmRace}>
                            Confirm Race
                        </button>
                    )}
                    <div
                        className={`connection-status ${connected ? "connected" : "disconnected"}`}
                    >
                        <span className="status-dot" />
                        {connected ? "Connected" : "Disconnected"}
                    </div>
                </div>
            </header>

            <StatusTiles
                connected={connected}
                latest={latest}
                lapMs={lapStartMs !== null ? lapElapsedMs : null}
                lapStopped={lapStopped}
                onLapToggle={handleLapToggle}
                onLapReset={handleLapReset}
                voltage={battery?.voltage ?? null}
                speedMeasured={speedMeasured}
                speedTarget={speedTarget}
            />

            <div className="dashboard-grid">
                <MapPanel
                    map={map}
                    arrivedAt={mapArrivedAt}
                    plannedPath={plannedPath}
                    inflatedMap={inflatedMap}
                    recordedPath={plannedPath ? null : recordedPath}
                    robotPose={robotPose}
                />
                <LidarPolar scan={scan} arrivedAt={scanArrivedAt} />
                <LidarCharts lidarSamples={lidarSamples} telSamples={samples} />
                <Charts samples={samples} />
                <LapLog
                    lapTimes={lapTimes}
                    currentMs={lapStartMs !== null ? lapElapsedMs : null}
                    stopped={lapStopped}
                />
                <CameraPanel
                    paintRef={cameraPaintRef}
                    format={imageFormat}
                    arrivedAt={imageArrivedAt}
                />

                <BatteryPanel
                    battery={battery}
                    arrivedAt={batteryArrivedAt}
                    samples={batterySamples}
                />
                <SpeedPanel
                    measured={speedMeasured}
                    setpoint={speedTarget}
                    samples={speedSamples}
                    arrivedAt={speedArrivedAt}
                />
                <EventLog events={events} />
                <MotorHeatPanel
                    temps={motorTemps}
                    arrivedAt={motorTempsArrivedAt}
                />
                <CameraPanel
                    title="Line Debug"
                    paintRef={lineDebugPaintRef}
                    format={lineDebugFormat}
                    arrivedAt={lineDebugArrivedAt}
                />
                <PathInspectorPanel onSendPath={sendPathToPlanner} />
            </div>

            <RawJson msg={latest} />
        </div>
    );
}

export default App;
