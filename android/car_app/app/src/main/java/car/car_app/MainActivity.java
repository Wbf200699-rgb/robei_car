package car.car_app;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Typeface;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.Gravity;
import android.view.HapticFeedbackConstants;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.text.InputType;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.ContextCompat;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.google.android.material.button.MaterialButton;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import car.car_app.bluetooth.BleFrame;
import car.car_app.bluetooth.BluetoothController;
import car.car_app.bluetooth.CarCommand;
import car.car_app.bluetooth.LidarMapAssembler;
import car.car_app.bluetooth.LidarMapChunk;
import car.car_app.bluetooth.LidarMapScan;
import car.car_app.bluetooth.MotionSensorFrame;
import car.car_app.bluetooth.OnboardMapChunk;
import car.car_app.bluetooth.OnboardMapStatus;
import car.car_app.bluetooth.RadarFrame;
import car.car_app.bluetooth.StatusFrame;
import car.car_app.bluetooth.Vl53Frame;
import car.car_app.calibration.MotionCalibrationEngine;
import car.car_app.calibration.MotionCalibrationResult;
import car.car_app.calibration.MotionCalibrationSample;
import car.car_app.calibration.MotionParameterEstimator;
import car.car_app.calibration.RelativeLidarPoseEstimator;
import car.car_app.control.DriveModeTransition;
import car.car_app.data.AppState;
import car.car_app.diagnostics.VehicleAlgorithmSelfCheck;
import car.car_app.geometry.VehicleGeometry;
import car.car_app.localization.Pose2d;
import car.car_app.localization.AnchorLayout;
import car.car_app.localization.AnchorLayoutStore;
import car.car_app.localization.MapWorldRegistration;
import car.car_app.localization.UwbLayoutView;
import car.car_app.localization.UwbPositionFilter;
import car.car_app.map.GridMapView;
import car.car_app.map.MapProfileStore;
import car.car_app.map.OccupancyGridView;
import car.car_app.map.RouteManager;
import car.car_app.navigation.GridPathPlanner;
import car.car_app.navigation.NavigationSafety;
import car.car_app.navigation.NavigationSteering;
import car.car_app.sensors.ImuAttitudeEstimator;
import car.car_app.ui.RadarView;
import car.car_app.uwb.UwbServerClient;
import car.car_app.uwb.UwbServerSnapshot;
import car.car_app.uwb.UwbVehiclePose;
import car.car_app.uwb.UwbEndpointConfig;
import car.car_app.uwb.UwbEndpointStore;
import car.car_app.util.PermissionHelper;

public class MainActivity extends AppCompatActivity {
    private static final String VEHICLE_CH9141_ADDRESS = "DC:32:62:C9:6B:C0";
    private static final long HEARTBEAT_INTERVAL_MS = 1000L;
    private static final long HEARTBEAT_START_DELAY_MS = 1200L;
    private static final long BT_OFF_TIMEOUT_MS = 3000L;
    private static final long HOLD_REPEAT_INTERVAL_MS = 100L;
    private static final long DATA_STALE_TIMEOUT_MS = 12000L;
    private static final long DATA_RECOVERY_INTERVAL_MS = 10000L;
    private static final long MODE_CONFIRM_TIMEOUT_MS = 1800L;
    private static final long UWB_SERVER_STALE_MS = 5000L;
    private static final long MAPPING_POSE_STALE_MS = 1800L;
    private static final float NAVIGATION_TURN_MIN_PROGRESS_DEG = 3f;
    private static final int NAVIGATION_TURN_NO_PROGRESS_LIMIT = 6;
    private static final int UWB_HEADING_INIT_SAMPLES = 3;
    private static final double UWB_HEADING_INIT_MAX_ERROR_DEG = 18.0;
    private static final double UWB_HEADING_CORRECTION_MAX_ERROR_DEG = 35.0;
    private static final double UWB_HEADING_CORRECTION_GAIN = 0.06;
    private static final double UWB_HEADING_MAX_CORRECTION_DEG = 0.75;
    private static final int ONBOARD_MAP_CELL_MM = 50;
    private static final int ONBOARD_MAP_SIZE = 128;
    private static final int ONBOARD_MAP_DESIRED_PADDING_CELLS = 40;
    private static final long ONBOARD_HEADING_STALE_MS = 2500L;
    private static final int GYRO_MISSING_SELF_LIMIT = 16;
    private static final int GYRO_MISSING_PEER_LIMIT = 256;
    private static final int CALIBRATION_FORWARD_MS = 320;
    private static final int AUTO_CALIBRATION_REPEATS = 3;
    // @M2 is now low-rate diagnostic telemetry. The FPGA consumes every fused
    // IMU sample locally, so heading stabilization does not depend on BLE.
    private static final int AUTO_CALIBRATION_TRANSLATION_MS = 650;
    private static final int AUTO_CALIBRATION_TURN_MS = 650;
    private static final long AUTO_CALIBRATION_SETTLE_MS = 650L;
    private static final long AUTO_CALIBRATION_SAMPLE_TIMEOUT_MS = 8000L;
    private static final long AUTO_CALIBRATION_RADAR_STALE_MS = 15000L;
    private static final int[][] DEFAULT_MOTION_PROFILES = {
            {675, 775, 675, 775},
            {775, 600, 800, 600},
            {700, 700, 700, 700}
    };
    private static final String[] LIDAR_SECTOR_LABELS = {
            "前", "右前", "右", "右后", "后", "左后", "左", "左前"
    };
    private static final String[] PARAMETER_LABELS = {
            "整体速度", "路线步长", "左前轮", "右前轮", "左后轮", "右后轮", "安全距离"
    };

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final DriveModeTransition driveModeTransition =
            new DriveModeTransition(MODE_CONFIRM_TIMEOUT_MS);
    private final List<BluetoothDevice> scanDevices = new ArrayList<>();
    private final List<String> scanDeviceLabels = new ArrayList<>();
    private final StringBuilder debugLog = new StringBuilder();
    private final SimpleDateFormat logTimeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.CHINA);
    private final LidarMapAssembler lidarMapAssembler = new LidarMapAssembler();
    private final ImuAttitudeEstimator imu0Attitude = new ImuAttitudeEstimator();
    private final ImuAttitudeEstimator imu1Attitude = new ImuAttitudeEstimator();
    private final RelativeLidarPoseEstimator relativeLidarPoseEstimator =
            new RelativeLidarPoseEstimator();
    private final MotionCalibrationEngine motionCalibrationEngine = new MotionCalibrationEngine();

    private BluetoothController bluetoothController;
    private UwbServerClient uwbServerClient;
    private UwbServerClient.Listener uwbServerListener;
    private UwbEndpointStore uwbEndpointStore;
    private UwbEndpointConfig currentUwbEndpoint;
    private RouteManager routeManager;
    private MapProfileStore mapProfileStore;
    private AnchorLayoutStore anchorLayoutStore;
    private AnchorLayout currentAnchorLayout;
    private AnchorLayout pendingAnchorLayout;
    private AppState appState;
    private ArrayAdapter<String> scanAdapter;

    private View pageMap;
    private View pageSettings;
    private View pageMapping;
    private View pageLocalization;
    private View pageConnect;
    private View pageLogs;
    private View controlPanel;
    private View routeMapPane;
    private View radarMapPane;
    private GridMapView gridMapView;
    private RadarView radarView;
    private OccupancyGridView occupancyGridView;
    private UwbLayoutView uwbLayoutView;
    private View statusIndicator;
    private ImageView statusLinkIcon;
    private TextView statusText;
    private TextView mapSummaryText;
    private TextView mapStateValue;
    private TextView mapStepsValue;
    private TextView mapSpeedValue;
    private TextView radarSummaryText;
    private TextView motionSensorSummaryText;
    private TextView lidarMapDebugText;
    private TextView settingsSummaryText;
    private TextView uwbEndpointSummaryText;
    private TextView mappingSummaryText;
    private TextView localizationSummaryText;
    private TextView anchorCalibrationText;
    private MaterialButton anchorCalibrationButton;
    private TextView connectionSummaryText;
    private TextView homeVehicleStateText;
    private TextView homeVehicleDetailText;
    private TextView homeSpeedValue;
    private TextView homeSafeValue;
    private TextView homeRouteValue;
    private TextView autoControlStatusText;
    private TextView logText;
    private TextView[] parameterValueTexts;
    private TextView[] parameterEstimateTexts;
    private TextView[] parameterStateTexts;
    private ScrollView logScrollView;
    private MaterialButton connectButton;
    private MaterialButton disconnectButton;
    private MaterialButton requestStatusButton;
    private MaterialButton routeMapButton;
    private MaterialButton radarMapButton;
    private MaterialButton mappingPauseButton;
    private MaterialButton mappingSaveButton;
    private MaterialButton mappingLoadButton;
    private MaterialButton mappingSyncButton;
    private MaterialButton mappingAnchorButton;
    private MaterialButton mappingNavigateButton;
    private MaterialButton mappingStopNavigationButton;
    private MaterialButton manualModeButton;
    private MaterialButton autoModeButton;
    private MaterialButton radarPowerButton;
    private MaterialButton immediateStopButton;
    private MaterialButton driveActionButton;
    private MaterialButton homePrimaryButton;
    private MaterialButton homeMapButton;
    private MaterialButton[] pageButtons;
    private MaterialButton[] parameterButtons;
    private View[] parameterRows;

    private long btWatchStartedAt;
    private long lastPongAt;
    private long lastStatusAt;
    private long lastRadarAt;
    private long lastVl53At;
    private long lastMotionSensorAt;
    private long lastLidarMapAt;
    private long previousLidarMapAt;
    private long lastDataRecoveryAt;
    private int lastLidarEnabled = -1;
    private int lastRadarOnline = -1;
    private int lastSpeed = -1;
    private int lastStepLength = -1;
    private int lastSafeDistanceMm = -1;
    private final int[] lastWheelSpeeds = {-1, -1, -1, -1};
    private final int[][] motionProfiles = new int[3][4];
    private MaterialButton[] motionProfileButtons;
    private int lastSelectedParameter = -1;
    private String lastParameterTableSignature = "";
    private String lastSettingsSummary = "";
    private String lastStatusText = "";
    private String lastConnectionSummary = "";
    private int lastParameterButtonsSelection = Integer.MIN_VALUE;
    private boolean heartbeatRunning;
    private boolean parameterSessionActive;
    private boolean manualDriveMode;
    private int lastManualMode = -1;
    private boolean radarMapMode;
    private View currentPage;
    private CarCommand heldCommand;
    private CarCommand explorationCommand;
    private RadarFrame latestRadarFrame;
    private Vl53Frame latestVl53Frame;
    private MotionSensorFrame latestMotionSensorFrame;
    private boolean imu0GyroSuppressed;
    private boolean imu1GyroSuppressed;
    private LidarMapScan latestLidarMapScan;
    private StatusFrame latestStatusFrame;
    private MaterialButton mappingExploreButton;
    private boolean explorationRunning;
    private long explorationCommandEndAt;
    private int explorationActionDurationMs;
    private int[] explorationStartSectors;
    private boolean explorationMotorConfirmed;
    private boolean explorationRadarMotionConfirmed;
    private boolean explorationAwaitingEvidence;
    private int explorationUnconfirmedActions;
    private boolean explorationPreflightPending;
    private int explorationStartAttempts;
    private long explorationStartRequestedAt;
    private boolean navigationRunning;
    private boolean navigationReplanning;
    private CarCommand navigationCommand;
    private long navigationCommandEndAt;
    private int navigationCommandDurationMs;
    private long lastNavigationObstacleAt;
    private int navigationReplanCount;
    private float navigationLastTurnTargetHeading = Float.NaN;
    private float navigationLastTurnErrorMagnitude = Float.NaN;
    private int navigationTurnNoProgressCount;
    private CarCommand calibrationCommand;
    private boolean calibrationRunning;
    private long calibrationEndAt;
    private int calibrationTestDurationMs;
    private AlertDialog parameterEditorDialog;
    private int editingParameter = -1;
    private float lidarMapRateHz;
    private int lidarMapCrcErrorCount;
    private int lidarMapFormatErrorCount;
    private boolean autoCalibrationRunning;
    private int autoCalibrationActionIndex;
    private long autoCalibrationActionEndAt;
    private long autoCalibrationStoppedAt;
    private CarCommand autoCalibrationCommand;
    private MotionSnapshot autoCalibrationStartSnapshot;
    private final List<MotionCalibrationSample> autoCalibrationSamples = new ArrayList<>();
    private final List<Double> autoForwardRates = new ArrayList<>();
    private final List<Double> autoReverseRates = new ArrayList<>();
    private final List<Double> autoLeftRates = new ArrayList<>();
    private final List<Double> autoRightRates = new ArrayList<>();
    private final List<Double> autoFlowCountsPerMm = new ArrayList<>();
    private int autoFlowXAxisVotes;
    private int autoFlowPositiveVotes;
    private int autoFlowNegativeVotes;
    private String autoCalibrationStatus = "";
    private OnboardMapStatus latestOnboardMapStatus;
    private long lastOnboardMapStatusAt;
    private UwbServerSnapshot latestUwbServerSnapshot;
    private final MapWorldRegistration mapWorldRegistration = new MapWorldRegistration();
    private long lastRegisteredUwbSourceMs = Long.MIN_VALUE;
    private long lastFilteredUwbSourceMs = Long.MIN_VALUE;
    private double filteredUwbXM;
    private double filteredUwbYM;
    private boolean filteredUwbValid;
    private final UwbPositionFilter uwbPositionFilter = new UwbPositionFilter();
    private double imuWorldHeadingOffsetDeg;
    private boolean imuWorldHeadingReferenceValid;
    private boolean imuWorldHeadingManuallyAligned;
    private double manualAbsoluteHeadingDeg;
    private double manualBoardHeadingReferenceDeg;
    private boolean manualBoardHeadingReferenceValid;
    private double automaticAbsoluteHeadingDeg;
    private double automaticBoardHeadingReferenceDeg;
    private boolean automaticBoardHeadingReferenceValid;
    private boolean automaticBoardHeadingTracking;
    private long lastUwbHeadingFusionSourceMs = Long.MIN_VALUE;
    private double pendingUwbImuOffsetDeg;
    private int pendingUwbHeadingSamples;
    private boolean uwbServerConnected;
    private String uwbServerConnectionDetail = "UWB服务器连接中";
    private boolean anchorCalibrationRunning;
    private boolean anchorLayoutConfiguring;
    private boolean startMappingAfterCalibration;
    private boolean syncAfterMapLoad;
    private boolean mapLoadOperationObserved;
    private String currentMapProfileId;
    private String currentMapProfileName = "";

    private final Runnable heartbeatRunnable = new Runnable() {
        @Override
        public void run() {
            if (!heartbeatRunning || !isCarConnected()) {
                return;
            }
            // Motion commands can keep the normal queue non-empty for several
            // seconds. Heartbeats must still reach the FPGA before its 3 s
            // link watchdog expires, so keep only one pending P at queue head.
            sendTransportPriorityCommand(CarCommand.HEARTBEAT);
            appendLog("发送", CarCommand.HEARTBEAT.getPayload() + " 心跳");
            handler.postDelayed(this, HEARTBEAT_INTERVAL_MS);
        }
    };

    private final Runnable heartbeatStartRunnable = this::startHeartbeat;

    private final Runnable statusRequestRunnable = () -> {
        if (isCarConnected()) {
            sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "查询");
        }
    };

    private final Runnable btTimeoutRunnable = new Runnable() {
        @Override
        public void run() {
            if (isCarConnected()
                    && getLastHeartbeatReferenceAt() > 0
                    && System.currentTimeMillis() - getLastHeartbeatReferenceAt() > BT_OFF_TIMEOUT_MS) {
                updateCompactStatus();
                updateConnectionUi();
            }
            handler.postDelayed(this, 500L);
        }
    };

    private final Runnable dataWatchdogRunnable = new Runnable() {
        @Override
        public void run() {
            recoverStaleDataIfNeeded();
            lidarMapAssembler.expire(System.currentTimeMillis());
            updateLidarMapDebug();
            updateRadarSummary();
            updateMotionSensorSummary();
            updateConnectionUi();
            handler.postDelayed(this, 700L);
        }
    };

    private final Runnable holdRepeatRunnable = new Runnable() {
        @Override
        public void run() {
            if (heldCommand == null) {
                return;
            }
            sendCarCommand(heldCommand, "长按发送");
            handler.postDelayed(this, HOLD_REPEAT_INTERVAL_MS);
        }
    };

    private final Runnable explorationDriveRunnable = new Runnable() {
        @Override
        public void run() {
            if (!explorationRunning || explorationCommand == null) return;
            if (System.currentTimeMillis() < explorationCommandEndAt) {
                sendTransportCommand(explorationCommand);
                handler.postDelayed(this, 90L);
                return;
            }
            CarCommand completed = explorationCommand;
            explorationCommand = null;
            explorationAwaitingEvidence = true;
            sendTransportEmergencyStop();
            appendLog("自动探索", "C 动作结束停止");
            handler.postDelayed(() -> finishExplorationAction(completed), 420L);
        }
    };

    private final Runnable navigationDriveRunnable = new Runnable() {
        @Override
        public void run() {
            if (!navigationRunning || navigationCommand == null) return;
            if (!isCarConnected()) {
                stopNavigation("蓝牙断开，FPGA安全看门狗停车", false);
                return;
            }
            if (System.currentTimeMillis() < navigationCommandEndAt) {
                sendTransportCommand(navigationCommand);
                handler.postDelayed(this, 90L);
                return;
            }
            CarCommand completed = navigationCommand;
            int duration = navigationCommandDurationMs;
            navigationCommand = null;
            navigationCommandDurationMs = 0;
            sendTransportEmergencyStop();
            occupancyGridView.setMotionActive(false);
            occupancyGridView.applyEstimatedMotion(completed.getPayload().charAt(0), duration);
            // The C5 frame, wheel inertia and @G0 publication all trail the
            // final BLE command. Decide from the settled board heading rather
            // than the first stale sample after the stop frame.
            handler.postDelayed(() -> runNavigationDecision("动作完成"), 500L);
        }
    };

    private final Runnable calibrationDriveRunnable = new Runnable() {
        @Override
        public void run() {
            if (!calibrationRunning || calibrationCommand == null) return;
            if (System.currentTimeMillis() < calibrationEndAt) {
                sendTransportCommand(calibrationCommand);
                handler.postDelayed(this, 90L);
                return;
            }
            CarCommand completed = calibrationCommand;
            calibrationRunning = false;
            calibrationCommand = null;
            sendTransportEmergencyStop();
            occupancyGridView.setMotionActive(false);
            appendLog("运动标定", "测试动作结束并停止");
            handler.postDelayed(() -> showCalibrationResultDialog(completed), 350L);
        }
    };

    private final Runnable uwbFreshnessRunnable = new Runnable() {
        @Override
        public void run() {
            updateMappingSummary();
            updateLocalizationSummary();
            handler.postDelayed(this, 500L);
        }
    };

    private final Runnable autoCalibrationDriveRunnable = new Runnable() {
        @Override
        public void run() {
            if (!autoCalibrationRunning || autoCalibrationCommand == null) return;
            if (System.currentTimeMillis() < autoCalibrationActionEndAt) {
                sendTransportCommand(autoCalibrationCommand);
                handler.postDelayed(this, 90L);
                return;
            }
            CarCommand completed = autoCalibrationCommand;
            autoCalibrationCommand = null;
            autoCalibrationStoppedAt = System.currentTimeMillis();
            sendTransportEmergencyStop();
            occupancyGridView.setMotionActive(false);
            appendLog("自动标定", describeCommand(completed) + "动作结束，已停车");
            handler.postDelayed(() -> waitForAutoCalibrationEndSample(
                    completed,
                    System.currentTimeMillis() + AUTO_CALIBRATION_SAMPLE_TIMEOUT_MS
            ), AUTO_CALIBRATION_SETTLE_MS);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);
        initState();
        initViews();
        runDebugAlgorithmSelfCheck();
        PermissionHelper.requestRuntimePermissions(this);
        if (PermissionHelper.hasRuntimePermissions(this)) {
            startVehicleAutoConnect();
        }

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            int edgePadding = dp(12);
            v.setPadding(
                    systemBars.left + edgePadding,
                    systemBars.top + edgePadding,
                    systemBars.right + edgePadding,
                    systemBars.bottom + edgePadding
            );
            return insets;
        });
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private void runDebugAlgorithmSelfCheck() {
        if ((getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) == 0) return;
        Thread worker = new Thread(() -> {
            try {
                String result = VehicleAlgorithmSelfCheck.run();
                Log.i("VehicleAlgorithm", result);
                runOnUiThread(() -> appendLog("算法回放自检", result));
            } catch (RuntimeException error) {
                String detail = "FAIL " + error.getMessage();
                Log.e("VehicleAlgorithm", detail, error);
                runOnUiThread(() -> appendLog("算法回放自检", detail));
            }
        }, "vehicle-algorithm-self-check");
        worker.start();
    }

    private void initState() {
        bluetoothController = new BluetoothController(this);
        routeManager = new RouteManager(5, 5);
        mapProfileStore = new MapProfileStore(this);
        anchorLayoutStore = new AnchorLayoutStore(this);
        currentAnchorLayout = anchorLayoutStore.load();
        uwbEndpointStore = new UwbEndpointStore(this);
        currentUwbEndpoint = uwbEndpointStore.load();
        appState = new AppState();
        uwbServerListener = new UwbServerClient.Listener() {
                    @Override
                    public void onConnectionChanged(boolean connected, String detail) {
                        runOnUiThread(() -> {
                            boolean changed = uwbServerConnected != connected
                                    || !detail.equals(uwbServerConnectionDetail);
                            uwbServerConnected = connected;
                            uwbServerConnectionDetail = detail;
                            if (changed) appendLog("UWB服务器", detail);
                            updateUwbEndpointSummary();
                            updateMappingSummary();
                            updateLocalizationSummary();
                        });
                    }

                    @Override
                    public void onSnapshot(UwbServerSnapshot snapshot) {
                        runOnUiThread(() -> handleUwbServerSnapshot(snapshot));
                    }

                    @Override
                    public void onAnchorCalibration(boolean running, String summary) {
                        runOnUiThread(() -> {
                            anchorCalibrationRunning = running;
                            appendLog("基站标定", summary.replace('\n', ' '));
                        });
                    }

                    @Override
                    public void onAnchorLayout(String state, String detail, AnchorLayout layout) {
                        runOnUiThread(() -> handleAnchorLayoutState(state, detail, layout));
                    }
                };
        uwbServerClient = new UwbServerClient(
                currentUwbEndpoint.endpoint(BuildConfig.UWB_SERVER_URL),
                BuildConfig.UWB_MOBILE_TOKEN,
                uwbServerListener
        );
        uwbServerClient.connect();
        handler.post(uwbFreshnessRunnable);
        bluetoothController.setListener(new BluetoothController.Listener() {
            @Override
            public void onDeviceFound(BluetoothDevice device, int rssi) {
                runOnUiThread(() -> {
                    addScanDevice(device, rssi);
                    appendLog("扫描", formatDeviceName(device) + " 信号=" + rssi);
                });
            }

            @Override
            public void onConnected() {
                runOnUiThread(() -> {
                    updateConnectionUi();
                    appendLog("蓝牙", "GATT 已连接，正在发现服务");
                    showStatus("蓝牙已连接，正在查找 CH9141K 串口服务...");
                });
            }

            @Override
            public void onDisconnected() {
                runOnUiThread(() -> {
                    stopHeldCommand(false);
                    if (explorationRunning) stopExploration("蓝牙断开");
                    if (navigationRunning) stopNavigation("蓝牙断开，已由车载看门狗停车", false);
                    if (calibrationRunning) stopCalibration("蓝牙断开");
                    if (autoCalibrationRunning) stopAutoCalibration("蓝牙断开", false);
                    stopHeartbeat();
                    manualDriveMode = false;
                    lastManualMode = -1;
                    driveModeTransition.clear();
                    lastLidarEnabled = -1;
                    resetLidarMapSession();
                    resetMotionSensorSession();
                    updateConnectionUi();
                    updateModeControls();
                    appendLog("蓝牙", "连接已断开");
                    showStatus(bluetoothController.isAutoConnectEnabled()
                            ? "蓝牙已断开，正在自动重连"
                            : "蓝牙未连接，点击前往连接");
                });
            }

            @Override
            public void onReady() {
                runOnUiThread(() -> {
                    lastPongAt = 0L;
                    btWatchStartedAt = System.currentTimeMillis();
                    resetLidarMapSession();
                    resetMotionSensorSession();
                    updateConnectionUi();
                    appendLog("蓝牙", "CH9141K 串口已就绪");
                    appendLog("蓝牙", HEARTBEAT_START_DELAY_MS + "ms 后启动心跳");
                    showStatus(getString(R.string.bluetooth_ready_waiting));
                    scheduleHeartbeatStart();
                    handler.postDelayed(() -> {
                        if (isCarConnected()) {
                            sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "连接同步");
                        }
                    }, 450L);
                });
            }

            @Override
            public void onFrameReceived(BleFrame frame) {
                runOnUiThread(() -> handleFrame(frame));
            }

            @Override
            public void onError(String message) {
                runOnUiThread(() -> {
                    appendLog("错误", message);
                    showStatus(toFriendlyBluetoothError(message));
                });
            }

            @Override
            public void onDebugLog(String tag, String message) {
                runOnUiThread(() -> appendLog(tag, message));
            }
        });
    }

    private void initViews() {
        pageMap = findViewById(R.id.pageMap);
        pageSettings = findViewById(R.id.pageSettings);
        pageMapping = findViewById(R.id.pageMapping);
        pageLocalization = findViewById(R.id.pageLocalization);
        pageConnect = findViewById(R.id.pageConnect);
        pageLogs = findViewById(R.id.pageLogs);
        controlPanel = findViewById(R.id.controlPanel);
        routeMapPane = findViewById(R.id.routeMapPane);
        radarMapPane = findViewById(R.id.radarMapPane);
        statusIndicator = findViewById(R.id.statusIndicator);
        statusLinkIcon = findViewById(R.id.statusLinkIcon);
        statusText = findViewById(R.id.statusText);
        mapSummaryText = findViewById(R.id.mapSummaryText);
        mapStateValue = findViewById(R.id.mapStateValue);
        mapStepsValue = findViewById(R.id.mapStepsValue);
        mapSpeedValue = findViewById(R.id.mapSpeedValue);
        radarSummaryText = findViewById(R.id.radarSummaryText);
        motionSensorSummaryText = findViewById(R.id.motionSensorSummaryText);
        lidarMapDebugText = findViewById(R.id.lidarMapDebugText);
        settingsSummaryText = findViewById(R.id.settingsSummaryText);
        uwbEndpointSummaryText = findViewById(R.id.uwbEndpointSummaryText);
        connectionSummaryText = findViewById(R.id.connectionSummaryText);
        homeVehicleStateText = findViewById(R.id.homeVehicleStateText);
        homeVehicleDetailText = findViewById(R.id.homeVehicleDetailText);
        homeSpeedValue = findViewById(R.id.homeSpeedValue);
        homeSafeValue = findViewById(R.id.homeSafeValue);
        homeRouteValue = findViewById(R.id.homeRouteValue);
        autoControlStatusText = findViewById(R.id.autoControlStatusText);
        logText = findViewById(R.id.logText);
        logScrollView = findViewById(R.id.logScrollView);
        gridMapView = findViewById(R.id.gridMapView);
        radarView = findViewById(R.id.radarView);
        occupancyGridView = findViewById(R.id.occupancyGridView);
        uwbLayoutView = findViewById(R.id.uwbLayoutView);
        occupancyGridView.setAnchorPositions(currentAnchorLayout.canonicalMapAnchorsM());
        occupancyGridView.setAnchorDisplayLabels(currentAnchorLayout.canonicalMapAnchorNames());
        uwbLayoutView.setAnchorLayout(currentAnchorLayout);
        mappingExploreButton = findViewById(R.id.mappingExploreButton);
        mappingNavigateButton = findViewById(R.id.mappingNavigateButton);
        mappingStopNavigationButton = findViewById(R.id.mappingStopNavigationButton);
        mappingSummaryText = findViewById(R.id.mappingSummaryText);
        localizationSummaryText = findViewById(R.id.localizationSummaryText);
        anchorCalibrationText = findViewById(R.id.anchorCalibrationText);
        anchorCalibrationButton = findViewById(R.id.anchorCalibrationButton);
        loadMotionCalibration();
        connectButton = findViewById(R.id.connectButton);
        disconnectButton = findViewById(R.id.disconnectButton);
        requestStatusButton = findViewById(R.id.requestStatusButton);
        routeMapButton = findViewById(R.id.routeMapButton);
        radarMapButton = findViewById(R.id.radarMapButton);
        mappingPauseButton = findViewById(R.id.mappingPauseButton);
        mappingSaveButton = findViewById(R.id.mappingSaveButton);
        mappingLoadButton = findViewById(R.id.mappingLoadButton);
        mappingSyncButton = findViewById(R.id.mappingSyncButton);
        mappingAnchorButton = findViewById(R.id.mappingAnchorButton);
        manualModeButton = findViewById(R.id.manualModeButton);
        autoModeButton = findViewById(R.id.autoModeButton);
        radarPowerButton = findViewById(R.id.radarPowerButton);
        immediateStopButton = findViewById(R.id.immediateStopButton);
        driveActionButton = findViewById(R.id.stopButton);
        homePrimaryButton = findViewById(R.id.homePrimaryButton);
        homeMapButton = findViewById(R.id.homeMapButton);
        pageButtons = new MaterialButton[] {
                findViewById(R.id.tabConnectButton),
                findViewById(R.id.tabMapButton),
                findViewById(R.id.tabMappingButton),
                findViewById(R.id.tabSettingsButton)
        };
        parameterButtons = new MaterialButton[] {
                findViewById(R.id.selectSpeedButton),
                findViewById(R.id.selectStepButton),
                findViewById(R.id.selectLfButton),
                findViewById(R.id.selectRfButton),
                findViewById(R.id.selectLrButton),
                findViewById(R.id.selectRrButton),
                findViewById(R.id.selectSafeButton)
        };
        parameterRows = new View[] {
                findViewById(R.id.parameterRowSpeed),
                findViewById(R.id.parameterRowStep),
                findViewById(R.id.parameterRowLf),
                findViewById(R.id.parameterRowRf),
                findViewById(R.id.parameterRowLr),
                findViewById(R.id.parameterRowRr),
                findViewById(R.id.parameterRowSafe)
        };
        parameterValueTexts = new TextView[] {
                findViewById(R.id.parameterValueSpeed),
                findViewById(R.id.parameterValueStep),
                findViewById(R.id.parameterValueLf),
                findViewById(R.id.parameterValueRf),
                findViewById(R.id.parameterValueLr),
                findViewById(R.id.parameterValueRr),
                findViewById(R.id.parameterValueSafe)
        };
        parameterEstimateTexts = new TextView[] {
                findViewById(R.id.parameterEstimateSpeed),
                findViewById(R.id.parameterEstimateStep),
                findViewById(R.id.parameterEstimateLf),
                findViewById(R.id.parameterEstimateRf),
                findViewById(R.id.parameterEstimateLr),
                findViewById(R.id.parameterEstimateRr),
                findViewById(R.id.parameterEstimateSafe)
        };
        parameterStateTexts = new TextView[] {
                findViewById(R.id.parameterStateSpeed),
                findViewById(R.id.parameterStateStep),
                findViewById(R.id.parameterStateLf),
                findViewById(R.id.parameterStateRf),
                findViewById(R.id.parameterStateLr),
                findViewById(R.id.parameterStateRr),
                findViewById(R.id.parameterStateSafe)
        };
        for (TextView valueText : parameterValueTexts) {
            valueText.setBackgroundResource(R.drawable.editable_value_background);
            valueText.setGravity(Gravity.CENTER);
            valueText.setPadding(dp(4), dp(4), dp(4), dp(4));
            valueText.setTextColor(ContextCompat.getColor(this, R.color.primary_dark));
            valueText.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        }
        motionProfileButtons = new MaterialButton[] {
                findViewById(R.id.forwardProfileButton),
                findViewById(R.id.reverseProfileButton),
                findViewById(R.id.rotationProfileButton)
        };
        loadMotionProfiles();

        gridMapView.clearRoute(routeManager.getCarPosition(), routeManager.getCarDirection());
        updateParameterButtons(-1);
        updateParameterTable(null, -1);
        bindTabs();
        bindMapTabs();
        bindMappingControls();
        bindConnectionControls();
        bindDriveControls();
        bindSettingsControls();
        bindProductNavigation();
        updateAnchorLayoutUi(formatAnchorLayout(currentAnchorLayout));
        updateUwbEndpointSummary();
        findViewById(R.id.statusPanel).setOnClickListener(view -> {
            view.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP);
            showPage(pageConnect);
        });
        showPage(pageConnect);
        updateConnectionUi();
        updateModeControls();
        updateCompactStatus();
        handler.post(btTimeoutRunnable);
        handler.post(dataWatchdogRunnable);
    }

    private void bindTabs() {
        findViewById(R.id.tabConnectButton).setOnClickListener(view -> showPage(pageConnect));
        findViewById(R.id.tabMapButton).setOnClickListener(view -> showPage(pageMap));
        findViewById(R.id.tabMappingButton).setOnClickListener(view -> showPage(pageMapping));
        findViewById(R.id.tabSettingsButton).setOnClickListener(view -> showPage(pageSettings));
        anchorCalibrationButton.setOnClickListener(view -> {
            showAnchorEditorDialog();
        });
    }

    private void bindMapTabs() {
        routeMapButton.setOnClickListener(view -> showMapMode(false));
        radarMapButton.setOnClickListener(view -> showMapMode(true));
        showMapMode(false);
    }

    private void showPage(View page) {
        if (currentPage == pageMapping && page != pageMapping && explorationRunning) {
            stopExploration("已离开建图页");
        }
        if (currentPage == pageMapping && page != pageMapping && calibrationRunning) {
            stopCalibration("已离开建图页");
        }
        if (currentPage == pageMapping && page != pageMapping && autoCalibrationRunning) {
            stopAutoCalibration("已离开建图页", true);
        }
        currentPage = page;
        pageMap.setVisibility(page == pageMap ? View.VISIBLE : View.GONE);
        pageMapping.setVisibility(page == pageMapping ? View.VISIBLE : View.GONE);
        pageLocalization.setVisibility(page == pageLocalization ? View.VISIBLE : View.GONE);
        pageSettings.setVisibility(page == pageSettings ? View.VISIBLE : View.GONE);
        pageConnect.setVisibility(page == pageConnect ? View.VISIBLE : View.GONE);
        pageLogs.setVisibility(page == pageLogs ? View.VISIBLE : View.GONE);
        updateControlPanelVisibility();
        updatePageTabs(page);
        if (page == pageMap) {
            gridMapView.postInvalidate();
            radarView.postInvalidate();
            requestStatusSoon();
        } else if (page == pageMapping) {
            occupancyGridView.postInvalidate();
            updateMappingSummary();
            requestStatusSoon();
        } else if (page == pageLocalization) {
            uwbLayoutView.postInvalidate();
            updateLocalizationSummary();
        }
    }

    private void showMapMode(boolean showRadar) {
        radarMapMode = showRadar;
        routeMapPane.setVisibility(showRadar ? View.GONE : View.VISIBLE);
        radarMapPane.setVisibility(showRadar ? View.VISIBLE : View.GONE);
        updateSegmentButton(routeMapButton, !showRadar);
        updateSegmentButton(radarMapButton, showRadar);
        updateControlPanelVisibility();
    }

    private void updateControlPanelVisibility() {
        controlPanel.setVisibility(currentPage == pageMap && !radarMapMode ? View.VISIBLE : View.GONE);
    }

    private void updatePageTabs(View page) {
        if (pageButtons == null) {
            return;
        }
        boolean[] selected = {
                page == pageConnect,
                page == pageMap,
                page == pageMapping || page == pageLocalization,
                page == pageSettings || page == pageLogs
        };
        for (int i = 0; i < pageButtons.length && i < selected.length; i++) {
            updateSegmentButton(pageButtons[i], selected[i]);
        }
    }

    private void bindProductNavigation() {
        homePrimaryButton.setOnClickListener(view -> {
            if (isCarConnected()) {
                showPage(pageMap);
            } else {
                connectButton.performClick();
            }
        });
        homeMapButton.setOnClickListener(view -> showPage(pageMapping));
        findViewById(R.id.mapLocalizationButton)
                .setOnClickListener(view -> showPage(pageLocalization));
        findViewById(R.id.localizationMapButton)
                .setOnClickListener(view -> showPage(pageMapping));
        findViewById(R.id.deviceConnectButton).setOnClickListener(view -> connectButton.performClick());
        findViewById(R.id.deviceDiagnosticsButton).setOnClickListener(view -> showPage(pageLogs));
        findViewById(R.id.deviceUwbServerButton).setOnClickListener(
                view -> showUwbEndpointDialog());
        findViewById(R.id.diagnosticsBackButton).setOnClickListener(view -> showPage(pageSettings));

        View parameterTable = findViewById(R.id.parameterTable);
        MaterialButton parameterToggle = findViewById(R.id.deviceParametersToggleButton);
        parameterToggle.setOnClickListener(view -> {
            boolean show = parameterTable.getVisibility() != View.VISIBLE;
            parameterTable.setVisibility(show ? View.VISIBLE : View.GONE);
            parameterToggle.setText(show ? "收起行驶参数 ︿" : "行驶参数与实测估算  ›");
        });
        View profilePanel = findViewById(R.id.motionProfilePanel);
        MaterialButton profileToggle = findViewById(R.id.deviceProfilesToggleButton);
        profileToggle.setOnClickListener(view -> {
            boolean show = profilePanel.getVisibility() != View.VISIBLE;
            profilePanel.setVisibility(show ? View.VISIBLE : View.GONE);
            profileToggle.setText(show ? "收起轮速补偿 ︿" : "轮速补偿与运动标定  ›");
        });

        MaterialButton diagnosticsToggle = findViewById(R.id.controlDiagnosticsButton);
        diagnosticsToggle.setOnClickListener(view -> {
            boolean show = motionSensorSummaryText.getVisibility() != View.VISIBLE;
            motionSensorSummaryText.setVisibility(show ? View.VISIBLE : View.GONE);
            lidarMapDebugText.setVisibility(show ? View.VISIBLE : View.GONE);
            diagnosticsToggle.setText(show ? "隐藏传感器详细数据" : "显示传感器详细数据");
        });
    }

    private void updateUwbEndpointSummary() {
        if (currentUwbEndpoint == null || uwbEndpointSummaryText == null) return;
        String state = uwbServerConnected ? "已连接" : uwbServerConnectionDetail;
        setTextIfChanged(uwbEndpointSummaryText,
                "当前：" + currentUwbEndpoint.summary() + "\n状态：" + state);
        MaterialButton button = findViewById(R.id.deviceUwbServerButton);
        if (button != null) {
            button.setText(currentUwbEndpoint.getMode() == UwbEndpointConfig.Mode.PUBLIC
                    ? "UWB 服务器 · 公网  ›" : "UWB 服务器 · 局域网  ›");
        }
    }

    private void showUwbEndpointDialog() {
        LinearLayout content = new LinearLayout(this);
        content.setOrientation(LinearLayout.VERTICAL);
        content.setPadding(dp(20), dp(4), dp(20), 0);

        RadioGroup modes = new RadioGroup(this);
        modes.setOrientation(RadioGroup.VERTICAL);
        RadioButton publicMode = new RadioButton(this);
        publicMode.setId(View.generateViewId());
        publicMode.setText("公网服务器（默认）");
        RadioButton lanMode = new RadioButton(this);
        lanMode.setId(View.generateViewId());
        lanMode.setText("局域网电脑");
        modes.addView(publicMode);
        modes.addView(lanMode);
        content.addView(modes);

        EditText hostInput = new EditText(this);
        hostInput.setHint("电脑内网 IP，例如 192.168.31.16");
        hostInput.setSingleLine(true);
        hostInput.setInputType(InputType.TYPE_CLASS_PHONE);
        hostInput.setText(currentUwbEndpoint.getLanHost());
        content.addView(hostInput, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(52)));

        EditText portInput = new EditText(this);
        portInput.setHint("端口，默认 18765");
        portInput.setSingleLine(true);
        portInput.setInputType(InputType.TYPE_CLASS_NUMBER);
        portInput.setText(String.valueOf(currentUwbEndpoint.getLanPort()));
        content.addView(portInput, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(52)));

        TextView warning = new TextView(this);
        warning.setText("局域网模式仅接受私有 IPv4/Tailscale 地址；切换后立即重连，不会自动回退。电脑、手机和 S3 需能互相访问。");
        warning.setTextColor(ContextCompat.getColor(this, R.color.text_secondary));
        warning.setTextSize(12f);
        warning.setPadding(0, dp(8), 0, 0);
        content.addView(warning);

        boolean lanSelected = currentUwbEndpoint.getMode() == UwbEndpointConfig.Mode.LAN;
        modes.check(lanSelected ? lanMode.getId() : publicMode.getId());
        hostInput.setEnabled(lanSelected);
        portInput.setEnabled(lanSelected);
        modes.setOnCheckedChangeListener((group, checkedId) -> {
            boolean useLan = checkedId == lanMode.getId();
            hostInput.setEnabled(useLan);
            portInput.setEnabled(useLan);
        });

        AlertDialog dialog = new AlertDialog.Builder(this)
                .setTitle("选择 UWB 服务器")
                .setView(content)
                .setNegativeButton("取消", null)
                .setPositiveButton("保存并重连", null)
                .create();
        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
                .setOnClickListener(view -> {
                    try {
                        boolean useLan = modes.getCheckedRadioButtonId() == lanMode.getId();
                        int port = useLan
                                ? Integer.parseInt(portInput.getText().toString().trim())
                                : currentUwbEndpoint.getLanPort();
                        UwbEndpointConfig selected = new UwbEndpointConfig(
                                useLan ? UwbEndpointConfig.Mode.LAN
                                        : UwbEndpointConfig.Mode.PUBLIC,
                                hostInput.getText().toString(), port);
                        applyUwbEndpoint(selected);
                        dialog.dismiss();
                    } catch (IllegalArgumentException error) {
                        Toast.makeText(this, error.getMessage(), Toast.LENGTH_LONG).show();
                    }
                }));
        dialog.show();
    }

    private void applyUwbEndpoint(UwbEndpointConfig selected) {
        if (uwbServerClient != null) uwbServerClient.shutdown();
        currentUwbEndpoint = selected;
        uwbEndpointStore.save(selected);
        uwbServerConnected = false;
        uwbServerConnectionDetail = "正在连接";
        latestUwbServerSnapshot = null;
        resetFilteredUwbPosition();
        uwbServerClient = new UwbServerClient(
                selected.endpoint(BuildConfig.UWB_SERVER_URL),
                BuildConfig.UWB_MOBILE_TOKEN,
                uwbServerListener);
        uwbServerClient.connect();
        updateUwbEndpointSummary();
        updateMappingSummary();
        updateLocalizationSummary();
        appendLog("UWB服务器", "手动切换为" + selected.summary());
        Toast.makeText(this, "已切换为" + selected.summary(), Toast.LENGTH_SHORT).show();
    }

    private void bindMappingControls() {
        mappingExploreButton.setOnClickListener(view -> {
            giveButtonFeedback(view, explorationRunning ? "停止探索" : "自动探索");
            if (explorationRunning) {
                stopExploration("用户停止");
            } else {
                startExploration();
            }
            updateMappingPauseButton();
            updateMappingSummary();
        });
        occupancyGridView.setGoalSelectionListener((pathFound, goalX, goalY, waypointCount) -> {
            if (!pathFound) {
                stopNavigation("目的地不可达", true);
                String detail = occupancyGridView.getLastPlanFailure();
                Toast.makeText(this, detail == null || detail.isEmpty()
                                ? "目的地不可达，请选择已建图的空闲区域" : detail,
                        Toast.LENGTH_LONG).show();
                mappingNavigateButton.setText("重新点选目的地");
                setStatusOnly(detail == null || detail.isEmpty() ? "目的地不可达" : detail);
                return;
            }
            startNavigationToSelectedGoal(waypointCount);
        });
        mappingNavigateButton.setOnClickListener(view -> {
            giveButtonFeedback(view, "选择目的地");
            if (!isCarConnected()) {
                Toast.makeText(this, "请先连接小车", Toast.LENGTH_SHORT).show();
                return;
            }
            if (explorationRunning) stopExploration("切换到目标导航");
            occupancyGridView.setGoalSelectionEnabled(true);
            mappingNavigateButton.setText("请点地图目的地");
            setStatusOnly("请在已建图的基站范围内点击目的地");
        });
        mappingStopNavigationButton.setOnClickListener(view -> {
            giveButtonFeedback(view, "停止导航");
            stopNavigation("用户停止", true);
        });
        mappingPauseButton.setOnClickListener(view -> {
            giveButtonFeedback(view, occupancyGridView.isPaused() ? "继续建图" : "暂停建图");
            boolean resume = occupancyGridView.isPaused();
            occupancyGridView.setPaused(!resume);
            if (isCarConnected()) {
                sendPriorityCarCommand(resume
                        ? CarCommand.START_ONBOARD_MAPPING
                        : CarCommand.PAUSE_ONBOARD_MAPPING, "手机建图位姿流");
            }
            updateMappingPauseButton();
            updateMappingSummary();
        });
        findViewById(R.id.mappingCenterButton).setOnClickListener(view -> {
            giveButtonFeedback(view, "地图适配场地");
            occupancyGridView.fitAnchorField();
        });
        findViewById(R.id.mappingCalibrationButton).setOnClickListener(view -> {
            giveButtonFeedback(view, "打开标定");
            showCalibrationTypeDialog();
        });
        findViewById(R.id.mappingClearButton).setOnClickListener(view -> {
            giveButtonFeedback(view, "清空地图");
            new AlertDialog.Builder(this)
                        .setTitle("清空建图数据")
                        .setMessage("确认清空当前栅格地图？此操作无法撤销。")
                        .setNegativeButton("取消", null)
                        .setPositiveButton("清空", (dialog, which) -> {
                            if (explorationRunning) stopExploration("清空地图");
                            if (calibrationRunning) stopCalibration("清空地图");
                            if (autoCalibrationRunning) stopAutoCalibration("清空地图", true);
                            if (isCarConnected())
                                sendPriorityCarCommand(CarCommand.CLEAR_ONBOARD_MAP, "位姿原点");
                            occupancyGridView.clearMap();
                            mapWorldRegistration.reset();
                            lastRegisteredUwbSourceMs = Long.MIN_VALUE;
                            updateMappingSummary();
                            Toast.makeText(this, "已清空手机地图", Toast.LENGTH_SHORT).show();
                        })
                        .show();
        });
        mappingSaveButton.setOnClickListener(view -> {
            giveButtonFeedback(view, "保存地图");
            showSaveMapProfileDialog();
        });
        mappingLoadButton.setOnClickListener(view -> {
            giveButtonFeedback(view, "读取地图");
            showLoadMapProfileDialog();
        });
        mappingAnchorButton.setOnClickListener(view -> {
            giveButtonFeedback(view, "配置基站");
            showAnchorEditorDialog();
        });
        mappingSyncButton.setOnClickListener(view -> {
            giveButtonFeedback(view, "刷新地图");
            occupancyGridView.fitAnchorField();
            if (isCarConnected())
                sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "刷新位姿");
        });
        updateMappingPauseButton();
    }

    private void showAnchorEditorDialog() {
        if (anchorLayoutConfiguring) {
            Toast.makeText(this, "基站配置正在写入，请稍候", Toast.LENGTH_SHORT).show();
            return;
        }
        LinearLayout container = new LinearLayout(this);
        container.setOrientation(LinearLayout.VERTICAL);
        container.setPadding(dp(20), dp(8), dp(20), 0);

        LinearLayout dimensions = new LinearLayout(this);
        dimensions.setOrientation(LinearLayout.HORIZONTAL);
        EditText widthInput = createDimensionInput("宽度 X（cm）",
                currentAnchorLayout.getWidthCm());
        EditText heightInput = createDimensionInput("长度 Y（cm）",
                currentAnchorLayout.getHeightCm());
        dimensions.addView(createDimensionField("宽度 X（cm）", widthInput),
                new LinearLayout.LayoutParams(0, dp(76), 1f));
        LinearLayout.LayoutParams heightParams = new LinearLayout.LayoutParams(0, dp(76), 1f);
        heightParams.setMarginStart(dp(8));
        dimensions.addView(createDimensionField("长度 Y（cm）", heightInput), heightParams);
        container.addView(dimensions);

        String[] anchorOptions = {"基站 A", "基站 B", "基站 C", "基站 D"};
        String[] cornerNames = {"左下位置", "右下位置", "右上位置", "左上位置"};
        Spinner[] cornerSpinners = new Spinner[4];
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this,
                android.R.layout.simple_spinner_dropdown_item, anchorOptions);
        for (int corner = 0; corner < 4; corner++) {
            LinearLayout row = new LinearLayout(this);
            row.setGravity(Gravity.CENTER_VERTICAL);
            TextView label = new TextView(this);
            label.setText(cornerNames[corner]);
            label.setTextColor(ContextCompat.getColor(this, R.color.text_primary));
            label.setTextSize(14f);
            row.addView(label, new LinearLayout.LayoutParams(0, dp(48), 1f));
            Spinner spinner = new Spinner(this);
            spinner.setAdapter(adapter);
            spinner.setSelection(currentAnchorLayout.getAnchorAtCorner(corner));
            cornerSpinners[corner] = spinner;
            row.addView(spinner, new LinearLayout.LayoutParams(0, dp(48), 1f));
            container.addView(row);
        }
        AlertDialog anchorDialog = new AlertDialog.Builder(this)
                .setTitle("设置 UWB 基站布局")
                .setMessage("只需填写场地长宽，并为四个角选择实际基站标签。"
                        + "写入成功后会清空旧地图，从新坐标系重新建图。")
                .setView(container)
                .setNegativeButton("取消", null)
                .setPositiveButton("应用", null)
                .create();
        anchorDialog.setOnShowListener(ignored ->
                anchorDialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(view -> {
                    if (applyRequestedAnchorLayout(widthInput, heightInput, cornerSpinners)) {
                        anchorDialog.dismiss();
                    }
                }));
        anchorDialog.show();
    }

    private EditText createDimensionInput(String hint, int valueCm) {
        EditText input = new EditText(this);
        input.setHint(hint);
        input.setSingleLine(true);
        input.setInputType(InputType.TYPE_CLASS_NUMBER);
        input.setText(String.valueOf(valueCm));
        return input;
    }

    private LinearLayout createDimensionField(String labelText, EditText input) {
        LinearLayout field = new LinearLayout(this);
        field.setOrientation(LinearLayout.VERTICAL);
        TextView label = new TextView(this);
        label.setText(labelText);
        label.setTextColor(ContextCompat.getColor(this, R.color.text_secondary));
        label.setTextSize(12f);
        field.addView(label, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(20)));
        field.addView(input, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(56)));
        return field;
    }

    private boolean applyRequestedAnchorLayout(EditText widthInput, EditText heightInput,
                                                Spinner[] cornerSpinners) {
        final AnchorLayout requested;
        try {
            int[] corners = new int[4];
            for (int corner = 0; corner < 4; corner++) {
                corners[corner] = cornerSpinners[corner].getSelectedItemPosition();
            }
            requested = new AnchorLayout(
                    Integer.parseInt(widthInput.getText().toString().trim()),
                    Integer.parseInt(heightInput.getText().toString().trim()),
                    corners);
        } catch (IllegalArgumentException error) {
            Toast.makeText(this,
                    "长宽需为 50–600cm，且 A/B/C/D 必须各使用一次",
                    Toast.LENGTH_LONG).show();
            return false;
        }
        if (!uwbServerClient.configureAnchorLayout(requested)) {
            Toast.makeText(this, "UWB服务器未连接", Toast.LENGTH_LONG).show();
            return false;
        }
        pendingAnchorLayout = requested;
        anchorLayoutConfiguring = true;
        updateAnchorLayoutUi("正在等待服务器应用新坐标系…");
        appendLog("基站布局", String.format(Locale.CHINA,
                "请求 %.2f×%.2fm，角标 %s", requested.getWidthM(), requested.getHeightM(),
                requested.cornerCsv()));
        return true;
    }

    private void handleAnchorLayoutState(String state, String detail, AnchorLayout serverLayout) {
        String normalized = state == null ? "unknown" : state.toLowerCase(Locale.US);
        if ("complete".equals(normalized)) {
            AnchorLayout confirmed = serverLayout != null ? serverLayout : pendingAnchorLayout;
            if (confirmed != null && !confirmed.equals(currentAnchorLayout)) {
                applyConfirmedAnchorLayout(confirmed);
            }
            pendingAnchorLayout = null;
            anchorLayoutConfiguring = false;
            updateAnchorLayoutUi("服务器已应用 · " + formatAnchorLayout(currentAnchorLayout));
            return;
        }
        if ("failed".equals(normalized) || "rejected".equals(normalized)) {
            pendingAnchorLayout = null;
            anchorLayoutConfiguring = false;
            updateAnchorLayoutUi("应用失败 · " + friendlyAnchorLayoutDetail(detail));
            Toast.makeText(this, "基站布局未改变，请检查 UWB 服务器连接", Toast.LENGTH_LONG).show();
            return;
        }
        if ("idle".equals(normalized) || "unknown".equals(normalized)) {
            if (!anchorLayoutConfiguring) {
                updateAnchorLayoutUi(formatAnchorLayout(currentAnchorLayout));
            }
            return;
        }
        anchorLayoutConfiguring = true;
        String stage;
        switch (normalized) {
            case "accepted": stage = "服务器已接收布局"; break;
            default: stage = "服务器正在应用布局"; break;
        }
        updateAnchorLayoutUi(stage + (detail == null || detail.isEmpty() ? "" : " · " + detail));
    }

    private void applyConfirmedAnchorLayout(AnchorLayout layout) {
        if (navigationRunning) stopNavigation("基站布局改变", true);
        if (explorationRunning) stopExploration("基站布局改变");
        if (calibrationRunning) stopCalibration("基站布局改变");
        if (autoCalibrationRunning) stopAutoCalibration("基站布局改变", true);
        if (isCarConnected()) {
            sendPriorityCarCommand(CarCommand.CLEAR_ONBOARD_MAP, "基站布局改变");
        }
        currentAnchorLayout = layout;
        anchorLayoutStore.save(layout);
        occupancyGridView.setAnchorPositions(layout.canonicalMapAnchorsM());
        occupancyGridView.setAnchorDisplayLabels(layout.canonicalMapAnchorNames());
        occupancyGridView.clearMap();
        uwbLayoutView.setAnchorLayout(layout);
        uwbLayoutView.resetToNominalCenter();
        currentMapProfileId = null;
        currentMapProfileName = "";
        mapWorldRegistration.reset();
        lastRegisteredUwbSourceMs = Long.MIN_VALUE;
        lastUwbHeadingFusionSourceMs = Long.MIN_VALUE;
        resetFilteredUwbPosition();
        latestUwbServerSnapshot = null;
        appendLog("基站布局", "服务器坐标系已更新，旧地图与UWB滤波/配准状态已清空");
        updateMappingSummary();
        updateLocalizationSummary();
        Toast.makeText(this, "基站布局已应用，已从新坐标系重新建图", Toast.LENGTH_LONG).show();
    }

    private void updateAnchorLayoutUi(String status) {
        if (anchorCalibrationText != null) anchorCalibrationText.setText(status);
        if (anchorCalibrationButton != null) {
            anchorCalibrationButton.setEnabled(!anchorLayoutConfiguring);
            anchorCalibrationButton.setText(anchorLayoutConfiguring ? "正在配置基站…" : "设置基站布局");
        }
        if (mappingAnchorButton != null) mappingAnchorButton.setEnabled(!anchorLayoutConfiguring);
    }

    private static String friendlyAnchorLayoutDetail(String detail) {
        if (detail == null || detail.isEmpty()) return "未知错误";
        return detail.replace('_', ' ');
    }

    private static String formatAnchorLayout(AnchorLayout layout) {
        return String.format(Locale.CHINA,
                "场地 %.2f×%.2fm · 左下%s / 右下%s / 右上%s / 左上%s",
                layout.getWidthM(), layout.getHeightM(),
                layout.getAnchorNameAtCorner(AnchorLayout.CORNER_LOWER_LEFT),
                layout.getAnchorNameAtCorner(AnchorLayout.CORNER_LOWER_RIGHT),
                layout.getAnchorNameAtCorner(AnchorLayout.CORNER_UPPER_RIGHT),
                layout.getAnchorNameAtCorner(AnchorLayout.CORNER_UPPER_LEFT));
    }

    private void showSaveMapProfileDialog() {
        EditText input = new EditText(this);
        input.setSingleLine(true);
        input.setHint("例如：决赛场地1");
        input.setText(currentMapProfileName);
        int padding = Math.round(20f * getResources().getDisplayMetrics().density);
        LinearLayout container = new LinearLayout(this);
        container.setPadding(padding, 0, padding, 0);
        container.addView(input, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
        new AlertDialog.Builder(this)
                .setTitle(currentMapProfileId == null ? "保存新地图" : "更新当前地图")
                .setView(container)
                .setNegativeButton("取消", null)
                .setPositiveButton("保存", (dialog, which) -> {
                    MapProfileStore.Profile profile = mapProfileStore.save(
                            currentMapProfileId, input.getText().toString(),
                            occupancyGridView.getAnchorPositions(),
                            occupancyGridView.exportPhoneMapSnapshot());
                    currentMapProfileId = profile.id;
                    currentMapProfileName = profile.name;
                    appendLog("手机地图", "已保存档案 " + profile.name + "，含四基站坐标");
                    Toast.makeText(this, "地图档案已保存", Toast.LENGTH_SHORT).show();
                    updateMappingSummary();
                })
                .show();
    }

    private void showLoadMapProfileDialog() {
        List<MapProfileStore.Profile> profiles = mapProfileStore.list();
        if (profiles.isEmpty()) {
            Toast.makeText(this, "没有已保存的地图档案", Toast.LENGTH_SHORT).show();
            return;
        }
        String[] labels = new String[profiles.size()];
        SimpleDateFormat format = new SimpleDateFormat("MM-dd HH:mm", Locale.CHINA);
        for (int index = 0; index < profiles.size(); index++) {
            MapProfileStore.Profile profile = profiles.get(index);
            labels[index] = profile.name + "  ·  " + format.format(new Date(profile.updatedAtMs));
        }
        new AlertDialog.Builder(this)
                .setTitle("选择地图档案")
                .setItems(labels, (dialog, which) -> loadMapProfile(profiles.get(which)))
                .setNegativeButton("取消", null)
                .show();
    }

    private void loadMapProfile(MapProfileStore.Profile profile) {
        if (!currentAnchorLayout.matchesMapAnchors(profile.anchors, 0.01)) {
            Toast.makeText(this,
                    "该地图使用旧基站尺寸，不能加载到当前坐标系；请重新建图",
                    Toast.LENGTH_LONG).show();
            return;
        }
        if (navigationRunning) stopNavigation("切换地图", true);
        if (explorationRunning) stopExploration("切换地图");
        occupancyGridView.setAnchorPositions(currentAnchorLayout.canonicalMapAnchorsM());
        if (!occupancyGridView.importPhoneMapSnapshot(profile.snapshot)) {
            Toast.makeText(this, "地图快照损坏，未加载", Toast.LENGTH_LONG).show();
            return;
        }
        currentMapProfileId = profile.id;
        currentMapProfileName = profile.name;
        appendLog("手机地图", "已加载 " + profile.name + " 及其四基站坐标");
        Toast.makeText(this, "已加载 " + profile.name, Toast.LENGTH_SHORT).show();
        updateMappingSummary();
    }

    private void updateMappingSummary() {
        if (mappingSummaryText == null || occupancyGridView == null) return;
        if (autoCalibrationRunning) {
            mappingSummaryText.setText("正在进行运动标定"
                    + "\n" + autoCalibrationStatus
                    + (startMappingAfterCalibration ? " · 完成后自动开始建图" : ""));
            return;
        }
        String state = !isCarConnected()
                ? "等待车辆"
                : (occupancyGridView.isPaused() ? "已暂停" : "建图中");
        if (explorationRunning) state = explorationCommand == null ? "自动探索决策中" : "自动探索：" + describeCommand(explorationCommand);
        if (navigationRunning) {
            state = navigationReplanning ? "自动导航重规划中"
                    : (navigationCommand == null ? "自动导航决策中"
                    : "自动导航：" + describeCommand(navigationCommand));
        }
        UwbVehiclePose serverPose = getFreshServerPose();
        String localizationState = serverPose == null || !serverPose.hasCoordinates()
                ? (uwbServerConnected ? "定位正在初始化" : "UWB 未连接")
                : "定位" + describeUwbQuality(serverPose.getQuality());
        String profile = currentMapProfileName.isEmpty() ? "未命名地图" : currentMapProfileName;
        String radarState = latestRadarFrame != null && latestRadarFrame.getRadarOnline() == 1
                ? (lidarMapRateHz > 0f
                ? String.format(Locale.CHINA, "雷达 %.1f Hz", lidarMapRateHz)
                : "雷达在线")
                : "雷达等待数据";
        String readiness = mapWorldRegistration.isRegistered()
                ? "地图与定位已对齐"
                : "移动小车以完成地图对齐";
        mappingSummaryText.setText(profile + " · " + state
                + "\n" + localizationState + " · " + radarState
                + "\n" + readiness);
    }

    private void updateLocalizationSummary() {
        if (localizationSummaryText == null) return;
        UwbVehiclePose serverPose = getFreshServerPose();
        String live = serverPose == null || !serverPose.hasCoordinates()
                ? "\n实时位姿：" + (uwbServerConnected ? "等待服务器UWB" : uwbServerConnectionDetail)
                : String.format(Locale.CHINA,
                "\nUWB中心 %.3f, %.3fm · IMU航向 %.1f° · %s · 标签 %d/4",
                serverPose.getXM(), serverPose.getYM(), getFusedWorldHeadingDeg(serverPose),
                describeUwbQuality(serverPose.getQuality()), serverPose.getValidTagCount());
        String gateway = latestUwbServerSnapshot != null
                && latestUwbServerSnapshot.isGatewayConnected() ? "A基站网关在线" : "A基站网关离线";
        localizationSummaryText.setText(
                "UWB八模块布局 · 固定基站 + 车载标签 T0/T1/T2/T3"
                        + String.format(Locale.US,
                        "\n场地 %.2f x %.2fm · 左下%s/右下%s/右上%s/左上%s"
                                + " · 左前T%d/右前T%d/右后T%d/左后T%d"
                                + " · 标签矩形 %.3f x %.3fm",
                        currentAnchorLayout.getWidthM(),
                        currentAnchorLayout.getHeightM(),
                        currentAnchorLayout.getAnchorNameAtCorner(0),
                        currentAnchorLayout.getAnchorNameAtCorner(1),
                        currentAnchorLayout.getAnchorNameAtCorner(2),
                        currentAnchorLayout.getAnchorNameAtCorner(3),
                        VehicleGeometry.FRONT_LEFT_TAG,
                        VehicleGeometry.FRONT_RIGHT_TAG,
                        VehicleGeometry.REAR_RIGHT_TAG,
                        VehicleGeometry.REAR_LEFT_TAG,
                        VehicleGeometry.TAG_RECTANGLE_LENGTH_M,
                        VehicleGeometry.TAG_RECTANGLE_WIDTH_M)
                        + "\n" + gateway + " · " + (uwbServerConnected ? "服务器在线" : "服务器离线")
                        + live
        );
    }

    private UwbVehiclePose getFreshServerPose() {
        if (latestUwbServerSnapshot == null
                || System.currentTimeMillis() - latestUwbServerSnapshot.getReceivedAtMs()
                > UWB_SERVER_STALE_MS) return null;
        return latestUwbServerSnapshot.getPose();
    }

    private String describeUwbQuality(String quality) {
        if ("full".equals(quality)) return "四标签可信";
        if ("partial".equals(quality)) return "标签不完整";
        if ("partial_rigid".equals(quality)) return "三标签刚体定位";
        if ("unsynchronized".equals(quality)) return "标签不同步";
        if ("unconfigured".equals(quality)) return "车体参数未配置";
        if ("rejected".equals(quality)) return "矩形校验失败";
        return "无有效位姿";
    }

    private void handleUwbServerSnapshot(UwbServerSnapshot snapshot) {
        latestUwbServerSnapshot = snapshot;
        if (uwbLayoutView != null) uwbLayoutView.setServerTagFixes(snapshot.getTagFixes());
        UwbVehiclePose pose = snapshot.getPose();
        if (pose != null && pose.hasCoordinates()) {
            updateFilteredUwbPosition(pose);
            updateImuWorldHeadingReference(pose);
            boolean reliable = pose.isNavigationReliable();
            updateMapWorldRegistration();
            boolean fusedMapPoseApplied = applyUwbImuMapPose();
            if ((getApplicationInfo().flags
                    & android.content.pm.ApplicationInfo.FLAG_DEBUGGABLE) != 0) {
                Double relativeYaw = getRelativeImuYawDeg();
                Float boardWorldHeading = getBoardWorldHeadingDeg();
                String boardRawHeading = latestOnboardMapStatus == null ? "NA"
                        : String.format(Locale.US, "%.2f",
                        latestOnboardMapStatus.getHeadingDegrees());
                Log.d("VehiclePoseTrace", String.format(Locale.US,
                        "uwb=(%.3f,%.3f) uwbHeading=%.2f imuRelative=%s "
                                + "boardRaw=%s boardWorld=%s manual=%s fused=%.2f",
                        pose.getXM(), pose.getYM(), pose.getHeadingDeg(),
                        relativeYaw == null ? "NA" : String.format(Locale.US, "%.2f", relativeYaw),
                        boardRawHeading,
                        boardWorldHeading == null ? "NA"
                                : String.format(Locale.US, "%.2f", boardWorldHeading),
                        imuWorldHeadingManuallyAligned,
                        getFusedWorldHeadingDeg(pose)));
            }
            if (uwbLayoutView != null) {
                // UWB fixes absolute translation. Its short 20 x 22 cm tag
                // baseline is too noisy for a precision compass, so rotation
                // comes from the FPGA's continuously integrated dual-IMU pose.
                double headingRad = Math.toRadians(getFusedWorldHeadingDeg(pose));
                // Both pages must render the same filtered world pose. A second
                // localization-only filter previously made the car appear at
                // two different positions relative to the same four anchors.
                uwbLayoutView.setVehiclePose(filteredUwbXM, filteredUwbYM,
                        headingRad, reliable,
                        "UWB位置 + IMU朝向 · " + describeUwbQuality(pose.getQuality()));
            }
            if (reliable && occupancyGridView != null && !fusedMapPoseApplied) {
                occupancyGridView.applyAbsolutePose(
                        (int) Math.round(filteredUwbXM * 1000.0),
                        (int) Math.round(filteredUwbYM * 1000.0),
                        getFusedWorldHeadingDeg(pose), true);
            }
        } else if (uwbLayoutView != null && pose != null && pose.hasPosition()) {
            Float boardHeading = getBoardWorldHeadingDeg();
            // Do not feed this approximate tag coordinate into the navigation
            // filter. It is only a stable display/manual-alignment fallback.
            uwbLayoutView.setVehiclePose(pose.getXM(), pose.getYM(),
                    Math.toRadians(boardHeading == null ? 0f : boardHeading), false,
                    "单标签近似位置 · 等待更多标签");
        }
        updateMappingSummary();
        updateLocalizationSummary();
    }

    private boolean applyUwbImuMapPose() {
        UwbVehiclePose uwbPose = getFreshServerPose();
        if (uwbPose == null || !uwbPose.isNavigationReliable()
                || !filteredUwbValid || latestOnboardMapStatus == null) {
            return false;
        }
        Float boardHeading = getBoardWorldHeadingDeg();
        boolean imuHeadingReady = boardHeading != null;
        float heading = getMappingWorldHeadingDeg(uwbPose);
        occupancyGridView.applyAbsolutePose(
                (int) Math.round(filteredUwbXM * 1000.0),
                (int) Math.round(filteredUwbYM * 1000.0),
                heading, imuHeadingReady);
        return true;
    }

    /**
     * Mapping uses the FPGA's per-sample, bias-corrected gyro integral once an
     * onboard map is running. UWB remains authoritative for translation and
     * supplies the absolute heading used to initialize that integral.
     */
    private float getMappingWorldHeadingDeg(UwbVehiclePose fallbackPose) {
        Float boardHeading = getBoardWorldHeadingDeg();
        if (boardHeading != null) return boardHeading;
        // @M2 is diagnostic telemetry and may be sparse or dropped. Never use
        // phone-side integration for mapping; hold the coarse UWB direction
        // until the next absolute FPGA @G0 snapshot arrives.
        return fallbackPose == null ? 0f : getWorldUwbHeadingDeg(fallbackPose);
    }

    /** Uses the FPGA's per-sample gyro integral instead of sparse @M2 samples. */
    private Float getBoardWorldHeadingDeg() {
        long now = System.currentTimeMillis();
        if (latestOnboardMapStatus == null
                || !isBoardHeadingAvailable(latestOnboardMapStatus)) return null;
        // @G0 is event-driven: once the chassis stops turning, a correct final
        // heading can remain unchanged for longer than the freshness window.
        // Keep that value after an explicit manual alignment. BLE disconnect
        // resets the reference, so this cannot leak into a later session.
        if (!imuWorldHeadingManuallyAligned && !automaticBoardHeadingTracking
                && now - lastOnboardMapStatusAt > ONBOARD_HEADING_STALE_MS) return null;
        double boardHeading = latestOnboardMapStatus.getHeadingDegrees();
        // An explicit user alignment defines the world-frame zero for the
        // entire BLE session. Keep it authoritative even after mapping starts;
        // otherwise mapActive used a second field-axis interpretation and the
        // display jumped back toward the startup/anchor direction.
        if (imuWorldHeadingManuallyAligned && manualBoardHeadingReferenceValid) {
            return NavigationSteering.rebaseBoardHeading(
                    (float) manualAbsoluteHeadingDeg,
                    (float) manualBoardHeadingReferenceDeg, (float) boardHeading);
        }
        if (automaticBoardHeadingTracking && automaticBoardHeadingReferenceValid) {
            return NavigationSteering.rebaseBoardHeading(
                    (float) automaticAbsoluteHeadingDeg,
                    (float) automaticBoardHeadingReferenceDeg, (float) boardHeading);
        }
        if (latestOnboardMapStatus.isMapActive()) {
            double[][] anchors = occupancyGridView.getAnchorPositions();
            double fieldAxisHeadingDeg = NavigationSteering.vehicleHeadingForWorldVector(
                    anchors[2][0] - anchors[0][0],
                    anchors[2][1] - anchors[0][1]);
            return (float) normalizeDegrees(fieldAxisHeadingDeg + boardHeading);
        }
        return null;
    }

    /** Bit 0 means IMU calibration is complete; bit 1 only gates map translation. */
    private static boolean isBoardHeadingAvailable(OnboardMapStatus status) {
        return status != null && (status.getPoseFlags() & 0x01) != 0;
    }

    /**
     * Establishes the absolute heading from stable four-tag UWB fixes, then uses
     * UWB only to remove slow IMU drift. Every UWB source sample is consumed at
     * most once so repeated server snapshots cannot over-correct the heading.
     */
    private void updateImuWorldHeadingReference(UwbVehiclePose pose) {
        if (pose == null || !pose.isNavigationReliable()
                || !"full".equals(pose.getQuality()) || pose.getValidTagCount() < 4) return;
        Double relativeImuYaw = getRelativeImuYawDeg();
        if (relativeImuYaw == null) return;

        long sourceMs = pose.getUpdatedAtMs() != null
                ? pose.getUpdatedAtMs()
                : (latestUwbServerSnapshot == null
                ? System.currentTimeMillis() : latestUwbServerSnapshot.getReceivedAtMs());
        if (sourceMs == lastUwbHeadingFusionSourceMs) return;
        lastUwbHeadingFusionSourceMs = sourceMs;

        if (imuWorldHeadingManuallyAligned) return;

        double candidateOffset = normalizeDegrees(
                getWorldUwbHeadingDeg(pose) - relativeImuYaw);
        if (!imuWorldHeadingReferenceValid) {
            if (pendingUwbHeadingSamples == 0) {
                pendingUwbImuOffsetDeg = candidateOffset;
                pendingUwbHeadingSamples = 1;
            } else {
                double initError = normalizeDegrees(candidateOffset - pendingUwbImuOffsetDeg);
                if (Math.abs(initError) > UWB_HEADING_INIT_MAX_ERROR_DEG) {
                    pendingUwbImuOffsetDeg = candidateOffset;
                    pendingUwbHeadingSamples = 1;
                } else {
                    pendingUwbHeadingSamples++;
                    pendingUwbImuOffsetDeg = normalizeDegrees(pendingUwbImuOffsetDeg
                            + initError / pendingUwbHeadingSamples);
                }
            }
            if (pendingUwbHeadingSamples >= UWB_HEADING_INIT_SAMPLES) {
                imuWorldHeadingOffsetDeg = pendingUwbImuOffsetDeg;
                imuWorldHeadingReferenceValid = true;
                // UWB's short tag baseline supplies one coarse absolute angle.
                // From this point the FPGA's high-rate gyro integral owns all
                // rotation; repeated noisy UWB fixes must not pull the display
                // back toward the initial anchor direction.
                automaticAbsoluteHeadingDeg = normalizeDegrees(
                        pendingUwbImuOffsetDeg + relativeImuYaw);
                automaticBoardHeadingTracking = true;
                automaticBoardHeadingReferenceValid =
                        isBoardHeadingAvailable(latestOnboardMapStatus);
                if (automaticBoardHeadingReferenceValid) {
                    automaticBoardHeadingReferenceDeg =
                            latestOnboardMapStatus.getHeadingDegrees();
                }
            }
            return;
        }
        // Do not continuously steer heading from UWB. With a 20 x 22 cm tag
        // rectangle, centimetre-level position noise becomes several degrees
        // of apparent yaw and previously caused visible snap-back.
    }

    private float getFusedWorldHeadingDeg(UwbVehiclePose fallbackPose) {
        Float boardHeading = getBoardWorldHeadingDeg();
        if (boardHeading != null) return boardHeading;
        // Do not integrate sparse @M2 packets on Android. The FPGA consumes
        // every dual-IMU sample and publishes an absolute modulo heading in
        // @G0, so a lost BLE packet is repaired by the next snapshot.
        return fallbackPose == null ? 0f : getWorldUwbHeadingDeg(fallbackPose);
    }

    /**
     * The server uses the A-C/A-D world frame but reports mathematical
     * CCW-positive angles. Convert once at this input boundary; all vehicle,
     * FPGA and navigation headings inside MainActivity are clockwise-positive.
     */
    private static float getWorldUwbHeadingDeg(UwbVehiclePose pose) {
        return NavigationSteering.fromMathematicalHeadingDegrees(
                (float) pose.getHeadingDeg());
    }

    private Double getRelativeImuYawDeg() {
        boolean imu0Ready = latestMotionSensorFrame != null
                && latestMotionSensorFrame.isBmi0Online() && !imu0GyroSuppressed
                && imu0Attitude.isInitialized();
        boolean imu1Ready = latestMotionSensorFrame != null
                && latestMotionSensorFrame.isBmi1Online() && !imu1GyroSuppressed
                && imu1Attitude.isInitialized();
        if (!imu0Ready && !imu1Ready) return null;
        if (!imu0Ready) return imu1Attitude.getYawDeg();
        if (!imu1Ready) return imu0Attitude.getYawDeg();
        double yaw0 = Math.toRadians(imu0Attitude.getYawDeg());
        double yaw1 = Math.toRadians(imu1Attitude.getYawDeg());
        return Math.toDegrees(Math.atan2(Math.sin(yaw0) + Math.sin(yaw1),
                Math.cos(yaw0) + Math.cos(yaw1)));
    }

    private boolean hasFreshWorldMappingPose(long now) {
        UwbVehiclePose pose = getFreshServerPose();
        return pose != null && pose.isNavigationReliable()
                && filteredUwbValid
                && getBoardWorldHeadingDeg() != null
                && latestOnboardMapStatus != null
                && now - lastOnboardMapStatusAt <= MAPPING_POSE_STALE_MS;
    }

    private void updateFilteredUwbPosition(UwbVehiclePose pose) {
        long sourceMs = pose.getUpdatedAtMs() == null
                ? latestUwbServerSnapshot.getReceivedAtMs() : pose.getUpdatedAtMs();
        if (sourceMs == lastFilteredUwbSourceMs) return;
        if (uwbPositionFilter.update(pose.getXM(), pose.getYM(), sourceMs)) {
            filteredUwbXM = uwbPositionFilter.getXM();
            filteredUwbYM = uwbPositionFilter.getYM();
            filteredUwbValid = uwbPositionFilter.isValid();
        }
        lastFilteredUwbSourceMs = sourceMs;
    }

    private void resetFilteredUwbPosition() {
        uwbPositionFilter.reset();
        filteredUwbValid = false;
        lastFilteredUwbSourceMs = Long.MIN_VALUE;
    }

    private MapWorldRegistration.WorldPose updateMapWorldRegistration() {
        UwbVehiclePose pose = getFreshServerPose();
        if (pose == null) return mapWorldRegistration.currentWorldPose();
        long sourceId = pose.getUpdatedAtMs() == null
                ? latestUwbServerSnapshot.getReceivedAtMs() : pose.getUpdatedAtMs();
        long receivedAtMs = latestUwbServerSnapshot.getReceivedAtMs();
        long receiveAgeMs = System.currentTimeMillis() - receivedAtMs;
        boolean registrationFix = pose.isNavigationReliable()
                && pose.getValidTagCount() >= 3
                && receiveAgeMs >= 0L && receiveAgeMs <= 2000L;
        // Server timestamps and BLE receive timestamps are different clocks and
        // include gateway/network latency. Pair both observations on the phone's
        // receive-time axis; keep the source timestamp only as a duplicate ID.
        if (registrationFix && sourceId != lastRegisteredUwbSourceMs
                && mapWorldRegistration.addWorldFix(receivedAtMs, pose.getXM(), pose.getYM(),
                pose.getHeadingDeg())) {
            lastRegisteredUwbSourceMs = sourceId;
        }
        occupancyGridView.setWorldTransform(mapWorldRegistration.getRotationRad(),
                mapWorldRegistration.getTranslationXM(),
                mapWorldRegistration.getTranslationYM(), mapWorldRegistration.isRegistered());
        return mapWorldRegistration.currentWorldPose();
    }

    private void updateMappingPauseButton() {
        if (mappingPauseButton == null || occupancyGridView == null) return;
        boolean paused = occupancyGridView.isPaused();
        setTextIfChanged(mappingPauseButton, paused ? "开始/继续" : "暂停");
        int background = paused ? R.color.accent_green : R.color.panel_background;
        int foreground = paused ? R.color.white : R.color.text_secondary;
        mappingPauseButton.setBackgroundTintList(ColorStateList.valueOf(
                ContextCompat.getColor(this, background)));
        mappingPauseButton.setTextColor(ContextCompat.getColor(this, foreground));
        mappingPauseButton.setStrokeColor(ColorStateList.valueOf(ContextCompat.getColor(
                this, paused ? R.color.accent_green : R.color.divider)));
    }

    private void showCalibrationTypeDialog() {
        if (autoCalibrationRunning) {
            new AlertDialog.Builder(this)
                    .setTitle("停止自动标定")
                    .setMessage("当前标定尚未完成，停止后本轮数据不会写入参数。")
                    .setNegativeButton("继续标定", null)
                    .setPositiveButton("立即停止", (dialog, which) ->
                            stopAutoCalibration("用户停止", true))
                    .show();
            return;
        }
        if (!isCarConnected()) {
            Toast.makeText(this, "请先连接小车", Toast.LENGTH_SHORT).show();
            return;
        }
        if (explorationRunning) stopExploration("开始运动标定");
        int leftDuration = calibratedQuarterTurnDuration(CarCommand.LEFT);
        int rightDuration = calibratedQuarterTurnDuration(CarCommand.RIGHT);
        String[] items = {
                "静止陀螺零偏学习（推荐）",
                "绝对朝向对齐（车头指向基站）",
                "FPGA闭环校准后自动探索建图",
                "仅运行板载闭环校准",
                "直行距离（前进 320ms）",
                "左转角度（预计90°，" + leftDuration + "ms）",
                "右转角度（预计90°，" + rightDuration + "ms）"
        };
        new AlertDialog.Builder(this)
                .setTitle("选择标定项目")
                .setItems(items, (dialog, which) -> {
                    if (which == 0) {
                        confirmStaticGyroAndHeadingAlignment();
                        return;
                    }
                    if (which == 1) {
                        showHeadingAnchorAlignmentDialog();
                        return;
                    }
                    if (which <= 3) {
                        confirmAutoCalibrationRun(which == 2);
                        return;
                    }
                    CarCommand command = which == 4 ? CarCommand.UP
                            : (which == 5 ? CarCommand.LEFT : CarCommand.RIGHT);
                    int duration = which == 4 ? CALIBRATION_FORWARD_MS
                            : (which == 5 ? leftDuration : rightDuration);
                    confirmCalibrationRun(command, duration);
                })
                .setNegativeButton("取消", null)
                .show();
    }

    private void confirmCalibrationRun(CarCommand command, int durationMs) {
        String action = describeCommand(command);
        new AlertDialog.Builder(this)
                .setTitle(action + "标定")
                .setMessage("请将小车放在平整、空旷的地面并标记起始位置。测试会执行 "
                        + durationMs + "ms " + action + "，随后自动停止。")
                .setNegativeButton("取消", null)
                .setPositiveButton("开始测试", (dialog, which) -> startCalibrationRun(command, durationMs))
                .show();
    }

    private void startCalibrationRun(CarCommand command, int durationMs) {
        calibrationRunning = true;
        calibrationCommand = command;
        calibrationTestDurationMs = durationMs;
        sendCarCommand(CarCommand.ENTER_MANUAL_MODE, "运动标定");
        setStatusOnly("运动标定准备：" + describeCommand(command));
        handler.postDelayed(() -> {
            if (!calibrationRunning || calibrationCommand != command) return;
            calibrationEndAt = System.currentTimeMillis() + durationMs;
            if (!sendProfiledMotion(command, "标定轮速")) {
                stopCalibration("四轮速度尚未同步");
                return;
            }
            occupancyGridView.setMotionActive(true);
            appendLog("运动标定", command.getPayload() + " " + describeCommand(command)
                    + " " + durationMs + "ms");
            handler.postDelayed(calibrationDriveRunnable, 90L);
        }, 400L);
    }

    private void showCalibrationResultDialog(CarCommand command) {
        EditText input = new EditText(this);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        input.setHint(command == CarCommand.UP ? "实测毫米" : "实测角度");
        input.setSingleLine(true);
        int horizontalPadding = dp(20);
        input.setPadding(horizontalPadding, dp(8), horizontalPadding, dp(8));
        AlertDialog dialog = new AlertDialog.Builder(this)
                .setTitle("输入" + describeCommand(command) + "实测结果")
                .setView(input)
                .setNegativeButton("放弃", null)
                .setPositiveButton("保存", null)
                .create();
        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
                .setOnClickListener(view -> {
                    String text = input.getText().toString().trim();
                    if (text.isEmpty()) {
                        input.setError("请输入测量值");
                        return;
                    }
                    float measured;
                    try {
                        measured = Float.parseFloat(text);
                    } catch (NumberFormatException error) {
                        input.setError("数值格式不正确");
                        return;
                    }
                    if (measured <= 0f) {
                        input.setError("测量值必须大于 0");
                        return;
                    }
                    saveMotionCalibration(command, measured);
                    dialog.dismiss();
                }));
        dialog.show();
    }

    private void saveMotionCalibration(CarCommand command, float measured) {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        android.content.SharedPreferences.Editor editor = preferences.edit();
        if (command == CarCommand.UP) {
            editor.putFloat("linear_mm_per_ms", measured / CALIBRATION_FORWARD_MS);
            if (lastSpeed > 0) editor.putInt("linear_speed_setting", lastSpeed);
        } else if (command == CarCommand.LEFT) {
            editor.putFloat("left_deg_per_ms", measured / calibrationTestDurationMs);
        } else if (command == CarCommand.RIGHT) {
            editor.putFloat("right_deg_per_ms", measured / calibrationTestDurationMs);
        }
        editor.apply();
        lastParameterTableSignature = "";
        loadMotionCalibration();
        updateParameterTable(null, lastSelectedParameter);
        updateMappingSummary();
        Toast.makeText(this, "标定结果已保存", Toast.LENGTH_SHORT).show();
    }

    private void loadMotionCalibration() {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        float linear = preferences.getFloat("linear_mm_per_ms", 0.38f);
        float left = preferences.getFloat("left_deg_per_ms", 0.18f);
        float right = preferences.getFloat("right_deg_per_ms", 0.18f);
        if (occupancyGridView != null) {
            occupancyGridView.setMotionCalibration(linear, left, right);
        }
    }

    private String formatMotionCalibration() {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        float linear = preferences.getFloat("linear_mm_per_ms", 0.38f);
        float left = preferences.getFloat("left_deg_per_ms", 0.18f);
        float right = preferences.getFloat("right_deg_per_ms", 0.18f);
        return String.format(Locale.CHINA, "标定 直行 %.3fmm/ms  左 %.3f°/ms  右 %.3f°/ms",
                linear, left, right);
    }

    private void stopCalibration(String reason) {
        if (!calibrationRunning) return;
        calibrationRunning = false;
        calibrationCommand = null;
        occupancyGridView.setMotionActive(false);
        handler.removeCallbacks(calibrationDriveRunnable);
        if (isCarConnected()) sendTransportEmergencyStop();
        appendLog("运动标定", "中止：" + reason);
        setStatusOnly("运动标定已中止");
    }

    private void confirmAutoCalibrationRun(boolean startMapAfterward) {
        String failure = autoCalibrationPreflightFailure();
        if (failure != null) {
            new AlertDialog.Builder(this)
                    .setTitle("暂不能自动标定")
                    .setMessage(failure + "\n\n请保持小车静止，等雷达和双IMU数据恢复后重试。")
                    .setPositiveButton("知道了", null)
                    .show();
            return;
        }
        new AlertDialog.Builder(this)
                .setTitle("板载自动校准与修正")
                .setMessage("请让车头前后和左右保留安全空间。手机只发送一次 X，"
                        + "随后由 FPGA 本地完成左右转向极性确认、短直行偏航采样、"
                        + "短后退回位和强化停车。\n\n校准结果会成为板载静态左右轮修正量，"
                        + "运行中的 PI+D 继续实时修正地面变化。按停止可随时中止。")
                .setNegativeButton("取消", null)
                .setPositiveButton("开始", (dialog, which) -> {
                    startMappingAfterCalibration = startMapAfterward;
                    startAutoCalibration();
                })
                .show();
    }

    private void confirmStaticGyroAndHeadingAlignment() {
        new AlertDialog.Builder(this)
                .setTitle("摆放小车")
                .setMessage("本次验证请将小车中心放在 B-C 连线中点，车头沿 B-C 连线朝向 B。"
                        + "\n\n保持车辆、四个标签和基站静止，然后点击开始。零偏学习约 2 秒后会自动弹出基站列表，请选择 B。")
                .setNegativeButton("取消", null)
                .setPositiveButton("已摆好，开始", (dialog, which) -> startStaticGyroBiasLearning())
                .show();
    }

    private void showHeadingAnchorAlignmentDialog() {
        UwbVehiclePose pose = getFreshServerPose();
        Double relativeImuYaw = getRelativeImuYawDeg();
        if (pose == null || !pose.hasPosition()) {
            Toast.makeText(this, "等待至少一个有效UWB标签位置", Toast.LENGTH_SHORT).show();
            return;
        }
        if (relativeImuYaw == null) {
            Toast.makeText(this, "等待双IMU姿态数据", Toast.LENGTH_SHORT).show();
            return;
        }
        String[] anchorLabels = currentAnchorLayout.canonicalMapAnchorNames();
        String[] anchors = new String[anchorLabels.length];
        for (int index = 0; index < anchorLabels.length; index++) {
            anchors[index] = "车头朝向 " + anchorLabels[index];
        }
        new AlertDialog.Builder(this)
                .setTitle("保持静止，选择车头朝向")
                .setItems(anchors, (dialog, which) ->
                        alignHeadingToAnchor(which, anchorLabels[which]))
                .setNegativeButton("取消", null)
                .show();
    }

    private void alignHeadingToAnchor(int anchorIndex, String anchorLabel) {
        UwbVehiclePose pose = getFreshServerPose();
        Double relativeImuYaw = getRelativeImuYawDeg();
        if (pose == null || !pose.hasPosition()
                || relativeImuYaw == null || anchorIndex < 0 || anchorIndex >= 4) {
            Toast.makeText(this, "定位或IMU数据已失效，请重试", Toast.LENGTH_SHORT).show();
            return;
        }
        double[][] anchors = occupancyGridView.getAnchorPositions();
        double alignmentXM = pose.isNavigationReliable() && filteredUwbValid
                ? filteredUwbXM : pose.getXM();
        double alignmentYM = pose.isNavigationReliable() && filteredUwbValid
                ? filteredUwbYM : pose.getYM();
        double dx = anchors[anchorIndex][0] - alignmentXM;
        double dy = anchors[anchorIndex][1] - alignmentYM;
        if (Math.hypot(dx, dy) < 0.10) {
            Toast.makeText(this, "车辆距离目标基站过近，无法可靠确定方向", Toast.LENGTH_SHORT).show();
            return;
        }

        sendTransportEmergencyStop();
        double absoluteHeadingDeg = NavigationSteering.vehicleHeadingForWorldVector(dx, dy);
        imuWorldHeadingOffsetDeg = normalizeDegrees(absoluteHeadingDeg - relativeImuYaw);
        imuWorldHeadingReferenceValid = true;
        imuWorldHeadingManuallyAligned = true;
        manualAbsoluteHeadingDeg = normalizeDegrees(absoluteHeadingDeg);
        automaticBoardHeadingTracking = false;
        automaticBoardHeadingReferenceValid = false;
        manualBoardHeadingReferenceValid =
                isBoardHeadingAvailable(latestOnboardMapStatus);
        if (manualBoardHeadingReferenceValid) {
            manualBoardHeadingReferenceDeg = latestOnboardMapStatus.getHeadingDegrees();
        }
        pendingUwbHeadingSamples = 0;
        lastUwbHeadingFusionSourceMs = Long.MIN_VALUE;
        mapWorldRegistration.reset();
        lastRegisteredUwbSourceMs = Long.MIN_VALUE;
        sendFieldMapConfiguration(pose, "车头朝向" + anchorLabel,
                true, absoluteHeadingDeg);
        appendLog("绝对朝向", String.format(Locale.CHINA,
                "车头朝向%s，设定 %.1f°，IMU相对 %.1f°，UWB标签=%d",
                anchorLabel, absoluteHeadingDeg, relativeImuYaw,
                pose.getValidTagCount()));
        updateLocalizationSummary();
        updateMappingSummary();
        Toast.makeText(this, String.format(Locale.CHINA,
                "朝向已对齐到 %s（%.1f°）", anchorLabel, absoluteHeadingDeg),
                Toast.LENGTH_LONG).show();
    }

    private void startStaticGyroBiasLearning() {
        sendPriorityCarCommand(CarCommand.CENTER, "静止零偏学习停车");
        setStatusOnly("请保持小车静止，正在学习陀螺零偏");
        handler.postDelayed(() -> {
            if (!isCarConnected()) return;
            sendPriorityCarCommand(CarCommand.LEARN_STATIC_GYRO_BIAS, "静止零偏学习");
            Toast.makeText(this, "请保持静止约 2 秒，随后选择车头朝向", Toast.LENGTH_LONG).show();
            handler.postDelayed(() -> {
                if (!isCarConnected()) {
                    Toast.makeText(this, "蓝牙已断开，无法继续朝向对齐", Toast.LENGTH_SHORT).show();
                    return;
                }
                showHeadingAnchorAlignmentDialog();
            }, 2400L);
        }, 350L);
    }

    private String autoCalibrationPreflightFailure() {
        if (!isCarConnected()) return "蓝牙尚未连接";
        if (latestStatusFrame != null && latestStatusFrame.getLocalCalibrationFlags() >= 0) {
            if (!latestStatusFrame.isCalibrationSensorsReady()) return "板载双 IMU 尚未就绪";
            if (latestStatusFrame.isCalibrationObstacleActive()) return "板载避障当前有效";
            return null;
        }
        long now = System.currentTimeMillis();
        if (latestMotionSensorFrame == null || now - lastMotionSensorAt > 15000L) {
            return "没有新鲜的 @M2 运动传感器帧";
        }
        if (!latestMotionSensorFrame.isCalibrationDone()) return "双 IMU 静止校准尚未完成";
        if (!latestMotionSensorFrame.isBmi0Online()
                || !latestMotionSensorFrame.isBmi1Online()) {
            return "两颗 BMI088 未全部在线";
        }
        return null;
    }

    private void startAutoCalibration() {
        if (autoCalibrationRunning) return;
        String failure = autoCalibrationPreflightFailure();
        if (failure != null) {
            startMappingAfterCalibration = false;
            setStatusOnly("自动标定未启动：" + failure);
            return;
        }
        if (explorationRunning) stopExploration("开始板载校准");
        if (calibrationRunning) stopCalibration("开始板载校准");
        autoCalibrationRunning = true;
        autoCalibrationStatus = "已发送启动命令，等待 FPGA 接管";
        updateAutoCalibrationButton();
        updateMappingSummary();
        sendPriorityCarCommand(CarCommand.START_LOCAL_CALIBRATION, "板载校准");
        appendLog("板载校准", "仅发送一次 X，后续动作由 FPGA 本地状态机执行");
        requestStatusSoon();
    }

    private int autoCalibrationActionCount() {
        return AUTO_CALIBRATION_REPEATS * 4;
    }

    private CarCommand autoCalibrationActionAt(int index) {
        switch (index & 3) {
            case 0: return CarCommand.UP;
            case 1: return CarCommand.DOWN;
            case 2: return CarCommand.LEFT;
            default: return CarCommand.RIGHT;
        }
    }

    private int autoCalibrationDuration(CarCommand command) {
        return command == CarCommand.UP || command == CarCommand.DOWN
                ? AUTO_CALIBRATION_TRANSLATION_MS : AUTO_CALIBRATION_TURN_MS;
    }

    private void waitForAutoCalibrationStartSample(long deadlineMs) {
        if (!autoCalibrationRunning) return;
        MotionSnapshot snapshot = captureMotionSnapshot();
        long now = System.currentTimeMillis();
        if (snapshot != null
                && now - snapshot.motionReceivedAtMs <= 1500L
                && now - snapshot.scan.getReceivedAtMs() <= 5000L) {
            beginAutoCalibrationAction(snapshot);
            return;
        }
        if (now >= deadlineMs) {
            stopAutoCalibration("等待动作前完整传感器快照超时", true);
            showAutoCalibrationFailure("雷达或 @M2 数据在 8 秒内没有形成完整快照");
            return;
        }
        autoCalibrationStatus = "等待动作前雷达/IMU快照 "
                + (autoCalibrationActionIndex + 1) + "/" + autoCalibrationActionCount();
        updateMappingSummary();
        handler.postDelayed(() -> waitForAutoCalibrationStartSample(deadlineMs), 200L);
    }

    private void beginAutoCalibrationAction(MotionSnapshot snapshot) {
        if (!autoCalibrationRunning) return;
        CarCommand command = autoCalibrationActionAt(autoCalibrationActionIndex);
        String safetyFailure = autoCalibrationSafetyFailure(command);
        if (safetyFailure != null) {
            stopAutoCalibration(safetyFailure, true);
            showAutoCalibrationFailure(safetyFailure);
            return;
        }
        int durationMs = autoCalibrationDuration(command);
        autoCalibrationStartSnapshot = snapshot;
        autoCalibrationCommand = command;
        autoCalibrationActionEndAt = System.currentTimeMillis() + durationMs;
        autoCalibrationStatus = String.format(Locale.CHINA, "%d/%d  %s %dms",
                autoCalibrationActionIndex + 1, autoCalibrationActionCount(),
                describeCommand(command), durationMs);
        updateAutoCalibrationButton();
        updateMappingSummary();
        if (!sendProfiledMotion(command, "自动标定轮速")) {
            stopAutoCalibration("四轮速度配置发送失败", true);
            showAutoCalibrationFailure("四轮速度配置发送失败");
            return;
        }
        occupancyGridView.setMotionActive(true);
        appendLog("自动标定", autoCalibrationStatus + " 开始");
        handler.postDelayed(autoCalibrationDriveRunnable, 90L);
    }

    private String autoCalibrationSafetyFailure(CarCommand command) {
        RadarFrame radar = latestRadarFrame;
        long now = System.currentTimeMillis();
        if (radar == null || radar.getRadarOnline() != 1
                || now - lastRadarAt > AUTO_CALIBRATION_RADAR_STALE_MS) {
            return "安全雷达数据过期";
        }
        if (radar.getMotorBlocked() == 1) return "电机当前被避障逻辑阻塞";
        int[] sectors = radar.getSectorDistancesMm();
        int safe = Math.max(450, radar.getSafeDistanceMm());
        int front = minimumValid(sectors[7], sectors[0], sectors[1], radar.getFrontDistance());
        int rear = minimumValid(sectors[3], sectors[4], sectors[5]);
        int left = minimumValid(sectors[5], sectors[6], sectors[7]);
        int right = minimumValid(sectors[1], sectors[2], sectors[3]);
        if (command == CarCommand.UP && (front == 0 || front < safe + 150)) {
            return "前方标定空间不足（至少 " + (safe + 150) + "mm）";
        }
        if (command == CarCommand.DOWN && (rear == 0 || rear < Math.max(500, safe + 80))) {
            return "后方标定空间不足";
        }
        if ((command == CarCommand.LEFT || command == CarCommand.RIGHT)
                && (left == 0 || right == 0 || Math.min(left, right) < 320)) {
            return "原地转向侧向空间不足";
        }
        return null;
    }

    private void waitForAutoCalibrationEndSample(CarCommand completed, long deadlineMs) {
        if (!autoCalibrationRunning || autoCalibrationStartSnapshot == null) return;
        MotionSnapshot end = captureMotionSnapshot();
        long now = System.currentTimeMillis();
        boolean freshMotion = end != null && end.motionReceivedAtMs > autoCalibrationStoppedAt;
        boolean newScan = end != null
                && end.scan.getReceivedAtMs() > autoCalibrationStartSnapshot.scan.getReceivedAtMs()
                && end.scan.getSequence() != autoCalibrationStartSnapshot.scan.getSequence();
        if (freshMotion && newScan) {
            processAutoCalibrationSample(completed, autoCalibrationStartSnapshot, end);
            return;
        }
        if (now >= deadlineMs) {
            stopAutoCalibration("等待动作后完整传感器快照超时", true);
            showAutoCalibrationFailure("停车后没有收到新的一整圈 72 点雷达或有效 @M2 帧");
            return;
        }
        autoCalibrationStatus = "停车稳定，等待第 " + (autoCalibrationActionIndex + 1)
                + " 段测量结果";
        updateMappingSummary();
        handler.postDelayed(() -> waitForAutoCalibrationEndSample(completed, deadlineMs), 200L);
    }

    private MotionSnapshot captureMotionSnapshot() {
        MotionSensorFrame frame = latestMotionSensorFrame;
        LidarMapScan scan = latestLidarMapScan;
        if (frame == null || scan == null || !frame.isCalibrationDone()
                || !frame.isPmw3901Online() || !frame.isBmi0Online() || !frame.isBmi1Online()
                || frame.getFlowSqual() < 20 || scan.getHighQualityPointCount() < 24
                || !imu0Attitude.isInitialized() || !imu1Attitude.isInitialized()) {
            return null;
        }
        return new MotionSnapshot(
                scan,
                lastMotionSensorAt,
                frame.getFlowAccumX(),
                frame.getFlowAccumY(),
                imu0Attitude.getYawDeg(),
                imu1Attitude.getYawDeg()
        );
    }

    private void processAutoCalibrationSample(
            CarCommand command,
            MotionSnapshot start,
            MotionSnapshot end
    ) {
        int durationMs = autoCalibrationDuration(command);
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        double predictedX = 0.0;
        double predictedHeading = 0.0;
        if (command == CarCommand.UP || command == CarCommand.DOWN) {
            double distance = preferences.getFloat("linear_mm_per_ms", 0.38f) * durationMs;
            predictedX = command == CarCommand.UP ? distance : -distance;
        } else {
            double rate = command == CarCommand.LEFT
                    ? preferences.getFloat("left_deg_per_ms", 0.18f)
                    : preferences.getFloat("right_deg_per_ms", 0.18f);
            predictedHeading = rate * durationMs * (command == CarCommand.LEFT ? 1.0 : -1.0);
        }
        final double expectedX = predictedX;
        final double expectedHeading = predictedHeading;
        autoCalibrationStatus = "计算第 " + (autoCalibrationActionIndex + 1) + " 段雷达匹配";
        updateMappingSummary();
        new Thread(() -> {
            RelativeLidarPoseEstimator.Result first = relativeLidarPoseEstimator.estimate(
                    start.scan, end.scan, expectedX, 0.0, expectedHeading);
            RelativeLidarPoseEstimator.Result match = first;
            if (command == CarCommand.LEFT || command == CarCommand.RIGHT) {
                RelativeLidarPoseEstimator.Result opposite = relativeLidarPoseEstimator.estimate(
                        start.scan, end.scan, 0.0, 0.0, -expectedHeading);
                match = betterLidarMatch(first, opposite);
            }
            RelativeLidarPoseEstimator.Result result = match;
            runOnUiThread(() -> recordAutoCalibrationSample(command, start, end, result));
        }, "auto-cal-lidar-match").start();
    }

    private RelativeLidarPoseEstimator.Result betterLidarMatch(
            RelativeLidarPoseEstimator.Result first,
            RelativeLidarPoseEstimator.Result second
    ) {
        if (first.isAccepted() != second.isAccepted()) return first.isAccepted() ? first : second;
        if (first.getInliers() != second.getInliers()) {
            return first.getInliers() > second.getInliers() ? first : second;
        }
        return first.getRmsMm() <= second.getRmsMm() ? first : second;
    }

    private void recordAutoCalibrationSample(
            CarCommand command,
            MotionSnapshot start,
            MotionSnapshot end,
            RelativeLidarPoseEstimator.Result match
    ) {
        if (!autoCalibrationRunning) return;
        int durationMs = autoCalibrationDuration(command);
        double yaw0 = normalizeDegrees(end.yaw0Deg - start.yaw0Deg);
        double yaw1 = normalizeDegrees(end.yaw1Deg - start.yaw1Deg);
        double yawMagnitude = (Math.abs(yaw0) + Math.abs(yaw1)) * 0.5;
        double yawDisagreement = Math.abs(Math.abs(yaw0) - Math.abs(yaw1));
        boolean yawValid = yawDisagreement <= Math.max(4.0, yawMagnitude * 0.35);
        int flowX = end.flowAccumX - start.flowAccumX;
        int flowY = end.flowAccumY - start.flowAccumY;
        boolean accepted = false;
        String reason = match.getReason();

        if (command == CarCommand.UP || command == CarCommand.DOWN) {
            double expectedSign = command == CarCommand.UP ? 1.0 : -1.0;
            double forwardMm = match.getXMm();
            double lateralLimit = Math.max(90.0, Math.abs(forwardMm) * 0.65);
            accepted = match.isAccepted()
                    && forwardMm * expectedSign >= 30.0
                    && Math.abs(match.getYMm()) <= lateralLimit;
            if (accepted) {
                double rate = Math.abs(forwardMm) / durationMs;
                Pose2d endPose = new Pose2d(
                        match.getXMm() / 1000.0,
                        match.getYMm() / 1000.0,
                        Math.toRadians(match.getHeadingDeg())
                );
                MotionCalibrationSample.Command sampleCommand = command == CarCommand.UP
                        ? MotionCalibrationSample.Command.FORWARD
                        : MotionCalibrationSample.Command.REVERSE;
                autoCalibrationSamples.add(new MotionCalibrationSample(
                        sampleCommand, durationMs, new Pose2d(0.0, 0.0, 0.0), endPose));
                (command == CarCommand.UP ? autoForwardRates : autoReverseRates).add(rate);
                int dominant = Math.abs(flowX) >= Math.abs(flowY) ? flowX : flowY;
                if (Math.abs(dominant) >= 2) {
                    autoFlowCountsPerMm.add(Math.abs(dominant) / Math.abs(forwardMm));
                    if (Math.abs(flowX) >= Math.abs(flowY)) autoFlowXAxisVotes++;
                    if (command == CarCommand.UP) {
                        if (dominant >= 0) autoFlowPositiveVotes++;
                        else autoFlowNegativeVotes++;
                    }
                }
                reason = String.format(Locale.CHINA,
                        "位移=(%.0f,%.0f)mm 速率=%.3fmm/ms",
                        match.getXMm(), match.getYMm(), rate);
            } else if (match.isAccepted()) {
                reason = String.format(Locale.CHINA,
                        "扫描匹配方向/横移不符 位移=(%.0f,%.0f)mm 航向=%.1f°",
                        match.getXMm(), match.getYMm(), match.getHeadingDeg());
            }
        } else {
            accepted = yawValid && yawMagnitude >= 5.0;
            if (accepted) {
                double signedHeading = command == CarCommand.LEFT ? yawMagnitude : -yawMagnitude;
                MotionCalibrationSample.Command sampleCommand = command == CarCommand.LEFT
                        ? MotionCalibrationSample.Command.TURN_LEFT
                        : MotionCalibrationSample.Command.TURN_RIGHT;
                autoCalibrationSamples.add(new MotionCalibrationSample(
                        sampleCommand,
                        durationMs,
                        new Pose2d(0.0, 0.0, 0.0),
                        new Pose2d(0.0, 0.0, Math.toRadians(signedHeading))
                ));
                double rate = yawMagnitude / durationMs;
                (command == CarCommand.LEFT ? autoLeftRates : autoRightRates).add(rate);
                reason = String.format(Locale.CHINA,
                        "双IMU角度=%.1f/%.1f° 速率=%.3f°/ms 雷达平移=%.0fmm",
                        yaw0, yaw1, rate, Math.hypot(match.getXMm(), match.getYMm()));
            } else {
                reason = String.format(Locale.CHINA,
                        "双IMU转角不可信 %.1f/%.1f°", yaw0, yaw1);
            }
        }

        appendLog("自动标定", String.format(Locale.CHINA,
                "%d/%d %s %s · flow=(%d,%d) · lidar=%s inliers=%d rms=%.0fmm",
                autoCalibrationActionIndex + 1, autoCalibrationActionCount(),
                describeCommand(command), accepted ? "通过" : "拒绝",
                flowX, flowY, reason, match.getInliers(), match.getRmsMm()));
        autoCalibrationActionIndex++;
        autoCalibrationStartSnapshot = null;
        if (autoCalibrationActionIndex >= autoCalibrationActionCount()) {
            finishAutoCalibration();
            return;
        }
        autoCalibrationStatus = "第 " + autoCalibrationActionIndex + " 段完成，准备下一段";
        updateMappingSummary();
        handler.postDelayed(() -> waitForAutoCalibrationStartSample(
                System.currentTimeMillis() + AUTO_CALIBRATION_SAMPLE_TIMEOUT_MS), 450L);
    }

    private void finishAutoCalibration() {
        if (!autoCalibrationRunning) return;
        autoCalibrationRunning = false;
        autoCalibrationCommand = null;
        handler.removeCallbacks(autoCalibrationDriveRunnable);
        occupancyGridView.setMotionActive(false);
        if (isCarConnected()) sendTransportEmergencyStop();
        updateAutoCalibrationButton();

        MotionCalibrationResult result = motionCalibrationEngine.solve(autoCalibrationSamples);
        boolean complete = autoForwardRates.size() >= 2
                && autoReverseRates.size() >= 2
                && autoLeftRates.size() >= 2
                && autoRightRates.size() >= 2;
        boolean stable = isStableCalibration(autoForwardRates, 0.25)
                && isStableCalibration(autoReverseRates, 0.25)
                && isStableCalibration(autoLeftRates, 0.22)
                && isStableCalibration(autoRightRates, 0.22);
        boolean plausible = isFiniteInRange(result.getForwardMmPerMs(), 0.05, 1.50)
                && isFiniteInRange(result.getReverseMmPerMs(), 0.05, 1.50)
                && isFiniteInRange(result.getLeftDegPerMs(), 0.03, 0.80)
                && isFiniteInRange(result.getRightDegPerMs(), 0.03, 0.80);
        boolean flowValid = autoFlowCountsPerMm.size() >= 2
                && isStableCalibration(autoFlowCountsPerMm, 0.40)
                && median(autoFlowCountsPerMm) > 0.005;
        boolean passed = complete && stable && plausible && flowValid;

        if (passed) {
            android.content.SharedPreferences.Editor editor =
                    getSharedPreferences("motion_calibration", MODE_PRIVATE).edit();
            editor.putFloat("linear_mm_per_ms", (float) result.getForwardMmPerMs());
            if (lastSpeed > 0) editor.putInt("linear_speed_setting", lastSpeed);
            editor.putFloat("reverse_mm_per_ms", (float) result.getReverseMmPerMs());
            editor.putFloat("left_deg_per_ms", (float) result.getLeftDegPerMs());
            editor.putFloat("right_deg_per_ms", (float) result.getRightDegPerMs());
            editor.putFloat("flow_counts_per_mm", (float) median(autoFlowCountsPerMm));
            editor.putBoolean("flow_forward_axis_x",
                    autoFlowXAxisVotes * 2 >= autoFlowCountsPerMm.size());
            editor.putBoolean("flow_forward_positive",
                    autoFlowPositiveVotes >= autoFlowNegativeVotes);
            editor.apply();
            lastParameterTableSignature = "";
            loadMotionCalibration();
            updateParameterTable(null, lastSelectedParameter);
            autoCalibrationStatus = String.format(Locale.CHINA,
                    "通过：前进 %.3fmm/ms，左 %.3f°/ms，右 %.3f°/ms",
                    result.getForwardMmPerMs(), result.getLeftDegPerMs(), result.getRightDegPerMs());
        } else {
            autoCalibrationStatus = "未通过：旧参数保持不变";
        }
        updateMappingSummary();
        appendLog("自动标定", autoCalibrationStatus + " / 前后左右样本="
                + autoForwardRates.size() + "/" + autoReverseRates.size() + "/"
                + autoLeftRates.size() + "/" + autoRightRates.size()
                + " / flow=" + autoFlowCountsPerMm.size());

        String details = String.format(Locale.CHINA,
                "前进 %.3f mm/ms（%d）\n后退 %.3f mm/ms（%d）\n"
                        + "左转 %.3f °/ms（%d）\n右转 %.3f °/ms（%d）\n"
                        + "光流 %.4f count/mm（%d）\n\n完整=%s  重复性=%s  数值范围=%s  光流=%s",
                result.getForwardMmPerMs(), autoForwardRates.size(),
                result.getReverseMmPerMs(), autoReverseRates.size(),
                result.getLeftDegPerMs(), autoLeftRates.size(),
                result.getRightDegPerMs(), autoRightRates.size(),
                median(autoFlowCountsPerMm), autoFlowCountsPerMm.size(),
                complete ? "通过" : "失败", stable ? "通过" : "失败",
                plausible ? "通过" : "失败", flowValid ? "通过" : "失败");
        new AlertDialog.Builder(this)
                .setTitle(passed ? "自动标定通过并已保存" : "自动标定未通过")
                .setMessage(details + (passed ? "" : "\n\n旧参数未被覆盖，请查看日志中的逐段拒绝原因。"))
                .setPositiveButton("确定", null)
                .show();
    }

    private void stopAutoCalibration(String reason, boolean sendStop) {
        if (!autoCalibrationRunning) return;
        autoCalibrationRunning = false;
        autoCalibrationCommand = null;
        autoCalibrationStartSnapshot = null;
        handler.removeCallbacks(autoCalibrationDriveRunnable);
        occupancyGridView.setMotionActive(false);
        if (sendStop && isCarConnected()) sendTransportEmergencyStop();
        autoCalibrationStatus = "中止：" + reason;
        startMappingAfterCalibration = false;
        updateAutoCalibrationButton();
        updateMappingSummary();
        appendLog("自动标定", autoCalibrationStatus + "，本轮未写入参数");
    }

    private void showAutoCalibrationFailure(String reason) {
        new AlertDialog.Builder(this)
                .setTitle("自动标定已停止")
                .setMessage(reason + "\n\n本轮数据没有覆盖原有参数。")
                .setPositiveButton("确定", null)
                .show();
    }

    private void handleLocalCalibrationStatus(StatusFrame frame) {
        if (frame.getLocalCalibrationState() < 0) return;
        boolean wasRunning = autoCalibrationRunning;
        if (frame.isLocalCalibrationBusy()) {
            autoCalibrationRunning = true;
            autoCalibrationStatus = describeLocalCalibrationState(
                    frame.getLocalCalibrationState());
        } else if (frame.isLocalCalibrationResultValid()) {
            autoCalibrationRunning = false;
            boolean beginMapping = wasRunning && startMappingAfterCalibration;
            startMappingAfterCalibration = false;
            autoCalibrationStatus = "完成：静态修正 " + frame.getLearnedYawTrim()
                    + "，陀螺峰值 " + frame.getCalibrationGyroPeak();
            if (wasRunning) {
                new AlertDialog.Builder(this)
                        .setTitle("板载校准完成")
                        .setMessage("FPGA 已保存本次运行期修正量。\n\n静态左右侧修正："
                                + frame.getLearnedYawTrim()
                                + "\n校准陀螺峰值：" + frame.getCalibrationGyroPeak()
                                + "\n极性学习："
                                + (frame.isGyroPolarityValid() ? "有效" : "无效"))
                        .setPositiveButton("确定", null)
                        .show();
            }
            if (beginMapping) {
                appendLog("板载建图", "FPGA校准通过，准备下发当前位置与场地边界");
                handler.postDelayed(this::startExploration, 350L);
            }
        } else if (frame.isLocalCalibrationFailed()) {
            autoCalibrationRunning = false;
            startMappingAfterCalibration = false;
            String failureReason = describeLocalCalibrationState(
                    frame.getLocalCalibrationState());
            autoCalibrationStatus = "失败：" + failureReason;
            if (wasRunning) showAutoCalibrationFailure(
                    "FPGA 本地校准未通过：" + failureReason
                            + "。陀螺峰值=" + frame.getCalibrationGyroPeak());
        }
        updateAutoCalibrationButton();
        updateMappingSummary();
    }

    private String describeLocalCalibrationState(int state) {
        switch (state) {
            case 0x0: return "待机";
            case 0x1: return "强化停车并等待稳定";
            case 0x2: return "左转采样";
            case 0x3: return "左转后稳定";
            case 0x4: return "右转采样";
            case 0x5: return "右转后验证极性";
            case 0x6: return "短前进并学习偏航修正";
            case 0x7: return "前进后稳定";
            case 0x8: return "短后退回位";
            case 0x9: return "最终强化停车";
            case 0xA: return "已完成";
            case 0xB: return "被停止、障碍或安全条件中止";
            case 0xC: return "回位后评估本轮收敛";
            case 0xD: return "融合陀螺样本中断";
            case 0xE: return "转向极性或陀螺运动不足";
            case 0xF: return "达到最大轮数仍未收敛";
            default: return "阶段 " + state;
        }
    }

    private void updateAutoCalibrationButton() {
        MaterialButton button = findViewById(R.id.mappingCalibrationButton);
        if (button == null) return;
        button.setText(autoCalibrationRunning ? "停止" : "标定");
    }

    private static boolean isFiniteInRange(double value, double minimum, double maximum) {
        return !Double.isNaN(value) && !Double.isInfinite(value)
                && value >= minimum && value <= maximum;
    }

    private static boolean isStableCalibration(List<Double> values, double relativeSpreadLimit) {
        if (values.size() < 2) return false;
        double center = median(values);
        if (center <= 0.0 || Double.isNaN(center)) return false;
        List<Double> deviations = new ArrayList<>();
        for (double value : values) deviations.add(Math.abs(value - center));
        return median(deviations) / center <= relativeSpreadLimit;
    }

    private static double median(List<Double> values) {
        if (values == null || values.isEmpty()) return Double.NaN;
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int middle = sorted.size() / 2;
        return (sorted.size() & 1) != 0
                ? sorted.get(middle)
                : (sorted.get(middle - 1) + sorted.get(middle)) * 0.5;
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    private static final class MotionSnapshot {
        private final LidarMapScan scan;
        private final long motionReceivedAtMs;
        private final int flowAccumX;
        private final int flowAccumY;
        private final double yaw0Deg;
        private final double yaw1Deg;

        private MotionSnapshot(LidarMapScan scan, long motionReceivedAtMs,
                               int flowAccumX, int flowAccumY,
                               double yaw0Deg, double yaw1Deg) {
            this.scan = scan;
            this.motionReceivedAtMs = motionReceivedAtMs;
            this.flowAccumX = flowAccumX;
            this.flowAccumY = flowAccumY;
            this.yaw0Deg = yaw0Deg;
            this.yaw1Deg = yaw1Deg;
        }
    }

    private void startExploration() {
        if (autoCalibrationRunning) {
            Toast.makeText(this, "请先完成或停止自动标定", Toast.LENGTH_SHORT).show();
            return;
        }
        if (!isCarConnected()) {
            Toast.makeText(this, "请先连接小车", Toast.LENGTH_SHORT).show();
            return;
        }
        UwbVehiclePose pose = getFreshServerPose();
        if (pose == null || !pose.hasPosition()) {
            Toast.makeText(this, "等待至少一个有效UWB标签位置", Toast.LENGTH_SHORT).show();
            return;
        }
        Double initialHeadingDeg = getExplorationInitialHeadingDeg(pose);
        if (initialHeadingDeg == null) {
            Toast.makeText(this, "单标签探索前请先完成方向标定", Toast.LENGTH_SHORT).show();
            return;
        }
        long now = System.currentTimeMillis();
        if (latestOnboardMapStatus == null || now - lastOnboardMapStatusAt > 3000L) {
            if (explorationPreflightPending) return;
            explorationPreflightPending = true;
            setStatusOnly("自动探索：正在读取FPGA最新状态");
            sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "探索启动预检");
            handler.postDelayed(() -> {
                if (!explorationPreflightPending) return;
                explorationPreflightPending = false;
                if (latestOnboardMapStatus != null
                        && System.currentTimeMillis() - lastOnboardMapStatusAt <= 3000L) {
                    startExploration();
                } else {
                    setStatusOnly("自动探索未启动：FPGA未返回最新@G0状态");
                    Toast.makeText(this, "未收到FPGA地图状态，请检查蓝牙数据上报",
                            Toast.LENGTH_LONG).show();
                }
            }, 900L);
            return;
        }
        if ((latestOnboardMapStatus.getPoseFlags() & 0x01) == 0) {
            Toast.makeText(this, "等待FPGA陀螺初始化完成", Toast.LENGTH_SHORT).show();
            setStatusOnly("自动探索未启动：融合陀螺尚未就绪");
            return;
        }
        if ((latestOnboardMapStatus.getPoseFlags() & 0x02) == 0) {
            Toast.makeText(this, "请先完成板载自动标定，再启动探索", Toast.LENGTH_LONG).show();
            setStatusOnly("自动探索未启动：陀螺转向极性尚未标定");
            return;
        }
        if (navigationRunning) stopNavigation("切换到自动探索", true);
        if (!sendFieldMapConfiguration(pose, "自动探索", true, initialHeadingDeg)) return;
        explorationRunning = true;
        explorationUnconfirmedActions = 0;
        explorationStartAttempts = 0;
        explorationStartRequestedAt = System.currentTimeMillis();
        mappingExploreButton.setText("立即停止");
        mappingExploreButton.setBackgroundTintList(ColorStateList.valueOf(
                ContextCompat.getColor(this, R.color.danger)));
        occupancyGridView.setPaused(false);
        if (lastLidarEnabled != 1) sendPriorityCarCommand(CarCommand.ENABLE_LIDAR, "自动探索");
        handler.postDelayed(this::requestOnboardExplorationStart, 320L);
        appendLog("自动探索", "当前位置和基站边界已下发，FPGA离线探索，UWB标签="
                + pose.getValidTagCount());
        updateMappingSummary();
    }

    private void requestOnboardExplorationStart() {
        if (!explorationRunning || !isCarConnected()) return;
        explorationStartAttempts++;
        sendPriorityCarCommand(CarCommand.ENTER_AUTO_MODE, "板载探索接管");
        handler.postDelayed(() -> {
            if (!explorationRunning || !isCarConnected()) return;
            sendPriorityCarCommand(CarCommand.START_ONBOARD_MAPPING, "板载探索建图");
            handler.postDelayed(() -> {
                if (!explorationRunning || !isCarConnected()) return;
                sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "板载探索启动确认");
                boolean acknowledged = latestOnboardMapStatus != null
                        && latestOnboardMapStatus.isMapActive()
                        && !latestOnboardMapStatus.isMapPaused()
                        && lastOnboardMapStatusAt >= explorationStartRequestedAt;
                if (acknowledged) {
                    setStatusOnly("自动探索已由FPGA接管");
                } else if (explorationStartAttempts < 3) {
                    appendLog("自动探索", "未收到板载启动确认，重试 "
                            + explorationStartAttempts + "/3");
                    handler.postDelayed(this::requestOnboardExplorationStart, 500L);
                } else {
                    stopExploration("连续三次未收到板载启动确认");
                    setStatusOnly("自动探索未启动：请检查陀螺标定和雷达状态");
                }
            }, 700L);
        }, 220L);
    }

    private Double getExplorationInitialHeadingDeg(UwbVehiclePose pose) {
        Float boardHeading = getBoardWorldHeadingDeg();
        if (boardHeading != null) return (double) boardHeading;
        if (imuWorldHeadingManuallyAligned) return manualAbsoluteHeadingDeg;
        if (pose != null && pose.hasCoordinates()) {
            return (double) getFusedWorldHeadingDeg(pose);
        }
        return null;
    }

    private void stopExploration(String reason) {
        explorationRunning = false;
        explorationPreflightPending = false;
        explorationStartAttempts = 0;
        explorationCommand = null;
        explorationAwaitingEvidence = false;
        occupancyGridView.setMotionActive(false);
        handler.removeCallbacks(explorationDriveRunnable);
        sendTransportEmergencyStop();
        if (isCarConnected())
            sendPriorityCarCommand(CarCommand.PAUSE_ONBOARD_MAPPING, "停止板载探索");
        if (mappingExploreButton != null) {
            mappingExploreButton.setText("自动探索");
            mappingExploreButton.setBackgroundTintList(ColorStateList.valueOf(
                    ContextCompat.getColor(this, R.color.primary)));
        }
        appendLog("自动探索", "停止：" + reason);
        updateMappingSummary();
    }

    /**
     * Sends the phone-owned field geometry and current UWB/IMU pose to FPGA.
     * The punctuation-only 17-byte frame coexists with the legacy one-byte
     * command protocol and fits in one BLE ATT write.
     */
    private boolean sendFieldMapConfiguration(UwbVehiclePose pose, String reason) {
        return sendFieldMapConfiguration(pose, reason, false, null);
    }

    private boolean sendFieldMapConfiguration(UwbVehiclePose pose, String reason,
                                                boolean allowApproximatePosition,
                                                Double headingOverrideDeg) {
        boolean positionAccepted = pose != null && (allowApproximatePosition
                ? pose.hasPosition() : pose.isNavigationReliable());
        if (!positionAccepted || (!allowApproximatePosition && !filteredUwbValid)) {
            Toast.makeText(this, allowApproximatePosition
                            ? "等待至少一个有效UWB标签位置"
                            : "等待四标签UWB可靠定位",
                    Toast.LENGTH_SHORT).show();
            return false;
        }
        double[][] anchors = occupancyGridView.getAnchorPositions();
        double axisX = anchors[2][0] - anchors[0][0];
        double axisY = anchors[2][1] - anchors[0][1];
        double sideX = anchors[3][0] - anchors[0][0];
        double sideY = anchors[3][1] - anchors[0][1];
        double widthM = Math.hypot(axisX, axisY);
        double heightM = Math.hypot(sideX, sideY);
        if (widthM < 0.30 || heightM < 0.30) {
            Toast.makeText(this, "基站A-C或A-D距离过小", Toast.LENGTH_LONG).show();
            return false;
        }
        double ux = axisX / widthM;
        double uy = axisY / widthM;
        double vx = sideX / heightM;
        double vy = sideY / heightM;
        if (Math.abs(ux * vx + uy * vy) > 0.30) {
            Toast.makeText(this, "基站A-C与A-D方向不接近垂直", Toast.LENGTH_LONG).show();
            return false;
        }

        int widthCells = Math.max(1, (int) Math.round(widthM * 1000.0 / ONBOARD_MAP_CELL_MM));
        int heightCells = Math.max(1, (int) Math.round(heightM * 1000.0 / ONBOARD_MAP_CELL_MM));
        int largestDimension = Math.max(widthCells, heightCells);
        int padding = Math.min(ONBOARD_MAP_DESIRED_PADDING_CELLS,
                Math.max(1, (ONBOARD_MAP_SIZE - 1 - largestDimension) / 2));
        int marginCells = Math.max(1, (int) Math.ceil(
                (Math.hypot(VehicleGeometry.TAG_RECTANGLE_LENGTH_M * 0.5,
                        VehicleGeometry.TAG_RECTANGLE_WIDTH_M * 0.5) + 0.075)
                        * 1000.0 / ONBOARD_MAP_CELL_MM));
        if (widthCells + 2 * padding >= ONBOARD_MAP_SIZE
                || heightCells + 2 * padding >= ONBOARD_MAP_SIZE
                || widthCells <= 2 * marginCells || heightCells <= 2 * marginCells) {
            Toast.makeText(this, "基站场地尺寸超出板载6.4m地图或小于车体安全范围",
                    Toast.LENGTH_LONG).show();
            return false;
        }

        boolean useFilteredPosition = pose.isNavigationReliable() && filteredUwbValid;
        double configurationXM = useFilteredPosition ? filteredUwbXM : pose.getXM();
        double configurationYM = useFilteredPosition ? filteredUwbYM : pose.getYM();
        double dx = configurationXM - anchors[0][0];
        double dy = configurationYM - anchors[0][1];
        int poseX = clampByte(padding + (int) Math.round((dx * ux + dy * uy)
                * 1000.0 / ONBOARD_MAP_CELL_MM));
        int poseY = clampByte(padding + (int) Math.round((dx * vx + dy * vy)
                * 1000.0 / ONBOARD_MAP_CELL_MM));
        double fieldAxisHeadingDeg =
                NavigationSteering.vehicleHeadingForWorldVector(axisX, axisY);
        double worldHeadingDeg = headingOverrideDeg == null
                ? getFusedWorldHeadingDeg(pose) : headingOverrideDeg;
        int heading1024 = ((int) Math.round(normalizeDegrees(
                worldHeadingDeg - fieldAxisHeadingDeg) * 1024.0 / 360.0)) & 0x3ff;
        int lowX = padding + marginCells;
        int highX = padding + widthCells - marginCells;
        int lowY = padding + marginCells;
        int highY = padding + heightCells - marginCells;
        String frame = encodeFieldMapFrame(
                poseX, poseY, heading1024, lowX, highX, lowY, highY);
        sendTransportScript(frame);
        // A duplicate complete frame is cheap and avoids making map start
        // depend on one no-flow-control BLE-to-UART transfer.
        handler.postDelayed(() -> {
            if (isCarConnected()) sendTransportScript(frame);
        }, 100L);
        appendLog("场地配置", String.format(Locale.CHINA,
                "%s pose=%d,%d heading=%d bounds=%d..%d,%d..%d",
                reason, poseX, poseY, heading1024, lowX, highX, lowY, highY));
        return true;
    }

    private static int clampByte(int value) {
        return Math.max(0, Math.min(127, value));
    }

    private static String encodeFieldMapFrame(int poseX, int poseY, int heading,
                                              int lowX, int highX,
                                              int lowY, int highY) {
        final String alphabet = "!#$%&'()*,./:;<>";
        int[] values = {poseX, poseY, heading, lowX, highX, lowY, highY};
        int[] widths = {2, 2, 3, 2, 2, 2, 2};
        StringBuilder result = new StringBuilder(17).append('{');
        for (int field = 0; field < values.length; field++) {
            for (int shift = (widths[field] - 1) * 4; shift >= 0; shift -= 4) {
                result.append(alphabet.charAt((values[field] >>> shift) & 0x0f));
            }
        }
        return result.append('}').toString();
    }

    private void startNavigationToSelectedGoal(int waypointCount) {
        if (!isCarConnected()) {
            stopNavigation("蓝牙未连接", false);
            return;
        }
        navigationRunning = true;
        navigationReplanning = false;
        navigationReplanCount = 0;
        resetNavigationTurnProgress();
        mappingNavigateButton.setText("重新选择目的地");
        mappingStopNavigationButton.setEnabled(true);
        occupancyGridView.setPaused(false);
        sendPriorityCarCommand(CarCommand.ENTER_MANUAL_MODE, "自动导航接管");
        if (lastLidarEnabled != 1) {
            sendPriorityCarCommand(CarCommand.ENABLE_LIDAR, "自动导航避障");
        }
        appendLog("自动导航", "目的地已确认，显示路径点=" + waypointCount);
        handler.postDelayed(() -> runNavigationDecision("开始导航"), 350L);
        updateMappingSummary();
    }

    private void runNavigationDecision(String reason) {
        if (!navigationRunning || navigationCommand != null) return;
        if (!isCarConnected()) {
            stopNavigation("蓝牙断开，FPGA安全看门狗停车", false);
            return;
        }
        navigationReplanning = true;
        boolean found = occupancyGridView.replanCurrentGoal();
        navigationReplanning = false;
        if (!found) {
            stopNavigation("障碍封闭通道，无可行绕行路径", true);
            setStatusOnly("自动导航停止：当前地图内没有安全路径");
            return;
        }
        GridPathPlanner.Cell goal = occupancyGridView.getNavigationGoal();
        int currentX = Math.round(occupancyGridView.getCarGridX());
        int currentY = Math.round(occupancyGridView.getCarGridY());
        if (goal == null || Math.hypot(goal.x - currentX, goal.y - currentY) <= 4.0) {
            stopNavigation("已到达目的地", true);
            occupancyGridView.clearNavigationPath();
            updateMappingSummary();
            setStatusOnly("自动导航完成：已到达目的地");
            return;
        }
        List<GridPathPlanner.Cell> raw = occupancyGridView.getRawPlannedPath();
        if (raw.size() < 2) {
            stopNavigation("路径退化，无可执行方向", true);
            return;
        }
        // Track a nearby point. A long look-ahead cuts corners around inflated
        // obstacles and makes UWB latency look like a steering error.
        GridPathPlanner.Cell next = raw.get(Math.min(2, raw.size() - 1));
        float targetHeading = NavigationSteering.vehicleHeadingForWorldVector(
                next.x - currentX, next.y - currentY);
        float headingError = NavigationSteering.normalizeHeadingError(
                targetHeading - occupancyGridView.getHeadingDegrees());
        Log.d("VehicleNavigationTrace", String.format(Locale.US,
                "%s cell=(%d,%d) next=(%d,%d) targetCW=%.1f currentCW=%.1f error=%.1f",
                reason, currentX, currentY, next.x, next.y, targetHeading,
                occupancyGridView.getHeadingDegrees(), headingError));
        CarCommand command;
        int duration;
        if (Math.abs(headingError) > 12f) {
            if (!navigationTurnIsConverging(targetHeading, Math.abs(headingError))) {
                stopNavigation("转向误差连续未收敛，已安全停车", true);
                setStatusOnly("自动导航停止：朝向反馈或左右转方向异常");
                return;
            }
            char turnPayload = NavigationSteering.commandForHeadingError(headingError);
            command = turnPayload == 'A' ? CarCommand.LEFT : CarCommand.RIGHT;
            duration = navigationTurnDurationMs(command, Math.abs(headingError));
        } else {
            resetNavigationTurnProgress();
            command = CarCommand.UP;
            duration = navigationForwardDurationMs();
        }
        Log.d("VehicleNavigationTrace", "decision=" + command.getPayload()
                + " durationMs=" + duration);
        if (command == CarCommand.UP && !navigationRadarAllowsForward()) {
            sendTransportEmergencyStop();
            setStatusOnly("自动导航等待前方安全雷达数据");
            handler.postDelayed(() -> runNavigationDecision("雷达安全复查"), 300L);
            return;
        }
        if (command == CarCommand.UP
                && !occupancyGridView.canExecuteMotionWithinAnchorBoundary('W', duration)) {
            stopNavigation("预测动作将越出基站安全边界", true);
            setStatusOnly("自动导航停止：车体将越出四基站范围");
            return;
        }
        navigationCommand = command;
        navigationCommandDurationMs = duration;
        navigationCommandEndAt = System.currentTimeMillis() + duration;
        if (!sendProfiledMotion(command, "自动导航")) {
            navigationCommand = null;
            navigationCommandDurationMs = 0;
            handler.postDelayed(() -> runNavigationDecision("等待轮速同步"), 500L);
            return;
        }
        occupancyGridView.setMotionActive(true);
        appendLog("自动导航", reason + " -> " + describeCommand(command)
                + "，航向误差=" + String.format(Locale.CHINA, "%.1f", headingError));
        handler.postDelayed(navigationDriveRunnable, 90L);
        updateMappingSummary();
    }

    private void handleNavigationObstacle(RadarFrame frame) {
        if (!navigationRunning || navigationReplanning) return;
        long now = System.currentTimeMillis();
        if (now - lastNavigationObstacleAt < 700L) return;
        lastNavigationObstacleAt = now;
        navigationReplanCount++;
        handler.removeCallbacks(navigationDriveRunnable);
        navigationCommand = null;
        navigationCommandDurationMs = 0;
        resetNavigationTurnProgress();
        sendTransportEmergencyStop();
        occupancyGridView.setMotionActive(false);
        // Integrate the safety frame again after motion is stopped. The normal
        // mapper intentionally ignores scans during movement to avoid smearing.
        occupancyGridView.addSafetyObstacleFrame(frame);
        appendLog("自动导航", "检测到障碍，停止当前动作并第 "
                + navigationReplanCount + " 次重规划");
        setStatusOnly("自动导航遇障，正在尝试绕行");
        handler.postDelayed(() -> runNavigationDecision("遇障重规划"), 300L);
    }

    private void stopNavigation(String reason, boolean sendStop) {
        boolean wasRunning = navigationRunning;
        navigationRunning = false;
        navigationReplanning = false;
        navigationCommand = null;
        navigationCommandDurationMs = 0;
        handler.removeCallbacks(navigationDriveRunnable);
        if (occupancyGridView != null) occupancyGridView.setMotionActive(false);
        if (sendStop && isCarConnected()) sendTransportEmergencyStop();
        if (mappingNavigateButton != null) mappingNavigateButton.setText("选择目的地");
        if (mappingStopNavigationButton != null) mappingStopNavigationButton.setEnabled(false);
        if (wasRunning) appendLog("自动导航", "停止：" + reason);
        updateMappingSummary();
    }

    private int navigationForwardDurationMs() {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        float mmPerMs = preferences.getFloat("linear_mm_per_ms", 0.38f);
        if (mmPerMs <= 0f) mmPerMs = 0.38f;
        // Short closed-loop bursts keep the real chassis close to the latest
        // UWB pose and leave enough time to stop before the next lidar scan.
        return Math.max(100, Math.min(180, Math.round(50f / mmPerMs)));
    }

    private int navigationTurnDurationMs(CarCommand command, float errorDegrees) {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        float degreesPerMs = command == CarCommand.LEFT
                ? preferences.getFloat("left_deg_per_ms", 0.18f)
                : preferences.getFloat("right_deg_per_ms", 0.18f);
        return NavigationSteering.boundedTurnPulseDurationMs(
                errorDegrees, degreesPerMs);
    }

    private boolean navigationRadarAllowsForward() {
        RadarFrame radar = latestRadarFrame;
        long now = System.currentTimeMillis();
        int configuredSafeMm = lastSafeDistanceMm > 0 ? lastSafeDistanceMm : 450;
        if (radar != null && radar.getRadarOnline() == 1 && now - lastRadarAt <= 2000L) {
            int[] sectors = radar.getSectorDistancesMm();
            if (sectors == null || sectors.length < 8) return false;
            int front = minimumValid(
                    sectors[7], sectors[0], sectors[1], radar.getFrontDistance());
            int threshold = Math.max(300, radar.hasValue(radar.getSafeDistanceMm())
                    ? radar.getSafeDistanceMm() : configuredSafeMm);
            boolean allowed = radar.getObstacleTriggered() != 1
                    && radar.getMotorBlocked() != 1 && front > threshold;
            Log.d("VehicleNavigationTrace", "forwardSafety=sector ageMs="
                    + (now - lastRadarAt) + " frontMm=" + front
                    + " thresholdMm=" + threshold + " allowed=" + allowed);
            return allowed;
        }

        boolean statusSafe = latestStatusFrame != null && now - lastStatusAt <= 2500L
                && latestStatusFrame.getRadarOnline() == 1
                && latestStatusFrame.getMotorBlocked() != 1;
        int detailedFront = NavigationSafety.detailedFrontClearanceMm(latestLidarMapScan);
        long detailedAge = lastLidarMapAt <= 0L ? Long.MAX_VALUE : now - lastLidarMapAt;
        int threshold = Math.max(300, latestStatusFrame != null
                && latestStatusFrame.getSafeDistanceMm() > 0
                ? latestStatusFrame.getSafeDistanceMm() : configuredSafeMm);
        boolean allowed = statusSafe && detailedAge <= 2000L && detailedFront > threshold;
        Log.d("VehicleNavigationTrace", "forwardSafety=detailed statusSafe=" + statusSafe
                + " ageMs=" + detailedAge + " frontMm=" + detailedFront
                + " thresholdMm=" + threshold + " allowed=" + allowed);
        return allowed;
    }

    private boolean navigationTurnIsConverging(float targetHeading, float errorMagnitude) {
        if (!Float.isNaN(navigationLastTurnTargetHeading)
                && Math.abs(NavigationSteering.normalizeHeadingError(
                targetHeading - navigationLastTurnTargetHeading)) <= 20f) {
            float improvement = navigationLastTurnErrorMagnitude - errorMagnitude;
            if (improvement < NAVIGATION_TURN_MIN_PROGRESS_DEG) {
                navigationTurnNoProgressCount++;
            } else {
                navigationTurnNoProgressCount = 0;
            }
        } else {
            navigationTurnNoProgressCount = 0;
        }
        navigationLastTurnTargetHeading = targetHeading;
        navigationLastTurnErrorMagnitude = errorMagnitude;
        return navigationTurnNoProgressCount < NAVIGATION_TURN_NO_PROGRESS_LIMIT;
    }

    private void resetNavigationTurnProgress() {
        navigationLastTurnTargetHeading = Float.NaN;
        navigationLastTurnErrorMagnitude = Float.NaN;
        navigationTurnNoProgressCount = 0;
    }

    private void runExplorationDecision() {
        if (!explorationRunning) return;
        if (currentPage != pageMapping || !isCarConnected()) {
            stopExploration("页面或连接状态改变");
            return;
        }
        RadarFrame radar = latestRadarFrame;
        if (radar == null || radar.getRadarOnline() != 1
                || System.currentTimeMillis() - lastRadarAt > 1500L) {
            sendTransportEmergencyStop();
            setStatusOnly("自动探索等待有效雷达数据");
            handler.postDelayed(this::runExplorationDecision, 500L);
            return;
        }

        int[] sectors = radar.getSectorDistancesMm();
        int threshold = Math.max(400, radar.getSafeDistanceMm() + 200);
        int front = minimumValid(sectors[7], sectors[0], sectors[1], radar.getFrontDistance());
        int left = clearanceScore(sectors[6], sectors[7]);
        int right = clearanceScore(sectors[1], sectors[2]);
        if (front == 0 && left == 0 && right == 0) {
            sendTransportEmergencyStop();
            setStatusOnly("自动探索等待有效方向距离");
            handler.postDelayed(this::runExplorationDecision, 500L);
            return;
        }
        CarCommand command;
        int duration;
        if (radar.getObstacleTriggered() != 1 && radar.getMotorBlocked() != 1 && front > threshold) {
            command = CarCommand.UP;
            duration = 320;
        } else {
            command = left >= right ? CarCommand.LEFT : CarCommand.RIGHT;
            duration = calibratedQuarterTurnDuration(command);
        }
        if (command == CarCommand.UP
                && !occupancyGridView.canExecuteMotionWithinAnchorBoundary('W', duration)) {
            float centerError = occupancyGridView.getHeadingErrorToAnchorCenter();
            command = centerError < 0f ? CarCommand.LEFT : CarCommand.RIGHT;
            duration = calibratedQuarterTurnDuration(command);
            appendLog("自动探索", "接近基站边界，转向场地中心");
        }
        beginExplorationAction(command, duration);
    }

    private void beginExplorationAction(CarCommand command, int durationMs) {
        explorationCommand = command;
        explorationActionDurationMs = durationMs;
        explorationMotorConfirmed = false;
        explorationRadarMotionConfirmed = false;
        explorationAwaitingEvidence = false;
        explorationStartSectors = latestRadarFrame == null
                ? null : latestRadarFrame.getSectorDistancesMm();
        explorationCommandEndAt = System.currentTimeMillis() + durationMs;
        if (!sendProfiledMotion(command, "自动探索轮速")) {
            explorationCommand = null;
            handler.postDelayed(this::runExplorationDecision, 500L);
            return;
        }
        occupancyGridView.setMotionActive(true);
        appendLog("自动探索", command.getPayload() + " " + describeCommand(command));
        updateMappingSummary();
        long confirmationDelayMs = Math.max(120L, Math.min(180L, durationMs / 2L));
        handler.postDelayed(() -> {
            if (explorationRunning && explorationCommand == command && !explorationAwaitingEvidence) {
                sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "自动探索运动确认");
            }
        }, confirmationDelayMs);
        handler.postDelayed(explorationDriveRunnable, 90L);
    }

    private void finishExplorationAction(CarCommand completed) {
        explorationAwaitingEvidence = false;
        if (!explorationRunning) return;
        boolean motionConfirmed = explorationMotorConfirmed || explorationRadarMotionConfirmed;
        if (motionConfirmed) {
            explorationUnconfirmedActions = 0;
            occupancyGridView.applyEstimatedMotion(
                    completed.getPayload().charAt(0), explorationActionDurationMs);
            appendLog("自动探索", "已确认运动，更新软件位姿：电机证据="
                    + explorationMotorConfirmed + " 雷达变化=" + explorationRadarMotionConfirmed);
        } else {
            explorationUnconfirmedActions++;
            appendLog("自动探索", "未更新位姿：电机证据=" + explorationMotorConfirmed
                    + " 雷达变化=" + explorationRadarMotionConfirmed);
            if (explorationUnconfirmedActions >= 2) {
                stopExploration("连续两次未检测到实际运动");
                setStatusOnly("自动探索已停止：小车未产生可确认的运动");
                return;
            }
        }
        occupancyGridView.setMotionActive(false);
        updateMappingSummary();
        handler.postDelayed(this::runExplorationDecision, 300L);
    }

    private int minimumValid(int... values) {
        int result = Integer.MAX_VALUE;
        for (int value : values) {
            if (value > 0 && value != 0xFFFF) result = Math.min(result, value);
        }
        return result == Integer.MAX_VALUE ? 0 : result;
    }

    private int clearanceScore(int first, int second) {
        int a = first <= 0 || first == 0xFFFF ? 0 : Math.min(first, 5000);
        int b = second <= 0 || second == 0xFFFF ? 0 : Math.min(second, 5000);
        return a + b;
    }

    private int calibratedQuarterTurnDuration(CarCommand command) {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        float degreesPerMs = command == CarCommand.LEFT
                ? preferences.getFloat("left_deg_per_ms", 0.18f)
                : preferences.getFloat("right_deg_per_ms", 0.18f);
        if (degreesPerMs <= 0f) degreesPerMs = 0.18f;
        return Math.max(300, Math.min(900, Math.round(90f / degreesPerMs)));
    }

    private int manualCommandValue(CarCommand command) {
        if (command == null) return -1;
        switch (command) {
            case UP: return 1;
            case DOWN: return 2;
            case LEFT: return 3;
            case RIGHT: return 4;
            default: return 0;
        }
    }

    private boolean hasMeaningfulRadarChange(int[] before, int[] after) {
        if (before == null || after == null) return false;
        int totalChange = 0;
        int largestChange = 0;
        int compared = 0;
        for (int i = 0; i < Math.min(before.length, after.length); i++) {
            int first = before[i];
            int second = after[i];
            if (first <= 0 || second <= 0 || first == 0xFFFF || second == 0xFFFF) continue;
            int change = Math.abs(first - second);
            totalChange += Math.min(change, 500);
            largestChange = Math.max(largestChange, change);
            compared++;
        }
        return compared >= 2 && (largestChange >= 80 || totalChange >= 180);
    }

    private void updateSegmentButton(MaterialButton button, boolean selected) {
        if (button == null) {
            return;
        }
        int background = selected ? R.color.primary_soft : R.color.panel_background;
        int foreground = selected ? R.color.primary_dark : R.color.text_secondary;
        button.setBackgroundTintList(ColorStateList.valueOf(ContextCompat.getColor(this, background)));
        button.setTextColor(ContextCompat.getColor(this, foreground));
        button.setIconTint(ColorStateList.valueOf(ContextCompat.getColor(this, foreground)));
        button.setStrokeColor(ColorStateList.valueOf(
                ContextCompat.getColor(this, selected ? R.color.primary_soft : R.color.divider)
        ));
    }

    private void bindConnectionControls() {
        connectButton.setOnClickListener(view -> showBluetoothDeviceDialog());
        disconnectButton.setOnClickListener(view -> {
            if (explorationRunning) stopExploration("主动断开连接");
            if (calibrationRunning) stopCalibration("主动断开连接");
            if (autoCalibrationRunning) stopAutoCalibration("主动断开连接", false);
            stopHeldCommand(false);
            stopHeartbeat();
            disconnectActiveTransport();
            updateConnectionUi();
            showStatus("BLE 已断开");
        });
        requestStatusButton.setOnClickListener(view -> {
            view.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP);
            sendCarCommand(CarCommand.REQUEST_STATUS);
        });
        findViewById(R.id.copyLogButton).setOnClickListener(view -> copyDebugLog());
        findViewById(R.id.clearLogButton).setOnClickListener(view -> clearDebugLog());
    }

    private void bindDriveControls() {
        bindHeldButton(findViewById(R.id.forwardButton), CarCommand.UP);
        bindHeldButton(findViewById(R.id.backButton), CarCommand.DOWN);
        bindHeldButton(findViewById(R.id.leftButton), CarCommand.LEFT);
        bindHeldButton(findViewById(R.id.rightButton), CarCommand.RIGHT);
        driveActionButton.setOnClickListener(view -> {
            giveButtonFeedback(view, manualDriveMode ? "停止" : "确定");
            stopHeldCommand(false);
            if (manualDriveMode) {
                sendTransportEmergencyStop();
            } else {
                // In automatic route-edit mode C confirms and starts the route.
                // Immediate parking is a separate H+C action in the mode row.
                sendPriorityCarCommand(CarCommand.CENTER, "路线确定");
            }
            requestStatusSoon();
        });
    }

    @SuppressLint("ClickableViewAccessibility")
    private void bindHeldButton(View button, CarCommand command) {
        button.setOnTouchListener((view, event) -> {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                view.performClick();
                setDriveButtonPressed(view, true);
                giveButtonFeedback(view, describeDriveCommand(command));
                if (manualDriveMode) {
                    startHeldCommand(command);
                } else {
                    sendCarCommand(command, "路线按键");
                    requestStatusSoon();
                }
                return true;
            }
            if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
                setDriveButtonPressed(view, false);
                if (manualDriveMode) {
                    stopHeldCommand(true);
                }
                return true;
            }
            return true;
        });
    }

    private void startHeldCommand(CarCommand command) {
        handler.removeCallbacks(holdRepeatRunnable);
        if (!isCarConnected()) {
            setStatusOnly(getActiveTransportName() + "未连接");
            return;
        }
        // Four wheel defaults are persistent chassis calibration values.  A
        // direction press must not rewrite them through a multi-packet script:
        // releasing during that script can leave only part of the wheels updated.
        sendPriorityCarCommand(command, "方向控制");
        heldCommand = command;
        occupancyGridView.setMotionActive(true);
        handler.postDelayed(holdRepeatRunnable, HOLD_REPEAT_INTERVAL_MS);
    }

    private boolean sendProfiledMotion(CarCommand command, String tag) {
        int profile = command == CarCommand.UP ? 0
                : (command == CarCommand.DOWN ? 1 : 2);
        for (int speed : lastWheelSpeeds) {
            if (speed < 0) {
                setStatusOnly("正在同步四轮速度，请稍后再试");
                requestStatusSoon();
                return false;
            }
        }
        StringBuilder script = new StringBuilder();
        for (int wheel = 0; wheel < 4; wheel++) {
            int difference = motionProfiles[profile][wheel] - lastWheelSpeeds[wheel];
            int steps = Math.abs(difference) / 25;
            if (steps == 0) continue;
            script.append(wheel + 1);
            char adjustment = difference > 0 ? '+' : '-';
            for (int step = 0; step < steps; step++) script.append(adjustment);
        }
        script.append(command.getPayload());
        String payload = script.toString();
        sendTransportScript(payload);
        System.arraycopy(motionProfiles[profile], 0, lastWheelSpeeds, 0, 4);
        lastParameterTableSignature = "";
        updateParameterTable(null, -1);
        appendLog(tag, profileName(profile) + "配置 + " + command.getPayload()
                + "，脚本=" + payload);
        setStatusOnly("已套用" + profileName(profile) + "轮速并发送" + describeCommand(command));
        return true;
    }

    private void stopHeldCommand(boolean sendStop) {
        handler.removeCallbacks(holdRepeatRunnable);
        heldCommand = null;
        if (occupancyGridView != null) occupancyGridView.setMotionActive(false);
        if (sendStop) {
            sendTransportEmergencyStop();
            requestStatusSoon();
        }
    }

    private void bindSettingsControls() {
        manualModeButton.setOnClickListener(view -> requestDriveMode(1));
        autoModeButton.setOnClickListener(view -> requestDriveMode(0));
        radarPowerButton.setOnClickListener(view -> setLidarEnabled(lastLidarEnabled != 1));
        immediateStopButton.setOnClickListener(view -> {
            if (!isCarConnected()) {
                showStatus("请先连接小车");
                return;
            }
            stopHeldCommand(false);
            if (navigationRunning) stopNavigation("用户立即停车", false);
            sendTransportEmergencyStop();
            setStatusOnly("已发送停车指令 · 当前为手动模式");
            requestStatusSoon();
        });
        bindParameterSelect(R.id.selectSpeedButton, CarCommand.SELECT_SPEED, 0);
        bindParameterSelect(R.id.selectStepButton, CarCommand.SELECT_STEP_LENGTH, 1);
        bindParameterSelect(R.id.selectLfButton, CarCommand.SELECT_LEFT_FRONT_SPEED, 2);
        bindParameterSelect(R.id.selectRfButton, CarCommand.SELECT_RIGHT_FRONT_SPEED, 3);
        bindParameterSelect(R.id.selectLrButton, CarCommand.SELECT_LEFT_REAR_SPEED, 4);
        bindParameterSelect(R.id.selectRrButton, CarCommand.SELECT_RIGHT_REAR_SPEED, 5);
        bindParameterSelect(R.id.selectSafeButton, CarCommand.SELECT_RADAR_DISTANCE, 6);
        findViewById(R.id.toggleSettingButton).setOnClickListener(view -> {
            parameterSessionActive = !parameterSessionActive;
            sendCarCommand(CarCommand.TOGGLE_SETTING);
            requestStatusSoon();
        });
        bindParameterAdjust(R.id.previousParameterButton, CarCommand.PREVIOUS_PARAMETER);
        bindParameterAdjust(R.id.nextParameterButton, CarCommand.NEXT_PARAMETER);
        bindParameterAdjust(R.id.increaseParameterButton, CarCommand.INCREASE_PARAMETER);
        bindParameterAdjust(R.id.decreaseParameterButton, CarCommand.DECREASE_PARAMETER);
        bindParameterTableEditors();
        bindMotionProfileControls();
    }

    private void bindParameterTableEditors() {
        if (parameterRows == null) {
            return;
        }
        for (int i = 0; i < parameterRows.length; i++) {
            final int parameter = i;
            View row = parameterRows[i];
            row.setClickable(true);
            row.setFocusable(true);
            row.setOnClickListener(view -> {
                view.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP);
                openParameterEditor(parameter);
            });
        }
    }

    private void bindMotionProfileControls() {
        for (int i = 0; i < motionProfileButtons.length; i++) {
            final int profile = i;
            motionProfileButtons[i].setOnClickListener(view -> showMotionProfileEditor(profile));
        }
        findViewById(R.id.resetProfilesButton).setOnClickListener(view -> {
            for (int i = 0; i < DEFAULT_MOTION_PROFILES.length; i++) {
                System.arraycopy(DEFAULT_MOTION_PROFILES[i], 0, motionProfiles[i], 0, 4);
            }
            saveMotionProfiles();
            updateMotionProfileButtons();
            Toast.makeText(this, "已恢复推荐默认值", Toast.LENGTH_SHORT).show();
        });
    }

    private void showMotionProfileEditor(int profile) {
        String[] wheelNames = {"左前轮", "右前轮", "左后轮", "右后轮"};
        LinearLayout content = new LinearLayout(this);
        content.setOrientation(LinearLayout.VERTICAL);
        content.setPadding(dp(20), dp(4), dp(20), 0);
        EditText[] inputs = new EditText[4];
        for (int i = 0; i < inputs.length; i++) {
            EditText input = new EditText(this);
            input.setInputType(InputType.TYPE_CLASS_NUMBER);
            input.setHint(wheelNames[i] + "（100-900，步进25）");
            input.setText(String.valueOf(motionProfiles[profile][i]));
            input.setSelectAllOnFocus(true);
            content.addView(input, new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT, dp(52)));
            inputs[i] = input;
        }
        AlertDialog dialog = new AlertDialog.Builder(this)
                .setTitle(profileName(profile) + "轮速")
                .setView(content)
                .setNegativeButton("取消", null)
                .setPositiveButton("保存", null)
                .create();
        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
                .setOnClickListener(view -> {
                    int[] values = new int[4];
                    for (int i = 0; i < inputs.length; i++) {
                        try {
                            values[i] = Integer.parseInt(inputs[i].getText().toString().trim());
                        } catch (NumberFormatException error) {
                            inputs[i].setError("请输入整数");
                            return;
                        }
                        if (values[i] < 100 || values[i] > 900 || values[i] % 25 != 0) {
                            inputs[i].setError("请输入100到900之间、能被25整除的数值");
                            return;
                        }
                    }
                    System.arraycopy(values, 0, motionProfiles[profile], 0, 4);
                    saveMotionProfiles();
                    updateMotionProfileButtons();
                    dialog.dismiss();
                }));
        dialog.show();
    }

    private void loadMotionProfiles() {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_profiles", MODE_PRIVATE);
        for (int profile = 0; profile < motionProfiles.length; profile++) {
            for (int wheel = 0; wheel < 4; wheel++) {
                motionProfiles[profile][wheel] = preferences.getInt(
                        "p" + profile + "w" + wheel, DEFAULT_MOTION_PROFILES[profile][wheel]);
            }
        }
        updateMotionProfileButtons();
    }

    private void saveMotionProfiles() {
        android.content.SharedPreferences.Editor editor =
                getSharedPreferences("motion_profiles", MODE_PRIVATE).edit();
        for (int profile = 0; profile < motionProfiles.length; profile++) {
            for (int wheel = 0; wheel < 4; wheel++) {
                editor.putInt("p" + profile + "w" + wheel, motionProfiles[profile][wheel]);
            }
        }
        editor.apply();
    }

    private void updateMotionProfileButtons() {
        if (motionProfileButtons == null) return;
        for (int i = 0; i < motionProfileButtons.length; i++) {
            int[] values = motionProfiles[i];
            motionProfileButtons[i].setText(profileName(i)
                    + "   LF " + values[0] + "   RF " + values[1]
                    + "   LR " + values[2] + "   RR " + values[3]);
        }
    }

    private String profileName(int profile) {
        return profile == 0 ? "前进" : (profile == 1 ? "后退" : "旋转");
    }

    private void openParameterEditor(int parameter) {
        editingParameter = parameter;
        selectParameterForEditing(parameter);

        parameterEditorDialog = new AlertDialog.Builder(this)
                .setTitle("编辑" + PARAMETER_LABELS[parameter])
                .setMessage(buildParameterEditorMessage(parameter))
                .setNegativeButton("减小", null)
                .setNeutralButton("完成", (dialog, which) -> editingParameter = -1)
                .setPositiveButton("增大", null)
                .create();
        parameterEditorDialog.setOnShowListener(dialog -> {
            parameterEditorDialog.getButton(AlertDialog.BUTTON_NEGATIVE)
                    .setOnClickListener(view -> adjustParameter(parameter, CarCommand.DECREASE_PARAMETER));
            parameterEditorDialog.getButton(AlertDialog.BUTTON_POSITIVE)
                    .setOnClickListener(view -> adjustParameter(parameter, CarCommand.INCREASE_PARAMETER));
        });
        parameterEditorDialog.setOnDismissListener(dialog -> {
            parameterEditorDialog = null;
            editingParameter = -1;
        });
        parameterEditorDialog.show();
    }

    private void selectParameterForEditing(int parameter) {
        updateParameterButtons(parameter);
        showPendingParameterSummary(parameter);
        sendCarCommand(getParameterSelectCommand(parameter), "参数选择");
        parameterSessionActive = true;
        requestStatusSoon();
    }

    private void adjustParameter(int parameter, CarCommand adjustment) {
        View row = parameterRows[parameter];
        row.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP);
        sendCarCommand(getParameterSelectCommand(parameter), "参数确认");
        handler.postDelayed(() -> {
            sendCarCommand(adjustment, "参数编辑");
            requestStatusSoon();
        }, 100L);
    }

    private CarCommand getParameterSelectCommand(int parameter) {
        CarCommand[] commands = {
                CarCommand.SELECT_SPEED,
                CarCommand.SELECT_STEP_LENGTH,
                CarCommand.SELECT_LEFT_FRONT_SPEED,
                CarCommand.SELECT_RIGHT_FRONT_SPEED,
                CarCommand.SELECT_LEFT_REAR_SPEED,
                CarCommand.SELECT_RIGHT_REAR_SPEED,
                CarCommand.SELECT_RADAR_DISTANCE
        };
        return commands[Math.max(0, Math.min(parameter, commands.length - 1))];
    }

    private String buildParameterEditorMessage(int parameter) {
        String value = parameterValueTexts == null ? "?" : parameterValueTexts[parameter].getText().toString();
        String unit = parameter == 6 && !value.endsWith("mm") ? "mm" : "";
        return "当前值：" + value + unit + "\n点击减小或增大后，当前值会自动同步。";
    }

    private void refreshParameterEditor() {
        if (parameterEditorDialog != null && parameterEditorDialog.isShowing()
                && editingParameter >= 0 && editingParameter < PARAMETER_LABELS.length) {
            parameterEditorDialog.setMessage(buildParameterEditorMessage(editingParameter));
        }
    }

    private void setDriveButtonPressed(View view, boolean pressed) {
        view.animate().cancel();
        view.animate()
                .scaleX(pressed ? 0.94f : 1f)
                .scaleY(pressed ? 0.94f : 1f)
                .alpha(pressed ? 0.78f : 1f)
                .setDuration(70L)
                .start();
    }

    private void giveButtonFeedback(View view, String action) {
        view.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP);
        setStatusOnly("按键已响应：" + action);
    }

    private String describeDriveCommand(CarCommand command) {
        switch (command) {
            case UP: return "前进";
            case DOWN: return "后退";
            case LEFT: return "左转";
            case RIGHT: return "右转";
            default: return "停止";
        }
    }

    private void bindCommand(int viewId, CarCommand command) {
        findViewById(viewId).setOnClickListener(view -> {
            sendCarCommand(command);
            requestStatusSoon();
        });
    }

    private void setLidarEnabled(boolean enabled) {
        if (!isCarConnected()) {
            showStatus("请先连接小车");
            return;
        }
        int target = enabled ? 1 : 0;
        if (lastLidarEnabled == target) {
            setStatusOnly(enabled ? "雷达已经是开启状态" : "雷达已经是关闭状态");
            requestStatusSoon();
            return;
        }
        lastLidarEnabled = target;
        updateModeControls();
        sendCarCommand(enabled ? CarCommand.ENABLE_LIDAR : CarCommand.DISABLE_LIDAR);
        requestStatusSoon();
    }

    private boolean isCarConnected() {
        return bluetoothController != null && bluetoothController.isConnected();
    }

    private boolean hasPendingTransportWrites() {
        return bluetoothController != null && bluetoothController.hasPendingWrites();
    }

    private void sendTransportCommand(CarCommand command) {
        bluetoothController.sendCommand(command);
    }

    private void sendTransportPriorityCommand(CarCommand command) {
        bluetoothController.sendPriorityCommand(command);
    }

    private void sendTransportEmergencyStop() {
        bluetoothController.sendEmergencyStop();
        // The transport-level safety sequence is H+C. H deliberately transfers
        // motor ownership to manual mode, so the UI must not keep showing an
        // optimistic automatic state while waiting for the next @S snapshot.
        driveModeTransition.clear();
        manualDriveMode = true;
        lastManualMode = 1;
        updateModeControls();
    }

    private void sendTransportScript(String payload) {
        for (int start = 0; start < payload.length(); start += 18) {
            bluetoothController.sendRaw(payload.substring(start, Math.min(start + 18, payload.length())));
        }
    }

    private String getActiveTransportName() {
        return "BLE";
    }

    private void startVehicleAutoConnect() {
        if (bluetoothController == null) return;
        bluetoothController.enableAutoConnect(VEHICLE_CH9141_ADDRESS);
        appendLog("蓝牙", "自动搜索本车 CH9141K " + VEHICLE_CH9141_ADDRESS);
        updateConnectionUi();
    }

    private void disconnectActiveTransport() {
        bluetoothController.disconnect();
    }

    private void updateModeControls() {
        int displayedMode = driveModeTransition.isPending()
                ? driveModeTransition.getExpectedMode() : lastManualMode;
        updateSegmentButton(manualModeButton, displayedMode == 1);
        updateSegmentButton(autoModeButton, displayedMode == 0);
        boolean pending = driveModeTransition.isPending();
        manualModeButton.setText(pending && displayedMode == 1 ? "切换中…" : "手动");
        autoModeButton.setText(pending && displayedMode == 0 ? "切换中…" : "自动");
        manualModeButton.setEnabled(!pending);
        autoModeButton.setEnabled(!pending);
        updateSegmentButton(radarPowerButton, lastLidarEnabled == 1);
        radarPowerButton.setText(lastLidarEnabled == 1 ? "关闭雷达" : "开启雷达");
        boolean automatic = displayedMode == 0;
        MaterialButton forwardButton = findViewById(R.id.forwardButton);
        MaterialButton backButton = findViewById(R.id.backButton);
        forwardButton.setText(automatic ? "前进一步" : "前进");
        backButton.setText(automatic ? "后退一步" : "后退");
        if (autoControlStatusText != null) {
            autoControlStatusText.setText(pending
                    ? "正在等待车辆确认模式…"
                    : (automatic
                    ? "路线编辑 · 添加行驶步骤，完成后点击执行路线"
                    : "手动驾驶 · 长按方向键，松开即停止"));
        }
        if (driveActionButton != null) {
            driveActionButton.setText(automatic ? "执行路线" : "停止");
            driveActionButton.setBackgroundTintList(ColorStateList.valueOf(
                    ContextCompat.getColor(this,
                            automatic ? R.color.primary : R.color.danger)));
        }
        updateHomeDashboard();
    }

    private void requestDriveMode(int targetMode) {
        if (!isCarConnected()) {
            showStatus("请先连接小车");
            return;
        }
        stopHeldCommand(false);
        driveModeTransition.request(targetMode, System.currentTimeMillis());
        manualDriveMode = targetMode == 1;
        lastManualMode = targetMode;
        updateModeControls();
        CarCommand command = targetMode == 1
                ? CarCommand.ENTER_MANUAL_MODE : CarCommand.ENTER_AUTO_MODE;
        // Mode ownership outranks ordinary queued traffic. Sending this through
        // the normal tail queue allowed an old status query to overtake it.
        sendPriorityCarCommand(command, "模式切换");
        handler.postDelayed(() -> {
            if (driveModeTransition.isPending()) requestStatusSoon();
        }, 260L);
        handler.postDelayed(() -> {
            if (!driveModeTransition.isPending()
                    || driveModeTransition.getExpectedMode() != targetMode) return;
            driveModeTransition.clear();
            lastManualMode = -1;
            updateModeControls();
            setStatusOnly("模式切换确认超时，请检查车辆状态");
            requestStatusSoon();
        }, MODE_CONFIRM_TIMEOUT_MS + 120L);
    }

    private void bindParameterSelect(int viewId, CarCommand command, int selectedParameter) {
        findViewById(viewId).setOnClickListener(view -> {
            updateParameterButtons(selectedParameter);
            showPendingParameterSummary(selectedParameter);
            runInParameterSession(command);
        });
    }

    private void bindParameterAdjust(int viewId, CarCommand command) {
        findViewById(viewId).setOnClickListener(view -> runInParameterSession(command));
    }

    private void runInParameterSession(CarCommand command) {
        if (!parameterSessionActive) {
            sendCarCommand(CarCommand.TOGGLE_SETTING);
            parameterSessionActive = true;
            handler.postDelayed(() -> {
                sendCarCommand(command);
                requestStatusSoon();
            }, 120L);
            return;
        }
        sendCarCommand(command);
        requestStatusSoon();
    }

    private void requestStatusSoon() {
        handler.removeCallbacks(statusRequestRunnable);
        handler.postDelayed(statusRequestRunnable, 180L);
    }

    private void sendCarCommand(CarCommand command) {
        sendCarCommand(command, "发送");
    }

    private void sendCarCommand(CarCommand command, String tag) {
        if (!isCarConnected()) {
            setStatusOnly(getActiveTransportName() + "未连接");
            return;
        }
        sendTransportCommand(command);
        appendLog(tag, command.getPayload() + " " + describeCommand(command));
        setStatusOnly("已发送 " + command.getPayload() + " - " + describeCommand(command));
    }

    private void sendPriorityCarCommand(CarCommand command, String tag) {
        if (!isCarConnected()) {
            setStatusOnly(getActiveTransportName() + "未连接");
            return;
        }
        sendTransportPriorityCommand(command);
        appendLog(tag, command.getPayload() + " " + describeCommand(command));
        setStatusOnly("已优先发送 " + command.getPayload() + " - " + describeCommand(command));
    }

    private void handleFrame(BleFrame frame) {
        if (frame.getType() != BleFrame.Type.LIDAR_MAP
                && frame.getType() != BleFrame.Type.MOTION
                && frame.getType() != BleFrame.Type.ONBOARD_MAP_CHUNK) {
            appendLog("接收", frame.getRawText());
        }
        if (frame.getType() == BleFrame.Type.PONG) {
            lastPongAt = System.currentTimeMillis();
            long dataAge = getDataAgeMs();
            if (dataAge > DATA_STALE_TIMEOUT_MS) {
                setStatusOnly(getBtPrefix() + "，心跳正常，状态/雷达未更新");
            } else if (dataAge > HEARTBEAT_INTERVAL_MS) {
                setStatusOnly(getBtPrefix() + "，心跳正常");
            }
        } else if (frame.getType() == BleFrame.Type.KEY_ACK) {
            setStatusOnly(getBtPrefix() + "，确认：" + describeAckKey(frame.getValue()));
            if (shouldRefreshStatusAfterAck(frame.getValue())) {
                requestStatusSoon();
            }
        } else if (frame.getType() == BleFrame.Type.STATUS) {
            lastPongAt = System.currentTimeMillis();
            handleStatusFrame(frame.getValue());
        } else if (frame.getType() == BleFrame.Type.RADAR) {
            lastPongAt = System.currentTimeMillis();
            handleRadarFrame(frame.getValue());
        } else if (frame.getType() == BleFrame.Type.VL53) {
            lastPongAt = System.currentTimeMillis();
            handleVl53Frame(frame.getValue());
        } else if (frame.getType() == BleFrame.Type.MOTION) {
            lastPongAt = System.currentTimeMillis();
            handleMotionSensorFrame(frame.getValue());
        } else if (frame.getType() == BleFrame.Type.LIDAR_MAP) {
            lastPongAt = System.currentTimeMillis();
            handleLidarMapFrame(frame.getRawText());
        } else if (frame.getType() == BleFrame.Type.ONBOARD_MAP_STATUS) {
            lastPongAt = System.currentTimeMillis();
            handleOnboardMapStatus(frame.getValue());
        } else if (frame.getType() == BleFrame.Type.ONBOARD_MAP_CHUNK) {
            lastPongAt = System.currentTimeMillis();
            handleOnboardMapChunk(frame.getValue());
        } else {
            setStatusOnly(getBtPrefix() + "，收到：" + frame.getRawText());
        }
    }

    private void handleStatusFrame(String payload) {
        StatusFrame statusFrame = StatusFrame.parse(payload);
        if (statusFrame == null) {
            appendLog("解析", "状态帧格式异常 " + payload);
            setStatusOnly(getBtPrefix() + "，@S 状态帧格式异常");
            return;
        }

        latestStatusFrame = statusFrame;
        if (explorationRunning && (explorationCommand != null || explorationAwaitingEvidence)) {
            int expectedCommand = manualCommandValue(explorationCommand);
            boolean commandMatched = expectedCommand < 0
                    || statusFrame.getManualCommand() == expectedCommand;
            boolean nonNeutralPulse = statusFrame.getC5LeftFrontPulse() >= 0
                    && (statusFrame.getC5LeftFrontPulse() != 1500
                    || statusFrame.getC5RightFrontPulse() != 1500
                    || statusFrame.getC5LeftRearPulse() != 1500
                    || statusFrame.getC5RightRearPulse() != 1500);
            if (commandMatched && nonNeutralPulse && statusFrame.getMotorBlocked() != 1) {
                explorationMotorConfirmed = true;
            }
        }

        gridMapView.setHardwareStatus(statusFrame);
        occupancyGridView.setStatusFrame(statusFrame);
        lastStatusAt = System.currentTimeMillis();
        lastDataRecoveryAt = lastStatusAt;
        if (statusFrame.getSettingMode() >= 0) {
            parameterSessionActive = statusFrame.getSettingMode() != 0;
        }
        if (statusFrame.getManualMode() >= 0) {
            DriveModeTransition.StatusDecision decision = driveModeTransition.onStatus(
                    statusFrame.getManualMode(), System.currentTimeMillis());
            if (decision != DriveModeTransition.StatusDecision.STALE_MISMATCH) {
                manualDriveMode = statusFrame.getManualMode() != 0;
                lastManualMode = statusFrame.getManualMode();
                if (decision == DriveModeTransition.StatusDecision.CONFIRMED) {
                    appendLog("模式切换", lastManualMode == 1 ? "手动模式已确认" : "自动模式已确认");
                } else if (decision == DriveModeTransition.StatusDecision.TIMED_OUT) {
                    appendLog("模式切换", "FPGA未确认目标模式，已恢复硬件上报状态");
                    Toast.makeText(this, "模式切换未确认，请重试", Toast.LENGTH_SHORT).show();
                }
            } else {
                // The status serializer snapshots a packet before transmitting
                // it. Ignore only the stale mode bit; all other telemetry in
                // this packet remains useful and is processed below.
                handler.postDelayed(this::requestStatusSoon, 220L);
            }
        }
        if (statusFrame.getLidarEnabled() >= 0) {
            lastLidarEnabled = statusFrame.getLidarEnabled();
        }
        if (statusFrame.getRadarOnline() >= 0) {
            lastRadarOnline = statusFrame.getRadarOnline();
        }
        if (statusFrame.getSpeed() >= 0) {
            lastSpeed = statusFrame.getSpeed();
        }
        if (statusFrame.getStepLength() >= 0) {
            lastStepLength = statusFrame.getStepLength();
        }
        int[] wheelSpeeds = {
                statusFrame.getLeftFrontSpeed(),
                statusFrame.getRightFrontSpeed(),
                statusFrame.getLeftRearSpeed(),
                statusFrame.getRightRearSpeed()
        };
        for (int i = 0; i < wheelSpeeds.length; i++) {
            if (wheelSpeeds[i] >= 0) {
                lastWheelSpeeds[i] = wheelSpeeds[i];
            }
        }
        if (statusFrame.getSafeDistanceMm() >= 0) {
            lastSafeDistanceMm = statusFrame.getSafeDistanceMm();
        }
        if (statusFrame.getSelectedParameter() >= 0) {
            lastSelectedParameter = statusFrame.getSelectedParameter();
        }
        handleLocalCalibrationStatus(statusFrame);
        String stateLine = "状态 " + statusFrame.getState()
                + " / 菜单 " + statusFrame.getMenu()
                + " / 步数 " + statusFrame.getCompletedRouteSteps()
                + "/" + statusFrame.getActiveRouteSteps()
                + " / 当前 (" + statusFrame.getNowX() + "," + statusFrame.getNowY() + ")"
                + " / 目标 (" + statusFrame.getTargetX() + "," + statusFrame.getTargetY() + ")"
                + " / 序号 " + formatOptionalValue(statusFrame.getSequence());
        String paramLine = "速度 " + formatOptionalValue(statusFrame.getSpeed())
                + " / 步长 " + formatOptionalValue(statusFrame.getStepLength())
                + " / 当前参数 " + describeSelectedParameter(statusFrame.getSelectedParameter())
                + " / 设置 " + formatOptionalValue(statusFrame.getSettingMode())
                + " / 手动 " + formatOnOff(statusFrame.getManualMode())
                + " / 雷达 " + formatOnOff(statusFrame.getLidarEnabled())
                + " / 动作 " + describeManualCommand(statusFrame.getManualCommand())
                + " / 看门狗 " + formatOnOff(statusFrame.getWatchdogTriggered())
                + " / 电机阻塞 " + formatOnOff(statusFrame.getMotorBlocked())
                + " / " + formatC5Diagnostics(statusFrame)
                + formatLocalCalibrationDiagnostics(statusFrame);

        String mapMode = statusFrame.getManualMode() < 0
                ? "模式待同步"
                : (statusFrame.getManualMode() == 1 ? "手动模式" : "自动模式");
        String mapAlert = statusFrame.getMotorBlocked() == 1
                ? " · 避障已拦截电机"
                : (statusFrame.getWatchdogTriggered() == 1 ? " · 指令超时已停止" : "");
        setTextIfChanged(mapSummaryText,
                mapMode + " · " + describeManualCommand(statusFrame.getManualCommand())
                        + " · 目标 " + statusFrame.getTargetX() + "," + statusFrame.getTargetY()
                        + mapAlert);
        setTextIfChanged(mapStateValue, describeVehicleState(statusFrame));
        setTextIfChanged(mapStepsValue,
                statusFrame.getCompletedRouteSteps() + "/" + statusFrame.getActiveRouteSteps());
        setTextIfChanged(mapSpeedValue, formatOptionalValue(lastSpeed));
        updateSettingsSummary(formatSettingsSummary(statusFrame));
        updateParameterButtons(statusFrame.getSelectedParameter());
        updateParameterTable(statusFrame, statusFrame.getSelectedParameter());
        updateModeControls();
        appendLog("状态", stateLine + " / 路线=" + statusFrame.getRouteInfo() + " / " + paramLine);
        updateCompactStatus();
    }

    private void handleRadarFrame(String payload) {
        RadarFrame radarFrame = RadarFrame.parse(payload);
        if (radarFrame == null) {
            appendLog("解析", "雷达帧格式异常 " + payload);
            setStatusOnly(getBtPrefix() + "，@L 雷达帧格式异常");
            return;
        }

        radarView.setRadarFrame(radarFrame);
        latestRadarFrame = radarFrame;
        if (explorationRunning && (explorationCommand != null || explorationAwaitingEvidence)
                && explorationStartSectors != null
                && hasMeaningfulRadarChange(explorationStartSectors, radarFrame.getSectorDistancesMm())) {
            explorationRadarMotionConfirmed = true;
        }
        gridMapView.setRadarFrame(radarFrame);
        occupancyGridView.addRadarFrame(radarFrame);
        boolean navigationDrivingForward = navigationCommand == null
                || navigationCommand == CarCommand.UP;
        if (navigationRunning && (radarFrame.getMotorBlocked() == 1
                || (navigationDrivingForward && radarFrame.getObstacleTriggered() == 1))) {
            handleNavigationObstacle(radarFrame);
        }
        updateMappingSummary();
        lastRadarAt = System.currentTimeMillis();
        lastDataRecoveryAt = lastRadarAt;
        if (radarFrame.getRadarOnline() >= 0) {
            lastRadarOnline = radarFrame.getRadarOnline();
        }
        if (radarFrame.hasValue(radarFrame.getSafeDistanceMm())) {
            lastSafeDistanceMm = radarFrame.getSafeDistanceMm();
        }
        updateParameterTable(null, -1);
        updateRadarSummary();
        appendLog("雷达", "在线=" + formatOnOff(radarFrame.getRadarOnline())
                + " / 雷达避障=" + formatOnOff(radarFrame.getLidarObstacleTriggered())
                + " / 超声避障=" + formatOnOff(radarFrame.getUltrasonicObstacleTriggered())
                + " / 合并避障=" + formatOnOff(radarFrame.getObstacleTriggered())
                + " / 电机阻塞=" + formatOnOff(radarFrame.getMotorBlocked())
                + " / 前方=" + formatDistance(radarFrame.getFrontDistance())
                + " / 序号=" + formatOptionalValue(radarFrame.getSequence()));
        updateCompactStatus();
    }

    private void handleOnboardMapStatus(String payload) {
        OnboardMapStatus status = OnboardMapStatus.parse(payload);
        if (status == null) {
            appendLog("FPGA位姿", "@G0 长度或十六进制字段异常");
            return;
        }
        latestOnboardMapStatus = status;
        lastOnboardMapStatusAt = System.currentTimeMillis();
        if (explorationRunning && status.isMapActive() && !status.isMapPaused()
                && lastOnboardMapStatusAt >= explorationStartRequestedAt) {
            setStatusOnly("自动探索已由FPGA接管");
        }
        // G0 is event-driven and may be older than the freshness window at the
        // exact moment the user aligns heading. Capture the first subsequent
        // valid board heading so sparse status timing cannot force the app back
        // to low-rate @M2 integration for the entire session.
        if (imuWorldHeadingManuallyAligned && !manualBoardHeadingReferenceValid
                && isBoardHeadingAvailable(status)) {
            manualBoardHeadingReferenceDeg = status.getHeadingDegrees();
            manualBoardHeadingReferenceValid = true;
            appendLog("绝对朝向", String.format(Locale.CHINA,
                    "已锁定FPGA航向基准 %.1f°", manualBoardHeadingReferenceDeg));
        } else if (!imuWorldHeadingManuallyAligned && automaticBoardHeadingTracking
                && !automaticBoardHeadingReferenceValid
                && isBoardHeadingAvailable(status)) {
            automaticBoardHeadingReferenceDeg = status.getHeadingDegrees();
            automaticBoardHeadingReferenceValid = true;
            appendLog("绝对朝向", String.format(Locale.CHINA,
                    "UWB粗对齐后已锁定FPGA航向基准 %.1f°",
                    automaticBoardHeadingReferenceDeg));
        }
        mapWorldRegistration.addLocalPose(lastOnboardMapStatusAt, status.getPoseXmm(),
                status.getPoseYmm(), status.getHeadingDegrees());
        // The FPGA map remains useful for offline safety, but the Android map
        // uses UWB translation + dual-IMU heading. Applying G0 here used to
        // switch the view back to drifting FPGA odometry on every status frame.
        updateMapWorldRegistration();
        applyUwbImuMapPose();
        updateMappingPauseButton();
        updateMappingSummary();
        updateLocalizationSummary();
    }

    private void handleOnboardMapChunk(String payload) {
        OnboardMapChunk chunk = OnboardMapChunk.parse(payload);
        if (chunk == null) {
            appendLog("板载地图", "@G1 长度、地址或 CRC 异常");
            return;
        }
        // Do not mix the FPGA-local 50 mm grid with the phone's UWB-world
        // 25 mm grid. The chunk is validated above so protocol health remains
        // observable, but only 72-point scans populate the displayed map.
        updateMappingSummary();
    }

    private void handleVl53Frame(String payload) {
        Vl53Frame vl53Frame = Vl53Frame.parse(payload);
        if (vl53Frame == null) {
            appendLog("解析", "VL53帧格式异常 " + payload);
            setStatusOnly(getBtPrefix() + "，@V 测距帧格式异常");
            return;
        }

        latestVl53Frame = vl53Frame;
        lastVl53At = System.currentTimeMillis();
        updateRadarSummary();
    }

    private void handleMotionSensorFrame(String payload) {
        MotionSensorFrame frame = MotionSensorFrame.parse(payload);
        if (frame == null) {
            appendLog("解析", "运动传感器帧格式异常 " + payload);
            return;
        }

        boolean calibrationJustCompleted = latestMotionSensorFrame != null
                && !latestMotionSensorFrame.isCalibrationDone()
                && frame.isCalibrationDone();
        if (calibrationJustCompleted) {
            imu0Attitude.reset();
            imu1Attitude.reset();
            resetImuWorldHeadingReference();
        }

        long now = System.currentTimeMillis();
        latestMotionSensorFrame = frame;
        lastMotionSensorAt = now;
        updateGyroSourceHealth(frame);
        if ((getApplicationInfo().flags
                & android.content.pm.ApplicationInfo.FLAG_DEBUGGABLE) != 0) {
            MotionSensorFrame.ImuSample imu0 = frame.getImu0();
            MotionSensorFrame.ImuSample imu1 = frame.getImu1();
            Log.d("VehicleMotionTrace", String.format(Locale.US,
                    "seq=%d status=%02X imu0_dps=(%.2f,%.2f,%.2f) "
                            + "imu1_dps=(%.2f,%.2f,%.2f)",
                    frame.getSequence(), frame.getStatus(),
                    imu0.getGyroXDps(), imu0.getGyroYDps(), imu0.getGyroZDps(),
                    imu1.getGyroXDps(), imu1.getGyroYDps(), imu1.getGyroZDps()));
        }
        if (frame.isBmi0Online()) {
            imu0Attitude.update(frame.getImu0(), frame.getSequence(), now);
        }
        if (frame.isBmi1Online()) {
            imu1Attitude.update(frame.getImu1(), frame.getSequence(), now);
        }
        UwbVehiclePose uwbPose = getFreshServerPose();
        if (uwbPose != null) {
            applyUwbImuMapPose();
        }
        updateMotionSensorSummary();
        updateMappingSummary();
        updateLocalizationSummary();
    }

    private void resetMotionSensorSession() {
        latestMotionSensorFrame = null;
        lastMotionSensorAt = 0L;
        imu0GyroSuppressed = false;
        imu1GyroSuppressed = false;
        imu0Attitude.reset();
        imu1Attitude.reset();
        resetImuWorldHeadingReference();
        updateMotionSensorSummary();
    }

    private void updateGyroSourceHealth(MotionSensorFrame frame) {
        if (!frame.isBmi0Online() || !frame.isBmi1Online()) {
            imu0GyroSuppressed = false;
            imu1GyroSuppressed = false;
            return;
        }

        int imu0Peak = gyroPeak(frame.getImu0());
        int imu1Peak = gyroPeak(frame.getImu1());
        boolean imu0Missing = imu0Peak <= GYRO_MISSING_SELF_LIMIT
                && imu1Peak >= GYRO_MISSING_PEER_LIMIT;
        boolean imu1Missing = imu1Peak <= GYRO_MISSING_SELF_LIMIT
                && imu0Peak >= GYRO_MISSING_PEER_LIMIT;

        if (imu0Missing) {
            imu0GyroSuppressed = true;
        } else if (imu0Peak >= GYRO_MISSING_PEER_LIMIT) {
            imu0GyroSuppressed = false;
        }
        if (imu1Missing) {
            imu1GyroSuppressed = true;
        } else if (imu1Peak >= GYRO_MISSING_PEER_LIMIT) {
            imu1GyroSuppressed = false;
        }
    }

    private static int gyroPeak(MotionSensorFrame.ImuSample sample) {
        return Math.max(Math.abs(sample.getGyroX()),
                Math.max(Math.abs(sample.getGyroY()), Math.abs(sample.getGyroZ())));
    }

    private void resetImuWorldHeadingReference() {
        // A transient BLE reconnect resets the phone-side sample integrators,
        // but must not erase the absolute direction explicitly selected by the
        // user. Freeze the latest FPGA-derived world heading, then rebase it on
        // the first valid @G0 after reconnect (or sensor recalibration).
        if (imuWorldHeadingManuallyAligned) {
            Float heldHeading = getBoardWorldHeadingDeg();
            if (heldHeading != null) manualAbsoluteHeadingDeg = heldHeading;
        } else if (automaticBoardHeadingTracking) {
            Float heldHeading = getBoardWorldHeadingDeg();
            if (heldHeading != null) automaticAbsoluteHeadingDeg = heldHeading;
        }
        imuWorldHeadingReferenceValid = false;
        manualBoardHeadingReferenceValid = false;
        automaticBoardHeadingReferenceValid = false;
        lastUwbHeadingFusionSourceMs = Long.MIN_VALUE;
        pendingUwbImuOffsetDeg = 0.0;
        pendingUwbHeadingSamples = 0;
    }

    private void handleLidarMapFrame(String line) {
        int dropsBefore = lidarMapAssembler.getIncompleteDropCount();
        final LidarMapChunk chunk;
        try {
            chunk = LidarMapChunk.parse(line);
        } catch (LidarMapChunk.ParseException exception) {
            if (exception.getReason() == LidarMapChunk.ErrorReason.CRC) {
                lidarMapCrcErrorCount++;
            } else {
                lidarMapFormatErrorCount++;
            }
            appendLog("雷达72错误", exception.getMessage() + " / 原始=" + line);
            updateLidarMapDebug();
            return;
        }

        long now = System.currentTimeMillis();
        LidarMapScan scan = lidarMapAssembler.add(chunk, now);
        if (lidarMapAssembler.getIncompleteDropCount() > dropsBefore) {
            appendLog("雷达72", "检测到新扫描，丢弃上一组不完整分包");
        }
        if (scan == null) {
            updateLidarMapDebug();
            return;
        }

        acceptLidarMapScan(scan, now, lidarMapAssembler.getCompletedScanCount(), "BLE直连");
    }

    private void acceptLidarMapScan(LidarMapScan scan, long now, int completed, String source) {
        if (previousLidarMapAt > 0L && now > previousLidarMapAt) {
            float instantRate = 1000f / (now - previousLidarMapAt);
            lidarMapRateHz = lidarMapRateHz <= 0f
                    ? instantRate
                    : lidarMapRateHz * 0.7f + instantRate * 0.3f;
        }
        previousLidarMapAt = now;
        lastLidarMapAt = now;
        latestLidarMapScan = scan;
        radarView.setLidarMapScan(scan);
        occupancyGridView.addLidarMapScan(scan, hasFreshWorldMappingPose(now));
        updateLidarMapDebug();
        updateRadarSummary();
        updateMappingSummary();

        if (completed <= 3 || completed % 10 == 0) {
            appendLog("雷达72", String.format(Locale.CHINA,
                    "%s 完整圈=%d seq=%02X tick=%04X 有效=%d/72 高质量=%d 频率=%.2fHz",
                    source,
                    completed,
                    scan.getSequence(),
                    scan.getTick10ms(),
                    scan.getValidPointCount(),
                    scan.getHighQualityPointCount(),
                    lidarMapRateHz));
        }
    }

    private void resetLidarMapSession() {
        lidarMapAssembler.resetSession();
        latestLidarMapScan = null;
        lastLidarMapAt = 0L;
        previousLidarMapAt = 0L;
        lidarMapRateHz = 0f;
        lidarMapCrcErrorCount = 0;
        lidarMapFormatErrorCount = 0;
        if (radarView != null) {
            radarView.setLidarMapScan(null);
        }
        updateLidarMapDebug();
        updateRadarSummary();
    }

    private void updateLidarMapDebug() {
        if (lidarMapDebugText == null) return;
        long now = System.currentTimeMillis();
        long age = lastLidarMapAt <= 0L ? Long.MAX_VALUE : now - lastLidarMapAt;
        String freshness = lastLidarMapAt <= 0L
                ? "等待首圈"
                : (age <= 1500L ? "数据新鲜" : "数据超时 " + age + "ms");
        String scanLine = latestLidarMapScan == null
                ? "序号 --  ·  有效 --/72  ·  高质量 --"
                : String.format(Locale.CHINA,
                "序号 %02X  ·  时基 %04X  ·  有效 %d/72  ·  高质量 %d",
                latestLidarMapScan.getSequence(),
                latestLidarMapScan.getTick10ms(),
                latestLidarMapScan.getValidPointCount(),
                latestLidarMapScan.getHighQualityPointCount());
        String counters = String.format(Locale.CHINA,
                "完整 %d  ·  待收 %d/6  ·  丢圈 %d  ·  CRC %d  ·  格式 %d",
                lidarMapAssembler.getCompletedScanCount(),
                lidarMapAssembler.getPendingPartCount(),
                lidarMapAssembler.getIncompleteDropCount(),
                lidarMapCrcErrorCount,
                lidarMapFormatErrorCount);
        String tail = "重复分包 " + lidarMapAssembler.getDuplicatePartCount()
                + "  ·  拒绝分包 " + lidarMapAssembler.getRejectedPartCount();
        setTextIfChanged(lidarMapDebugText,
                "72点链路 · " + freshness + " · "
                        + String.format(Locale.CHINA, "%.2fHz", lidarMapRateHz)
                        + "\n" + scanLine
                        + "\n" + counters
                        + "\n" + tail);
    }

    private void updateRadarSummary() {
        if (radarSummaryText == null) return;
        long now = System.currentTimeMillis();
        String safetyLine;
        String distanceLine;
        String sectorLine;
        if (latestRadarFrame == null) {
            safetyLine = "8方向安全链路等待数据";
            distanceLine = "前方 --  ·  超声 --  ·  安全距离 --";
            sectorLine = "扇区距离等待同步";
        } else {
            boolean fresh = lastRadarAt > 0L && now - lastRadarAt <= DATA_STALE_TIMEOUT_MS;
            String radarState = latestRadarFrame.getRadarOnline() == 1 ? "雷达在线" : "雷达离线";
            String obstacleState = latestRadarFrame.getObstacleTriggered() == 1
                    ? "检测到障碍"
                    : "环境正常";
            safetyLine = radarState + " · " + obstacleState + (fresh ? "" : " · @L超时");
            distanceLine = "前方 " + formatDistance(latestRadarFrame.getFrontDistance())
                    + " · 超声 " + formatUltrasonicDistance(latestRadarFrame)
                    + " · 安全距离 " + formatDistance(latestRadarFrame.getSafeDistanceMm());
            sectorLine = formatLidarSectorSummary(latestRadarFrame);
        }

        String detailedLine;
        if (latestLidarMapScan == null) {
            detailedLine = "72点扫描等待同步";
        } else {
            long age = Math.max(0L, now - lastLidarMapAt);
            detailedLine = String.format(Locale.CHINA,
                    "72点 seq=%02X · 有效 %d/72 · %.2fHz · %dms",
                    latestLidarMapScan.getSequence(),
                    latestLidarMapScan.getValidPointCount(),
                    lidarMapRateHz,
                    age);
        }
        setTextIfChanged(radarSummaryText,
                safetyLine + "\n" + distanceLine + "\n" + detailedLine + "\n" + sectorLine);
    }

    private void updateMotionSensorSummary() {
        if (motionSensorSummaryText == null) return;
        MotionSensorFrame frame = latestMotionSensorFrame;
        if (frame == null) {
            setTextIfChanged(motionSensorSummaryText,
                    "运动传感器 @M2 等待同步\n"
                            + "IMU0  等待数据\n"
                            + "IMU1  等待数据\n"
                            + "光流  等待数据");
            return;
        }

        long ageMs = Math.max(0L, System.currentTimeMillis() - lastMotionSensorAt);
        String calibration = frame.isCalibrationDone() ? "校准完成" : "静止校准中";
        String fusion;
        if (frame.isImuDisagreement()) {
            fusion = "双IMU分歧";
        } else if (frame.isImuDegraded()) {
            fusion = "单IMU降级";
        } else if (frame.isFusedDataReady()) {
            fusion = "双IMU一致";
        } else {
            fusion = "融合等待";
        }
        String freshness = ageMs <= DATA_STALE_TIMEOUT_MS ? ageMs + "ms" : "数据超时";
        String header = String.format(Locale.CHINA,
                "运动传感器 @M2 #%04X · %s · %s · %s",
                frame.getSequence(), calibration, fusion, freshness);
        String flowState = frame.isPmw3901Online()
                ? (frame.hasFlowData() ? "在线" : "在线/无运动帧")
                : "离线";
        String flowLine = String.format(Locale.CHINA,
                "光流 %s  Q=%d  d=(%+d,%+d)  sum=(%+d,%+d)",
                flowState,
                frame.getFlowSqual(),
                frame.getFlowDeltaX(),
                frame.getFlowDeltaY(),
                frame.getFlowAccumX(),
                frame.getFlowAccumY());
        setTextIfChanged(motionSensorSummaryText,
                header + "\n"
                        + formatImuSummary("IMU0", frame.isBmi0Online(),
                        frame.getImu0(), imu0Attitude) + "\n"
                        + formatImuSummary("IMU1", frame.isBmi1Online(),
                        frame.getImu1(), imu1Attitude) + "\n"
                        + flowLine);
    }

    private static String formatImuSummary(
            String label,
            boolean online,
            MotionSensorFrame.ImuSample sample,
            ImuAttitudeEstimator attitude
    ) {
        if (!online) return label + " 离线";
        return String.format(Locale.CHINA,
                "%s 在线  相对R=%+.1f° P=%+.1f° Y=%+.1f°\n"
                        + "  A=(%+.2f,%+.2f,%+.2f)g  G=(%+.1f,%+.1f,%+.1f)°/s",
                label,
                attitude.getRollDeg(),
                attitude.getPitchDeg(),
                attitude.getYawDeg(),
                sample.getAccelXG(),
                sample.getAccelYG(),
                sample.getAccelZG(),
                sample.getGyroXDps(),
                sample.getGyroYDps(),
                sample.getGyroZDps());
    }

    static String formatVl53SensorLines(Vl53Frame frame) {
        StringBuilder builder = new StringBuilder();
        for (int sensor = 0; sensor < Vl53Frame.SENSOR_COUNT; sensor++) {
            if (sensor > 0) builder.append('\n');

            String distance = "----";
            String state = "等待";
            if (frame != null) {
                if (!frame.isOnline(sensor)) {
                    state = "离线";
                } else if (!frame.isDistanceValid(sensor)) {
                    int rawDistance = frame.getRawDistanceMm(sensor);
                    if (rawDistance == 0) {
                        distance = "0";
                        state = "原始值0";
                    } else if (rawDistance != 0xFFFF && rawDistance > 4000) {
                        distance = Integer.toString(rawDistance);
                        state = "超4m范围";
                    } else {
                        state = "测距无效";
                    }
                } else {
                    distance = Integer.toString(frame.getDistanceMm(sensor));
                    state = "有效";
                }
            }

            builder.append(String.format(Locale.CHINA,
                    "%d号  0x%02X  %4s mm  %s",
                    sensor,
                    VehicleGeometry.VL53L1X_I2C_ADDRESSES_7BIT[sensor],
                    distance,
                    state));
        }
        return builder.toString();
    }

    private void showBluetoothDeviceDialog() {
        if (!bluetoothController.isBluetoothAvailable()) {
            showStatus("当前设备不支持蓝牙");
            appendLog("蓝牙", "当前设备不支持蓝牙");
            return;
        }
        if (!bluetoothController.isBluetoothEnabled()) {
            showStatus("请先打开手机蓝牙");
            appendLog("蓝牙", "手机蓝牙未开启");
            return;
        }

        scanDevices.clear();
        scanDeviceLabels.clear();
        BluetoothDevice vehicleDevice =
                bluetoothController.getRemoteDevice(VEHICLE_CH9141_ADDRESS);
        if (vehicleDevice != null) {
            scanDevices.add(vehicleDevice);
            scanDeviceLabels.add("本车 CH9141K " + VEHICLE_CH9141_ADDRESS
                    + "\n已知地址，可直接连接");
        } else {
            scanDeviceLabels.add(getString(R.string.scan_empty));
        }
        scanAdapter = new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, scanDeviceLabels);

        AlertDialog dialog = new AlertDialog.Builder(this)
                .setTitle(getString(R.string.select_bluetooth_device))
                .setAdapter(scanAdapter, (dialogInterface, which) -> {
                    if (which >= 0 && which < scanDevices.size()) {
                        BluetoothDevice selectedDevice = scanDevices.get(which);
                        appendLog("蓝牙", "正在连接 " + formatDeviceName(selectedDevice));
                        bluetoothController.connect(selectedDevice);
                        showStatus(getString(R.string.bluetooth_connecting));
                    }
                })
                .setNegativeButton(getString(R.string.cancel), (dialogInterface, which) -> bluetoothController.stopScan())
                .create();
        dialog.setOnDismissListener(dialogInterface -> bluetoothController.stopScan());
        dialog.show();

        showStatus(getString(R.string.bluetooth_scanning));
        appendLog("蓝牙", "开始扫描 BLE 设备");
        bluetoothController.startScan();
    }

    @SuppressLint("MissingPermission")
    private void addScanDevice(BluetoothDevice device, int rssi) {
        for (int index = 0; index < scanDevices.size(); index++) {
            BluetoothDevice existingDevice = scanDevices.get(index);
            if (existingDevice.getAddress().equals(device.getAddress())) {
                scanDeviceLabels.set(index,
                        formatDeviceName(device) + "\n信号 " + rssi + " dBm");
                if (scanAdapter != null) {
                    scanAdapter.notifyDataSetChanged();
                }
                return;
            }
        }
        scanDevices.add(device);
        if (scanDevices.size() == 1) {
            scanDeviceLabels.clear();
        }
        scanDeviceLabels.add(formatDeviceName(device) + "\n信号 " + rssi + " dBm");
        if (scanAdapter != null) {
            scanAdapter.notifyDataSetChanged();
        }
    }

    private void startHeartbeat() {
        stopHeartbeat();
        heartbeatRunning = true;
        handler.post(heartbeatRunnable);
    }

    private void stopHeartbeat() {
        heartbeatRunning = false;
        handler.removeCallbacks(heartbeatStartRunnable);
        handler.removeCallbacks(heartbeatRunnable);
    }

    private void scheduleHeartbeatStart() {
        stopHeartbeat();
        handler.postDelayed(heartbeatStartRunnable, HEARTBEAT_START_DELAY_MS);
    }

    private String getBtPrefix() {
        if (lastPongAt > 0 && System.currentTimeMillis() - lastPongAt <= BT_OFF_TIMEOUT_MS) {
            return "BLE车辆在线";
        }
        return "BLE车辆离线";
    }

    private long getLastHeartbeatReferenceAt() {
        return lastPongAt > 0 ? lastPongAt : btWatchStartedAt;
    }

    private long getDataAgeMs() {
        long reference = Math.max(lastStatusAt, lastRadarAt);
        if (reference <= 0L) {
            return Long.MAX_VALUE;
        }
        return System.currentTimeMillis() - reference;
    }

    private void recoverStaleDataIfNeeded() {
        if (!isCarConnected()) {
            return;
        }
        long now = System.currentTimeMillis();
        long dataAge = getDataAgeMs();
        boolean dataStale = dataAge > DATA_STALE_TIMEOUT_MS;
        boolean heartbeatAlive = lastPongAt > 0 && now - lastPongAt <= BT_OFF_TIMEOUT_MS;
        if (!dataStale || !heartbeatAlive || now - lastDataRecoveryAt < DATA_RECOVERY_INTERVAL_MS) {
            return;
        }
        lastDataRecoveryAt = now;
        sendPriorityCarCommand(CarCommand.REQUEST_STATUS, "同步恢复");
    }

    private void updateConnectionUi() {
        boolean connected = isCarConnected();
        boolean bluetoothAvailable = bluetoothController != null && bluetoothController.isBluetoothAvailable();
        boolean bluetoothEnabled = bluetoothAvailable && bluetoothController.isBluetoothEnabled();
        boolean dataOnline = connected && lastPongAt > 0
                && System.currentTimeMillis() - lastPongAt <= BT_OFF_TIMEOUT_MS;
        if (statusLinkIcon != null) {
            statusLinkIcon.setImageResource(R.drawable.ic_nav_bluetooth);
        }
        if (connectButton != null) {
            setTextIfChanged(connectButton,
                    connected ? "重新扫描/连接" : getString(R.string.connect_bluetooth));
            connectButton.setIconResource(R.drawable.ic_nav_bluetooth);
        }
        setConnectionActionEnabled(disconnectButton, connected);
        setConnectionActionEnabled(requestStatusButton, connected);
        if (connectionSummaryText != null) {
            boolean hadData = lastPongAt > 0;
            String dataState = !connected ? "等待连接"
                    : (dataOnline ? "正常" : (hadData ? "响应超时" : "等待首帧"));
            String phoneState = !bluetoothAvailable ? "当前手机不支持 BLE"
                    : (bluetoothEnabled ? "已开启" : "未开启");
            String linkState = connected ? "已连接" : "未连接";
            String advice = !bluetoothAvailable ? "无法建立蓝牙连接"
                    : (!bluetoothEnabled ? "请先开启手机蓝牙"
                    : (!connected ? (bluetoothController.isAutoConnectEnabled()
                    ? "正在附近自动搜索车辆，也可手动选择设备"
                    : "自动连接已暂停，点击连接可重新选择设备")
                    : (dataOnline ? "FFF1 通知与 FFF2 写入均可用"
                    : (hadData ? "正在自动查询并恢复车辆状态" : "正在初始化串口服务"))));
            String summary = "通信方式：BLE 直连\n"
                    + "手机蓝牙：" + phoneState + "\n"
                    + "CH9141K 串口：" + linkState + "\n"
                    + "车辆数据：" + dataState + "\n"
                    + advice;
            if (!summary.equals(lastConnectionSummary)) {
                lastConnectionSummary = summary;
                setTextIfChanged(connectionSummaryText, summary);
            }
        }
        int indicatorColor = !connected ? R.color.danger
                : (dataOnline ? R.color.accent_green : R.color.accent_yellow);
        tintStatusIndicator(indicatorColor);
        updateHomeDashboard();
    }

    private void updateHomeDashboard() {
        if (homeVehicleStateText == null) return;
        boolean connected = isCarConnected();
        boolean dataOnline = connected && lastPongAt > 0
                && System.currentTimeMillis() - lastPongAt <= BT_OFF_TIMEOUT_MS;
        String mode = lastManualMode < 0 ? "模式同步中"
                : (lastManualMode == 1 ? "手动模式" : "自动模式");
        setTextIfChanged(homeVehicleStateText, !connected ? "未连接 · 等待车辆"
                : (dataOnline ? "在线 · " + mode : "已连接 · 数据同步中"));
        String radar = lastLidarEnabled < 0 ? "雷达待同步"
                : (lastLidarEnabled == 1 ? "雷达开启" : "雷达关闭");
        String uwb = uwbServerConnected ? "UWB 在线" : "UWB 离线";
        setTextIfChanged(homeVehicleDetailText, connected
                ? radar + " · " + uwb : "连接后显示模式与传感器状态");

        String[] estimates = buildParameterEstimates(latestStatusFrame);
        String speed = estimates[0];
        if (speed == null || speed.startsWith("待") || "—".equals(speed)) speed = "--";
        setTextIfChanged(homeSpeedValue, speed);
        setTextIfChanged(homeSafeValue,
                lastSafeDistanceMm >= 0 ? lastSafeDistanceMm + " mm" : "--");
        setTextIfChanged(homeRouteValue, latestStatusFrame == null ? "--"
                : latestStatusFrame.getCompletedRouteSteps() + "/"
                + latestStatusFrame.getActiveRouteSteps());
        homePrimaryButton.setText(connected ? "进入车辆控制" : "连接车辆");
    }

    private void setConnectionActionEnabled(MaterialButton button, boolean enabled) {
        if (button == null) return;
        button.setEnabled(enabled);
        button.setAlpha(enabled ? 1f : 0.42f);
    }

    private void tintStatusIndicator(int colorResource) {
        if (statusIndicator == null) return;
        ViewCompat.setBackgroundTintList(statusIndicator, ColorStateList.valueOf(
                ContextCompat.getColor(this, colorResource)));
    }

    private void updateCompactStatus() {
        boolean connected = isCarConnected();
        if (!connected) {
            setStatusOnly(getActiveTransportName() + "未连接，点击前往连接");
            tintStatusIndicator(R.color.danger);
            return;
        }

        boolean dataOnline = lastPongAt > 0
                && System.currentTimeMillis() - lastPongAt <= BT_OFF_TIMEOUT_MS;
        if (!dataOnline) {
            setStatusOnly(lastPongAt > 0
                    ? getActiveTransportName() + "已连接 · 车辆数据响应超时"
                    : getActiveTransportName() + "已连接 · 等待车辆数据");
            tintStatusIndicator(R.color.accent_yellow);
            return;
        }

        long now = System.currentTimeMillis();
        boolean blocked = latestStatusFrame != null
                && now - lastStatusAt <= DATA_STALE_TIMEOUT_MS
                && latestStatusFrame.getMotorBlocked() == 1;
        boolean obstacle = latestRadarFrame != null
                && now - lastRadarAt <= DATA_STALE_TIMEOUT_MS
                && latestRadarFrame.getObstacleTriggered() == 1;
        String mode = lastManualMode < 0 ? "模式待同步"
                : (lastManualMode == 1 ? "手动" : "自动");
        String action = latestStatusFrame == null
                ? "状态同步中"
                : describeManualCommand(latestStatusFrame.getManualCommand());
        String speed = lastSpeed < 0 ? "--" : String.valueOf(lastSpeed);
        String alert = blocked ? " · 电机已拦截"
                : (obstacle ? " · 前方有障碍" : "");
        setStatusOnly(getActiveTransportName() + "已连接 · " + mode + " · " + action
                + " · 速度 " + speed + alert);
        // The header indicator represents only link/data health. Obstacle and
        // motor-block states remain visible in the adjacent status text and
        // radar panel; reusing this dot for both made a healthy BLE link flash.
        tintStatusIndicator(R.color.accent_green);
    }

    private void showStatus(String message) {
        setStatusOnly(message);
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }

    private void setStatusOnly(String message) {
        appState.setStatusText(message);
        appState.setBluetoothConnected(isCarConnected());
        if (!message.equals(lastStatusText)) {
            lastStatusText = message;
            setTextIfChanged(statusText, message);
            updateConnectionUi();
        }
    }

    @SuppressLint("MissingPermission")
    private String formatDeviceName(BluetoothDevice device) {
        String name = device.getName();
        if (name == null || name.trim().isEmpty()) {
            name = "未知蓝牙设备";
        }
        return name + " " + device.getAddress();
    }

    private void appendLog(String tag, String message) {
        if ((getApplicationInfo().flags
                & android.content.pm.ApplicationInfo.FLAG_DEBUGGABLE) != 0) {
            Log.d("VehicleBleTrace", tag + "|" + message);
        }
        if (logText == null) {
            return;
        }
        debugLog.append(logTimeFormat.format(new Date()))
                .append(" [")
                .append(tag)
                .append("] ")
                .append(message)
                .append('\n');

        int maxLength = 12000;
        if (debugLog.length() > maxLength) {
            debugLog.delete(0, debugLog.length() - maxLength);
        }

        logText.setText(debugLog.toString());
        if (logScrollView != null) {
            logScrollView.post(() -> logScrollView.fullScroll(ScrollView.FOCUS_DOWN));
        }
    }

    private void copyDebugLog() {
        if (debugLog.length() == 0) {
            Toast.makeText(this, getString(R.string.log_empty), Toast.LENGTH_SHORT).show();
            return;
        }
        ClipboardManager clipboardManager =
                (ClipboardManager) getSystemService(Context.CLIPBOARD_SERVICE);
        if (clipboardManager == null) {
            Toast.makeText(this, "无法访问剪贴板", Toast.LENGTH_SHORT).show();
            return;
        }
        clipboardManager.setPrimaryClip(ClipData.newPlainText("car_app_debug_log", debugLog.toString()));
        Toast.makeText(this, getString(R.string.log_copied), Toast.LENGTH_SHORT).show();
    }

    private void clearDebugLog() {
        debugLog.setLength(0);
        logText.setText("");
    }

    private String formatLidarSectorSummary(RadarFrame radarFrame) {
        int[] sectors = radarFrame.getSectorDistancesMm();
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < sectors.length && i < LIDAR_SECTOR_LABELS.length; i++) {
            if (builder.length() > 0) {
                builder.append("  ");
            }
            builder.append(LIDAR_SECTOR_LABELS[i])
                    .append(":")
                    .append(formatDistance(sectors[i]));
        }
        return builder.toString();
    }

    private String formatSettingsSummary(StatusFrame statusFrame) {
        String driveMode = statusFrame.getManualMode() == 1 ? "手动控制" : "自动运行";
        String radarState = statusFrame.getLidarEnabled() == 1 ? "雷达在线" : "雷达关闭";
        return "车辆状态已同步"
                + "\n" + driveMode + " · " + radarState
                + "\n行驶参数与传感器配置可在下方展开查看";
    }

    private void showPendingParameterSummary(int selectedParameter) {
        if (settingsSummaryText == null) {
            return;
        }
        updateSettingsSummary("当前参数：" + describeSelectedParameter(selectedParameter)
                + "\n速度：同步中    步长：同步中"
                + "\n设置：开启    手动：" + (manualDriveMode ? "开启" : "关闭")
                + "    雷达：" + formatOnOff(lastLidarEnabled));
        lastSelectedParameter = selectedParameter;
        updateParameterTable(null, selectedParameter);
    }

    private void updateParameterButtons(int selectedParameter) {
        if (parameterButtons == null) {
            return;
        }
        if (selectedParameter == lastParameterButtonsSelection) {
            return;
        }
        lastParameterButtonsSelection = selectedParameter;
        for (int i = 0; i < parameterButtons.length && i < PARAMETER_LABELS.length; i++) {
            MaterialButton button = parameterButtons[i];
            if (button == null) {
                continue;
            }
            setTextIfChanged(button, i == selectedParameter ? "当前 " + PARAMETER_LABELS[i] : PARAMETER_LABELS[i]);
            button.setSelected(i == selectedParameter);
        }
    }

    private void updateSettingsSummary(String summary) {
        if (settingsSummaryText == null || summary.equals(lastSettingsSummary)) {
            return;
        }
        lastSettingsSummary = summary;
        setTextIfChanged(settingsSummaryText, summary);
    }

    private void updateParameterTable(StatusFrame statusFrame, int selectedParameter) {
        if (parameterRows == null || parameterValueTexts == null
                || parameterEstimateTexts == null || parameterStateTexts == null) {
            return;
        }

        int selected = selectedParameter;
        if (selected < 0 && statusFrame != null) {
            selected = statusFrame.getSelectedParameter();
        }
        if (selected < 0) {
            selected = lastSelectedParameter;
        }

        String[] values = new String[] {
                formatOptionalValue(statusFrame == null ? lastSpeed : statusFrame.getSpeed()),
                formatOptionalValue(statusFrame == null ? lastStepLength : statusFrame.getStepLength()),
                formatOptionalValue(lastWheelSpeeds[0]),
                formatOptionalValue(lastWheelSpeeds[1]),
                formatOptionalValue(lastWheelSpeeds[2]),
                formatOptionalValue(lastWheelSpeeds[3]),
                formatDistance(lastSafeDistanceMm)
        };
        String[] states = new String[] {
                lastSpeed >= 0 || statusFrame != null ? "点击编辑 ›" : "等待同步",
                lastStepLength >= 0 || statusFrame != null ? "点击编辑 ›" : "等待同步",
                lastWheelSpeeds[0] >= 0 ? "点击编辑 ›" : "等待同步",
                lastWheelSpeeds[1] >= 0 ? "点击编辑 ›" : "等待同步",
                lastWheelSpeeds[2] >= 0 ? "点击编辑 ›" : "等待同步",
                lastWheelSpeeds[3] >= 0 ? "点击编辑 ›" : "等待同步",
                lastSafeDistanceMm >= 0 ? "点击编辑 ›" : "等待雷达帧"
        };
        String[] estimates = buildParameterEstimates(statusFrame);

        String signature = buildParameterTableSignature(selected, values, estimates, states);
        if (signature.equals(lastParameterTableSignature)) {
            return;
        }
        lastParameterTableSignature = signature;

        for (int i = 0; i < parameterRows.length && i < values.length; i++) {
            boolean active = i == selected;
            parameterRows[i].setBackgroundColor(ContextCompat.getColor(
                    this,
                    active ? R.color.primary_soft : R.color.panel_background
            ));
            setTextIfChanged(parameterValueTexts[i], values[i]);
            setTextIfChanged(parameterEstimateTexts[i], estimates[i]);
            setTextIfChanged(parameterStateTexts[i], active ? "编辑中" : states[i]);
            parameterStateTexts[i].setTextColor(ContextCompat.getColor(
                    this,
                    active ? R.color.primary_dark : R.color.text_secondary
            ));
        }
        refreshParameterEditor();
    }

    private String[] buildParameterEstimates(StatusFrame statusFrame) {
        android.content.SharedPreferences preferences =
                getSharedPreferences("motion_calibration", MODE_PRIVATE);
        boolean calibrated = preferences.contains("linear_mm_per_ms")
                && preferences.contains("linear_speed_setting");
        int speed = statusFrame == null ? lastSpeed : statusFrame.getSpeed();
        int stepLength = statusFrame == null ? lastStepLength : statusFrame.getStepLength();
        double calibratedRate = calibrated
                ? preferences.getFloat("linear_mm_per_ms", Float.NaN) : Double.NaN;
        int calibrationSpeed = calibrated
                ? preferences.getInt("linear_speed_setting", -1) : -1;
        double speedEstimate = MotionParameterEstimator.estimateSpeedMmPerSecond(
                calibratedRate, speed, calibrationSpeed);
        double stepEstimate = MotionParameterEstimator.estimateStepDistanceMm(
                calibratedRate, speed, calibrationSpeed, stepLength);
        String unavailable = calibrated ? "—" : "待标定";
        return new String[] {
                Double.isFinite(speedEstimate)
                        ? String.format(Locale.CHINA, "%.0f mm/s", speedEstimate) : unavailable,
                Double.isFinite(stepEstimate)
                        ? String.format(Locale.CHINA, "%.0f mm", stepEstimate) : unavailable,
                "—", "—", "—", "—", "—"
        };
    }

    private String buildParameterTableSignature(
            int selected, String[] values, String[] estimates, String[] states) {
        StringBuilder builder = new StringBuilder();
        builder.append(selected);
        for (int i = 0; i < values.length; i++) {
            builder.append('|').append(values[i]).append(':').append(estimates[i])
                    .append(':').append(i == selected ? "编辑中" : states[i]);
        }
        return builder.toString();
    }

    private void setTextIfChanged(TextView textView, String text) {
        if (!text.equals(textView.getText().toString())) {
            textView.setText(text);
        }
    }

    private boolean shouldRefreshStatusAfterAck(String key) {
        if (key == null) {
            return false;
        }
        return "W".equalsIgnoreCase(key)
                || "U".equalsIgnoreCase(key)
                || "S".equalsIgnoreCase(key)
                || "D".equalsIgnoreCase(key)
                || "A".equalsIgnoreCase(key)
                || "L".equalsIgnoreCase(key)
                || "R".equalsIgnoreCase(key)
                || "C".equalsIgnoreCase(key)
                || "M".equalsIgnoreCase(key)
                || "H".equalsIgnoreCase(key)
                || "O".equalsIgnoreCase(key)
                || "X".equalsIgnoreCase(key);
    }

    private String formatOptionalValue(int value) {
        return value >= 0 ? String.valueOf(value) : "?";
    }

    private String formatOnOff(int value) {
        if (value < 0) {
            return "?";
        }
        return value == 0 ? "关闭" : "开启";
    }

    private String formatDistance(int value) {
        if (value < 0) {
            return "?";
        }
        if (value == 0xFFFF) {
            return "无效";
        }
        return value + "mm";
    }

    private String formatUltrasonicDistance(RadarFrame frame) {
        if (frame.getUltrasonicTimeout() == 1) {
            return "超时";
        }
        if (frame.getUltrasonicValid() == 0) {
            return "无效";
        }
        int value = frame.getUltrasonicDistanceCm();
        return value < 0 || value == 0xFFFF ? "?" : value + "cm";
    }

    private String describeManualCommand(int command) {
        switch (command) {
            case 0: return "停止";
            case 1: return "前进";
            case 2: return "后退";
            case 3: return "左转";
            case 4: return "右转";
            default: return "?";
        }
    }

    private String describeVehicleState(StatusFrame frame) {
        if (frame.getMotorBlocked() == 1) return "避障停止";
        if (frame.getWatchdogTriggered() == 1) return "超时停止";
        if (frame.getManualCommand() >= 1 && frame.getManualCommand() <= 4) {
            return describeManualCommand(frame.getManualCommand());
        }
        if (frame.getActiveRouteSteps() > frame.getCompletedRouteSteps()) return "路线运行";
        return frame.getState() == 0 ? "待机" : "状态 " + frame.getState();
    }

    private String formatStopProgress(StatusFrame frame) {
        if (frame.getStopFramesRemaining() > 0) {
            return " / 停止帧发送中，剩余 " + frame.getStopFramesRemaining();
        }
        if (frame.getStopCompleted() == 1) {
            return " / 三次停止帧已完成";
        }
        return "";
    }

    private String formatC5Diagnostics(StatusFrame frame) {
        if (frame.getC5FrameCount() < 0) {
            return "C5 ?";
        }
        return "C5帧 " + frame.getC5FrameCount()
                + " / 脉宽 LF=" + formatOptionalValue(frame.getC5LeftFrontPulse())
                + " RF=" + formatOptionalValue(frame.getC5RightFrontPulse())
                + " LR=" + formatOptionalValue(frame.getC5LeftRearPulse())
                + " RR=" + formatOptionalValue(frame.getC5RightRearPulse())
                + " / 停止完成 " + formatOnOff(frame.getStopCompleted())
                + " 剩余 " + formatOptionalValue(frame.getStopFramesRemaining())
                + " 忙 " + formatOnOff(frame.getC5Busy());
    }

    private String formatLocalCalibrationDiagnostics(StatusFrame frame) {
        if (frame.getLocalCalibrationState() < 0) return "";
        return " / 板载校准 " + describeLocalCalibrationState(
                frame.getLocalCalibrationState())
                + " 静态修正=" + formatOptionalValue(frame.getLearnedYawTrim())
                + " 实时修正=" + formatOptionalValue(frame.getYawCorrection())
                + " gyro峰值=" + formatOptionalValue(frame.getCalibrationGyroPeak());
    }

    private String describeCommand(CarCommand command) {
        switch (command) {
            case UP:
                return "前进";
            case DOWN:
                return "后退";
            case LEFT:
                return "左转";
            case RIGHT:
                return "右转";
            case CENTER:
                return "停止";
            case HEARTBEAT:
                return "心跳";
            case REQUEST_STATUS:
                return "查询状态";
            case ENTER_MANUAL_MODE:
                return "手动模式";
            case ENTER_AUTO_MODE:
                return "自动模式";
            case START_LOCAL_CALIBRATION:
                return "启动板载自动校准";
            case START_ONBOARD_MAPPING:
                return "开始或继续板载建图";
            case PAUSE_ONBOARD_MAPPING:
                return "暂停板载建图";
            case CLEAR_ONBOARD_MAP:
                return "清空板载地图";
            case SAVE_ONBOARD_MAP:
                return "保存板载地图";
            case LOAD_ONBOARD_MAP:
                return "读取板载地图";
            case SYNC_ONBOARD_MAP:
                return "同步板载地图";
            case ENABLE_LIDAR:
                return "开启雷达";
            case DISABLE_LIDAR:
                return "关闭雷达";
            case TOGGLE_SETTING:
                return "切换设置模式";
            case NEXT_PARAMETER:
                return "下一个参数";
            case PREVIOUS_PARAMETER:
                return "上一个参数";
            case INCREASE_PARAMETER:
                return "增加参数";
            case DECREASE_PARAMETER:
                return "减少参数";
            case SELECT_SPEED:
                return "选择整体速度";
            case SELECT_STEP_LENGTH:
                return "选择路线步长";
            case SELECT_LEFT_FRONT_SPEED:
                return "选择左前轮速度";
            case SELECT_RIGHT_FRONT_SPEED:
                return "选择右前轮速度";
            case SELECT_LEFT_REAR_SPEED:
                return "选择左后轮速度";
            case SELECT_RIGHT_REAR_SPEED:
                return "选择右后轮速度";
            case SELECT_RADAR_DISTANCE:
                return "选择雷达安全距离";
            default:
                return command.name();
        }
    }

    private String describeAckKey(String key) {
        if ("W".equalsIgnoreCase(key) || "U".equalsIgnoreCase(key)) {
            return "前进";
        }
        if ("S".equalsIgnoreCase(key)) {
            return "后退";
        }
        if ("A".equalsIgnoreCase(key) || "L".equalsIgnoreCase(key)) {
            return "左转";
        }
        if ("D".equalsIgnoreCase(key) || "R".equalsIgnoreCase(key)) {
            return "右转";
        }
        if ("C".equalsIgnoreCase(key) || "M".equalsIgnoreCase(key)) {
            return "停止";
        }
        if ("5".equals(key) || "Z".equalsIgnoreCase(key)) {
            return "雷达安全距离";
        }
        for (CarCommand command : CarCommand.values()) {
            if (command.getPayload().equalsIgnoreCase(key)) {
                return describeCommand(command);
            }
        }
        return key;
    }

    private String describeSelectedParameter(int selectedParameter) {
        switch (selectedParameter) {
            case 0:
                return "整体速度";
            case 1:
                return "路线步长";
            case 2:
                return "左前轮";
            case 3:
                return "右前轮";
            case 4:
                return "左后轮";
            case 5:
                return "右后轮";
            case 6:
                return "雷达安全距离";
            default:
                return formatOptionalValue(selectedParameter);
        }
    }

    private String toFriendlyBluetoothError(String message) {
        if ("BT OFF".equals(message)) {
            return getString(R.string.bluetooth_off);
        }
        if (message.contains("permission")) {
            return "蓝牙权限不足，请允许蓝牙和定位权限";
        }
        if (message.contains("not enabled")) {
            return "请先打开手机蓝牙";
        }
        if (message.contains("scanner")) {
            return "当前设备无法启动 BLE 扫描";
        }
        if (message.contains("scan failed")) {
            return "蓝牙扫描失败：" + message;
        }
        if (message.contains("service")) {
            return "未找到 CH9141K 串口服务";
        }
        if (message.contains("characteristic")) {
            return "未找到 CH9141K 透传特征值";
        }
        return message;
    }

    @Override
    protected void onStop() {
        if (explorationRunning) stopExploration("App 进入后台");
        if (calibrationRunning) stopCalibration("App 进入后台");
        if (autoCalibrationRunning) stopAutoCalibration("App 进入后台", true);
        super.onStop();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (PermissionHelper.hasRuntimePermissions(this)) {
            bluetoothController.resumeAutoConnect();
        }
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode,
            String[] permissions,
            int[] grantResults
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != PermissionHelper.REQUEST_CODE) return;
        boolean granted = grantResults.length > 0;
        for (int result : grantResults) {
            granted &= result == PackageManager.PERMISSION_GRANTED;
        }
        if (granted && PermissionHelper.hasRuntimePermissions(this)) {
            startVehicleAutoConnect();
        } else {
            showStatus("需要蓝牙扫描与连接权限才能自动连接小车");
        }
    }

    @Override
    protected void onDestroy() {
        handler.removeCallbacksAndMessages(null);
        if (uwbServerClient != null) uwbServerClient.shutdown();
        bluetoothController.shutdown();
        super.onDestroy();
    }
}
