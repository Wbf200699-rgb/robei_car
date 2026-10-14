package car.car_app.bluetooth;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;

import androidx.core.content.ContextCompat;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

public class BluetoothController {
    private static final long WRITE_CALLBACK_TIMEOUT_MS = 5000L;
    private static final long WRITE_START_RETRY_MS = 80L;
    private static final long AUTO_SCAN_TIMEOUT_MS = 12000L;
    private static final long AUTO_RECONNECT_MIN_MS = 1200L;
    private static final long AUTO_RECONNECT_MAX_MS = 10000L;
    private static final String PREFERENCES_NAME = "vehicle_ble";
    private static final String LAST_DEVICE_ADDRESS_KEY = "last_device_address";

    public interface Listener {
        void onDeviceFound(BluetoothDevice device, int rssi);

        void onConnected();

        void onDisconnected();

        void onReady();

        void onFrameReceived(BleFrame frame);

        void onError(String message);

        void onDebugLog(String tag, String message);
    }

    private static final UUID CH9141_SERVICE_UUID =
            UUID.fromString("0000FFF0-0000-1000-8000-00805F9B34FB");
    private static final UUID CH9141_NOTIFY_UUID =
            UUID.fromString("0000FFF1-0000-1000-8000-00805F9B34FB");
    private static final UUID CH9141_WRITE_UUID =
            UUID.fromString("0000FFF2-0000-1000-8000-00805F9B34FB");
    private static final UUID CLIENT_CHARACTERISTIC_CONFIG_UUID =
            UUID.fromString("00002902-0000-1000-8000-00805F9B34FB");

    private final Context context;
    private final BluetoothAdapter bluetoothAdapter;
    private final BleFrameParser frameParser = new BleFrameParser();
    private final Map<String, BluetoothDevice> discoveredDevices = new LinkedHashMap<>();
    private final Handler handler = new Handler(Looper.getMainLooper());

    private BluetoothGatt bluetoothGatt;
    private BluetoothGattCharacteristic notifyCharacteristic;
    private BluetoothGattCharacteristic writeCharacteristic;
    private BluetoothLeScanner scanner;
    private ScanCallback scanCallback;
    private Listener listener;
    private boolean ready;
    private boolean writeInProgress;
    private boolean writeRetryScheduled;
    private boolean manualScan;
    private boolean autoConnectEnabled;
    private boolean autoReconnectSuppressed;
    private boolean connectionAttemptInProgress;
    private int reconnectAttempt;
    private String preferredAddress;
    private int writeSequence;
    private int rxNotificationCount;
    private int rxByteCount;
    private long rxMetricsStartedAtMs;
    private final ArrayDeque<String> writeQueue = new ArrayDeque<>();

    private final Runnable autoScanTimeoutRunnable = () -> {
        if (manualScan || scanCallback == null) return;
        log("BLE", "automatic scan timed out");
        stopScanInternal();
        scheduleAutoReconnect();
    };

    private final Runnable autoReconnectRunnable = () -> {
        if (!shouldAutoConnect()) return;
        startScanInternal(false);
    };

    public BluetoothController(Context context) {
        this.context = context.getApplicationContext();
        BluetoothManager bluetoothManager =
                (BluetoothManager) this.context.getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager != null ? bluetoothManager.getAdapter() : null;
    }

    public void setListener(Listener listener) {
        this.listener = listener;
    }

    public boolean isBluetoothAvailable() {
        return bluetoothAdapter != null;
    }

    public boolean isBluetoothEnabled() {
        return bluetoothAdapter != null && bluetoothAdapter.isEnabled();
    }

    public boolean isConnected() {
        return bluetoothGatt != null && ready;
    }

    public List<BluetoothDevice> getDiscoveredDevices() {
        return new ArrayList<>(discoveredDevices.values());
    }

    /**
     * Enables foreground auto-connect. The known vehicle address is preferred;
     * a manually selected CH9141K address is persisted and used on later runs.
     */
    public void enableAutoConnect(String vehicleAddress) {
        preferredAddress = normalizeAddress(vehicleAddress);
        autoConnectEnabled = true;
        autoReconnectSuppressed = false;
        reconnectAttempt = 0;
        scheduleAutoReconnect(0L);
    }

    /** Resumes scanning only when the user has not explicitly disconnected. */
    public void resumeAutoConnect() {
        if (shouldAutoConnect()) scheduleAutoReconnect(0L);
    }

    public boolean isAutoConnectEnabled() {
        return autoConnectEnabled && !autoReconnectSuppressed;
    }

    public BluetoothDevice getRemoteDevice(String address) {
        if (bluetoothAdapter == null || !BluetoothAdapter.checkBluetoothAddress(address)) {
            return null;
        }
        try {
            return bluetoothAdapter.getRemoteDevice(address);
        } catch (IllegalArgumentException exception) {
            return null;
        }
    }

    @SuppressLint("MissingPermission")
    public void startScan() {
        startScanInternal(true);
    }

    @SuppressLint("MissingPermission")
    private void startScanInternal(boolean requestedManually) {
        if (!isBluetoothEnabled()) {
            if (requestedManually) notifyError("Bluetooth is not enabled");
            return;
        }
        if (!hasBluetoothPermission()) {
            if (requestedManually) notifyError("Bluetooth permission is required");
            return;
        }
        if (!requestedManually && !shouldAutoConnect()) return;

        stopScanInternal();
        manualScan = requestedManually;
        discoveredDevices.clear();
        scanner = bluetoothAdapter.getBluetoothLeScanner();
        if (scanner == null) {
            notifyError("BLE scanner is not available");
            if (!requestedManually) scheduleAutoReconnect();
            return;
        }

        scanCallback = new ScanCallback() {
            @Override
            public void onScanResult(int callbackType, ScanResult result) {
                BluetoothDevice device = result.getDevice();
                if (device == null || device.getAddress() == null) {
                    return;
                }
                if (!discoveredDevices.containsKey(device.getAddress())) {
                    discoveredDevices.put(device.getAddress(), device);
                    if (listener != null) {
                        listener.onDeviceFound(device, result.getRssi());
                    }
                }
                if (!manualScan && shouldAutoConnect() && isAutoConnectTarget(result)) {
                    log("BLE", "automatic scan matched " + device.getAddress()
                            + " rssi=" + result.getRssi());
                    connectionAttemptInProgress = true;
                    handler.post(() -> connectInternal(device, true));
                }
            }

            @Override
            public void onScanFailed(int errorCode) {
                notifyError("BLE scan failed: " + errorCode);
                stopScanInternal();
                scheduleAutoReconnect();
            }
        };

        scanner.startScan(scanCallback);
        log("BLE", requestedManually ? "manual scan started" : "automatic scan started");
        if (!requestedManually) {
            handler.postDelayed(autoScanTimeoutRunnable, AUTO_SCAN_TIMEOUT_MS);
        }
    }

    @SuppressLint("MissingPermission")
    public void stopScan() {
        boolean resumeAutomaticScan = manualScan;
        stopScanInternal();
        if (resumeAutomaticScan) scheduleAutoReconnect();
    }

    @SuppressLint("MissingPermission")
    private void stopScanInternal() {
        handler.removeCallbacks(autoScanTimeoutRunnable);
        if (scanner != null && scanCallback != null && hasBluetoothPermission()) {
            scanner.stopScan(scanCallback);
        }
        scanCallback = null;
        scanner = null;
        manualScan = false;
    }

    @SuppressLint("MissingPermission")
    public void connect(BluetoothDevice device) {
        if (device == null) return;
        autoConnectEnabled = true;
        autoReconnectSuppressed = false;
        reconnectAttempt = 0;
        preferredAddress = normalizeAddress(device.getAddress());
        rememberDeviceAddress(preferredAddress);
        connectInternal(device, false);
    }

    @SuppressLint("MissingPermission")
    private void connectInternal(BluetoothDevice device, boolean automatic) {
        if (!hasBluetoothPermission()) {
            notifyError("Bluetooth permission is required");
            connectionAttemptInProgress = false;
            return;
        }
        if (device == null || (!automatic && autoReconnectSuppressed)) {
            connectionAttemptInProgress = false;
            return;
        }
        stopScanInternal();
        handler.removeCallbacks(autoReconnectRunnable);
        closeCurrentGatt();
        ready = false;
        resetReceiveSession();
        connectionAttemptInProgress = true;
        log("BLE", (automatic ? "auto" : "manual") + " connecting " + device.getAddress());
        bluetoothGatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
    }

    public void sendCommand(CarCommand command) {
        sendRaw(command.getPayload());
    }

    public void sendPriorityCommand(CarCommand command) {
        sendPriorityRaw(command.getPayload());
    }

    public void sendEmergencyStop() {
        if (!isConnected() || writeCharacteristic == null) return;
        writeQueue.clear();
        // H first makes STOP persistent even on older FPGA images where C only
        // emitted a finite stop-frame burst without leaving automatic mode.
        writeQueue.offerLast("H");
        writeQueue.offerLast("C");
        drainWriteQueue();
    }

    public boolean hasPendingWrites() {
        return writeInProgress || !writeQueue.isEmpty();
    }

    @SuppressLint("MissingPermission")
    public void sendRaw(String payload) {
        if (!isConnected() || writeCharacteristic == null) {
            notifyError("BT OFF");
            return;
        }
        if ("P".equals(payload)) {
            // A delayed GATT callback must not build a backlog of obsolete
            // heartbeats ahead of a later vehicle command.
            writeQueue.removeIf("P"::equals);
        }
        if (isSingleMotionPayload(payload)) {
            // Only the newest held direction matters.  Keeping obsolete W/A/S/D
            // writes behind telemetry creates visible command residue.
            writeQueue.removeIf(this::isSingleMotionPayload);
        }
        writeQueue.offer(payload);
        drainWriteQueue();
    }

    @SuppressLint("MissingPermission")
    public void sendPriorityRaw(String payload) {
        if (!isConnected() || writeCharacteristic == null) {
            notifyError("BT OFF");
            return;
        }
        if (isSingleMotionPayload(payload)) {
            writeQueue.removeIf(this::isSingleMotionPayload);
        } else {
            writeQueue.removeIf(payload::equals);
        }
        writeQueue.offerFirst(payload);
        drainWriteQueue();
    }

    private boolean isSingleMotionPayload(String payload) {
        if (payload == null || payload.length() != 1) return false;
        char command = Character.toUpperCase(payload.charAt(0));
        return command == 'W' || command == 'U' || command == 'S'
                || command == 'A' || command == 'L' || command == 'D'
                || command == 'R' || command == 'C' || command == 'M';
    }

    @SuppressLint("MissingPermission")
    private void drainWriteQueue() {
        if (writeInProgress || writeRetryScheduled
                || !isConnected() || writeCharacteristic == null) {
            return;
        }
        String payload = writeQueue.poll();
        if (payload == null) {
            return;
        }

        byte[] data = payload.getBytes(StandardCharsets.US_ASCII);
        int writeType = getWriteType(writeCharacteristic);
        boolean started;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            int result = bluetoothGatt.writeCharacteristic(writeCharacteristic, data, writeType);
            started = result == BluetoothGatt.GATT_SUCCESS;
            log("TX", "write " + payload + " result=" + result + " type=" + writeType);
        } else {
            writeCharacteristic.setWriteType(writeType);
            writeCharacteristic.setValue(data);
            started = bluetoothGatt.writeCharacteristic(writeCharacteristic);
            log("TX", "write " + payload + " started=" + started + " type=" + writeType);
        }
        if (!started) {
            // Android can report GATT_WRITE_REQUEST_BUSY while the stack is
            // still completing the preceding request. Keep the command at
            // the head of the queue and retry later; recursively draining the
            // queue here drops commands and can turn one delayed callback into
            // a stream of overlapping writes.
            writeQueue.offerFirst(payload);
            log("TX", "write start busy; retry queued");
            writeRetryScheduled = true;
            handler.postDelayed(() -> {
                writeRetryScheduled = false;
                drainWriteQueue();
            }, WRITE_START_RETRY_MS);
        } else {
            writeInProgress = true;
            int sequence = ++writeSequence;
            handler.postDelayed(() -> {
                if (writeInProgress && sequence == writeSequence) {
                    // A write-with-response must remain the only outstanding
                    // GATT operation until its callback arrives. Starting the
                    // next write after an arbitrary timeout produces Android
                    // error 201 and can destabilize the connection with 133.
                    log("TX", "write callback timeout; disconnect without overlapping writes");
                    notifyError("BLE write callback timeout");
                    ready = false;
                    if (bluetoothGatt != null && hasBluetoothPermission()) {
                        bluetoothGatt.disconnect();
                    }
                }
            }, WRITE_CALLBACK_TIMEOUT_MS);
        }
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        autoReconnectSuppressed = true;
        reconnectAttempt = 0;
        handler.removeCallbacks(autoReconnectRunnable);
        stopScanInternal();
        closeCurrentGatt();
    }

    /** Releases resources without changing the user's reconnect preference. */
    @SuppressLint("MissingPermission")
    public void shutdown() {
        autoConnectEnabled = false;
        handler.removeCallbacks(autoReconnectRunnable);
        stopScanInternal();
        closeCurrentGatt();
    }

    @SuppressLint("MissingPermission")
    private void closeCurrentGatt() {
        ready = false;
        connectionAttemptInProgress = false;
        writeInProgress = false;
        writeRetryScheduled = false;
        writeSequence++;
        writeQueue.clear();
        notifyCharacteristic = null;
        writeCharacteristic = null;
        resetReceiveSession();
        if (bluetoothGatt != null && hasBluetoothPermission()) {
            bluetoothGatt.disconnect();
            bluetoothGatt.close();
        }
        bluetoothGatt = null;
    }

    @SuppressLint("MissingPermission")
    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            log("GATT", "connection state status=" + status + " newState=" + newState);
            if (gatt != bluetoothGatt) {
                gatt.close();
                return;
            }
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                connectionAttemptInProgress = false;
                reconnectAttempt = 0;
                rememberDeviceAddress(gatt.getDevice().getAddress());
                if (listener != null) {
                    listener.onConnected();
                }
                gatt.discoverServices();
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                ready = false;
                connectionAttemptInProgress = false;
                writeInProgress = false;
                writeRetryScheduled = false;
                writeSequence++;
                writeQueue.clear();
                notifyCharacteristic = null;
                writeCharacteristic = null;
                if (listener != null) {
                    listener.onDisconnected();
                }
                gatt.close();
                if (bluetoothGatt == gatt) {
                    bluetoothGatt = null;
                }
                scheduleAutoReconnect();
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            log("GATT", "service discovery status=" + status);
            if (status != BluetoothGatt.GATT_SUCCESS) {
                notifyError("BLE service discovery failed: " + status);
                gatt.disconnect();
                return;
            }

            BluetoothGattService service = gatt.getService(CH9141_SERVICE_UUID);
            if (service == null) {
                notifyError("CH9141 service FFF0 not found");
                gatt.disconnect();
                return;
            }

            log("GATT", "found CH9141 service FFF0");
            log("BLE", "CH9141K UART baud is 115200 on hardware side; BLE protocol unchanged");
            for (BluetoothGattCharacteristic characteristic : service.getCharacteristics()) {
                log("GATT", characteristic.getUuid() + " properties=" + describeProperties(characteristic));
            }

            notifyCharacteristic = service.getCharacteristic(CH9141_NOTIFY_UUID);
            writeCharacteristic = service.getCharacteristic(CH9141_WRITE_UUID);
            if (notifyCharacteristic == null || writeCharacteristic == null) {
                notifyError("CH9141 must expose FFF1 notify and FFF2 write");
                gatt.disconnect();
                return;
            }
            if (!hasAnyProperty(
                    notifyCharacteristic,
                    BluetoothGattCharacteristic.PROPERTY_NOTIFY | BluetoothGattCharacteristic.PROPERTY_INDICATE
            )) {
                notifyError("FFF1 does not support notify");
                gatt.disconnect();
                return;
            }
            if (!hasAnyProperty(
                    writeCharacteristic,
                    BluetoothGattCharacteristic.PROPERTY_WRITE
                            | BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE
            )) {
                notifyError("FFF2 does not support write");
                gatt.disconnect();
                return;
            }

            log("GATT", "notify characteristic=" + notifyCharacteristic.getUuid());
            log("GATT", "write characteristic=" + writeCharacteristic.getUuid());
            enableNotifications(gatt, notifyCharacteristic);
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            handleIncoming(characteristic.getValue());
        }

        @Override
        public void onCharacteristicChanged(
                BluetoothGatt gatt,
                BluetoothGattCharacteristic characteristic,
                byte[] value
        ) {
            handleIncoming(value);
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            if (!CLIENT_CHARACTERISTIC_CONFIG_UUID.equals(descriptor.getUuid())) {
                return;
            }
            log("GATT", "notify descriptor write status=" + status);
            ready = status == BluetoothGatt.GATT_SUCCESS;
            if (ready && listener != null) {
                boolean priorityRequested = gatt.requestConnectionPriority(
                        BluetoothGatt.CONNECTION_PRIORITY_HIGH
                );
                log("GATT", "high connection priority requested=" + priorityRequested);
                listener.onReady();
            } else if (!ready) {
                notifyError("Notify descriptor write failed: " + status);
                gatt.disconnect();
            }
        }

        @Override
        public void onCharacteristicWrite(
                BluetoothGatt gatt,
                BluetoothGattCharacteristic characteristic,
                int status
        ) {
            handler.post(() -> {
                if (gatt != bluetoothGatt) {
                    log("GATT", "ignore stale write callback");
                    return;
                }
                log("GATT", "write complete status=" + status + " uuid=" + characteristic.getUuid());
                writeSequence++;
                writeInProgress = false;
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    notifyError("BLE write failed: " + status);
                }
                drainWriteQueue();
            });
        }
    };

    @SuppressLint("MissingPermission")
    private void enableNotifications(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
        boolean notificationStarted = gatt.setCharacteristicNotification(characteristic, true);
        log("GATT", "local notification enabled=" + notificationStarted);
        BluetoothGattDescriptor descriptor = characteristic.getDescriptor(CLIENT_CHARACTERISTIC_CONFIG_UUID);
        if (descriptor == null) {
            log("GATT", "no CCCD descriptor, mark ready directly");
            ready = true;
            if (listener != null) {
                listener.onReady();
            }
            return;
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            int result = gatt.writeDescriptor(descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            log("GATT", "write notify descriptor result=" + result);
        } else {
            descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            boolean started = gatt.writeDescriptor(descriptor);
            log("GATT", "write notify descriptor started=" + started);
        }
    }

    private void handleIncoming(byte[] data) {
        if (data == null || listener == null) {
            return;
        }
        aggregateReceiveMetrics(data.length);
        for (BleFrame frame : frameParser.append(data)) {
            listener.onFrameReceived(frame);
        }
    }

    private void aggregateReceiveMetrics(int byteCount) {
        long now = System.currentTimeMillis();
        if (rxMetricsStartedAtMs == 0L) {
            rxMetricsStartedAtMs = now;
        }
        rxNotificationCount++;
        rxByteCount += byteCount;
        long elapsed = now - rxMetricsStartedAtMs;
        if (elapsed < 1000L) return;

        float bytesPerSecond = elapsed > 0L ? rxByteCount * 1000f / elapsed : 0f;
        log("RX", String.format(
                java.util.Locale.CHINA,
                "notify=%d bytes=%d rate=%.0fB/s",
                rxNotificationCount,
                rxByteCount,
                bytesPerSecond
        ));
        rxNotificationCount = 0;
        rxByteCount = 0;
        rxMetricsStartedAtMs = now;
    }

    private void resetReceiveSession() {
        frameParser.reset();
        rxNotificationCount = 0;
        rxByteCount = 0;
        rxMetricsStartedAtMs = 0L;
    }

    private int getWriteType(BluetoothGattCharacteristic characteristic) {
        if (hasAnyProperty(characteristic, BluetoothGattCharacteristic.PROPERTY_WRITE)) {
            return BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT;
        }
        return BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE;
    }

    private boolean hasAnyProperty(BluetoothGattCharacteristic characteristic, int propertyMask) {
        return (characteristic.getProperties() & propertyMask) != 0;
    }

    private String describeProperties(BluetoothGattCharacteristic characteristic) {
        int properties = characteristic.getProperties();
        List<String> labels = new ArrayList<>();
        if ((properties & BluetoothGattCharacteristic.PROPERTY_READ) != 0) {
            labels.add("READ");
        }
        if ((properties & BluetoothGattCharacteristic.PROPERTY_WRITE) != 0) {
            labels.add("WRITE");
        }
        if ((properties & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0) {
            labels.add("WRITE_NO_RESPONSE");
        }
        if ((properties & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0) {
            labels.add("NOTIFY");
        }
        if ((properties & BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0) {
            labels.add("INDICATE");
        }
        return labels.toString();
    }

    private boolean hasBluetoothPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT)
                    == PackageManager.PERMISSION_GRANTED
                    && ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN)
                    == PackageManager.PERMISSION_GRANTED;
        }
        return true;
    }

    private boolean shouldAutoConnect() {
        return autoConnectEnabled
                && !autoReconnectSuppressed
                && bluetoothGatt == null
                && !ready
                && !connectionAttemptInProgress
                && isBluetoothEnabled()
                && hasBluetoothPermission();
    }

    private void scheduleAutoReconnect() {
        long exponent = Math.min(reconnectAttempt, 4);
        long delay = Math.min(AUTO_RECONNECT_MAX_MS,
                AUTO_RECONNECT_MIN_MS << exponent);
        reconnectAttempt++;
        scheduleAutoReconnect(delay);
    }

    private void scheduleAutoReconnect(long delayMs) {
        handler.removeCallbacks(autoReconnectRunnable);
        if (!shouldAutoConnect()) return;
        log("BLE", "automatic reconnect scheduled in " + delayMs + "ms");
        handler.postDelayed(autoReconnectRunnable, delayMs);
    }

    @SuppressLint("MissingPermission")
    private boolean isAutoConnectTarget(ScanResult result) {
        BluetoothDevice device = result.getDevice();
        String address = normalizeAddress(device.getAddress());
        String lastAddress = normalizeAddress(context
                .getSharedPreferences(PREFERENCES_NAME, Context.MODE_PRIVATE)
                .getString(LAST_DEVICE_ADDRESS_KEY, null));
        if (address != null && (address.equals(preferredAddress) || address.equals(lastAddress))) {
            return true;
        }

        // Name/service fallback is used only when no known address exists, so
        // another nearby CH9141K cannot steal an established vehicle profile.
        if (preferredAddress != null || lastAddress != null) return false;
        String name = device.getName();
        boolean nameMatches = name != null
                && name.toUpperCase(java.util.Locale.ROOT).contains("CH9141");
        boolean serviceMatches = result.getScanRecord() != null
                && result.getScanRecord().getServiceUuids() != null
                && result.getScanRecord().getServiceUuids().contains(
                new ParcelUuid(CH9141_SERVICE_UUID));
        return nameMatches || serviceMatches;
    }

    private String normalizeAddress(String address) {
        if (address == null || !BluetoothAdapter.checkBluetoothAddress(address)) return null;
        return address.toUpperCase(java.util.Locale.ROOT);
    }

    private void rememberDeviceAddress(String address) {
        String normalized = normalizeAddress(address);
        if (normalized == null) return;
        preferredAddress = normalized;
        context.getSharedPreferences(PREFERENCES_NAME, Context.MODE_PRIVATE)
                .edit()
                .putString(LAST_DEVICE_ADDRESS_KEY, normalized)
                .apply();
    }

    private void notifyError(String message) {
        if (listener != null) {
            listener.onError(message);
        }
    }

    private void log(String tag, String message) {
        if (listener != null) {
            listener.onDebugLog(tag, message);
        }
    }
}
