package car.car_app.uwb;

import org.json.JSONException;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;
import car.car_app.localization.AnchorLayout;

/** Maintains the phone's UWB stream for either the public or a user-selected LAN relay. */
public final class UwbServerClient {
    public interface Listener {
        void onConnectionChanged(boolean connected, String detail);
        void onSnapshot(UwbServerSnapshot snapshot);
        void onAnchorCalibration(boolean running, String summary);
        void onAnchorLayout(String state, String detail, AnchorLayout layout);
    }

    private static final long RECONNECT_MIN_MS = 1000L;
    private static final long RECONNECT_MAX_MS = 15000L;

    private final OkHttpClient httpClient;
    private final String endpoint;
    private final String token;
    private final Listener listener;
    private final Map<Integer, UwbTagFix> tagFixes = new HashMap<>();
    private final Map<Integer, ArrayDeque<double[]>> tagPositionHistory = new HashMap<>();

    private WebSocket socket;
    private boolean closed;
    private boolean reconnectScheduled;
    private boolean gatewayConnected;
    private long reconnectDelayMs = RECONNECT_MIN_MS;
    private UwbVehiclePose latestPose = unavailablePose();

    public UwbServerClient(String endpoint, String token, Listener listener) {
        this.endpoint = endpoint == null ? "" : endpoint.trim();
        this.token = token == null ? "" : token.trim();
        this.listener = listener;
        httpClient = new OkHttpClient.Builder()
                .pingInterval(15, TimeUnit.SECONDS)
                .retryOnConnectionFailure(true)
                .build();
    }

    public synchronized void connect() {
        if (closed || socket != null) return;
        if (endpoint.isEmpty() || token.isEmpty()) {
            listener.onConnectionChanged(false, "UWB服务器配置缺失");
            return;
        }
        String separator = endpoint.contains("?") ? "&" : "?";
        Request request = new Request.Builder()
                .url(endpoint + separator + "token=" + token)
                .build();
        socket = httpClient.newWebSocket(request, new SocketListener());
    }

    public synchronized void shutdown() {
        closed = true;
        reconnectScheduled = false;
        if (socket != null) socket.close(1000, "app shutdown");
        socket = null;
        httpClient.dispatcher().executorService().shutdown();
    }

    public synchronized boolean startAnchorCalibration() {
        return socket != null && socket.send("{\"type\":\"anchor_calibrate\"}");
    }

    public synchronized boolean cancelAnchorCalibration() {
        return socket != null && socket.send("{\"type\":\"anchor_calibrate_cancel\"}");
    }

    public synchronized boolean configureAnchorLayout(AnchorLayout layout) {
        if (socket == null || layout == null) return false;
        try {
            JSONArray corners = new JSONArray();
            for (int anchor : layout.getCornerAnchors()) corners.put(anchor);
            JSONObject message = new JSONObject()
                    .put("type", "anchor_layout_set")
                    .put("width_cm", layout.getWidthCm())
                    .put("height_cm", layout.getHeightCm())
                    .put("corner_anchor_ids", corners);
            return socket.send(message.toString());
        } catch (JSONException error) {
            return false;
        }
    }

    public synchronized void resetCoordinateSession() {
        tagFixes.clear();
        tagPositionHistory.clear();
        latestPose = unavailablePose();
        emitSnapshot();
    }

    private void handleMessage(String text) {
        try {
            JSONObject root = new JSONObject(text);
            String type = root.optString("type", "");
            if ("snapshot".equals(type)) {
                parseSnapshot(root.getJSONObject("data"));
            } else if ("uwb_position".equals(type)) {
                UwbTagFix fix = parseTag(root.getJSONObject("tag"), true,
                        root.getJSONObject("tag").getInt("id"));
                tagFixes.put(fix.getTagId(), fix);
                latestPose = parsePose(root.optJSONObject("pose"));
                emitSnapshot();
            } else if ("gateway".equals(type)) {
                gatewayConnected = root.optBoolean("connected", false);
                emitSnapshot();
            } else if ("anchor_calibration".equals(type)) {
                emitCalibration(root.optJSONObject("data"));
            } else if ("anchor_layout".equals(type)) {
                emitAnchorLayout(root.optJSONObject("data"));
            }
        } catch (JSONException | IllegalArgumentException error) {
            listener.onConnectionChanged(true, "UWB数据格式异常");
        }
    }

    private void parseSnapshot(JSONObject data) throws JSONException {
        gatewayConnected = data.optBoolean("gateway_connected", false);
        tagFixes.clear();
        tagPositionHistory.clear();
        JSONObject fixes = data.optJSONObject("tag_fixes");
        if (fixes != null) {
            Iterator<String> ids = fixes.keys();
            while (ids.hasNext()) {
                String id = ids.next();
                UwbTagFix fix = parseTag(fixes.getJSONObject(id), false, Integer.parseInt(id));
                tagFixes.put(fix.getTagId(), fix);
            }
        }
        latestPose = parsePose(data.optJSONObject("pose"));
        emitCalibration(data.optJSONObject("anchor_calibration"));
        emitAnchorLayout(data.optJSONObject("anchor_layout"));
        emitSnapshot();
    }

    private void emitAnchorLayout(JSONObject data) {
        if (data == null) return;
        AnchorLayout layout = null;
        JSONArray corners = data.optJSONArray("corner_anchor_ids");
        if (data.has("width_cm") && data.has("height_cm")
                && corners != null && corners.length() == 4) {
            try {
                int[] anchorAtCorner = new int[4];
                for (int index = 0; index < 4; index++) {
                    anchorAtCorner[index] = corners.getInt(index);
                }
                layout = new AnchorLayout(data.getInt("width_cm"),
                        data.getInt("height_cm"), anchorAtCorner);
            } catch (JSONException | IllegalArgumentException ignored) {
                // Still report state/detail so malformed server data remains visible.
            }
        }
        listener.onAnchorLayout(
                data.optString("state", "unknown"),
                data.optString("detail", ""), layout);
    }

    private void emitCalibration(JSONObject data) {
        if (data == null) return;
        String state = data.optString("state", "idle");
        boolean running = !("idle".equals(state) || "complete".equals(state)
                || "failed".equals(state) || "rejected".equals(state));
        StringBuilder summary = new StringBuilder("状态 ").append(state);
        JSONObject pairs = data.optJSONObject("pairs_cm");
        if (pairs != null && pairs.length() > 0) {
            summary.append(" · ");
            String[] names = {"AB", "AC", "AD", "BC", "BD", "CD"};
            for (String name : names) {
                if (pairs.has(name)) summary.append(name).append('=').append(pairs.optInt(name)).append("cm ");
            }
        }
        JSONObject result = data.optJSONObject("result");
        if (result != null) {
            summary.append("\n解算 ").append(result.optString("quality", "unknown"));
            if (!result.isNull("residual_cm")) {
                summary.append(" · BD残差 ")
                        .append(String.format(java.util.Locale.CHINA, "%.1fcm", result.optDouble("residual_cm")));
            }
        } else if (data.has("detail")) {
            summary.append(" · ").append(data.optString("detail"));
        }
        listener.onAnchorCalibration(running, summary.toString().trim());
    }

    private UwbTagFix parseTag(JSONObject tag, boolean event, int tagId) {
        boolean positionValid = tag.optBoolean("position_valid", false);
        boolean fresh = event || tag.optBoolean("fresh", false);
        double xCm = tag.optDouble("x_cm", 0.0);
        double yCm = tag.optDouble("y_cm", 0.0);
        double zCm = tag.optDouble("z_cm", 0.0);
        if (positionValid && fresh) {
            ArrayDeque<double[]> history = tagPositionHistory.computeIfAbsent(
                    tagId, ignored -> new ArrayDeque<>(5));
            if (history.size() == 5) history.removeFirst();
            history.addLast(new double[]{xCm, yCm, zCm});
            xCm = medianCoordinate(history, 0);
            yCm = medianCoordinate(history, 1);
            zCm = medianCoordinate(history, 2);
        }
        return new UwbTagFix(
                tagId,
                positionValid,
                fresh,
                xCm,
                yCm,
                zCm
        );
    }

    private static double medianCoordinate(ArrayDeque<double[]> history, int axis) {
        List<Double> values = new ArrayList<>(history.size());
        for (double[] sample : history) values.add(sample[axis]);
        Collections.sort(values);
        int middle = values.size() / 2;
        if ((values.size() & 1) != 0) return values.get(middle);
        return (values.get(middle - 1) + values.get(middle)) / 2.0;
    }

    private UwbVehiclePose parsePose(JSONObject pose) {
        if (pose == null) return unavailablePose();
        Double yCm = nullableDouble(pose, "y_cm");
        Double headingDeg = nullableDouble(pose, "heading_deg");
        return new UwbVehiclePose(
                pose.optString("quality", "unavailable"),
                nullableDouble(pose, "x_cm"),
                yCm,
                headingDeg,
                pose.optJSONArray("valid_tags") == null ? 0 : pose.optJSONArray("valid_tags").length(),
                nullableDouble(pose, "fix_skew_ms"),
                nullableEpochMs(pose, "updated_at")
        );
    }

    private static Double nullableDouble(JSONObject object, String key) {
        return object.has(key) && !object.isNull(key) ? object.optDouble(key) : null;
    }

    private static Long nullableEpochMs(JSONObject object, String key) {
        if (!object.has(key) || object.isNull(key)) return null;
        double seconds = object.optDouble(key, Double.NaN);
        return Double.isFinite(seconds) ? Math.round(seconds * 1000.0) : null;
    }

    private static UwbVehiclePose unavailablePose() {
        return new UwbVehiclePose("unavailable", null, null, null, 0, null, null);
    }

    private void emitSnapshot() {
        listener.onSnapshot(new UwbServerSnapshot(
                gatewayConnected, tagFixes, latestPose, System.currentTimeMillis()));
    }

    private void scheduleReconnect(WebSocket failedSocket) {
        final long delay;
        synchronized (this) {
            // A callback from an older socket must never clear or disconnect a
            // newer connection that has already succeeded.
            if (closed || socket != failedSocket || reconnectScheduled) return;
            socket = null;
            reconnectScheduled = true;
            delay = reconnectDelayMs;
            reconnectDelayMs = Math.min(RECONNECT_MAX_MS, reconnectDelayMs * 2L);
        }
        httpClient.dispatcher().executorService().execute(() -> {
            try {
                Thread.sleep(delay);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
            synchronized (UwbServerClient.this) {
                reconnectScheduled = false;
            }
            connect();
        });
    }

    private final class SocketListener extends WebSocketListener {
        @Override
        public void onOpen(WebSocket webSocket, Response response) {
            synchronized (UwbServerClient.this) {
                if (closed || socket != webSocket) {
                    webSocket.close(1000, "stale connection");
                    return;
                }
                reconnectDelayMs = RECONNECT_MIN_MS;
                reconnectScheduled = false;
            }
            listener.onConnectionChanged(true, "UWB服务器已连接");
        }

        @Override
        public void onMessage(WebSocket webSocket, String text) {
            handleMessage(text);
        }

        @Override
        public void onClosed(WebSocket webSocket, int code, String reason) {
            synchronized (UwbServerClient.this) {
                if (socket != webSocket) return;
            }
            listener.onConnectionChanged(false, "UWB服务器连接关闭");
            scheduleReconnect(webSocket);
        }

        @Override
        public void onFailure(WebSocket webSocket, Throwable error, Response response) {
            synchronized (UwbServerClient.this) {
                if (socket != webSocket) return;
            }
            listener.onConnectionChanged(false, "UWB服务器暂不可用");
            scheduleReconnect(webSocket);
        }
    }
}
