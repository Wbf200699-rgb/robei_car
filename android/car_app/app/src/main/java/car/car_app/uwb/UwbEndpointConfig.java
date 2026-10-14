package car.car_app.uwb;

import java.util.Locale;

/** User-selected UWB relay route. Public and LAN routes are switched manually. */
public final class UwbEndpointConfig {
    public enum Mode { PUBLIC, LAN }

    public static final int DEFAULT_LAN_PORT = 18765;

    private final Mode mode;
    private final String lanHost;
    private final int lanPort;

    public UwbEndpointConfig(Mode mode, String lanHost, int lanPort) {
        if (mode == null) throw new IllegalArgumentException("mode is required");
        if (lanPort < 1 || lanPort > 65535) {
            throw new IllegalArgumentException("port must be between 1 and 65535");
        }
        String normalizedHost = lanHost == null ? "" : lanHost.trim();
        if (mode == Mode.LAN && !isPrivateIpv4(normalizedHost)) {
            throw new IllegalArgumentException("请输入有效的局域网 IPv4 地址");
        }
        this.mode = mode;
        this.lanHost = normalizedHost;
        this.lanPort = lanPort;
    }

    public static UwbEndpointConfig publicServer() {
        return new UwbEndpointConfig(Mode.PUBLIC, "", DEFAULT_LAN_PORT);
    }

    public Mode getMode() { return mode; }
    public String getLanHost() { return lanHost; }
    public int getLanPort() { return lanPort; }

    public String endpoint(String publicEndpoint) {
        if (mode == Mode.PUBLIC) return publicEndpoint == null ? "" : publicEndpoint.trim();
        return String.format(Locale.US, "ws://%s:%d/ws/mobile", lanHost, lanPort);
    }

    public String summary() {
        if (mode == Mode.PUBLIC) return "公网服务器 · 使用构建配置地址";
        return String.format(Locale.US, "局域网电脑 · %s:%d", lanHost, lanPort);
    }

    public static boolean isPrivateIpv4(String value) {
        if (value == null) return false;
        String[] parts = value.trim().split("\\.", -1);
        if (parts.length != 4) return false;
        int[] octets = new int[4];
        try {
            for (int index = 0; index < 4; index++) {
                if (parts[index].isEmpty() || parts[index].length() > 3) return false;
                octets[index] = Integer.parseInt(parts[index]);
                if (octets[index] < 0 || octets[index] > 255) return false;
            }
        } catch (NumberFormatException error) {
            return false;
        }
        return octets[0] == 10
                || (octets[0] == 172 && octets[1] >= 16 && octets[1] <= 31)
                || (octets[0] == 192 && octets[1] == 168)
                || (octets[0] == 100 && octets[1] >= 64 && octets[1] <= 127);
    }
}
