package car.car_app.uwb;

import android.content.Context;
import android.content.SharedPreferences;

/** Persists the explicit public/LAN relay selection without storing credentials. */
public final class UwbEndpointStore {
    private static final String PREFS = "uwb_endpoint_v1";
    private static final String MODE = "mode";
    private static final String LAN_HOST = "lan_host";
    private static final String LAN_PORT = "lan_port";

    private final SharedPreferences preferences;

    public UwbEndpointStore(Context context) {
        preferences = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    public UwbEndpointConfig load() {
        String modeValue = preferences.getString(MODE, UwbEndpointConfig.Mode.PUBLIC.name());
        try {
            return new UwbEndpointConfig(
                    UwbEndpointConfig.Mode.valueOf(modeValue),
                    preferences.getString(LAN_HOST, ""),
                    preferences.getInt(LAN_PORT, UwbEndpointConfig.DEFAULT_LAN_PORT));
        } catch (IllegalArgumentException error) {
            return UwbEndpointConfig.publicServer();
        }
    }

    public void save(UwbEndpointConfig config) {
        preferences.edit()
                .putString(MODE, config.getMode().name())
                .putString(LAN_HOST, config.getLanHost())
                .putInt(LAN_PORT, config.getLanPort())
                .apply();
    }
}
