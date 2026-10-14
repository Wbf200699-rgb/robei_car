package car.car_app.localization;

import android.content.Context;
import android.content.SharedPreferences;

/** Persists the last anchor layout confirmed by the UWB server. */
public final class AnchorLayoutStore {
    private static final String PREFS = "uwb_anchor_layout_v1";
    private static final String WIDTH_CM = "width_cm";
    private static final String HEIGHT_CM = "height_cm";
    private static final String CORNERS = "corner_anchor_ids";

    private final SharedPreferences preferences;

    public AnchorLayoutStore(Context context) {
        preferences = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    public AnchorLayout load() {
        AnchorLayout fallback = AnchorLayout.defaultLayout();
        String[] parts = preferences.getString(CORNERS, fallback.cornerCsv()).split(",");
        if (parts.length != 4) return fallback;
        int[] corners = new int[4];
        try {
            for (int index = 0; index < 4; index++) corners[index] = Integer.parseInt(parts[index]);
            return new AnchorLayout(
                    preferences.getInt(WIDTH_CM, fallback.getWidthCm()),
                    preferences.getInt(HEIGHT_CM, fallback.getHeightCm()),
                    corners);
        } catch (IllegalArgumentException error) {
            return fallback;
        }
    }

    public void save(AnchorLayout layout) {
        preferences.edit()
                .putInt(WIDTH_CM, layout.getWidthCm())
                .putInt(HEIGHT_CM, layout.getHeightCm())
                .putString(CORNERS, layout.cornerCsv())
                .apply();
    }
}
