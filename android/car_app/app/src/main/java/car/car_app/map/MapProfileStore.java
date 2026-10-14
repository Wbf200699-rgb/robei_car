package car.car_app.map;

import android.content.Context;
import android.content.SharedPreferences;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Stores named phone maps and their anchor coordinate frames independently. */
public final class MapProfileStore {
    private static final String PREFS = "phone_map_profiles_v1";
    private static final String INDEX = "profile_index";
    private static final String SNAPSHOT_PREFIX = "snapshot_";

    public static final class Profile {
        public final String id;
        public final String name;
        public final long updatedAtMs;
        public final double[][] anchors;
        public final String snapshot;

        Profile(String id, String name, long updatedAtMs, double[][] anchors, String snapshot) {
            this.id = id;
            this.name = name;
            this.updatedAtMs = updatedAtMs;
            this.anchors = copyAnchors(anchors);
            this.snapshot = snapshot;
        }
    }

    private final SharedPreferences preferences;

    public MapProfileStore(Context context) {
        preferences = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    public Profile save(String existingId, String name, double[][] anchors, String snapshot) {
        String id = existingId == null || existingId.isEmpty()
                ? Long.toHexString(System.currentTimeMillis()) : existingId;
        long now = System.currentTimeMillis();
        List<Profile> profiles = list();
        profiles.removeIf(profile -> profile.id.equals(id));
        profiles.add(new Profile(id, sanitizeName(name), now, anchors, snapshot));
        writeIndex(profiles);
        preferences.edit().putString(SNAPSHOT_PREFIX + id, snapshot).apply();
        return new Profile(id, sanitizeName(name), now, anchors, snapshot);
    }

    public List<Profile> list() {
        ArrayList<Profile> result = new ArrayList<>();
        try {
            JSONArray array = new JSONArray(preferences.getString(INDEX, "[]"));
            for (int index = 0; index < array.length(); index++) {
                JSONObject item = array.getJSONObject(index);
                String id = item.getString("id");
                double[][] anchors = new double[4][2];
                JSONArray anchorArray = item.getJSONArray("anchors");
                if (anchorArray.length() != 4) continue;
                for (int anchor = 0; anchor < 4; anchor++) {
                    JSONArray coordinate = anchorArray.getJSONArray(anchor);
                    anchors[anchor][0] = coordinate.getDouble(0);
                    anchors[anchor][1] = coordinate.getDouble(1);
                }
                String snapshot = preferences.getString(SNAPSHOT_PREFIX + id, null);
                if (snapshot == null) continue;
                result.add(new Profile(id, item.getString("name"),
                        item.getLong("updated_at_ms"), anchors, snapshot));
            }
        } catch (JSONException ignored) {
            return new ArrayList<>();
        }
        result.sort(Comparator.comparingLong((Profile profile) -> profile.updatedAtMs).reversed());
        return result;
    }

    public void delete(String id) {
        List<Profile> profiles = list();
        profiles.removeIf(profile -> profile.id.equals(id));
        writeIndex(profiles);
        preferences.edit().remove(SNAPSHOT_PREFIX + id).apply();
    }

    private void writeIndex(List<Profile> profiles) {
        JSONArray array = new JSONArray();
        try {
            for (Profile profile : profiles) {
                JSONObject item = new JSONObject();
                item.put("id", profile.id);
                item.put("name", profile.name);
                item.put("updated_at_ms", profile.updatedAtMs);
                JSONArray anchors = new JSONArray();
                for (double[] anchor : profile.anchors) {
                    anchors.put(new JSONArray().put(anchor[0]).put(anchor[1]));
                }
                item.put("anchors", anchors);
                array.put(item);
            }
        } catch (JSONException error) {
            throw new IllegalStateException("map profile index serialization failed", error);
        }
        preferences.edit().putString(INDEX, array.toString()).apply();
    }

    private static String sanitizeName(String name) {
        String value = name == null ? "" : name.trim();
        return value.isEmpty() ? "未命名地图" : value.substring(0, Math.min(40, value.length()));
    }

    private static double[][] copyAnchors(double[][] source) {
        if (source == null || source.length != 4) throw new IllegalArgumentException("four anchors required");
        double[][] result = new double[4][2];
        for (int index = 0; index < 4; index++) {
            result[index][0] = source[index][0];
            result[index][1] = source[index][1];
        }
        return result;
    }
}
