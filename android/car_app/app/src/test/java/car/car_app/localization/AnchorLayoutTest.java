package car.car_app.localization;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public final class AnchorLayoutTest {
    @Test
    public void mapsSwappedLabelsToTheirPhysicalCorners() {
        AnchorLayout layout = new AnchorLayout(250, 180, new int[]{2, 0, 3, 1});
        int[][] coordinates = layout.coordinatesByAnchorCm();
        assertArrayEquals(new int[]{250, 0, 0}, coordinates[0]);
        assertArrayEquals(new int[]{0, 180, 0}, coordinates[1]);
        assertArrayEquals(new int[]{0, 0, 0}, coordinates[2]);
        assertArrayEquals(new int[]{250, 180, 0}, coordinates[3]);
    }

    @Test
    public void exposesCanonicalMapGeometryIndependentOfLabels() {
        AnchorLayout layout = new AnchorLayout(250, 180, new int[]{2, 0, 3, 1});
        double[][] anchors = layout.canonicalMapAnchorsM();
        assertEquals(2.5, anchors[2][0], 0.0001);
        assertEquals(0.0, anchors[2][1], 0.0001);
        assertEquals(1.8, anchors[3][1], 0.0001);
        assertArrayEquals(new String[]{"C", "D", "A", "B"},
                layout.canonicalMapAnchorNames());
        assertTrue(layout.matchesMapAnchors(anchors, 0.0001));
    }

    @Test
    public void equalityIncludesDimensionsAndPhysicalCornerOrder() {
        AnchorLayout layout = new AnchorLayout(600, 500, new int[]{3, 2, 1, 0});
        assertEquals(layout, new AnchorLayout(600, 500, new int[]{3, 2, 1, 0}));
        assertTrue(!layout.equals(new AnchorLayout(500, 600, new int[]{3, 2, 1, 0})));
        assertTrue(!layout.equals(new AnchorLayout(600, 500, new int[]{0, 1, 2, 3})));
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsDuplicateLabels() {
        new AnchorLayout(150, 170, new int[]{0, 1, 1, 3});
    }
}
