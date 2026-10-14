package car.car_app;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import car.car_app.localization.MapWorldRegistration;

import org.junit.Test;

public class MapWorldRegistrationTest {
    @Test
    public void delayedUwbFixPairsWithHistoricalLocalPose() {
        MapWorldRegistration registration = new MapWorldRegistration();
        registration.addLocalPose(1_000L, 3200, 3200, 0f);
        registration.addLocalPose(1_300L, 3500, 3200, 0f);

        assertTrue(registration.addWorldFix(1_000L, 0.75, 0.835, 90f));
        MapWorldRegistration.WorldPose current = registration.currentWorldPose();
        assertNotNull(current);

        // The UWB fix belongs to the older FPGA pose. The fused current pose
        // must retain the later 300 mm local motion, rotated into world +Y.
        assertEquals(0.75, current.xM, 0.001);
        assertEquals(1.135, current.yM, 0.001);
        assertEquals(90.0, Math.toDegrees(current.headingRad), 0.01);
    }

    @Test
    public void rejectsFixOutsidePoseHistoryWindow() {
        MapWorldRegistration registration = new MapWorldRegistration();
        registration.addLocalPose(10_000L, 3200, 3200, 0f);
        assertTrue(!registration.addWorldFix(1_000L, 0.75, 0.835, 0f));
    }

    @Test
    public void rejectsTransformJumpInsteadOfMovingTheMap() {
        MapWorldRegistration registration = new MapWorldRegistration();
        registration.addLocalPose(1_000L, 0, 0, 0f);
        assertTrue(registration.addWorldFix(1_000L, 1.0, 1.0, 0f));

        registration.addLocalPose(1_100L, 0, 0, 0f);
        assertFalse(registration.addWorldFix(1_100L, 1.8, 1.0, 0f));
        MapWorldRegistration.WorldPose pose = registration.currentWorldPose();
        assertNotNull(pose);
        assertEquals(1.0, pose.xM, 0.001);
        assertEquals(1.0, pose.yM, 0.001);
    }

    @Test
    public void uwbTranslationMovesInFixedLocalMapFrame() {
        MapWorldRegistration registration = new MapWorldRegistration();
        registration.addLocalPose(1_000L, 3200, 3200, 0f);
        assertTrue(registration.addWorldFix(1_000L, 1.0, 1.0, 90f));

        double[] local = registration.worldToLocal(1.0, 1.4);
        assertNotNull(local);
        assertEquals(3.6, local[0], 0.001);
        assertEquals(3.2, local[1], 0.001);
    }

    @Test
    public void resetRequiresFreshRegistration() {
        MapWorldRegistration registration = new MapWorldRegistration();
        registration.addLocalPose(1_000L, 3200, 3200, 0f);
        assertTrue(registration.addWorldFix(1_000L, 1.0, 1.0, 0f));
        registration.reset();
        assertFalse(registration.isRegistered());
        assertTrue(registration.worldToLocal(1.0, 1.0) == null);
    }
}
