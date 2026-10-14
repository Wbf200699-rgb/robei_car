package car.car_app;

import org.junit.Test;

import car.car_app.uwb.UwbVehiclePose;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class UwbVehiclePoseTest {
    @Test
    public void singleTagPoseProvidesPositionWithoutClaimingReliableHeading() {
        UwbVehiclePose pose = new UwbVehiclePose(
                "partial", 86.0, 89.0, null, 1, 0.0, 1234L);

        assertTrue(pose.hasPosition());
        assertFalse(pose.hasCoordinates());
        assertFalse(pose.isNavigationReliable());
    }

    @Test
    public void fullPoseStillProvidesReliableNavigationCoordinates() {
        UwbVehiclePose pose = new UwbVehiclePose(
                "full", 86.0, 89.0, 45.0, 4, 20.0, 1234L);

        assertTrue(pose.hasPosition());
        assertTrue(pose.hasCoordinates());
        assertTrue(pose.isNavigationReliable());
    }
}
