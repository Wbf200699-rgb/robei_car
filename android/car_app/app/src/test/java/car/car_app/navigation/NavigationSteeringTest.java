package car.car_app.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import car.car_app.geometry.VehicleGeometry;

public class NavigationSteeringTest {
    @Test
    public void positiveHeadingErrorCommandsVehicleRightTurn() {
        assertEquals('R', NavigationSteering.commandForHeadingError(90f));
        assertEquals(90f, NavigationSteering.signedTurnDegrees('R', 90f), 0f);
    }

    @Test
    public void negativeHeadingErrorCommandsVehicleLeftTurn() {
        assertEquals('A', NavigationSteering.commandForHeadingError(-90f));
        assertEquals(-90f, NavigationSteering.signedTurnDegrees('A', 90f), 0f);
    }

    @Test
    public void selectedTurnAlwaysReducesWrappedHeadingError() {
        float[] errors = {145f, 45f, -45f, -145f, 179f, -179f};
        for (float error : errors) {
            char command = NavigationSteering.commandForHeadingError(error);
            float delta = NavigationSteering.signedTurnDegrees(command, 10f);
            float after = NavigationSteering.normalizeHeadingError(error - delta);
            assertEquals(Math.abs(error) - 10f, Math.abs(after), 0.001f);
        }
    }

    @Test
    public void clockwiseBoardIncreaseProducesClockwiseWorldIncreaseAcrossWrap() {
        assertEquals(105f, NavigationSteering.rebaseBoardHeading(90f, 330f, 345f), 0.001f);
        assertEquals(115f, NavigationSteering.rebaseBoardHeading(90f, 330f, 355f), 0.001f);
        assertEquals(125f, NavigationSteering.rebaseBoardHeading(90f, 330f, 5f), 0.001f);
    }

    @Test
    public void clockwiseVehicleHeadingIsMirroredOnlyForCanvasRotation() {
        assertEquals(270f, NavigationSteering.toCanvasHeadingDegrees(90f), 0.001f);
        assertEquals(90f, NavigationSteering.toCanvasHeadingDegrees(270f), 0.001f);
        assertEquals(0f, NavigationSteering.toCanvasHeadingDegrees(360f), 0.001f);
    }

    @Test
    public void anchorDirectionsRoundTripBetweenVehicleAndWorldConventions() {
        // Default field center is (0.75, 0.835). Anchor row order is A/B/C/D.
        double[][] anchors = {
                {0.0, 0.0}, {1.5, 1.67}, {1.5, 0.0}, {0.0, 1.67}
        };
        double centerX = 0.75;
        double centerY = 0.835;
        for (double[] anchor : anchors) {
            double dx = anchor[0] - centerX;
            double dy = anchor[1] - centerY;
            float vehicleHeading = NavigationSteering.vehicleHeadingForWorldVector(dx, dy);
            double worldHeading = Math.toRadians(
                    NavigationSteering.toMathematicalHeadingDegrees(vehicleHeading));
            double length = Math.hypot(dx, dy);
            assertEquals(dx / length, Math.cos(worldHeading), 0.0001);
            assertEquals(dy / length, Math.sin(worldHeading), 0.0001);
        }
    }

    @Test
    public void selectingUpperRightBDoesNotMirrorToLowerRightC() {
        float towardB = NavigationSteering.vehicleHeadingForWorldVector(0.75, 0.835);
        float drawnWorldHeading = NavigationSteering.toCanvasHeadingDegrees(towardB);
        assertEquals((float) Math.toDegrees(Math.atan2(0.835, 0.75)),
                drawnWorldHeading, 0.001f);
        // The old bug stored the positive atan2 angle as a clockwise heading,
        // then drawing mirrored it into the lower-right C quadrant.
        assertTrue(drawnWorldHeading > 0f && drawnWorldHeading < 90f);
    }

    @Test
    public void tagsOneAndTwoFormTheNoseThatPointsTowardSelectedB() {
        double dxToB = 0.75;
        double dyToB = 0.835;
        float vehicleHeading = NavigationSteering.vehicleHeadingForWorldVector(dxToB, dyToB);
        double worldHeading = Math.toRadians(
                NavigationSteering.toMathematicalHeadingDegrees(vehicleHeading));

        double noseLocalX = (VehicleGeometry.TAG_POSITIONS_M[1][0]
                + VehicleGeometry.TAG_POSITIONS_M[2][0]) * 0.5;
        double noseLocalY = (VehicleGeometry.TAG_POSITIONS_M[1][1]
                + VehicleGeometry.TAG_POSITIONS_M[2][1]) * 0.5;
        double noseWorldX = Math.cos(worldHeading) * noseLocalX
                - Math.sin(worldHeading) * noseLocalY;
        double noseWorldY = Math.sin(worldHeading) * noseLocalX
                + Math.cos(worldHeading) * noseLocalY;

        assertTrue(noseWorldX * dxToB + noseWorldY * dyToB > 0.0);
        assertEquals(0.0, noseWorldX * dyToB - noseWorldY * dxToB, 0.0001);
    }

    @Test
    public void pathFollowerUsesClockwiseTargetInsteadOfOscillatingMirror() {
        float targetTowardUpperRight =
                NavigationSteering.vehicleHeadingForWorldVector(2.0, 2.0);
        assertEquals(315f, targetTowardUpperRight, 0.001f);
        assertEquals(0f, NavigationSteering.normalizeHeadingError(
                targetTowardUpperRight - 315f), 0.001f);
        assertEquals('A', NavigationSteering.commandForHeadingError(
                NavigationSteering.normalizeHeadingError(
                        targetTowardUpperRight - 0f)));
    }

    @Test
    public void turnControllerUsesBoundedClosedLoopPulses() {
        assertEquals(180, NavigationSteering.boundedTurnPulseDurationMs(153f, 0.186f));
        assertEquals(118, NavigationSteering.boundedTurnPulseDurationMs(55f, 0.186f));
        assertEquals(80, NavigationSteering.boundedTurnPulseDurationMs(13f, 0.186f));
    }
}
