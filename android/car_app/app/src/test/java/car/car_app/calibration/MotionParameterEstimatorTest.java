package car.car_app.calibration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MotionParameterEstimatorTest {
    @Test
    public void speedEstimateScalesFromCalibrationSetting() {
        assertEquals(500.0,
                MotionParameterEstimator.estimateSpeedMmPerSecond(0.4, 700, 560),
                0.001);
    }

    @Test
    public void stepEstimateMatchesFpgaStepDuration() {
        assertEquals(250.0,
                MotionParameterEstimator.estimateStepDistanceMm(0.4, 500, 500, 5),
                0.001);
    }

    @Test
    public void missingCalibrationDoesNotInventAnEstimate() {
        assertTrue(Double.isNaN(
                MotionParameterEstimator.estimateSpeedMmPerSecond(0.4, 500, -1)));
    }
}
