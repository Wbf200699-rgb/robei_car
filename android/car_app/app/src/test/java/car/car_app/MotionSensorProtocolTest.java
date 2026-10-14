package car.car_app;

import org.junit.Test;

import java.nio.charset.StandardCharsets;
import java.util.List;

import car.car_app.bluetooth.BleFrame;
import car.car_app.bluetooth.BleFrameParser;
import car.car_app.bluetooth.MotionSensorFrame;
import car.car_app.sensors.ImuAttitudeEstimator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class MotionSensorProtocolTest {
    private static final String PAYLOAD =
            "212349F48FFFE000300000064FFFFFF38"
                    + "F0001000E0000083FF7D0041"
                    + "FF000200E80000FF0040FFC0";

    @Test
    public void parserDecodesSignedFlowAndBothImus() {
        MotionSensorFrame frame = MotionSensorFrame.parse(PAYLOAD);

        assertNotNull(frame);
        assertEquals(0x1234, frame.getSequence());
        assertEquals(0x9F, frame.getStatus());
        assertEquals(0x48, frame.getFlowSqual());
        assertEquals(-2, frame.getFlowDeltaX());
        assertEquals(3, frame.getFlowDeltaY());
        assertEquals(100, frame.getFlowAccumX());
        assertEquals(-200, frame.getFlowAccumY());
        assertEquals(-4096, frame.getImu0().getAccelX());
        assertEquals(4096, frame.getImu0().getAccelY());
        assertEquals(-8192, frame.getImu0().getAccelZ());
        assertEquals(131, frame.getImu0().getGyroX());
        assertEquals(-131, frame.getImu0().getGyroY());
        assertEquals(65, frame.getImu0().getGyroZ());
        assertEquals(-256, frame.getImu1().getAccelX());
        assertEquals(512, frame.getImu1().getAccelY());
        assertEquals(-6144, frame.getImu1().getAccelZ());
        assertEquals(255, frame.getImu1().getGyroX());
        assertEquals(64, frame.getImu1().getGyroY());
        assertEquals(-64, frame.getImu1().getGyroZ());
        assertTrue(frame.isPmw3901Online());
        assertTrue(frame.isBmi0Online());
        assertTrue(frame.isBmi1Online());
        assertTrue(frame.isCalibrationDone());
        assertTrue(frame.isFusedDataReady());
        assertFalse(frame.isImuDegraded());
        assertFalse(frame.isImuDisagreement());
        assertTrue(frame.hasFlowData());
    }

    @Test
    public void parserRejectsWrongVersionLengthAndHex() {
        assertNull(MotionSensorFrame.parse("1" + PAYLOAD.substring(1)));
        assertNull(MotionSensorFrame.parse(PAYLOAD.substring(1)));
        assertNull(MotionSensorFrame.parse(PAYLOAD.substring(0, 20) + "ZZZZ"
                + PAYLOAD.substring(24)));
    }

    @Test
    public void bleParserReassemblesFragmentedMotionLine() {
        BleFrameParser parser = new BleFrameParser();
        String line = "@M" + PAYLOAD;
        assertTrue(parser.append(line.substring(0, 17)
                .getBytes(StandardCharsets.US_ASCII)).isEmpty());
        List<BleFrame> frames = parser.append((line.substring(17) + "\n")
                .getBytes(StandardCharsets.US_ASCII));

        assertEquals(1, frames.size());
        assertEquals(BleFrame.Type.MOTION, frames.get(0).getType());
        assertEquals(PAYLOAD, frames.get(0).getValue());
    }

    @Test
    public void attitudeStartsAtZeroAndIntegratesRelativeYaw() {
        MotionSensorFrame frame = MotionSensorFrame.parse(PAYLOAD);
        assertNotNull(frame);
        ImuAttitudeEstimator estimator = new ImuAttitudeEstimator();
        estimator.update(frame.getImu0(), 100, 1000L);
        assertEquals(0.0, estimator.getRollDeg(), 0.001);
        assertEquals(0.0, estimator.getPitchDeg(), 0.001);

        estimator.update(frame.getImu0(), 120, 1100L);
        assertEquals(frame.getImu0().getGyroZDps() * 0.1,
                estimator.getYawDeg(), 0.001);
    }
}
