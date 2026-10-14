package car.car_app;

import org.junit.Test;

import car.car_app.bluetooth.RadarFrame;
import car.car_app.bluetooth.StatusFrame;
import car.car_app.bluetooth.Vl53Frame;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class ExampleUnitTest {
    @Test
    public void statusFrameParsesRouteAndParameterFields() {
        StatusFrame frame = StatusFrame.parse("032010080FF807F123456789ABCDEF0001C12C11601");

        assertNotNull(frame);
        assertEquals(3, frame.getState());
        assertEquals(2, frame.getMenu());
        assertEquals(1, frame.getActiveRouteSteps());
        assertEquals(0, frame.getCompletedRouteSteps());
        assertEquals(128, frame.getNowX());
        assertEquals(255, frame.getNowY());
        assertEquals(128, frame.getTargetX());
        assertEquals(127, frame.getTargetY());
        assertEquals("123456789ABCDEF0", frame.getRouteInfo());
        assertEquals(28, frame.getStepLength());
        assertEquals(300, frame.getSpeed());
        assertEquals(1, frame.getSettingMode());
        assertEquals(1, frame.getSelectedParameter());
        assertEquals(6, frame.getRadarOnline());
        assertEquals(0, frame.getManualMode());
        assertEquals(1, frame.getLidarEnabled());
    }

    @Test
    public void statusFrameParsesBoardCalibrationExtension() {
        StringBuilder payload = new StringBuilder();
        for (int i = 0; i < 83; i++) payload.append('0');
        payload.append("A52FEB00C01F4");

        StatusFrame frame = StatusFrame.parse(payload.toString());

        assertNotNull(frame);
        assertEquals(0xA, frame.getLocalCalibrationState());
        assertEquals(0x52, frame.getLocalCalibrationFlags());
        assertEquals(-21, frame.getLearnedYawTrim());
        assertEquals(12, frame.getYawCorrection());
        assertEquals(500, frame.getCalibrationGyroPeak());
        assertTrue(frame.isLocalCalibrationResultValid());
        assertFalse(frame.isLocalCalibrationBusy());
        assertTrue(frame.isGyroPolarityValid());
        assertTrue(frame.isCalibrationSensorsReady());
    }

    @Test
    public void radarFrameRejectsTruncatedPacket() {
        RadarFrame frame = RadarFrame.parse("1001C2012C");

        assertNull(frame);
    }

    @Test
    public void radarFrameParsesEightSectorDistancesWhenPresent() {
        RadarFrame frame = RadarFrame.parse("1001C2012C01000200030004000500060007000800");

        assertNotNull(frame);
        assertEquals(1, frame.getRadarOnline());
        assertEquals(0, frame.getObstacleTriggered());
        assertEquals(450, frame.getSafeDistanceMm());
        assertEquals(300, frame.getFrontDistance());
        assertEquals(256, frame.getSectorDistancesMm()[0]);
        assertEquals(512, frame.getSectorDistancesMm()[1]);
        assertEquals(2048, frame.getSectorDistancesMm()[7]);
    }

    @Test
    public void vl53FrameParsesIndexedDistancesAndMasks() {
        Vl53Frame frame = Vl53Frame.parse("00951301230456FFFFFFFF0ABC002A");

        assertNotNull(frame);
        assertEquals(0, frame.getSequence());
        assertEquals(true, frame.isInitializationDone());
        assertEquals(0x15, frame.getOnlineMask());
        assertEquals(0x13, frame.getValidMask());
        assertEquals(3, frame.getOnlineCount());
        assertEquals(0x0123, frame.getDistanceMm(0));
        assertEquals(0x0456, frame.getDistanceMm(1));
        assertEquals(Vl53Frame.INVALID_DISTANCE, frame.getDistanceMm(2));
        assertEquals(Vl53Frame.INVALID_DISTANCE, frame.getDistanceMm(3));
        assertEquals(0x0ABC, frame.getDistanceMm(4));
        assertEquals(0xFFFF, frame.getRawDistanceMm(2));
        assertEquals(0xFFFF, frame.getRawDistanceMm(3));
        assertEquals(0x002A, frame.getErrorCount());

        String display = MainActivity.formatVl53SensorLines(frame);
        assertTrue(display.split("\\n")[1].contains("离线"));
        assertTrue(display.split("\\n")[2].contains("测距无效"));
    }

    @Test
    public void vl53FrameRejectsTruncatedPacket() {
        assertNull(Vl53Frame.parse("0095130123"));
    }

    @Test
    public void vl53DisplayListsAllFiveIndexedDistances() {
        Vl53Frame frame = Vl53Frame.parse("009F1F0123045607890ABC0DEF002A");

        assertNotNull(frame);
        String display = MainActivity.formatVl53SensorLines(frame);
        String[] lines = display.split("\\n");
        assertEquals(5, lines.length);
        assertTrue(lines[0].contains("0号  0x30"));
        assertTrue(lines[0].contains("291 mm"));
        assertTrue(lines[1].contains("1号  0x31"));
        assertTrue(lines[1].contains("1110 mm"));
        assertTrue(lines[2].contains("2号  0x32"));
        assertTrue(lines[2].contains("1929 mm"));
        assertTrue(lines[3].contains("3号  0x33"));
        assertTrue(lines[3].contains("2748 mm"));
        assertTrue(lines[4].contains("4号  0x34"));
        assertTrue(lines[4].contains("3567 mm"));
        assertFalse(display.contains("前"));
        assertFalse(display.contains("后"));
    }

    @Test
    public void vl53DisplayExplainsZeroAndOverRangeRawSamples() {
        Vl53Frame frame = Vl53Frame.parse("009F18000013880123045607890000");

        assertNotNull(frame);
        assertEquals(Vl53Frame.INVALID_DISTANCE, frame.getDistanceMm(0));
        assertEquals(0, frame.getRawDistanceMm(0));
        assertEquals(Vl53Frame.INVALID_DISTANCE, frame.getDistanceMm(1));
        assertEquals(5000, frame.getRawDistanceMm(1));

        String[] lines = MainActivity.formatVl53SensorLines(frame).split("\\n");
        assertTrue(lines[0].contains("0 mm"));
        assertTrue(lines[0].contains("原始值0"));
        assertTrue(lines[1].contains("5000 mm"));
        assertTrue(lines[1].contains("超4m范围"));
    }
}
