package car.car_app.bluetooth;

import org.junit.Test;

import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class OnboardMapProtocolTest {
    private static final String G0_PAYLOAD =
            "00D52A0C800C4E1551234001100220003A527F0020BA1801020304";

    @Test
    public void statusFrameParsesFixedMapAndFlashFields() {
        OnboardMapStatus status = OnboardMapStatus.parse(G0_PAYLOAD);

        assertNotNull(status);
        assertEquals(0, status.getSequence());
        assertEquals(0xD5, status.getFlags());
        assertEquals(0x2A, status.getEpoch());
        assertEquals(3200, status.getPoseXmm());
        assertEquals(3150, status.getPoseYmm());
        assertEquals(0x155, status.getHeading1024());
        assertEquals(0x1234, status.getScanCount());
        assertEquals(17, status.getOccupiedCount());
        assertEquals(34, status.getFreeCount());
        assertEquals(3, status.getDroppedCount());
        assertEquals(0xA5, status.getPoseFlags());
        assertEquals(2, status.getFlashOperation());
        assertEquals(0x7F, status.getFlashProgress());
        assertEquals(0, status.getFlashError());
        assertEquals(0x20BA18, status.getFlashJedecId());
        assertEquals(0x01020304L, status.getSavedSequence());
        assertTrue(status.isMapActive());
        assertFalse(status.isMapPaused());
        assertTrue(status.isMapValid());
        assertTrue(status.isFlashReady());
        assertTrue(status.isLidarOnline());
        assertTrue(status.areMotionSensorsReady());
    }

    @Test
    public void chunkFrameChecksCrcAndDecodesLowBitsFirst() {
        String data = "9" + repeat('0', 63);
        String protectedAscii = "G1A70180" + data;
        String payload = "A70180" + data
                + String.format(Locale.ROOT, "%02X", OnboardMapChunk.crc8(protectedAscii));

        OnboardMapChunk chunk = OnboardMapChunk.parse(payload);

        assertNotNull(chunk);
        assertEquals(0xA7, chunk.getSequence());
        assertEquals(0x0180, chunk.getBaseAddress());
        assertEquals(1, chunk.getCellClasses()[0]);
        assertEquals(2, chunk.getCellClasses()[1]);
        assertEquals(0, chunk.getCellClasses()[2]);
        assertNull(OnboardMapChunk.parse(payload.substring(0, 70) + "00"));
    }

    @Test
    public void streamingParserKeepsG0AndG1FramesSeparate() {
        String data = repeat('0', 64);
        String protectedAscii = "G1010000" + data;
        String g1Payload = "010000" + data
                + String.format(Locale.ROOT, "%02X", OnboardMapChunk.crc8(protectedAscii));
        BleFrameParser parser = new BleFrameParser();

        List<BleFrame> first = parser.append(
                ("@PONG\n@G0" + G0_PAYLOAD.substring(0, 12))
                        .getBytes(StandardCharsets.US_ASCII));
        List<BleFrame> second = parser.append(
                (G0_PAYLOAD.substring(12) + "\n@G1" + g1Payload + "\n")
                        .getBytes(StandardCharsets.US_ASCII));

        assertEquals(1, first.size());
        assertEquals(BleFrame.Type.PONG, first.get(0).getType());
        assertEquals(2, second.size());
        assertEquals(BleFrame.Type.ONBOARD_MAP_STATUS, second.get(0).getType());
        assertEquals(BleFrame.Type.ONBOARD_MAP_CHUNK, second.get(1).getType());
        assertNotNull(OnboardMapStatus.parse(second.get(0).getValue()));
        assertNotNull(OnboardMapChunk.parse(second.get(1).getValue()));
    }

    private static String repeat(char value, int count) {
        StringBuilder builder = new StringBuilder(count);
        for (int index = 0; index < count; index++) builder.append(value);
        return builder.toString();
    }
}
