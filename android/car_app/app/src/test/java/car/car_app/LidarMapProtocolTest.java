package car.car_app;

import org.junit.Test;

import java.nio.charset.StandardCharsets;
import java.util.List;

import car.car_app.bluetooth.BleFrame;
import car.car_app.bluetooth.BleFrameParser;
import car.car_app.bluetooth.LidarMapAssembler;
import car.car_app.bluetooth.LidarMapChunk;
import car.car_app.bluetooth.LidarMapScan;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class LidarMapProtocolTest {
    @Test
    public void chunkDecodesQualityDistanceAndInvalidPoint() throws Exception {
        int[] words = words(0x401);
        words[0] = 0x84B;
        words[1] = 0xC7D;
        words[2] = 0xFFF;
        LidarMapChunk chunk = LidarMapChunk.parse(buildLine(0x2A, 0x1234, 0, words));

        assertEquals(0x2A, chunk.getSequence());
        assertEquals(0x1234, chunk.getTick10ms());
        assertEquals(0, chunk.getPart());
        assertEquals(0x04B * 16, chunk.getPoints().get(0).getDistanceMm());
        assertEquals(2, chunk.getPoints().get(0).getQualityClass());
        assertEquals(2.5f, chunk.getPoints().get(0).getAngleDegrees(), 0.01f);
        assertEquals(3, chunk.getPoints().get(1).getQualityClass());
        assertTrue(!chunk.getPoints().get(2).isValid());
    }

    @Test
    public void corruptCrcIsRejectedWithCrcReason() {
        String line = buildLine(1, 2, 0, words(0xFFF));
        String corrupt = line.substring(0, 47) + (line.endsWith("00") ? "01" : "00");

        LidarMapChunk.ParseException exception = assertThrows(
                LidarMapChunk.ParseException.class,
                () -> LidarMapChunk.parse(corrupt)
        );
        assertEquals(LidarMapChunk.ErrorReason.CRC, exception.getReason());
    }

    @Test
    public void bleParserReassemblesFragmentedMapLine() {
        String line = buildLine(3, 4, 2, words(0x455));
        BleFrameParser parser = new BleFrameParser();

        assertTrue(parser.append(line.substring(0, 7).getBytes(StandardCharsets.US_ASCII)).isEmpty());
        assertTrue(parser.append(line.substring(7, 31).getBytes(StandardCharsets.US_ASCII)).isEmpty());
        List<BleFrame> frames = parser.append(
                (line.substring(31) + "\n").getBytes(StandardCharsets.US_ASCII)
        );

        assertEquals(1, frames.size());
        assertEquals(BleFrame.Type.LIDAR_MAP, frames.get(0).getType());
        assertEquals(line, frames.get(0).getRawText());
    }

    @Test
    public void assemblerRequiresAllMatchingPartsAndRestoresBinOrder() throws Exception {
        LidarMapAssembler assembler = new LidarMapAssembler();
        LidarMapScan scan = null;
        int[] order = {0, 2, 1, 5, 3, 4};
        for (int index = 0; index < order.length; index++) {
            int part = order[index];
            scan = assembler.add(
                    LidarMapChunk.parse(buildLine(7, 100, part, words(0x400 + part + 1))),
                    1000L + index * 10L
            );
            if (index < order.length - 1) {
                assertNull(scan);
            }
        }

        assertNotNull(scan);
        assertEquals(72, scan.getPoints().size());
        assertEquals(0, scan.getPoints().get(0).getBinIndex());
        assertEquals(71, scan.getPoints().get(71).getBinIndex());
        assertEquals(1, assembler.getCompletedScanCount());
        assertEquals(0, assembler.getPendingPartCount());
    }

    @Test
    public void assemblerCountsMismatchDuplicateAndTimeout() throws Exception {
        LidarMapAssembler assembler = new LidarMapAssembler();
        LidarMapChunk first = LidarMapChunk.parse(buildLine(1, 10, 0, words(0x401)));
        assembler.add(first, 0L);
        assembler.add(first, 10L);
        assembler.add(LidarMapChunk.parse(buildLine(2, 20, 1, words(0x402))), 20L);
        assembler.add(LidarMapChunk.parse(buildLine(2, 20, 0, words(0x402))), 30L);
        assembler.expire(30L + LidarMapAssembler.PARTIAL_TIMEOUT_MS + 1L);

        assertEquals(1, assembler.getDuplicatePartCount());
        assertEquals(1, assembler.getRejectedPartCount());
        assertEquals(2, assembler.getIncompleteDropCount());
        assertEquals(0, assembler.getPendingPartCount());
    }

    private static int[] words(int value) {
        int[] result = new int[LidarMapChunk.POINTS_PER_PART];
        for (int index = 0; index < result.length; index++) {
            result[index] = value;
        }
        return result;
    }

    private static String buildLine(int sequence, int tick, int part, int[] words) {
        StringBuilder body = new StringBuilder(String.format(
                "R1%02X%04X%X6",
                sequence & 0xFF,
                tick & 0xFFFF,
                part
        ));
        for (int word : words) {
            body.append(String.format("%03X", word & 0xFFF));
        }
        int crc = LidarMapChunk.crc8(body.toString().getBytes(StandardCharsets.US_ASCII));
        return "@" + body + String.format("%02X", crc);
    }
}
