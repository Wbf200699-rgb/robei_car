package car.car_app.bluetooth;

/** One CRC-protected @G1 chunk containing 128 packed 2-bit map cells. */
public final class OnboardMapChunk {
    public static final int CELL_COUNT = 128;
    public static final int PAYLOAD_LENGTH = 72;

    private final int sequence;
    private final int baseAddress;
    private final byte[] cellClasses;
    private final int crc;

    private OnboardMapChunk(int sequence, int baseAddress, byte[] cellClasses, int crc) {
        this.sequence = sequence;
        this.baseAddress = baseAddress;
        this.cellClasses = cellClasses;
        this.crc = crc;
    }

    public static OnboardMapChunk parse(String payload) {
        if (payload == null || payload.length() != PAYLOAD_LENGTH || !isHex(payload)) return null;
        String crcInput = "G1" + payload.substring(0, 70);
        int expectedCrc;
        try {
            expectedCrc = Integer.parseInt(payload.substring(70, 72), 16);
        } catch (NumberFormatException error) {
            return null;
        }
        if (crc8(crcInput) != expectedCrc) return null;

        try {
            int sequence = Integer.parseInt(payload.substring(0, 2), 16);
            int baseAddress = Integer.parseInt(payload.substring(2, 6), 16);
            if (baseAddress < 0 || baseAddress + CELL_COUNT > 128 * 128) return null;
            byte[] cells = new byte[CELL_COUNT];
            String packed = payload.substring(6, 70);
            for (int nibbleIndex = 0; nibbleIndex < packed.length(); nibbleIndex++) {
                int nibble = Character.digit(packed.charAt(nibbleIndex), 16);
                cells[nibbleIndex * 2] = (byte) (nibble & 0x03);
                cells[nibbleIndex * 2 + 1] = (byte) ((nibble >>> 2) & 0x03);
            }
            return new OnboardMapChunk(sequence, baseAddress, cells, expectedCrc);
        } catch (NumberFormatException error) {
            return null;
        }
    }

    static int crc8(String ascii) {
        int crc = 0;
        for (int index = 0; index < ascii.length(); index++) {
            crc ^= ascii.charAt(index) & 0xFF;
            for (int bit = 0; bit < 8; bit++) {
                crc = (crc & 0x80) != 0 ? ((crc << 1) ^ 0x07) & 0xFF : (crc << 1) & 0xFF;
            }
        }
        return crc;
    }

    private static boolean isHex(String value) {
        for (int index = 0; index < value.length(); index++) {
            if (Character.digit(value.charAt(index), 16) < 0) return false;
        }
        return true;
    }

    public int getSequence() { return sequence; }
    public int getBaseAddress() { return baseAddress; }
    public int getCrc() { return crc; }
    public byte[] getCellClasses() { return cellClasses.clone(); }
}
