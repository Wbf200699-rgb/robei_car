package car.car_app.uwb;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class UwbEndpointConfigTest {
    @Test
    public void publicModeUsesBuildEndpoint() {
        UwbEndpointConfig config = UwbEndpointConfig.publicServer();
        assertEquals("wss://example/ws/mobile", config.endpoint("wss://example/ws/mobile"));
    }

    @Test
    public void lanModeBuildsExpectedWebSocketEndpoint() {
        UwbEndpointConfig config = new UwbEndpointConfig(
                UwbEndpointConfig.Mode.LAN, "192.168.31.16", 18765);
        assertEquals("ws://192.168.31.16:18765/ws/mobile", config.endpoint("ignored"));
    }

    @Test
    public void privateAndTailscaleRangesAreAccepted() {
        assertTrue(UwbEndpointConfig.isPrivateIpv4("10.1.2.3"));
        assertTrue(UwbEndpointConfig.isPrivateIpv4("172.20.1.2"));
        assertTrue(UwbEndpointConfig.isPrivateIpv4("192.168.1.9"));
        assertTrue(UwbEndpointConfig.isPrivateIpv4("100.108.120.74"));
        assertFalse(UwbEndpointConfig.isPrivateIpv4("8.8.8.8"));
    }

    @Test
    public void lanModeRejectsPublicOrMalformedAddress() {
        assertThrows(IllegalArgumentException.class, () -> new UwbEndpointConfig(
                UwbEndpointConfig.Mode.LAN, "uwb.example.com", 18765));
        assertThrows(IllegalArgumentException.class, () -> new UwbEndpointConfig(
                UwbEndpointConfig.Mode.LAN, "192.168.1.999", 18765));
    }
}
