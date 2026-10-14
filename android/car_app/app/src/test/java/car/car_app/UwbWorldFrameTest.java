package car.car_app;

import static org.junit.Assert.assertEquals;

import car.car_app.uwb.UwbWorldFrame;

import org.junit.Test;

public class UwbWorldFrameTest {
    @Test
    public void preservesSharedServerAndAppWorldFrame() {
        assertEquals(0.0, UwbWorldFrame.yCm(0.0), 0.001);
        assertEquals(90.0, UwbWorldFrame.yCm(90.0), 0.001);
        assertEquals(-160.0, UwbWorldFrame.headingDeg(-160.0), 0.001);
        assertEquals(170.0, UwbWorldFrame.headingDeg(170.0), 0.001);
        assertEquals(-170.0, UwbWorldFrame.headingDeg(190.0), 0.001);
    }
}
