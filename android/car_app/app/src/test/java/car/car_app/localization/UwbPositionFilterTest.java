package car.car_app.localization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class UwbPositionFilterTest {
    @Test
    public void normalTravelIsNotShortenedByASecondEma() {
        UwbPositionFilter filter = new UwbPositionFilter();
        assertTrue(filter.update(0.0, 0.0, 1_000));
        assertTrue(filter.update(0.20, 0.0, 1_100));
        assertTrue(filter.update(0.42, 0.0, 1_200));
        assertEquals(0.42, filter.getXM(), 1e-9);
    }

    @Test
    public void isolatedLargeSpikeIsRejected() {
        UwbPositionFilter filter = new UwbPositionFilter();
        filter.update(1.0, 1.0, 1_000);
        assertFalse(filter.update(5.0, 5.0, 1_100));
        assertTrue(filter.update(1.1, 1.0, 1_200));
        assertEquals(1.1, filter.getXM(), 1e-9);
        assertEquals(1.0, filter.getYM(), 1e-9);
    }

    @Test
    public void persistentRelocationReacquiresInsteadOfLockingForever() {
        UwbPositionFilter filter = new UwbPositionFilter();
        filter.update(0.0, 0.0, 1_000);
        assertFalse(filter.update(2.0, 2.0, 1_100));
        assertTrue(filter.update(2.05, 2.02, 1_200));
        assertEquals(2.05, filter.getXM(), 1e-9);
        assertEquals(2.02, filter.getYM(), 1e-9);
    }

    @Test
    public void serverClockRestartStartsANewSession() {
        UwbPositionFilter filter = new UwbPositionFilter();
        filter.update(4.0, 3.0, 10_000);
        assertTrue(filter.update(0.5, 0.6, 100));
        assertEquals(0.5, filter.getXM(), 1e-9);
        assertEquals(0.6, filter.getYM(), 1e-9);
    }
}
