package car.car_app.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DriveModeTransitionTest {
    @Test
    public void ignoresOldModeUntilExpectedStatusArrives() {
        DriveModeTransition transition = new DriveModeTransition(1800L);
        transition.request(0, 1000L);

        assertEquals(DriveModeTransition.StatusDecision.STALE_MISMATCH,
                transition.onStatus(1, 1200L));
        assertTrue(transition.isPending());
        assertEquals(DriveModeTransition.StatusDecision.CONFIRMED,
                transition.onStatus(0, 1400L));
        assertFalse(transition.isPending());
    }

    @Test
    public void acceptsHardwareModeAfterConfirmationTimeout() {
        DriveModeTransition transition = new DriveModeTransition(1800L);
        transition.request(0, 1000L);

        assertEquals(DriveModeTransition.StatusDecision.TIMED_OUT,
                transition.onStatus(1, 2800L));
        assertFalse(transition.isPending());
    }

    @Test
    public void acceptsNormalStatusWhenNoSwitchIsPending() {
        DriveModeTransition transition = new DriveModeTransition(1800L);
        assertEquals(DriveModeTransition.StatusDecision.ACCEPTED,
                transition.onStatus(1, 1000L));
    }
}
