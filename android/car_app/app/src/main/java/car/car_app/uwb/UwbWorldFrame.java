package car.car_app.uwb;

import car.car_app.geometry.VehicleGeometry;

/** Converts LinkPG device coordinates into the application's +Y-up field frame. */
public final class UwbWorldFrame {
    private UwbWorldFrame() {}

    /** Server and app share the generated A-C/A-D world frame. */
    public static double yCm(double deviceYCm) {
        return deviceYCm;
    }

    public static double headingDeg(double deviceHeadingDeg) {
        double heading = deviceHeadingDeg;
        while (heading > 180.0) heading -= 360.0;
        while (heading <= -180.0) heading += 360.0;
        return heading;
    }
}
