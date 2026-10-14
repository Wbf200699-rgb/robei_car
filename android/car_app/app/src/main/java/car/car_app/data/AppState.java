package car.car_app.data;

public class AppState {
    private boolean bluetoothConnected;
    private String statusText = "Bluetooth: disconnected";

    public boolean isBluetoothConnected() {
        return bluetoothConnected;
    }

    public void setBluetoothConnected(boolean bluetoothConnected) {
        this.bluetoothConnected = bluetoothConnected;
    }

    public String getStatusText() {
        return statusText;
    }

    public void setStatusText(String statusText) {
        this.statusText = statusText;
    }
}
