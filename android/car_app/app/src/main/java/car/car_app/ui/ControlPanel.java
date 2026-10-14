package car.car_app.ui;

import android.view.View;

import com.google.android.material.button.MaterialButton;

import car.car_app.bluetooth.CarCommand;

public class ControlPanel {
    public interface CommandListener {
        void onCommand(CarCommand command);
    }

    private final CommandListener commandListener;

    public ControlPanel(CommandListener commandListener) {
        this.commandListener = commandListener;
    }

    public void bindDirectionalButton(MaterialButton button, CarCommand command) {
        button.setOnClickListener(view -> commandListener.onCommand(command));
    }

    public void bindClickButton(View button, CarCommand command) {
        button.setOnClickListener(view -> commandListener.onCommand(command));
    }
}
