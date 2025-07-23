package org.firstinspires.ftc.teamcode.util;

public class ButtonBlock {
    boolean pressed = false;
    RunCommand onTrue, onFalse;
    RunCommand whileTrue, whileFalse;
    public ButtonBlock() {}
    public void update(boolean button) {
        if (button) {
            if (whileTrue != null) whileTrue.Invoke();
            if (!pressed && onTrue != null) {
                onTrue.Invoke();
            }
            pressed = true;
        } else {
            if (whileFalse != null) whileFalse.Invoke();
            if (pressed && onFalse != null) {
                onFalse.Invoke();
            }
            pressed = false;
        }
    }
    public ButtonBlock onTrue(RunCommand runCommand) {
        this.onTrue = runCommand;
        return this;
    }
    public ButtonBlock onFalse(RunCommand runCommand) {
        this.onFalse = runCommand;
        return this;
    }
    public ButtonBlock whileTrue(RunCommand runCommand) {
        this.whileTrue = runCommand;
        return this;
    }
    public ButtonBlock whileFalse(RunCommand runCommand) {
        this.whileFalse = runCommand;
        return this;
    }
}
