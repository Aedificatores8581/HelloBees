package org.firstinspires.ftc.teamcode.util;

public class ButtonBlock {
    boolean pressed = false;
    RunCommand onTrue;
    RunCommand onFalse;
    public ButtonBlock() {}
    public void update(boolean button) {
        if (button) {
            if (!pressed && onTrue != null) {
                onTrue.Invoke();
            }
            pressed = true;
        } else {
            if (pressed && onFalse != null) {
                onFalse.Invoke();
            }
            pressed = false;
        }
    }
//    public static ButtonBlock onTrue(RunCommand runCommand) {
//        ButtonBlock this2 = new ButtonBlock();
//        this2.onTrue = runCommand;
//        return this2;
//    }
    public ButtonBlock onTrue(RunCommand runCommand) {
        this.onTrue = runCommand;
        return this;
    }
}
