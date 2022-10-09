package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CervoButton extends GamepadButtonMix{

    private Gamepad m_gamepad;

    public CervoButton() {
        super();
    }

    public CervoButton(@NonNull Gamepad gamepad) {
        this();
        m_gamepad = gamepad;
    }

    @Override
    public boolean get() {
        boolean res = true;
//        boolean button = m_gamepad.setLedColor();
//        res = res && m_gamepad

        return res;
    }

}
