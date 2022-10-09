package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

public class GamepadButtonMix {
    private JustOnce whenPressed;
    private JustOnce whileHeld;
    private JustOnce whileActiveOnce;
    private JustOnce whenReleased;
    private JustOnce toggleWhenActive;
    private JustOnce cancelWhenActive;

    private boolean m_isActive = false;

    public GamepadButtonMix(){
        whenPressed = new JustOnce();
        whileHeld = new JustOnce();
        whileActiveOnce = new JustOnce();
        whenReleased = new JustOnce();
        toggleWhenActive = new JustOnce();
        cancelWhenActive = new JustOnce();
    }

    public GamepadButtonMix(boolean m_isActive){
        this();
        this.m_isActive = m_isActive;
    }

    public boolean get(){return m_isActive;}

    public GamepadButtonMix whenPressed(final boolean get, final Command command, boolean interruptible) {
        whenPressed.JustOneTime(get, new Runnable() {
            private boolean m_pressedLast = get;

            @Override
            public void run() {
                boolean pressed = get;

                if (m_pressedLast && pressed) {
                    command.schedule(interruptible);
                }

                m_pressedLast = pressed;
            }
        });

        return this;
    }

    public GamepadButtonMix whenPressed(final boolean get, final Command command) {
        return whenPressed(get, command, true);
    }

    public GamepadButtonMix whenPressed(final boolean get, Runnable toRun) {
        return whenPressed(get, new InstantCommand(toRun));
    }

    public GamepadButtonMix whileHeld(final boolean get, Command command, boolean interruptible) {
        whileHeld.JustOneTime(get, new Runnable() {
        //whileHeld.loopRunning(new Runnable() {
            private boolean m_pressedLast = get;

            @Override
            public void run() {
                boolean pressed = get;

                if (pressed) {
                    command.schedule(interruptible);
                } else if (m_pressedLast) {
                    command.cancel();
                }

                m_pressedLast = pressed;
            }
        });

        return this;
    }
    public GamepadButtonMix whileHeld(final boolean get, Command command) {
        return whileHeld(get, command, true);
    }

    public GamepadButtonMix whileHeld(final boolean get, final Runnable toRun) {
        return whileHeld(get, new InstantCommand(toRun));
    }

    public GamepadButtonMix whileActiveOnce(final boolean get, Command command, boolean interruptible) {
        whileActiveOnce.JustOneTime(get, new Runnable() {
            private boolean m_pressedLast = get;

            @Override
            public void run() {
                boolean pressed = get;

                if (!m_pressedLast && pressed) {
                    command.schedule(interruptible);
                } else if (m_pressedLast && !pressed) {
                    command.cancel();
                }

                m_pressedLast = pressed;
            }
        });

        return this;
    }

    public GamepadButtonMix whileActiveOnce(final boolean get, final Command command) {
        return whileActiveOnce(get, command, true);
    }

    public GamepadButtonMix whenReleased(final boolean get, final Command command, boolean interruptible) {
        whenReleased.JustOneTime(!get, new Runnable() {
            private boolean m_pressedLast = !get;

            @Override
            public void run() {
                boolean pressed = get;

                if (m_pressedLast && !pressed) {
                    command.schedule(interruptible);
                }

                m_pressedLast = pressed;
            }
        });

        return this;
    }

    public GamepadButtonMix whenReleased(final boolean get, final Command command) {
        return whenReleased(get, command, true);
    }

    public GamepadButtonMix whenReleased(final boolean get, final Runnable toRun) {
        return whenReleased(get, new InstantCommand(toRun));
    }

    public GamepadButtonMix toggleWhenActive(final boolean get, final Command command, boolean interruptible) {
        toggleWhenActive.JustOneTime(get, new Runnable() {
            private boolean m_pressedLast = get;

            @Override
            public void run() {
                boolean pressed = get;

                if (!m_pressedLast && pressed) {
                    if (command.isScheduled()) {
                        command.cancel();
                    } else {
                        command.schedule(interruptible);
                    }
                }

                m_pressedLast = pressed;
            }
        });

        return this;
    }

    public GamepadButtonMix toggleWhenActive(final boolean get, final Command command) {
        return toggleWhenActive(get,command, true);
    }

    public GamepadButtonMix toggleWhenActive(final boolean get, final Command commandOne, final Command commandTwo, boolean interruptible) {
        toggleWhenActive.JustOneTime(get, new Runnable() {
            private boolean m_pressedLast = get;
            private boolean m_firstCommandActive = false;

            @Override
            public void run() {
                boolean pressed = get;

                if (!m_pressedLast && pressed) {
                    if (m_firstCommandActive) {
                        if (commandOne.isScheduled()) {
                            commandOne.cancel();
                        }
                        commandTwo.schedule(interruptible);
                    } else {
                        if (commandTwo.isScheduled()) {
                            commandTwo.cancel();
                        }
                        commandOne.schedule(interruptible);
                    }

                    m_firstCommandActive = !m_firstCommandActive;
                }

                m_pressedLast = pressed;
            }
        });

        return this;
    }

    public GamepadButtonMix toggleWhenActive(final boolean get, final Command commandOne, final Command commandTwo) {
        return toggleWhenActive(get, commandOne, commandTwo, true);
    }

    public GamepadButtonMix toggleWhenActive(final boolean get, final Runnable runnableOne, final Runnable runnableTwo) {
        return toggleWhenActive(get, new InstantCommand(runnableOne), new InstantCommand(runnableTwo));
    }

    public GamepadButtonMix cancelWhenActive(final boolean get, final Command command) {
        cancelWhenActive.JustOneTime(get, new Runnable() {
            private boolean m_pressedLast = get;

            @Override
            public void run() {
                boolean pressed = get;

                if (!m_pressedLast && pressed) {
                    command.cancel();
                }

                m_pressedLast = pressed;
            }
        });
        return this;
    }

    /**
     * Composes this trigger with another trigger, returning a new trigger that is active when both
     * triggers are active.
     *
     * @param trigger the trigger to compose with
     * @return the trigger that is active when both triggers are active
     */
    public Trigger and(final boolean get, Trigger trigger) {
        return new Trigger(() -> get && trigger.get());
    }

    /**
     * Composes this trigger with another trigger, returning a new trigger that is active when either
     * trigger is active.
     *
     * @param trigger the trigger to compose with
     * @return the trigger that is active when either trigger is active
     */
    public Trigger or(final boolean get, Trigger trigger) {
        return new Trigger(() -> get || trigger.get());
    }

    /**
     * Creates a new trigger that is active when this trigger is inactive, i.e. that acts as the
     * negation of this trigger.
     *
     * @return the negated trigger
     */
    public Trigger negate(final boolean get) {
        return new Trigger(() -> !get);
    }
}