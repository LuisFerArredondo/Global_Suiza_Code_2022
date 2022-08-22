package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

public enum ColorFormatter {
    RED("FF2222"), ORANGE("EA5D00"), YELLOW("FFBF00"), LIME("69FF00"),
    GREEN("00FF1D"), CYAN("00FFFF"), BLUE("172EAE"), PURPLE("822EFF"),
    MAGENTA("BE00FF"), PINK("FF3ADC"), BLACK("000000"), WHITE("FFFFFF"),
    LIGHT_GRAY("A3A3A3"), DARK_GRAY("404040"), NO_COLOR("FFFFFF");
    String hexValue;

    ColorFormatter(String hex) {
        hexValue = hex;
    }

    /**
     * Get the hex value
     *
     * @return The hex for the color
     */
    public String getHexValue() {
        return hexValue;
    }

    /**
     * Format the supplied object with the HTML to become this color
     *
     * @param object The object
     * @return The formatted String
     */
    public String format(String object) {
        return "<font" + (this == NO_COLOR ? "" : " color=#" + getHexValue()) + ">" + object + "</font>";
    }

    /**
     * Format the supplied object with the HTML and a format String to become this color
     *
     * @param objects The objects
     * @param format  The format for the supplied String
     * @return The formatted String
     */
    public String format(String format, Object... objects) {
        return format(String.format(format, objects));
    }

}
