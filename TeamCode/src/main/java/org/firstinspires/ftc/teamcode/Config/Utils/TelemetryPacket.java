package org.firstinspires.ftc.teamcode.Config.Utils;

import androidx.annotation.NonNull;

/**
 * TelemetryPacket - A container for telemetry data that supports all primitive types
 * This class stores a name-value pair for telemetry display
 */
public class TelemetryPacket {
    private String name;
    private Object value;
    private Class<?> type;

    /**
     * Create a TelemetryPacket with a name and value
     * @param name The name/label for this telemetry item
     * @param value The value (supports all primitive types and String)
     */
    public TelemetryPacket(String name, Object value) {
        this.name = name;
        this.value = value;
        this.type = value != null ? value.getClass() : Object.class;
    }

    // Constructors for all primitive types (for type safety and autoboxing)

    public TelemetryPacket(String name, int value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, double value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, float value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, long value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, boolean value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, byte value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, short value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, char value) {
        this(name, (Object) value);
    }

    public TelemetryPacket(String name, String value) {
        this(name, (Object) value);
    }

    // Getters

    public String getName() {
        return name;
    }

    public Object getValue() {
        return value;
    }

    public Class<?> getType() {
        return type;
    }

    // Type-safe getters with defaults

    public int getInt() {
        if (value instanceof Number) {
            return ((Number) value).intValue();
        }
        return 0;
    }

    public double getDouble() {
        if (value instanceof Number) {
            return ((Number) value).doubleValue();
        }
        return 0.0;
    }

    public float getFloat() {
        if (value instanceof Number) {
            return ((Number) value).floatValue();
        }
        return 0.0f;
    }

    public long getLong() {
        if (value instanceof Number) {
            return ((Number) value).longValue();
        }
        return 0L;
    }

    public boolean getBoolean() {
        if (value instanceof Boolean) {
            return (Boolean) value;
        }
        return false;
    }

    public byte getByte() {
        if (value instanceof Number) {
            return ((Number) value).byteValue();
        }
        return 0;
    }

    public short getShort() {
        if (value instanceof Number) {
            return ((Number) value).shortValue();
        }
        return 0;
    }

    public char getChar() {
        if (value instanceof Character) {
            return (Character) value;
        }
        return '\0';
    }

    public String getString() {
        return value != null ? value.toString() : "null";
    }

    // Setters

    public void setValue(Object value) {
        this.value = value;
        this.type = value != null ? value.getClass() : Object.class;
    }

    public void setValue(int value) {
        setValue((Object) value);
    }

    public void setValue(double value) {
        setValue((Object) value);
    }

    public void setValue(float value) {
        setValue((Object) value);
    }

    public void setValue(long value) {
        setValue((Object) value);
    }

    public void setValue(boolean value) {
        setValue((Object) value);
    }

    public void setValue(byte value) {
        setValue((Object) value);
    }

    public void setValue(short value) {
        setValue((Object) value);
    }

    public void setValue(char value) {
        setValue((Object) value);
    }

    public void setValue(String value) {
        setValue((Object) value);
    }

    public void setName(String name) {
        this.name = name;
    }

    // Utility methods

    /**
     * Check if the stored value is a numeric type
     */
    public boolean isNumeric() {
        return value instanceof Number;
    }

    /**
     * Check if the stored value is a boolean
     */
    public boolean isBoolean() {
        return value instanceof Boolean;
    }

    /**
     * Check if the stored value is a string
     */
    public boolean isString() {
        return value instanceof String;
    }

    /**
     * Check if the stored value is a character
     */
    public boolean isChar() {
        return value instanceof Character;
    }

    /**
     * Get a formatted string representation with format specifier
     * @param format Format string (e.g., "%.2f" for 2 decimal places)
     * @return Formatted string
     */
    public String getFormattedValue(String format) {
        if (value == null) {
            return "null";
        }
        try {
            return String.format(format, value);
        } catch (Exception e) {
            return value.toString();
        }
    }

    /**
     * Get a formatted telemetry line
     * @return Formatted string like "Name: Value"
     */
    public String getFormattedLine() {
        return name + ": " + getString();
    }

    /**
     * Get a formatted telemetry line with custom format
     * @param format Format string for the value
     * @return Formatted string like "Name: Value"
     */
    public String getFormattedLine(String format) {
        return name + ": " + getFormattedValue(format);
    }

    @NonNull
    @Override
    public String toString() {
        return getFormattedLine();
    }
}
