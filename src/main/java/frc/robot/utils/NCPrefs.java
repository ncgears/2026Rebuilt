package frc.robot.utils;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Utility wrapper for WPILib {@link Preferences} access.
 */
public final class NCPrefs {
    private static NetworkTable prefs = Preferences.getNetworkTable();

    /**
     * Creates a non-instantiable preferences utility class.
     */
    private NCPrefs() {}

    /**
     * Refreshes the cached preferences table reference.
     */
    public static void init() {
        prefs = Preferences.getNetworkTable();
    }

    /**
     * Returns the table entry for a given preferences key.
     *
     * @param key Preferences key.
     * @return Backing NetworkTable entry.
     */
    private static NetworkTableEntry entry(String key) {
        return prefs.getEntry(key);
    }

    /**
     * Checks whether a key is reserved by the preferences backing table.
     *
     * @param key Preferences key.
     * @return True when the key is reserved metadata.
     */
    private static boolean isReservedKey(String key) {
        return ".type".equals(key);
    }

    /**
     * Escapes a String so it can be pasted into Java code as a literal.
     *
     * @param value String value to escape.
     * @return Escaped Java String literal contents.
     */
    private static String escapeJava(String value) {
        return value
            .replace("\\", "\\\\")
            .replace("\"", "\\\"")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
            .replace("\t", "\\t");
    }

    /**
     * Converts a preferences key into an UpperCamelCase Java identifier.
     *
     * @param key Preferences key.
     * @return Identifier-safe name for constants code generation.
     */
    private static String keyToIdentifier(String key) {
        StringBuilder name = new StringBuilder();
        String[] tokens = key.split("[^A-Za-z0-9]+");

        for (String token : tokens) {
            if (token.isEmpty()) {
                continue;
            }

            String safeToken = token.replaceAll("[^A-Za-z0-9]", "");
            if (safeToken.isEmpty()) {
                continue;
            }

            name.append(Character.toUpperCase(safeToken.charAt(0)));
            if (safeToken.length() > 1) {
                name.append(safeToken.substring(1));
            }
        }

        if (name.length() == 0) {
            return "PrefValue";
        }

        if (Character.isDigit(name.charAt(0))) {
            name.insert(0, '_');
        }

        return name.toString();
    }

    /**
     * Checks whether a preferences key currently exists.
     *
     * @param key Preferences key.
     * @return True when the key exists.
     */
    public static boolean hasKey(String key) {
        return prefs.containsKey(key);
    }

    /**
     * Removes a preferences key if it exists.
     *
     * @param key Preferences key.
     */
    public static void remove(String key) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.clearPersistent();
        prefEntry.unpublish();
    }

    /**
     * Creates a command that removes a preferences key once.
     *
     * @param key Preferences key.
     * @return Command that removes the key.
     */
    public static Command removeC(String key) {
        return Commands.runOnce(() -> remove(key));
    }

    /**
     * Stores a numeric value in preferences.
     *
     * <p>Numeric values are stored as NetworkTables doubles.
     *
     * @param key Preferences key.
     * @param value Value to store.
     */
    public static void setNumber(String key, Number value) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.setDouble(value.doubleValue());
        prefEntry.setPersistent();
    }

    /**
     * Creates a command that stores a numeric value once.
     *
     * @param key Preferences key.
     * @param value Value to store.
     * @return Command that writes the value.
     */
    public static Command setNumberC(String key, Number value) {
        return Commands.runOnce(() -> setNumber(key, value));
    }

    /**
     * Initializes a numeric key with a default only if missing.
     *
     * <p>Numeric values are stored as NetworkTables doubles.
     *
     * @param key Preferences key.
     * @param defaultValue Default value to initialize.
     */
    public static void initNumber(String key, Number defaultValue) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.setDefaultDouble(defaultValue.doubleValue());
        prefEntry.setPersistent();
    }

    /**
     * Creates a command that initializes a numeric key if missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value to initialize.
     * @return Command that initializes the key.
     */
    public static Command initNumberC(String key, Number defaultValue) {
        return Commands.runOnce(() -> initNumber(key, defaultValue));
    }

    /**
     * Gets a numeric value from preferences.
     *
     * <p>Returned value type is {@link Double}.
     *
     * @param key Preferences key.
     * @param defaultValue Fallback value when key does not exist.
     * @return Stored value or fallback.
     */
    public static Number getNumber(String key, Number defaultValue) {
        return entry(key).getDouble(defaultValue.doubleValue());
    }

    /**
     * Gets a numeric value and initializes the key when missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value used for initialization and fallback.
     * @return Stored value or default value.
     */
    public static Number getOrInitNumber(String key, Number defaultValue) {
        initNumber(key, defaultValue);
        return getNumber(key, defaultValue);
    }

    /**
     * Stores a boolean value in preferences.
     *
     * @param key Preferences key.
     * @param value Value to store.
     */
    public static void setBoolean(String key, boolean value) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.setBoolean(value);
        prefEntry.setPersistent();
    }

    /**
     * Creates a command that stores a boolean value once.
     *
     * @param key Preferences key.
     * @param value Value to store.
     * @return Command that writes the value.
     */
    public static Command setBooleanC(String key, boolean value) {
        return Commands.runOnce(() -> setBoolean(key, value));
    }

    /**
     * Initializes a boolean key with a default only if missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value to initialize.
     */
    public static void initBoolean(String key, boolean defaultValue) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.setDefaultBoolean(defaultValue);
        prefEntry.setPersistent();
    }

    /**
     * Creates a command that initializes a boolean key if missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value to initialize.
     * @return Command that initializes the key.
     */
    public static Command initBooleanC(String key, boolean defaultValue) {
        return Commands.runOnce(() -> initBoolean(key, defaultValue));
    }

    /**
     * Gets a boolean value from preferences.
     *
     * @param key Preferences key.
     * @param defaultValue Fallback value when key does not exist.
     * @return Stored value or fallback.
     */
    public static boolean getBoolean(String key, boolean defaultValue) {
        return entry(key).getBoolean(defaultValue);
    }

    /**
     * Gets a boolean value and initializes the key when missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value used for initialization and fallback.
     * @return Stored value or default value.
     */
    public static boolean getOrInitBoolean(String key, boolean defaultValue) {
        initBoolean(key, defaultValue);
        return getBoolean(key, defaultValue);
    }

    /**
     * Stores a String value in preferences.
     *
     * @param key Preferences key.
     * @param value Value to store.
     */
    public static void setString(String key, String value) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.setString(value);
        prefEntry.setPersistent();
    }

    /**
     * Creates a command that stores a String value once.
     *
     * @param key Preferences key.
     * @param value Value to store.
     * @return Command that writes the value.
     */
    public static Command setStringC(String key, String value) {
        return Commands.runOnce(() -> setString(key, value));
    }

    /**
     * Initializes a String key with a default only if missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value to initialize.
     */
    public static void initString(String key, String defaultValue) {
        NetworkTableEntry prefEntry = entry(key);
        prefEntry.setDefaultString(defaultValue);
        prefEntry.setPersistent();
    }

    /**
     * Creates a command that initializes a String key if missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value to initialize.
     * @return Command that initializes the key.
     */
    public static Command initStringC(String key, String defaultValue) {
        return Commands.runOnce(() -> initString(key, defaultValue));
    }

    /**
     * Gets a String value from preferences.
     *
     * @param key Preferences key.
     * @param defaultValue Fallback value when key does not exist.
     * @return Stored value or fallback.
     */
    public static String getString(String key, String defaultValue) {
        return entry(key).getString(defaultValue);
    }

    /**
     * Gets a String value and initializes the key when missing.
     *
     * @param key Preferences key.
     * @param defaultValue Default value used for initialization and fallback.
     * @return Stored value or default value.
     */
    public static String getOrInitString(String key, String defaultValue) {
        initString(key, defaultValue);
        return getString(key, defaultValue);
    }

    /**
     * Gets a snapshot of all stored preference values keyed by preference key.
     *
     * <p>Returned values are one of: {@link Boolean}, {@link Double}, {@link Long},
     * {@link Float}, or {@link String}. Unsupported value types are omitted.
     *
     * @return Sorted map of key to current value.
     */
    public static Map<String, Object> snapshotValues() {
        List<String> keys = new ArrayList<>(prefs.getKeys());
        Collections.sort(keys);

        Map<String, Object> snapshot = new LinkedHashMap<>();
        for (String key : keys) {
            if (isReservedKey(key)) {
                continue;
            }

            NetworkTableValue value = entry(key).getValue();
            if (!value.isValid()) {
                continue;
            }

            if (value.isBoolean()) {
                snapshot.put(key, value.getBoolean());
            } else if (value.isDouble()) {
                snapshot.put(key, value.getDouble());
            } else if (value.isInteger()) {
                snapshot.put(key, value.getInteger());
            } else if (value.isFloat()) {
                snapshot.put(key, value.getFloat());
            } else if (value.isString()) {
                snapshot.put(key, value.getString());
            }
        }

        return snapshot;
    }

    /**
     * Builds Java constants lines using current preference values.
     *
     * <p>Numeric values are exported as doubles by default.
     *
     * @return Multiline Java snippet for {@code PersistentDefaults} fields.
     */
    public static String exportDefaults() {
        return exportDefaults(Set.of());
    }

    /**
     * Builds Java constants lines using current preference values.
     *
     * <p>Use {@code intKeys} to force specific numeric keys to export as ints.
     *
     * @param intKeys Set of keys that should export using {@code int}.
     * @return Multiline Java snippet for {@code PersistentDefaults} fields.
     */
    public static String exportDefaults(Set<String> intKeys) {
        StringBuilder lines = new StringBuilder();
        Map<String, Object> snapshot = snapshotValues();

        for (Map.Entry<String, Object> pref : snapshot.entrySet()) {
            String key = pref.getKey();
            Object value = pref.getValue();
            String constantName = keyToIdentifier(key);

            if (value instanceof Boolean boolValue) {
                lines.append("public static final boolean ")
                    .append(constantName)
                    .append(" = ")
                    .append(boolValue)
                    .append(";");
            } else if (value instanceof Number numberValue) {
                if (intKeys.contains(key)) {
                    lines.append("public static final int ")
                        .append(constantName)
                        .append(" = ")
                        .append(numberValue.intValue())
                        .append(";");
                } else {
                    lines.append("public static final double ")
                        .append(constantName)
                        .append(" = ")
                        .append(Double.toString(numberValue.doubleValue()))
                        .append(";");
                }
            } else if (value instanceof String stringValue) {
                lines.append("public static final String ")
                    .append(constantName)
                    .append(" = \"")
                    .append(escapeJava(stringValue))
                    .append("\";");
            } else {
                continue;
            }

            lines.append(System.lineSeparator());
        }

        return lines.toString();
    }

    /**
     * Prints current preference values as Java defaults lines.
     */
    public static void printDefaults() {
        System.out.println(exportDefaults());
    }

    /**
     * Creates a command that prints current preference values as Java defaults lines.
     *
     * @return Command that prints the export snippet once.
     */
    public static Command printDefaultsC() {
        return Commands.runOnce(NCPrefs::printDefaults);
    }

    /**
     * Builds Java constants lines using current preference values.
     *
     * <p>This is retained for backwards compatibility.
     *
     * @return Multiline Java snippet for {@code PersistentDefaults} fields.
     */
    public static String exportDefaultsLines() {
        return exportDefaults();
    }

    /**
     * Builds Java constants lines using current preference values.
     *
     * <p>This is retained for backwards compatibility.
     *
     * @param intKeys Set of keys that should export using {@code int}.
     * @return Multiline Java snippet for {@code PersistentDefaults} fields.
     */
    public static String exportDefaultsLines(Set<String> intKeys) {
        return exportDefaults(intKeys);
    }
}
