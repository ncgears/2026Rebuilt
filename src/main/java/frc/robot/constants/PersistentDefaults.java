package frc.robot.constants;

/**
 * Default values for persistent preferences-backed settings.
 *
 * <p>How to refresh these defaults from the robot's current persistent values:
 *
 * <ol>
 *   <li>Run a one-time call to {@code NCPrefs.printDefaults()} (or schedule
 *       {@code NCPrefs.printDefaultsC()}) while connected to the robot.
 *   <li>Copy the printed output from the console.
 *   <li>Update the values in this class to match the latest known-good settings.
 *   <li>Commit the updated defaults before events so a replacement rio can be restored quickly.
 * </ol>
 */
public final class PersistentDefaults {
    //#region Defaults
    public static final double IntakeDeployMagOffset = 0.010820;
    //#endregion Defaults

    /**
     * Creates a non-instantiable persistent defaults constants class.
     */
    private PersistentDefaults() {}
}
