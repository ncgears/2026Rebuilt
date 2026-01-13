
package frc.robot.classes;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;

/**
 * The Lighting Subsystem handles getting and managing the Lighting from the CANdle.
 * It is responsible for changing lighting colors and animations.
 */
@SuppressWarnings("unused")
public class Lighting {
	private static Lighting instance;
  private final CANdle m_candle1 = new CANdle(LightingConstants.kCandle1ID, LightingConstants.canBus);
  private final CANdle m_candle2 = new CANdle(LightingConstants.kCandle2ID, LightingConstants.canBus);
  private Colors m_currentColor, m_oldColor = Colors.OFF;
  private boolean m_blinking, m_oldBlinking = false;
  private int m_intensity, m_oldIntensity = 100;
  //private and public variables defined here
  public enum Colors {
    OFF(0,0,0),
    NCBLUE(0,119,181),
    NCGREEN(0,255,0),
    AMBER(255,100,0),
    AQUA(50,255,255),
    BLACK(0,0,0),
    BLUE(0,0,255),
    CYAN(0,255,255),
    GOLD(255,222,30),
    GREEN(0,255,0),
    JADE(0,255,40),
    MAGENTA(255,0,20),
    LACE(253,245,230),
    ORANGE(255,40,0),
    PINK(242,90,255),
    PURPLE(180,0,255),
    RED(255,0,0),
    WHITE(255,255,255),
    TEAL(0,255,120),
    YELLOW(255,150,0);
    private final int r, g, b;
    Colors(int r, int g, int b) { this.r = r; this.g = g; this.b = b; }
    /**
     * Gets the red component for this color.
     *
     * @return Red value (0-255).
     */
    public int R() { return this.r; }
    /**
     * Gets the green component for this color.
     *
     * @return Green value (0-255).
     */
    public int G() { return this.g; }
    /**
     * Gets the blue component for this color.
     *
     * @return Blue value (0-255).
     */
    public int B() { return this.b; }
  }
  /**
	 * Returns the instance of the LightingSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return LightingSubsystem instance
	 */
  public static Lighting getInstance() {
		if (instance == null)
			instance = new Lighting();
		return instance;
	}
  
  /** Creates the lighting subsystem and initializes dashboards. */
  public Lighting() {
    //initialize values for private and public variables, etc.
    init();
    createDashboards();
  }
  
    
  /**
   * The init method resets and operational state of the class
   */
  public void init() {
    m_currentColor = Colors.OFF;
    NCDebug.Debug.debug("Lighting: Initialized");
  }
  

  /** Creates Shuffleboard widgets for lighting state. */
  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("LED Color", this::getColor)
      .withSize(8, 2)
      .withWidget("Single Color View")
      .withPosition(0, 5);  
		if(LightingConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Lighting");
      debugTab.addString("LED Color", this::getColor)
        .withSize(6, 4)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      // debugTab.addString("LED Hex", this::getColor)
      //   .withSize(6, 2)
      //   .withWidget("Text String")
      //   .withPosition(0, 4);  
    }
  }

  /**
   * Gets the current LED color as a hex string.
   *
   * @return Hex string representing the current color.
   */
  public String getColor() {
    if(m_currentColor == null) m_currentColor = Colors.OFF;
    return new Color(m_currentColor.R(), m_currentColor.G(), m_currentColor.B()).toHexString();
  }

  /**
   * Sets the CANdle LEDs to the requested color.
   *
   * @param color Color to set.
   */
  public void setColor(Colors color) {
    if(color != m_currentColor) {
      m_currentColor = color;
      
      // Phoenix 6 CANdle uses setControl() with SolidColor control requests
      // 0-7 are onboard LEDs
      RGBWColor rgbwColor = new RGBWColor(color.R(), color.G(), color.B(), 0);
      m_candle1.setControl(new SolidColor(0, 7).withColor(rgbwColor));
      m_candle2.setControl(new SolidColor(0, 7).withColor(rgbwColor));
    }
  }
  

  /**
   * Creates a command that sets the LED color.
   *
   * @param color Color to set.
   * @return Command that updates LED color.
   */
  public Command setColorCommand(Colors color) {
    return new InstantCommand(() -> setColor(color)).ignoringDisable(true);
  }

  /**
   * Returns a command that cycles team colors for a dance party effect.
   *
   * @return Repeat command with a color sequence.
   */
  public Command danceParty() {
    return new RepeatCommand(  //Lighting Dance Party!
      setColorCommand(Colors.NCBLUE)
      .andThen(wait(0.5))
      .andThen(setColorCommand(Colors.NCGREEN))
      .andThen(wait(0.5))
      .andThen(setColorCommand(Colors.NCBLUE))
      .andThen(wait(0.15))
      .andThen(setColorCommand(Colors.OFF))
      .andThen(wait(0.15))
      .andThen(setColorCommand(Colors.NCBLUE))
      .andThen(wait(0.15))
      .andThen(setColorCommand(Colors.NCGREEN))
      .andThen(wait(0.25))
    );
  }

  /**
   * Creates a command to set climb start color.
   *
   * @return Command that sets LED color to gold.
   */
  public Command climbStartColor() {
    return setColorCommand(Colors.GOLD);
  }

  /**
   * Creates a command that blinks the climb-done color.
   *
   * @return Repeat command that blinks gold.
   */
  public Command climbDoneColor() {
    return new RepeatCommand(  //blinking
      setColorCommand(Colors.OFF)
      .andThen(wait(0.2))
      .andThen(setColorCommand(Colors.GOLD))
      .andThen(wait(0.2))
    );
  }

  /**
   * Convenience wrapper for {@link WaitCommand}.
   *
   * @param seconds Time to wait.
   * @return Wait command.
   */
  private Command wait(double seconds) {
    return new WaitCommand(seconds);
  }

}
