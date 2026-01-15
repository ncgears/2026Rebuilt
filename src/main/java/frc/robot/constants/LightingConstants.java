
package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants for the Lighting Subsystem
 */
public class LightingConstants {
    public static final CANBus canBus = new CANBus("rio");
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final int kCandle1ID = ID.CANdle.candle1;
    public static final int kCandle2ID = ID.CANdle.candle2;
}
