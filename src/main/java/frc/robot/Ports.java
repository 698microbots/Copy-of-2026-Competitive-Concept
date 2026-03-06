package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 11; //Intake Pivot On Feeder (right on bot)
    public static final int kIntakeRollers = 9; //Intake motor on rollers
    public static final int kFloor = 13; //Feeder to column (below shooter)
    public static final int kFeeder = 12; //Feeder (left on bot)


    public static final int kShooterLeft = 16; //Column
    public static final int kShooterMiddle = 14; //Column
    public static final int kShooterRight = 15; // Column
    public static final int kHanger = 17; // Climber hook

    // PWM Ports
    public static final int kHoodLeftServo = 9;
    public static final int kHoodRightServo = 8;
}
