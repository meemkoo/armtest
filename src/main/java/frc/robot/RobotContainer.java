// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveTeleoperated;
import frc.robot.generated.TunerConstantsArkelon0416;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private static RobotContainer instance;
    public static RobotContainer getInstance() {
        if (instance==null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final CommandXboxController driverCtl = new CommandXboxController(0);
    private final CommandXboxController coDriverCtl = new CommandXboxController(1);

    private Intake intake = new Intake(); 
    private Indexer indexer = new Indexer();
    private Shooter shooter = new Shooter(() -> Degrees.of(SmartDashboard.getNumber("testAngle", 20)), 
                                          () -> RPM.of(SmartDashboard.getNumber("testRpm", 3000)));
    private Drivetrain drivetrain = TunerConstantsArkelon0416.createDrivetrain();
    public Locator locator;

    public RobotContainer() {
        locator = new Locator(drivetrain::getPos, drivetrain);
        drivetrain.registerTelemetry(Logging.getInstance()::logCTREChassis);

        SmartDashboard.putNumber("testAngle", 20);
        SmartDashboard.putNumber("testRpm", 3000);
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            new DriveTeleoperated(drivetrain,
                driverCtl::getLeftY,
                driverCtl::getLeftX,
                driverCtl::getRightX,
                driverCtl::getRightTriggerAxis,
                () -> Locator.getInstance().hubPose,
                driverCtl.leftStick(),
                driverCtl.a(),
                driverCtl.b()
            )
        );

        // Brendan shoot is: x
        // Brendan intake up/down NORMAL is: leftBumper
        // Brendan rin intake: left trigger
        // For my you asked me what i wanted my loc kpose on right? Oh i already did that! Yep yep okay for my all wheels pointing forwards
        // I want that to be on B
        // Idle all: povUp
        // Reset odometry: povDown
        // Toggle vision: povRight
        // Keep flywheel spun up while holding: rightBumper
        // Reverse intake povLeft
        // 
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
