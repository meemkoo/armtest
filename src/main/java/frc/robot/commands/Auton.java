package frc.robot.commands;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.wpilibj.util.WPILibVersion;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Locator;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Utils.Direction;

@SuppressWarnings("unused")
public class Auton {
    private AutoChooser autoChooser = new AutoChooser();
    private AutoFactory autoFactory;

    // Subsystem refs
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private Vision vision;
    private RobotContainer rcontainer;

    public Trigger astop = new Trigger(() -> SmartDashboard.getBoolean("astop",
                        false
                    ));

    public Auton(
        Drivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        Intake intake,
        RobotContainer rcontainer,
        Vision vision
    ) {
        this.rcontainer = rcontainer;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.vision = vision;

        autoFactory = new AutoFactory(
            this.drivetrain::getPos,
            this.drivetrain::resetPose,
            this.drivetrain::followTrajectory,
            true,
            this.drivetrain
        );

        autoChooser.addCmd("Do nothing", Commands::none);
        autoChooser.addRoutine("Outpost", this::outpost);

        SmartDashboard.putData("Auton Selector", autoChooser);
        SmartDashboard.putBoolean("astop", false);
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.selectedCommandScheduler()
                .unless(astop)
        );

        RobotModeTriggers.teleop()
            .or(astop)
            .onTrue(
                Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
                    .alongWith(Commands.runOnce(() -> vision.usePose = true))
            );
    }

    public AutoRoutine outpost() {
        var routine = autoFactory.newRoutine("outpost");

        var outpost = routine.trajectory("outpost");

        routine.active().onTrue(
            Commands.sequence(
                outpost.resetOdometry(),
                outpost.cmd()
            )
        );

        return routine;
    }
}
