// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem("photon");

  private final Joystick driverJoystick = new Joystick(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveSubsystem,
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      )
    );

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2).toggleOnTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(driverJoystick, 3).whileTrue(new AutoAlignCommand(swerveSubsystem, visionSubsystem));
  }

  public Command getAutonomousCommand() {
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(DriveConstants.kDriveKinematics);

    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)),
      List.of(
        new Translation2d(1,0),
        new Translation2d(1,-1)
      ),
      new Pose2d(2,-1, Rotation2d.fromDegrees(180)),
      trajectoryConfig
    );

    
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController,
      0, 0,
      AutoConstants.kThetaControllerConstraints
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerveSubsystem::getPoseMeters,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem
    );

    
    return new SequentialCommandGroup(
      new InstantCommand(()-> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(()-> swerveSubsystem.stopModules())
    );

  } 
}
