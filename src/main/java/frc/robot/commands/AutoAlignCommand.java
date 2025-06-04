package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    
    private final PIDController turningPIDController;


    
    private final PIDController yPIDController;

    public AutoAlignCommand(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem
    ){
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.turningPIDController = new PIDController(0, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.yPIDController = new PIDController(0.3, 0, 0);
        turningPIDController.setTolerance(Math.toRadians(2));
        yPIDController.setTolerance(0.03);
        addRequirements(swerveSubsystem,visionSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        

        if(visionSubsystem.hasTarget()){
            double targetYaw = visionSubsystem.getTargetYaw();
            double distance = visionSubsystem.estimateDistance() -VisionConstants.TARGET_DISTANCE_METERS;
            double turnOutput = MathUtil.clamp(
                turningPIDController.calculate(0, targetYaw),
                -AutoConstants.MAX_TURN_SPEED,
                AutoConstants.MAX_TURN_SPEED
            );

            double yOutput = MathUtil.clamp(
                yPIDController.calculate(distance, 0),
                -AutoConstants.MAX_FORWARD_SPEED,
                AutoConstants.MAX_FORWARD_SPEED
            );



            ChassisSpeeds chassisSpeeds;
            if(!turningPIDController.atSetpoint()){
                chassisSpeeds = new ChassisSpeeds(
                    0,
                    0,
                    turnOutput
                );
            }else{
                chassisSpeeds = new ChassisSpeeds(
                    0,
                    yOutput,
                    0
                );
            }
            var moduleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);

        }else{
            swerveSubsystem.stopModules();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        
    }

    @Override
    public boolean isFinished() {
        if(yPIDController.atSetpoint() && turningPIDController.atSetpoint()){
            return true;
        }else{
            return false;
        }
    }
}
