package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

    
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    
    private final SparkMaxConfig driveMotorConfig;
    private final SparkMaxConfig turningMotorConfig;

    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    
    private final AnalogInput absoluteEncoder;
    private final double absoluteEncoderOffsetRad;
    private final boolean absoluteEncoderReversed;

    
    private final PIDController turningPidController;

    public SwerveModule(
            int driveMotorId,
            int turningMotorId,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderId,
            double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        
        this.driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        this.turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        
        this.driveMotorConfig = new SparkMaxConfig();
        this.turningMotorConfig = new SparkMaxConfig();

        
        driveMotor.configure(driveMotorConfig, null, null);
        turningMotor.configure(turningMotorConfig, null, null);

        
        this.driveEncoder = driveMotor.getEncoder();
        this.turningEncoder = turningMotor.getEncoder();

        
        this.absoluteEncoder = new AnalogInput(absoluteEncoderId);
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        
        driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        
        resetEncoders();

    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle = angle * Math.PI * 2;
        angle = angle - absoluteEncoderOffsetRad;
        angle = angle * (absoluteEncoderReversed ? -1 : 1);
        angle = Math.IEEEremainder(angle, 2 * Math.PI);
        return angle;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                new Rotation2d(getTurningPosition())
            );
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(getTurningPosition())
        );
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)< 0.001){
            stop();
            return;
        }

        state.optimize(getState().angle );
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
        SmartDashboard.putString("Swerve["+absoluteEncoder.getChannel()+ "] state", state.toString());
    }

}
