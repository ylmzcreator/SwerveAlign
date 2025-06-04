package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

    
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        getKinematics(),
        getRotation2d(),
        new SwerveModulePosition[]{
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
        }
    );

    public SwerveSubsystem(){
        new Thread(()->{
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch(Exception e){

            }
        }).start();
    }

    public AHRS getGyro(){
        return gyro;
    }

    public SwerveDriveKinematics getKinematics(){
        return DriveConstants.kDriveKinematics;
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPoseMeters(){
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[]{
                frontRight.getPosition(),
                frontLeft.getPosition(),
                backRight.getPosition(),
                backLeft.getPosition()
        },
        pose
        );
    }

    @Override
    public void periodic() {
        odometry.update(
            getRotation2d(),
            new SwerveModulePosition[]{
                frontRight.getPosition(),
                frontLeft.getPosition(),
                backRight.getPosition(),
                backLeft.getPosition()
            }
        );
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString(
            "Robot Location",
            getPoseMeters()
                .getTranslation()
                .toString()
        );
    }
    public void stopModules(){
        frontRight.stop();
        frontLeft.stop();
        backRight.stop();
        backLeft.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredState){
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredState,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );
        frontRight.setDesiredState(desiredState[0]);
        frontLeft.setDesiredState(desiredState[1]);
        backRight.setDesiredState(desiredState[2]);
        backLeft.setDesiredState(desiredState[3]);
    }
}
