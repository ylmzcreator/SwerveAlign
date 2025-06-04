package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera photonCamera;
    private PhotonPipelineResult latestResult;
    private final double cameraHeight = VisionConstants.CAMERA_HEIGHT_METERS;
    private final double targetHeight = VisionConstants.TARGET_HEIGHT_METERS;
    private final double cameraPitch = VisionConstants.CAMERA_PITCH_RADIANS;
    private final Timer timer = new Timer();

    public VisionSubsystem(String cameraName){
        this.photonCamera = new PhotonCamera(cameraName);
    }
    
    public PhotonCamera getCamera(){
        return photonCamera;
    }

    public double estimateDistance(){
        var distance = (targetHeight - cameraHeight) / Math.tan(cameraPitch + Units.degreesToRadians(getTargetPitch()));
        return distance;
    }
    
    public boolean hasTarget(){
        var result = latestResult;
        boolean hasTarget = false;
        
        if(result != null && result.hasTargets()){
            hasTarget = true;
        }else{
            if(hasTarget){
                timer.reset();
            }else{
                if(timer.get() > 1){
                    hasTarget = false;
                }
            }
        }
        return hasTarget;
        
    }
    public double getTargetArea(){
        var target = getTarget();
        return(target != null) ? target.getArea() : Double.NaN;
    }

    public double getTargetPitch(){
        var target = getTarget();
        return(target != null) ? target.getPitch() : Double.NaN;
    }

    public double getTargetYaw(){
        var target = getTarget();
        return(target != null) ? target.getYaw() : Double.NaN;
    }

    public PhotonTrackedTarget getTarget(){
        var result = latestResult;
        if(result != null && result.hasTargets()){
            return result.getBestTarget();
        }else{
            return null;
        }
    }
    
    
    @Override
    public void periodic() {
        
        
        var results = photonCamera.getAllUnreadResults();
        latestResult = results.isEmpty() ? null : results.get(results.size()-1);
        SmartDashboard.putBoolean("hasTarget", hasTarget());
    }
    
    
}
