package frc.robot.Components;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A singleton class that encapsulates the {@link SwerveDrivePoseEstimator}
 * to give positions from limelight and odometry.
 * @author Darren Ringer
 */
public class PositionComponent{
    //-------------------------------------------Variables--------------------------------------------//
    private static PositionComponent instance;
    private static SwerveDrivePoseEstimator poseEstimator;
    private static Supplier<SwerveModulePosition[]> swerveModulePositionSupplier; 
    private static Supplier<SwerveModuleState[]> swerveModuleStatesSupplier;
    private static SwerveDriveKinematics kinematics;
    private static AHRS gyro;
    private static Pose2d lastCache;
    private static Pose2d origin;

    //------------------------------------------Core Methods------------------------------------------//

    private PositionComponent(SwerveDriveKinematics m_kinematics, Supplier<SwerveModulePosition[]> m_swerveModulePositionsSupplier, 
                              Supplier<SwerveModuleState[]> m_swerveModuleStatesSupplier, Pose2d initialPose, Pose2d originCompensator){
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        gyro.zeroYaw();

        lastCache = originCompensator.transformBy(new Transform2d(initialPose.toMatrix()));
        origin = originCompensator;
        kinematics = m_kinematics;
        swerveModulePositionSupplier = m_swerveModulePositionsSupplier;
        swerveModuleStatesSupplier = m_swerveModuleStatesSupplier;
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), m_swerveModulePositionsSupplier.get(), originCompensator.transformBy(new Transform2d(initialPose.toMatrix())));
    }
    /**
     * Gets the instance of PositionComponent or throws an error if none exists yet
     * @return Current instance of PositionComponent
     */
    public static PositionComponent getInstance(){
        if(instance == null){
            throw new Error("Cannot get instance of PositionComponent before initialization :(");
        }
        return instance;
    }
    /**
     * Initializes the PositionComponent
     * @param kinematics Swerve kinematics
     * @param swerveModulePositionsSupplier A supplier of the current drivetrain swerve module positions
     * @param swerveModuleStatesSupplier A supplier of the current drivetrain swerve module states
     * @param initialPose Starting Pose
     * @return Instance of PositionComponent
     */
    public static PositionComponent initialize(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier, 
                                               Supplier<SwerveModuleState[]> swerveModuleStatesSupplier, Pose2d initialPose, Pose2d originCompensator){
        instance = new PositionComponent(kinematics, swerveModulePositionsSupplier, swerveModuleStatesSupplier, initialPose, originCompensator);
        return instance;
    }
    /**
     * PositionComponent periodic loop (should be called in ComponentManager.periodic())
     */
    static int d = 0;
    static int q = 20;

    public static void periodic(){
        lastCache = poseEstimator.update(gyro.getRotation2d(), swerveModulePositionSupplier.get());

        if(++d==q) d=0;
    }
    //--------------------------------------------Getters---------------------------------------------//

    /**
     * Fetches the current robot pose
     * @return The current RobotPose
     */
    public static Pose2d getPose2d(){
        Transform2d temp = new Transform2d(origin,lastCache);
        return new Pose2d(temp.getTranslation(),temp.getRotation());
    }

    /**
     * Gets the robot's raw pose without any origins applied
     * @return Raw Pose2d
     */
    public static Pose2d getRawPose2d(){
        return lastCache;
    }

    /**
     * Gets the robot's current chassis speeds
     * @return A {@link ChassisSpeeds} object with the robot's current speeds.
     */
    public static ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] swerveModuleStates = swerveModuleStatesSupplier.get();
        return kinematics.toChassisSpeeds(
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[2],
            swerveModuleStates[3]
        );
    }

    /**
     * Gets the gyro 
     * @return the gyro
     */
    public static AHRS getGyro(){
        return gyro;
    }
    //--------------------------------------------Setters---------------------------------------------//

    /**
     * Resets the Pose of the robot to a given pose (i.e. if newPose = (1,1,0) then wherever the robot is currently will
     * be treated as (1,1,0)).
     * @param newPose The new pose for the robot
     */
    public static void resetPose(Pose2d newPose){
        origin = lastCache.transformBy(new Transform2d(newPose.getTranslation().times(-1),newPose.getRotation().times(-1)));
        gyro.setAngleAdjustment(newPose.getRotation().getDegrees()-gyro.getYaw());
    }
    /**
     * Sets the current pose to be the new zero
     */
    public static void zeroPos(){
        origin = lastCache;
        gyro.zeroYaw();
    }
    
    /**
     * Does a true reset of the position (internally as well) <p>
     * <strong>DO NOT USE UNLESS YOU KNOW WHAT YOU'RE DOING</strong>
     */
    public static void trueZero(){
        poseEstimator.resetPose(Pose2d.kZero);
        gyro.zeroYaw();
        origin = Pose2d.kZero;
        lastCache = Pose2d.kZero;
    }
}