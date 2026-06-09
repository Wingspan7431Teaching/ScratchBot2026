package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.PositionComponent;
import frc.robot.Utilities.MiscMathFunctions;

public class GoToPoint extends Command{
    Pose2d targetPosition;
    double startTime = 0;
    double distance;
    double rotDistance;
    Supplier<Boolean> stopButton;
    boolean checkIsRobotStopped;
    boolean scratchbot = false;
    double moveX;
    Rotation2d turn;
    double moveY;

    TrapezoidProfile.State xStartState;
    TrapezoidProfile.State xEndState;
    TrapezoidProfile.State yStartState;
    TrapezoidProfile.State yEndState;
    TrapezoidProfile.State rotationStartState;
    TrapezoidProfile.State rotationEndState;
    
    /**
     * Constructor for a GoToPoint object
     * @author Giahna C.
     * @param x The desired position for the bot along the x-axis (uses NWU coords)
     * @param y The desired position for the bot along the y-axis (uses NWU coords)
     * @param rotation The desired angle for the bot (uses NWU coords)
     * @param stopButton A supplier that will stop the command if true.
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(double x, double y, double rotation, Supplier<Boolean> stopButton, boolean checkIsRobotStopped){
        targetPosition = new Pose2d(x, y, new Rotation2d(rotation));
        this.stopButton = stopButton;
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    /**
     *  Constructor for a GoToPoint object
     * @author Giahna C.
     * @param targetPosition The position the robot should be in at the end, contains x, y, and rotation. (uses NWU coords)
     * @param stopButton A supplier that will stop the command if true.
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(Pose2d targetPosition, Supplier<Boolean> stopButton, boolean checkIsRobotStopped){
        this.targetPosition = targetPosition;
        this.stopButton = stopButton;
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    /**
     *  Constructor for a GoToPoint object
     * @author Giahna C.
     * @param x The desired position for the bot along the x-axis (uses NWU coords)
     * @param y The desired position for the bot along the y-axis (uses NWU coords)
     * @param rotation The desired angle for the bot (uses NWU coords)
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(double x, double y, double rotation, boolean checkIsRobotStopped){
        targetPosition = new Pose2d(x, y, new Rotation2d(rotation));
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    /**
     *  Constructor for a GoToPoint object
     * @author Giahna C.
     * @param targetPosition The position the robot should be in at the end, contains x, y, and rotation. (uses NWU coords)
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(Pose2d targetPosition, boolean checkIsRobotStopped){
        this.targetPosition = targetPosition;
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    public GoToPoint(double moveForward, double moveLeft, Rotation2d turn, boolean checkIsRobotStopped){
        scratchbot = true;
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        this.moveX = moveForward;
        this.moveY = moveLeft;
        this.turn = turn;
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void initialize(){
        if(scratchbot){
            Pose2d curPose = PositionComponent.getPose2d();
            this.targetPosition = curPose.transformBy(new Transform2d(moveX, moveY, turn));
            // this.targetPosition = new Pose2d(curPose.getX() + moveX, curPose.getY()+ moveY, curPose.getRotation());
        }
        System.out.println("x" + targetPosition.getX() + " y:" + targetPosition.getY() + "rot: " + targetPosition.getRotation().getRadians());
        startTime = Timer.getFPGATimestamp();
        //sets the overall states for the profiles
        xStartState = new TrapezoidProfile.State(PositionComponent.getPose2d().getX(), PositionComponent.getChassisSpeeds().vxMetersPerSecond);
        xEndState = new TrapezoidProfile.State(targetPosition.getX(), 0);
        yStartState = new TrapezoidProfile.State(PositionComponent.getPose2d().getY(), PositionComponent.getChassisSpeeds().vyMetersPerSecond);
        yEndState = new TrapezoidProfile.State(targetPosition.getY(), 0);
        rotationStartState = new TrapezoidProfile.State(PositionComponent.getPose2d().getRotation().getRadians(), PositionComponent.getChassisSpeeds().omegaRadiansPerSecond);
        rotationEndState = new TrapezoidProfile.State(targetPosition.getRotation().getRadians(), 0);
    }

    @Override
    public void execute(){
        double velX;
        double velY;
        double velR;

        distance = getDistanceFromPoint();
        rotDistance = Math.abs(getDifferenceFromAngle());

        System.out.println("dist Away:: " + distance);
        //get the current setpoints for the profiles
        TrapezoidProfile.State xSetpoint = DrivetrainConstants.xProfile.calculate((Timer.getFPGATimestamp() - startTime), xStartState, xEndState);
        TrapezoidProfile.State ySetpoint = DrivetrainConstants.yProfile.calculate((Timer.getFPGATimestamp() - startTime), yStartState, yEndState);
        TrapezoidProfile.State rotSetpoint = DrivetrainConstants.rotProfile.calculate((Timer.getFPGATimestamp() - startTime), rotationStartState, rotationEndState);
        
        //if the velocity is 0 and we're still not where we need to be, calculate the velocities
        //else just grab it from the setpoint.
        if (xSetpoint.velocity == 0 && ySetpoint.velocity == 0 && distance > DrivetrainConstants.atPointTarget){
            Vector velocities = calculateVelocitiesXY(targetPosition, PositionComponent.getPose2d());
            velX = velocities.x;
            velY = velocities.y;
        }else{
            velX = xSetpoint.velocity;
            velY = ySetpoint.velocity;
        }
        
        if (rotSetpoint.velocity ==0 && rotDistance > DrivetrainConstants.atRotTarget){
            velR = calculateVelocityRot(targetPosition, PositionComponent.getPose2d());
        }else{
            velR = rotSetpoint.velocity;
        }

        //set the PIDs based on what has been calculated
        Drivetrain.getInstance().setVelocityPIDs(velX, velY, velR, false, false);
    }
    
    @Override
    public void end(boolean interrupted){
        Drivetrain.getInstance().setVelocityPIDs(0,0,0, false,false);
    }

    @Override
    public boolean isFinished(){
        boolean isRobotStopped = checkIsRobotStopped? Drivetrain.getInstance().isRobotStopped(): true;
        return stopButton.get() || (
         distance < DrivetrainConstants.atPointTarget &&
         rotDistance < DrivetrainConstants.atRotTarget &&
         isRobotStopped
        );
    }

    /**
     * Gets the distance between the robot and the point the robot is going to that was set in the constructor
     * @author Giahna C.
     * @return the distance
     */
    public double getDistanceFromPoint(){
        Pose2d curPos = PositionComponent.getPose2d();
        return MiscMathFunctions.distance(curPos.getX(), targetPosition.getX(), curPos.getY(), targetPosition.getY());
    }

    /**
     * Gets the difference in radians between the robot's rotation and the rotation the robot was set to in the constructor
     * @author Giahna C.
     * @return the differnce
     */
    public double getDifferenceFromAngle(){
        double currentAngle = PositionComponent.getPose2d().getRotation().getRadians();
        return MiscMathFunctions.mod(targetPosition.getRotation().getRadians() - currentAngle -Math.PI, 2*Math.PI) - Math.PI;
    }

    /**
     * Gets the position this command is set to go to
     * @author Giahna C. 
     * @return the target position
     */
    public Pose2d getTargetPosition(){
        return targetPosition;
    }
    
    /**
     * 
     * @param desiredPose the position the robot should be in
     * @param currentPose the position the robot is currently in
     * @return returns the needed velocity as a 2D vector
     * @author Giahna C.
     */
    public Vector calculateVelocitiesXY(Pose2d desiredPose, Pose2d currentPose){
        Vector velocity = new Vector(desiredPose.getX() - currentPose.getX(), desiredPose.getY() - currentPose.getY());
        Vector normalVelocity = velocity.normalize();
        double multiplier = distance/DrivetrainConstants.multiplierConstant;
        return normalVelocity.multiplyVector(multiplier);
    }

    /**
     * 
     * @param desiredPose the position the robot should be in
     * @param currentPose the position the robot is currently in
     * @return returns the needed velocity as a double
     * @author Giahna C.
     */
    public double calculateVelocityRot(Pose2d desiredPose, Pose2d currentPose){
        return MathUtil.clamp(getDifferenceFromAngle(),-1,1)/DrivetrainConstants.rotMultiplierConstant;
    }
}


