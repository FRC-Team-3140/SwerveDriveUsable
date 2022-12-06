// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class SwerveModule extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANCoder cANCoder;
private CANSparkMax driveSparkMax;
private CANSparkMax turnSparkMax;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private RelativeEncoder drive_encoder;
    private RelativeEncoder turn_encoder;

    private double maxTurnSpeed;
    private double turnIntegratorRange = 0.01;
    private boolean activated = true;
    private double turnVel = 0.0;

    private PIDController angle_pid;

    /**
    *
    */
    public SwerveModule() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
cANCoder = new CANCoder(0);
 
 

driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
 
 driveSparkMax.restoreFactoryDefaults();  
driveSparkMax.setInverted(false);
driveSparkMax.setIdleMode(IdleMode.kCoast);
driveSparkMax.burnFlash();
  

turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
 
 turnSparkMax.restoreFactoryDefaults();  
turnSparkMax.setInverted(false);
turnSparkMax.setIdleMode(IdleMode.kCoast);
turnSparkMax.burnFlash();
  


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drive_encoder = driveSparkMax.getEncoder();

        turn_encoder = turnSparkMax.getEncoder();
        maxTurnSpeed = 0.3;

        angle_pid = new PIDController(0.007500, 0.0200, 0.0000);
        angle_pid.enableContinuousInput(0.0, 360);
        //angle_pid.disableContinuousInput();
        angle_pid.setTolerance(5.0,5.0);
        angle_pid.setIntegratorRange(-turnIntegratorRange, turnIntegratorRange);


    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("cancoder_abs_position", this::getCancoderAbsPosition, null);
        //builder.addDoubleProperty("cancoder_id", this::getCancoderDeviceId, null);
        //builder.addDoubleProperty("cancoder_position", this::getCancoderPosition, null);
        //builder.addDoubleProperty("cancoder_velocity", this::getCancoderVelocity, null);
        builder.addDoubleProperty("turn_value", this::getTurnMotorValue, null);
        builder.addDoubleProperty("drive_value", this::getDriveMotorValue, null);
        //builder.addDoubleProperty("turn_position", this::getTurnPosition, null);
        //builder.addDoubleProperty("drive_position", this::getDrivePosition, null);
        builder.addDoubleProperty("turn_velocity", this::getTurnVelocity, null);
        builder.addDoubleProperty("drive_velocity", this::getDriveVelocity, null);

        builder.addDoubleProperty("maxTurnSpeed", this::getMaxTurnSpeed, this::setMaxTurnSpeed);
        builder.addBooleanProperty("activated", this::getActivated, this::setActivated);
        builder.addDoubleProperty("turnAngle", this::getTurnAngle, this::setTurnAngle);
        builder.addDoubleProperty("turnP", this::getTurnP, null);
        builder.addDoubleProperty("turnI", this::getTurnI, null);
        builder.addDoubleProperty("turnD", this::getTurnD, null);
        builder.addDoubleProperty("turnIRange", this::getTurnIRange, this::setTurnIRange);
        builder.addDoubleProperty("turnVel", this::getTurnVel, null);


    }

    public boolean getActivated(){
        return activated;
    }

    public void setActivated(boolean value){
        activated = value;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //if(!activated){
        //    setTurnSpeed(0.0);
        //    setDriveSpeed(0.0);
        //    return;
        //}
            turnVel = 0;
        turnVel = angle_pid.calculate(getCancoderAbsPosition());
        //if(angle_pid.atSetpoint()){
        //    setTurnSpeed(0.0);
        //}
        //else{
        setTurnSpeed(-turnVel);
        //}
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setMaxTurnSpeed(double value){
        maxTurnSpeed = value;
    }

    public double getMaxTurnSpeed(){
        return maxTurnSpeed;
    }

    public void setTurnAngle(double value){
        angle_pid.setSetpoint(value);
    }

    public double getTurnAngle(){
        return angle_pid.getSetpoint();
    }

    public void setTurnP(double value){
        angle_pid.setP(value);
    }

    public double getTurnP(){
        return angle_pid.getP();
    }

    public void setTurnI(double value){
        angle_pid.setI(value);
    }


    public double getTurnI(){
        return angle_pid.getI();
    }

    public void setTurnD(double value){
        angle_pid.setD(value);
    }

    public double getTurnD(){
        return angle_pid.getP();
    }

    public void setTurnIRange(double value){
        turnIntegratorRange = value;
        angle_pid.setIntegratorRange(-turnIntegratorRange,turnIntegratorRange);
    }

    public double getTurnIRange(){
        return turnIntegratorRange;
    }

    public void setTurnSpeed(double velocity){
        if(velocity > maxTurnSpeed){
            velocity = maxTurnSpeed;
        } 
        if(velocity < -maxTurnSpeed){
            velocity = -maxTurnSpeed;
        }
        turnSparkMax.set(velocity);
    }

    public void setDriveSpeed(double velocity){
        driveSparkMax.set(velocity);
    }

    double getCancoderAbsPosition() {
        return cANCoder.getAbsolutePosition();
    }

    int getCancoderDeviceId() {
        return cANCoder.getDeviceID();
    }

    double getCancoderPosition() {
        return cANCoder.getPosition();
    }

    double getCancoderVelocity() {
        return cANCoder.getVelocity();
    }

    double getTurnMotorValue() {
        return turnSparkMax.get();
    }

    double getDriveMotorValue() {
        return driveSparkMax.get();
    }

    double getTurnVel(){
        return turnVel;
    }

    double getDrivePosition() {
        return drive_encoder.getPosition();
    }

    double getDriveVelocity() {
        return drive_encoder.getVelocity();
    }

    double getTurnPosition() {
        return turn_encoder.getPosition();
    }

    double getTurnVelocity() {
        return turn_encoder.getVelocity();
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
