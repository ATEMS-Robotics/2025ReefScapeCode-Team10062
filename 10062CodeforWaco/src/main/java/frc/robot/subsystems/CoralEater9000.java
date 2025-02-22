package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.TalonFX; // Kraken X60 motors use TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*; // Needed for motor control modes

public class CoralEater9000 extends SubsystemBase {
    private final TalonFX armMotor; // Rotates the whole arm
    private final TalonFX intakeMotor; // Spins Omni wheels for intake

    private MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);
        private final MotionMagicConfigs motionConfigs = new MotionMagicConfigs();
    @SuppressWarnings("unused")
        private final PIDController pidController = new PIDController(0.3, 0.0, 0.0);
    
        private static final double EncoderTicksPerRevolution = 2048;
    
        public static final double Position_one = 5;
        public static final double Position_two = 10;
        public static final double Position_three = 15;
    
    
        public CoralEater9000() {
            System.out.println("SigmaNonchallanting Initializing Coral EATER");
            armMotor = new TalonFX(23);  // Replace with actual CAN ID
            intakeMotor = new TalonFX(22);  // Replace with actual CAN ID 
            armMotor.setNeutralMode(NeutralModeValue.Brake); 
    
            motionConfigs.MotionMagicCruiseVelocity = 30;
            motionConfigs.MotionMagicAcceleration = 15;
            armMotor.getConfigurator().apply(motionConfigs);
    
            armMotor.setControl(new MotionMagicDutyCycle(0));
            motionMagic = new MotionMagicDutyCycle(0);        
    }

    /** Moves the arm up/down. Positive = up, Negative = down */
    public void setArmSpeed(double speed) {
        armMotor.setControl(new MotionMagicDutyCycle(speed));    }

    public Command moveToPosition(double targetRotations) { 
        double currentPosition = armMotor.getPosition().getValueAsDouble();
        double targetPosition = targetRotations * EncoderTicksPerRevolution;

        System.out.println("Current Position: " + currentPosition + " Target Position: " + targetPosition);

        armMotor.setControl(motionMagic.withPosition(targetPosition));
        
        
    }

    /** Spins the intake wheels. Positive = intake, Negative = outtake */
    public void setIntakeSpeed(double speed) {
        intakeMotor.setControl(new DutyCycleOut(speed));
    }

    /** Stops all motors */
    public void stop() {
        armMotor.setControl(new MotionMagicDutyCycle(0));
        intakeMotor.setControl(new DutyCycleOut(0));
    }

    public double getCurrentPosition() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public void printEncoderValue(){
        double position = armMotor.getPosition().getValueAsDouble();
        double rotations = position / EncoderTicksPerRevolution;
        System.out.println("Arm Motor Encoder Value:" + position + " (" + rotations + "rotations" );
    }

    @Override
    public void periodic() {
        printEncoderValue();
    }
}
