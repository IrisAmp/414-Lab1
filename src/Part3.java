import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.EncoderMotor;


public class Part3
{
	public static void main(String[] args)
	{
		DeadReckoningRobot robot = new DeadReckoningRobot();
		
		// Do something.
	}
	
	public static class DeadReckoningRobot
	{
		EncoderMotor motorL;
		EncoderMotor motorR;
		
		public DeadReckoningRobot()
		{
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);
		}
	}
}
