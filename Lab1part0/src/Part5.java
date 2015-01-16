import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.LightDetector;


public class Part5 
{
	public static void main(String[] args)
	{
		
	}
	
	public class Braitenberg
	{
		private EncoderMotor motorL;
		private EncoderMotor motorR;
		private EV3ColorSensor sensor;
		
		public Braitenberg()
		{
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);
			sensor = new EV3ColorSensor(SensorPort.S1);
		}
		
		public void doTheThing()
		{
			float[] sample = new float[sensor.sampleSize()];
			sensor.fetchSample(sample, 0);
			
			
		}
	}
}
