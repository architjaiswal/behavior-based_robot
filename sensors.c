// Find a goal (empty soft drink can) in the unknown environment where walls are marked with blue tape and goal is sitting on a red tape

//-----------------------------------------------------------------------------------------------------------------------
// Hardware Configuration: LEGO EV3 Mindstorm
// PORTB: right long motor
// PORTC: left long motor
// PORT2: Ultrasonic sensor
// PORT3: Colour Sensor
//-----------------------------------------------------------------------------------------------------------------------


#include <ev3.h>
#include <ev3_output.h>
#include <ev3_lcd.h>
#include <ev3_button.h>
#include <ev3_command.h>
#include <ev3_sensor.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int last  = 0;  	// For turning by 90-degrees during the wall following function
int second = 0; 	// For turning by 90-degrees during the wall following function
time_t interval;    // Used to count 15 seconds for triggering LookForGoal() in wallFollowing()


void RotateClockWiseBy11() // robot will rotate in clockwise direction by approximately 11 degrees on the same place
{
	OnFwdReg(OUT_C, 19);
	OnRevReg(OUT_B, 19);
	Wait(155);
	Off(OUT_BC);
}


// Robot will rotate 360 degrees and scan the surroundings using an ultrasonic sensor
int LookForTarget()
{
	int Slots = 50;       // number derived after several experiments
	int distance[Slots];
	int angle = 0;
	int min;
	int i;

	for (i = 0; i < Slots; i++)
	{
		RotateClockWiseBy11();
		distance[i] = readSensor(IN_2);
	}

	min = distance[0];

	for (i = 0; i < Slots; i++)
	{
		if (min >= distance[i])
		{
			angle = i;
			min = distance[i];
		}
	}

	// turn the robot in the direction of the goal and let it move forward
	if (min >= 0 && min <= 65)
	{

		for (i = 0; i <angle; i++ )
		{
			RotateClockWiseBy11();
		}
		return angle;
	}
	else
	{
		return -1;
	}

}


// Play the siren and hit the goal
 int clearing()
 {
	 PlayTone(TONE_C2, 500);
	 Wait(1000);
	 MoveFwdByOneTile();
	 MoveFwdByOneTile();
	 exit(0);
 }


// This function will be called within wall Following
// It will either return 0 (go back to wall following) Or 1 (object has been cleared)
int goalFinding()
{

	int lightSensor_value = readSensor(IN_3) & 127;
	int distance = readSensor(IN_2);

	while (lightSensor_value >= 20)
	{

		//Need to read color sensor and distance
		//As long as the reflect value stays above 35 and distance less than 45 cm , scan 360 degrees and look for the target
		//Distance is the value returned from ultrasonic, the number is in cm unit and the REFLECT value should be greater than 20 for the free space

		if(distance <= 45)
		{
			clearing();
		}

		else
		{
			OnFwdSync(OUT_BC, 50);
			Wait(50);
			Off(OUT_BC);
		}

		lightSensor_value = readSensor(IN_3) & 127;
		distance = readSensor(IN_2);
	}

		return 0;

}


// Let the robot move freely until a wall is found
void Wander()
{
	int ligthSensor_value = readSensor(IN_3) & 127;


	// light sensor value is less than 20 as it approaches to the wall
	while(ligthSensor_value >= 20)
	{
		OnFwdSync(OUT_BC, 15);
		Wait(50);
		ligthSensor_value = readSensor(IN_3) & 127;
	}

	return;
}



void WallFollowing() // Only controls the wall following commands and behavior
{

	interval = time(NULL);

	while(ButtonIsUp(BTNEXIT))
	{
		int ligthSensor_value = readSensor(IN_3) & 127; // only taking lower 8-bits of the sensor value since its REFLECT value output range is 0 - 100

		LcdClean();
		LcdPrintf(1,"Light Sensor = %d \n", ligthSensor_value);

		if( (ligthSensor_value >= 10) && (ligthSensor_value <= 20)) // perfect location to keep moving forward
		{
			OnFwdSync(OUT_BC, 80);
			Wait(50);
			Off(OUT_BC);
			ligthSensor_value = readSensor(IN_3) & 127;
		}
		else if(ligthSensor_value > 20) // OUTSIDE tape
		{

			while ( !(( ligthSensor_value >= 10) && (ligthSensor_value <= 20)) && ButtonIsUp(BTNEXIT) )
			{

				OnFwdReg(OUT_B, 30);
				Wait(100);
				Off(OUT_ALL);
				ligthSensor_value = readSensor(IN_3) & 127;
				if (ligthSensor_value < 10)
				{
					second = ligthSensor_value;
					break;
				}
			}

			Off(OUT_ALL);

		}
		else if( (ligthSensor_value >= 0) && (ligthSensor_value < 10) ) // ON tape
		{
			if ( ((last >= 0) && (last < 10)) && ((ligthSensor_value >= 0) && (ligthSensor_value < 10)) )
			{
				OnRevReg(OUT_B, 30);
				OnFwdReg(OUT_C, 30);
				Wait(50);
				Off(OUT_BC);

				while ( !(( ligthSensor_value >= 10) && (ligthSensor_value <= 20)) && ButtonIsUp(BTNEXIT) )
				{

					OnFwdReg(OUT_C, 30);
					OnRevReg(OUT_B, 30);
					Wait(100);
					Off(OUT_ALL);
					ligthSensor_value = readSensor(IN_3) & 127;

					if(ligthSensor_value > 20)
					{
						second = ligthSensor_value;

						break;
					}
				}

			}

			//Turn away from the wall
			//These need to be small turns
			OnRevReg(OUT_B, 30);
			OnFwdReg(OUT_C, 30);
			Wait(50);
			Off(OUT_BC);

			while ( !(( ligthSensor_value >= 10) && (ligthSensor_value <= 20)) && ButtonIsUp(BTNEXIT) )
			{

				OnFwdReg(OUT_C, 30);
				OnRevReg(OUT_B, 30);
				Wait(100);
				Off(OUT_ALL);
				ligthSensor_value = readSensor(IN_3) & 127;

				if(ligthSensor_value > 20)
				{
					second = ligthSensor_value;

					break;
				}
			}

			Off(OUT_ALL);

		}

		last = second;
		second = ligthSensor_value;

		if(time(NULL) - interval >= 15)
		{
			if (LookForTarget() != -1)
			{
				goalFinding();
			}
			interval = time(NULL);

		}

		if (ButtonIsDown(BTNEXIT))
		{
			exit(1);
		}
		
	}
}


// Initializes the robot
void StartRobot()
{
	InitEV3();
	ResetRotationCount(OUT_ALL);
	Off(OUT_ALL);

	LcdClean(); 				 // Erase everything on the display
	TermPrintf("Team 8 rocks!!\n");
	Wait(1000);
}


int main(void)
{
	StartRobot();

	setAllSensorMode(NO_SEN, US_DIST_CM, COL_REFLECT, NO_SEN);

	Wander();
	WallFollowing();

	return 0;
}
