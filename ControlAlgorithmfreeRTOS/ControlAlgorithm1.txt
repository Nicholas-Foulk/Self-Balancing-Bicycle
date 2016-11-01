if(fabs(result) > 1000)
		{
			while(accY > 3600)
			{
				//If the bicycle is tipping to the right, we want the motor to constantly spin counterclockwise for
				//counter weight until we reach our limit
				//the reason we have this if loop is because we only want the stepper motor to take 100 microsteps
				if(steppermotor < 50) //this limit is arbitrary
				{
					++steppermotor;
					stepperTurnF(2, 0, 2, 11, 1);
					ACC_Data[3] = SSPReceive(0x2B);
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
//					printf("steppermotor: %d\n", steppermotor);
//					printf("accX > %d ", accX);
//					printf("accY > %d ", accY);
//					printf("accZ > %d\n", accZ);
				}
				else
				{
					//We still want the data from the accelerometer to print out
					ACC_Data[3] = SSPReceive(0x2B);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
//					printf("steppermotor: %d\n", steppermotor);
//					printf("accX > %d ", accX);
//					printf("accY > %d ", accY);
//					printf("accZ > %d\n", accZ);
				}
			}
			//The reason we don't reset steppermotor variable is because if the pendulum system is tipping left,
			//we have to go more than 100 steps to hit out limit on the left.
			while(accY < -900)
			{
				//If the bicycle is tipping to the left, we want the motor to constantly spin clockwise for counter weight
				//until we reach our limit
				//the reason we have this if loop is because we only want the stepper motor to take 100 microsteps
				if(steppermotor > -50)
				{
					--steppermotor;
					stepperTurnR(2, 0, 2, 11, 1);
					ACC_Data[3] = SSPReceive(0x2B); //We are taking data from the SSP setup accelerometer and putting it into data
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
//					printf("steppermotor: %d\n", steppermotor);
//					printf("accX > %d ", accX); //We decided to print here to make sure the accelerometer is still working.
//					printf("accY > %d ", accY);
//					printf("accZ > %d\n", accZ);
				}
				else
				{
					//We still want the data from the accelerometer to print out
					ACC_Data[3] = SSPReceive(0x2B);
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
//					printf("steppermotor: %d\n", steppermotor);
//					printf("accX > %d ", accX);
//					printf("accY > %d ", accY);
//					printf("accZ > %d\n", accZ);
				}
			}
			while((accY >= -900) &&  (accY <= 3600) && (steppermotor != 0))
			{
				//If the bicycle is not tipping at all, we want to move back to being balanced.
				//the reason we have this if loop is because we only want the stepper motor to take 100 microsteps
				if(steppermotor > 0)
				{
					--steppermotor;
					stepperTurnR(2, 0, 2, 11, 1);
					ACC_Data[3] = SSPReceive(0x2B); //We are taking data from the SSP setup accelerometer and putting it into data
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
//					printf("steppermotor: %d\n", steppermotor);
//					printf("accX > %d ", accX); //We decided to print here to make sure the accelerometer is still working.
//					printf("accY > %d ", accY);
//					printf("accZ > %d\n", accZ);
				}
				else if(steppermotor < 0)
				{
					++steppermotor;
					stepperTurnF(2, 0, 2, 11, 1);
					ACC_Data[3] = SSPReceive(0x2B);
					ACC_Data[2] = SSPReceive(0x2A);
					accY = (int)(ACC_Data[3] << 8) | ACC_Data[2];
//					printf("steppermotor: %d\n", steppermotor);
//					printf("accX > %d ", accX);
//					printf("accY > %d ", accY);
//					printf("accZ > %d\n", accZ);
				}
			}
		}