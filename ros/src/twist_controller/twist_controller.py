
from pid import PID
from yaw_controller import YawController
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self):
        # TODO: Implement
        self.Acc_controller = PID(0.7,0.01,0.3,mn=0.1,mx=1.0)
        
	self.yaw_controller = YawController()
        

    def control(self,proposed_linear_velocity,proposed_angular_velocity,current_linear_velocity,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        ###try implimenting the  linear pid controller

        # 1. find the error
        Linear_vel_error = 20 - current_linear_velocity[0]
        #print ('this is the linear error ',Linear_vel_error)
        Acceleration = self.Acc_controller.step(Linear_vel_error,1.0/50.0)
	if dbw_enabled !=True:
		Acceleration = 0.0
		Linear_vel_error = 0.0
		self.Acc_controller.reset()
	print ('this is the linear error ',Linear_vel_error)

        # +1.0 left 0.0 -1.0 right
        
        #print ('this is the Angular error ',Angular_vel_error)
        #steering = self.steer_controller.step(Angular_vel_error,1.0/50.0)# add a constant to center the steering 
        #print ('this is y' , steering)
        #steering = self.translate(steering, 0.0, 1.0, -1.0, 1.0)
	steering = self.yaw_controller.get_steering(proposed_linear_velocity[0], 
						    proposed_angular_velocity[2],
						    current_linear_velocity[0])
        

        return Acceleration, 0.,steering

    def reset(self):
	    self.Acc_controller.reset()
	    self.steer_controller.reset()


    def translate(self,value, ipMin, ipMax, opMin, opMax):
	    # Figure out how 'wide' each range is
	    leftSpan = ipMax - ipMin
	    rightSpan = opMax - opMin 

	    # Convert the left range into a 0-1 range (float)
	    valueScaled = float(value - ipMin) / float(leftSpan)

	    # Convert the 0-1 range into a value in the right range.
	    return opMin + (valueScaled * rightSpan)

