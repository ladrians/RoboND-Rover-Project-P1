import numpy as np

# Added a bias to the default steering angle
def get_biased_steer_angle(nav_angles, min, max, bias):
    return get_steer_angle(nav_angles, min, max)+bias

# Get steering angle within expected max and min values
def get_steer_angle(nav_angles, min, max):
    try:
        return np.clip(np.mean(nav_angles * 180 / np.pi), min, max)
    except:
        return 0.

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    if Rover.nav_angles is not None:

        if Rover.at_home: # Record Home
            Rover.start_position = Rover.pos
            Rover.at_home = False
        if Rover.samples_collected == 6: # TODO: Should go Home
            pass

        if Rover.sample_detected: # Try collecting a rock
            if Rover.near_sample:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
            else:
                #print("detection Rover.vel", Rover.vel)
                if 0 < Rover.vel < Rover.max_vel - 1.0:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                elif Rover.vel <= 0:
                    if Rover.throttle == 0:
                        Rover.throttle = 2.0
                        Rover.steer = get_biased_steer_angle(Rover.nav_angles, -10, 10, 8)
                    else:
                        Rover.throttle = 0
                        Rover.steer = -15
                        Rover.mode = 'stop'
                else:  # try to decelerate
                    Rover.throttle = -0.1
                Rover.brake = 0
                Rover.steer = get_steer_angle(Rover.nav_angles, -5, 15) # Bias to the right
            Rover.sample_detected = False

        elif Rover.mode == 'forward':
            #print("forward Rover.vel", Rover.vel)
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    # If the rover is stuck
                    if abs(Rover.vel) < 0.05:
                        # Wait a little
                        Rover.vel_count += 1
                        if Rover.vel_count > 400:
                            Rover.vel_count = 0
                            Rover.stuck_count = 0
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = 0
                            Rover.mode = 'stuck' # change mode
                else:
                    Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = get_steer_angle(Rover.nav_angles, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            #print("stop Rover.vel", Rover.vel)
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Just turn right
                    #print("turning -15 (right)")
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = get_steer_angle(Rover.nav_angles, -15, 15)
                    Rover.mode = 'forward'
        # This is the Rover stuck mode - that is if the wheels are caught on the terrain        
        elif Rover.mode == 'stuck':
            #print("stuck Rover.vel", Rover.vel)
            Rover.stuck_count += 1 # accumulate the count
            Rover.steer = 0
            Rover.throttle = -0.1 # slowly reverse mode
            Rover.brake = 0
            
            if Rover.stuck_count > 300: # end of the stuck mode
                # Reset counters and change mode
                Rover.stuck_count = 0
                Rover.steer = 0
                Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.sample_detected = False
        Rover.mode = 'forward'
    
    return Rover

