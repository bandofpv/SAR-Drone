# USAGE
# python takeoff_test.py

# Connect to the Vehicle.
print("Connecting to vehicle on: serial0")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")

    # don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.mode.name == 'GUIDED':  # wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)

    print ("\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed)
    vehicle.armed = True
    while not vehicle.armed: # wait until copter is armed
        print (" Waiting for arming...")
        time.sleep(1)
    print (" Vehicle is armed: %s" % vehicle.armed)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # take off to target altitude

    # wait until the vehicle reaches a safe height before processing the goto (otherwise the command after
    # Vehicle.simple_takeoff will execute immediately).
    while True:
        print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        # break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

# take off to 10m
arm_and_takeoff(10)

print("Take off complete")

# hover for 10 seconds
time.sleep(10)

# land
print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# close vehicle object
vehicle.close()
