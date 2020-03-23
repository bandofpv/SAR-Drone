print ("Start simulator (SITL)")
import dronekit_sitl
import time

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:")
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print ("Attitude: %s" % vehicle.attitude)
print ("Velocity: %s" % vehicle.velocity)
print ("GPS: %s" % vehicle.gps_0)
print ("Groundspeed: %s" % vehicle.groundspeed)
print ("Airspeed: %s" % vehicle.airspeed)
print ("Battery: %s" % vehicle.battery)
print ("Heading: %s" % vehicle.heading)
print ("Is Armable?: %s" % vehicle.is_armable)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name)
print ("Armed: %s" % vehicle.armed)

# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print (" Waiting for home location ...")

# We have a home location.
print ("\n Home location: %s" % vehicle.home_location)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
    print (" Getting ready to take off ...")
    time.sleep(1)


# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")