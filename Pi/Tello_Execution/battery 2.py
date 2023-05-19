from djitellopy import Tello

# create a Tello object
tello = Tello()

# connect to the Tello
tello.connect()

# get the current battery level
battery_level = tello.get_battery()

# print the battery level
print(f"Battery level: {battery_level}%")

# disconnect from the Tello
tello.disconnect()
