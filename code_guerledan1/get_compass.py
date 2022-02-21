from imu9_driver_v2 import Imu9IO



imu = Imu9IO()


print("\n\n--------------")
print("IMU OK")
print("--------------\n\n")

ask_new_val = True

input("Press Enter to continue...")

while ask_new_val == True :
    print (imu.read_mag_raw())
    a=input("Press Enter to continue...('q' and Enter to quit)\n\n")
    if a=='q':
        ask_new_val = False
