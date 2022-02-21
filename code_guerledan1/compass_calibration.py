from imu9_driver_v2 import Imu9IO
import numpy as np
imu = Imu9IO()

ask_new_val = True

# with open('log.txt', 'w') as log:
#     while ask_new_val == True :
#         val = 0
#         for i in range(3):
#             print(valeur 1 :)
#             val += imu.read_mag_raw()
#         log.write(str(val))
#         log.write("\n")
#         print (val)
#         a=input("Press Enter to continue...('q' and Enter to quit)\n")
#         if a=='q':
#             ask_new_val = False
#

with open('log.txt', 'w') as log:
    for i in range(4):
        val = np.array([[0,0,0]])
        for j in range(3):
            print(i+1,j+1)
            input("Press Enter to continue...\n")
            val += np.array(imu.read_mag_raw())
        log.write(str(val/3))
        log.write("\n")
