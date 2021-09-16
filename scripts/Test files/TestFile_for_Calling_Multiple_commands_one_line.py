
from PID import PID, DronePID 

drone_1 = DronePID(minOutput = -100, maxOutput = 100, derivativeFilterFreq=15,
                     PID_X = [200,0,150], PID_Y = [400,0,170], PID_Z = [300,0,130], current_time = None)

#drone_1.UPDATE([1,1,1], [1,1,2])
drone_2 = DronePID(minOutput = -100, maxOutput = 100, derivativeFilterFreq=15,
                     PID_X = [200,0,150], PID_Y = [400,0,170], PID_Z = [300,0,130], current_time = None)

# For Drone 1
present = [-20,1,-2]
goal = [2,1,-2]

# For Drone 2
present_2 = [2,10,-2]
goal_2 = [2,1,-2]
print("For Drone 1")
controls = drone_1.UPDATE(present, goal) 
print(controls)
print("For Drone 2")
print(drone_1.UPDATE(present_2, goal_2))

print("-------------------------------------------")
print("Actual Update in PID Class")

###################################################################################################################
# How we were Calling Earlier

drone1PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)


x = drone1PIDx.update([present[0]], [goal[0]])
y = drone1PIDy.update([present[1]], [goal[1]])
z = drone1PIDz.update([present[2]], [goal[2]])
print("For Drone 1")
print(x,y,z)

# =========

drone2PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone2PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=20,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

x_2 = drone2PIDx.update([present_2[0]], [goal_2[0]])
y_2 = drone2PIDy.update([present_2[1]], [goal_2[1]])
z_2 = drone2PIDz.update([present_2[2]], [goal_2[2]])
print("For Drone 2")
print(x_2,y_2,z_2)
