import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])
p.setGravity(0, 0, -20)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
#objectId = p.loadURDF("random_urdfs/083/083.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos = 1
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])

object_path ="random_urdfs/175/175.urdf"
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])


######################################################### Load tray Here!!!!#############################################################################

tray_x = 0.95
tray_y = 0.3



trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])



###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]

#hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
# [0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 
# 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 
# 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 
# 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])


######################################################### detected information ##########################################################################
'''
19 b'palm'
20 b'rb1'
21 b'finger1.1a'
22 b'finger1.1b'
23 b'finger1.1c'
24 b'fingertip1'
25 b'rb2'
26 b'finger1.2a'
27 b'finger1.2b'
28 b'finger1.2c'
29 b'rb3'
30 b'finger2.1a'
31 b'finger2.1b'
32 b'finger2.1c'
33 b'fingertip2'
34 b'rb4'
35 b'finger2.2a'
36 b'finger2.2b'
37 b'finger2.2c'
38 b'rb5'
39 b'finger3.1a'
40 b'finger3.1b'
41 b'finger3.1c'
42 b'fingertip3'
43 b'rb6'
44 b'finger3.2a'
45 b'finger3.2b'
46 b'finger3.2c'
47 b'rb7'
48 b'finger4.1a'
49 b'finger4.1b'
50 b'finger4.1c'
51 b'fingertip4'
52 b'rb8'
53 b'finger4.2a'
54 b'finger4.2b'
55 b'finger4.2c'
56 b'rb9'
57 b'rb10'
58 b'thumb1'
59 b'thumb2'
60 b'rb11'
61 b'thumb3.1'
62 b'thumbtip'
63 b'rb12'
64 b'thumb3.2'
hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
[0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]
'''
def info():
    palmContact = []
    thumbContact = []
    indexContact = []
    midContact = []
    ringContact = []
    pinkyContact = []
    palmLinks = [19, 20, 25, 29, 34, 38, 43, 47, 52, 56, 57]
    thumbLinks = [58, 59, 60, 61, 62, 63, 64]
    indexLinks = [48, 49, 50, 51, 53, 54, 55]
    middleLinks = [39, 40, 41, 42, 44, 45, 46]
    ringLinks = [30, 31, 32, 33, 35, 36, 37]
    pinkyLinks = [21, 22, 23, 24, 26, 27, 28]

    contact = p.getContactPoints(sawyerId, objectId)  # pubullet quick guide
    nums = len(contact)
    if (nums == 0):
        print("There are no contact points")
        return [], [], [], [], [], []
    for i in range(nums):
        temp = []
        if (contact[i][3] in palmLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            palmContact.append(temp)

        if (contact[i][3] in thumbLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            thumbContact.append(temp)

        if (contact[i][3] in indexLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            indexContact.append(temp)

        if (contact[i][3] in middleLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            midContact.append(temp)

        if (contact[i][3] in ringLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            ringContact.append(temp)

        if (contact[i][3] in pinkyLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            pinkyContact.append(temp)

    return palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact
######################################################### Simulation ##########################################################################
currentP = [0] * 65
k = 0

##################################################################### Input Value Here############################################################

#handInitial =
grasp_orientation = [1.573832769393921, 2.890657767453466, 0.5493183326721191]

grasp_palmPosition = [0.87, 0.09956837789155543, -0.14557867154479026]

handClose = [0.2422690080999104, 0.9444851009895664, 0.5030800374659657,
0.31285218263202195, 0.9731120768050122, 0.6742382160148981, 0.583375044315802, 0.5842577849323328,
0.29474036912766877, 0.5975214466752685, 0.5856938599356889, 0.1700047575816217, 0.9349378953438807,
1.414292683409878, 0.1700017988186381, 0.9352728140808059, 1.4156440098784877, 0.1699999999996, 
0.9058133871001239, 1.5694870229576992, 0.199969276558996, 0.9054163522456206, 1.570000322829452,
0.17006153425493656, 1.5676822635012848, 0.6232114935242119, 0.61011872546111]

pu_palmPosition = [0.85, 0.06956837789155543, 0.04557867154479026]

pu_orientation = [1.563832769393921, 2.8720657767453466, 0.5283183326721191]

final_palmPosition = [0.85, 0.42956837789155543, 0.04557867154479026]

final_orientation = [1.563832769393921, 2.8720657767453466, 0.5283183326721191]

handOpen = [0.23415813347966705, 0.9670352906874953, 0.1699967368611223, 0.2331431971216663, 0.9739927886985636,
0.16999974215320662, 0.5812942348429904, 0.5834941139972684, 0.17689542090304175, 0.5783157598547679, 0.5856246576786355,
0.17003810233685732, 0.17147425787559695, 0.2678029535516616, 0.17001225737141265, 0.17026460556008577, 0.26926118525833936,
0.16999999999999996, 0.1702982843054628, 0.17884454038902028, 0.1699998796566938, 0.17009497880540192, 0.17042690109918343,
0.17003575992710027, 0.8499999549650139, 0.34007728523358827, 0.3398998920037342]

##################################################################################################################################################################################################
initial_palmPosition = [0.95, 0.0, 0.2]
initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

initial_palmPosition = [initial_palmPosition[0]-0.1,initial_palmPosition[1]-0.05,initial_palmPosition[2]]
##################################################################################################################################################################################################

# write the code for step 9-12

k = 1
while 1:
    k = k + 1
# move palm to target postion
    i = 0
    while 1:
       i += 1
#p.stepSimulation()
       currentP = palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]],
       p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
       time.sleep(0.03)
       p.stepSimulation()
   
       if (i == 100):
          print('Reached Initial Position')
          break
    i = 0
    while 1:
       i += 1

#p.stepSimulation()
       currentP = palmP([grasp_palmPosition[0], grasp_palmPosition[1], grasp_palmPosition[2]],
       p.getQuaternionFromEuler([grasp_orientation [0], grasp_orientation[1], grasp_orientation[2]]))
       time.sleep(0.03)
       p.stepSimulation()
       if (i == 100):
           print('Reached Grasp position')
           break

    i = 0 
    while 1:
       i += 1
       thumb (handClose [24], handClose [25]) 
       indexF (handClose [18], handClose [19]) 
       midF (handClose [12], handClose[13]) 
       p.stepSimulation() 
       if i == 300:
          print("Object Grasped")
          break
    i = 0
    while 1:
      i += 1
#p.stepSimulation()
      currentP = palmP([pu_palmPosition[0], pu_palmPosition [1], pu_palmPosition[2]],
      p.getQuaternionFromEuler([pu_orientation[0], pu_orientation [1], pu_orientation [2]]))
      time.sleep(0.03)
      p.stepSimulation()
      if (i == 100):
         print('Reached Up Position')
         break
    i = 0
    while 1:
      i += 1
# p.stepSimulation()
      currentP = palmP([final_palmPosition[0], final_palmPosition[1], final_palmPosition [2]],
      p.getQuaternionFromEuler([final_orientation [0], final_orientation [1], final_orientation [2]]))
      time.sleep(0.03)
      p.stepSimulation()
      if (i == 100):
          print('Reached Top of Tray ')
          break

    i = 0
    while 1:
       i += 1
       thumb (handOpen [24], handOpen [25])
       indexF (handOpen [18], handOpen[19]) 
       midF (handOpen [12], handOpen[13])
       p.stepSimulation()
       if i == 300:
          print("Object Released")
          break

p.disconnect()
print("disconnected")







