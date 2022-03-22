import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sympy import *
from pyquaternion import Quaternion
import time
import serial

class StewartPlatform:
    def __init__(   self,
                    BaseAngleOffset,
                    BaseRadius,
                    ActStroke,
                    ArmLen,
                    PlatformRadius,
                    PlatformAngleOffset,
                    EndEffectorPositionOffset
                ):

        # Conversion Constants
        self.D2R = np.pi / 180                                                              # Degrees to radians conversion
        self.R2D = 180 / np.pi                                                              # Radians to degrees conversion

        # Design Constants
        self.REF_SPACING = 120                                                              # Radial symetry of actuator spacing [deg]

        # Design Parameters
        self.BaseAngleOffset = BaseAngleOffset                                              # Actuator angle offset from symmetry [deg]
        self.BaseRadius = BaseRadius                                                        # Actuator radial spacing from base [m]
        self.ActStroke = ActStroke                                                          # Actuator stroke [m]
        self.ArmLen = ArmLen                                                                # Control arm length [m]
        self.PlatformRadius = PlatformRadius                                                # Platfrom radius [m]
        self.PlatformAngleOffset = PlatformAngleOffset                                      # Control arm radial offset from platform symetry [deg]
        self.EndEffectorPositionOffset = np.array(EndEffectorPositionOffset, dtype=np.float64)                            # End Effector Position from the platform [Coordinate Transformation]

        # Initial States
        self.PlatformQuaternion = np.array([1,0,0,0], dtype=np.float64)                     # Initial orientation of the platform [quaternion]
        self.EndEffectorOrientationOffset = np.array([1,0,0,0], dtype=np.float64)           # End Effector Rotation from the platform [quaternion]

        # Control Parameters
        self.MaxActuatorSpeed = None                                                        # The maximum speed of the actuator [m/s]
        self.MaxStepSize = None                                                             # The maximum distance between sucsessive position signals [m]
        self.SamplingPeriod = None                                                          # The sampling period for the controller [Hz]

        #Actuator angle relative to origion
        self.A1B_Angle = None
        self.A2B_Angle = None
        self.A3B_Angle = None
        self.A4B_Angle = None
        self.A5B_Angle = None
        self.A6B_Angle = None

        #Control arm angle on platform
        self.C1P_Angle = None
        self.C2P_Angle = None
        self.C3P_Angle = None
        self.C4P_Angle = None
        self.C5P_Angle = None
        self.C6P_Angle = None

        #Actuator position relative to base
        self.A1B = None
        self.A2B = None
        self.A3B = None
        self.A4B = None
        self.A5B = None
        self.A6B = None

        #Position of actuator and control arm connection
        self.A1C = None
        self.A2C = None
        self.A3C = None
        self.A4C = None
        self.A5C = None
        self.A6C = None

        #Control arm positions relative to platform center
        self.C1 = None
        self.C2 = None
        self.C3 = None
        self.C4 = None
        self.C5 = None
        self.C6 = None

        #Position of control arm and platform connection
        self.C1P = None
        self.C2P = None
        self.C3P = None
        self.C4P = None
        self.C5P = None
        self.C6P = None

        #Platform Position
        self.PlatformPosition = None

        #The home height
        self.HomeHeight = None


        try:
            SerialRate = 115200
            self.ser1 = serial.Serial('/dev/cu.usbmodem11103', SerialRate, timeout=1)
            self.ser2 = serial.Serial('/dev/cu.usbmodem11303', SerialRate, timeout=1)
            self.ser3 = serial.Serial('/dev/cu.usbmodem114303', SerialRate, timeout=1)
            self.ser4 = serial.Serial('/dev/cu.usbmodem114403', SerialRate, timeout=1)
            self.ser5 = serial.Serial('/dev/cu.usbmodem114203', SerialRate, timeout=1)
            self.ser6 = serial.Serial('/dev/cu.usbmodem11203', SerialRate, timeout=1)
        except Exception as e:
            print(e)

        # Setup the Stewart Platform
        self.setup()

    def setup(self):
        # Solve for the angle of the actuator on the base
        self.A1B_Angle = 0 * self.REF_SPACING + self.BaseAngleOffset
        self.A2B_Angle = 1 * self.REF_SPACING - self.BaseAngleOffset
        self.A3B_Angle = 1 * self.REF_SPACING + self.BaseAngleOffset
        self.A4B_Angle = 2 * self.REF_SPACING - self.BaseAngleOffset
        self.A5B_Angle = 2 * self.REF_SPACING + self.BaseAngleOffset
        self.A6B_Angle = 3 * self.REF_SPACING - self.BaseAngleOffset

        # Solve for the angle of the ball joint on the base
        self.C1P_Angle = 0 * self.REF_SPACING + self.PlatformAngleOffset
        self.C2P_Angle = 1 * self.REF_SPACING - self.PlatformAngleOffset
        self.C3P_Angle = 1 * self.REF_SPACING + self.PlatformAngleOffset
        self.C4P_Angle = 2 * self.REF_SPACING - self.PlatformAngleOffset
        self.C5P_Angle = 2 * self.REF_SPACING + self.PlatformAngleOffset
        self.C6P_Angle = 3 * self.REF_SPACING - self.PlatformAngleOffset

        # Compute the x, y and z location of the start of the actuators on the base
        self.A1B = np.array([self.BaseRadius * np.cos(self.A1B_Angle * self.D2R),self.BaseRadius * np.sin(self.A1B_Angle * self.D2R),0], dtype=np.float64)
        self.A2B = np.array([self.BaseRadius * np.cos(self.A2B_Angle * self.D2R),self.BaseRadius * np.sin(self.A2B_Angle * self.D2R),0], dtype=np.float64)
        self.A3B = np.array([self.BaseRadius * np.cos(self.A3B_Angle * self.D2R),self.BaseRadius * np.sin(self.A3B_Angle * self.D2R),0], dtype=np.float64)
        self.A4B = np.array([self.BaseRadius * np.cos(self.A4B_Angle * self.D2R),self.BaseRadius * np.sin(self.A4B_Angle * self.D2R),0], dtype=np.float64)
        self.A5B = np.array([self.BaseRadius * np.cos(self.A5B_Angle * self.D2R),self.BaseRadius * np.sin(self.A5B_Angle * self.D2R),0], dtype=np.float64)
        self.A6B = np.array([self.BaseRadius * np.cos(self.A6B_Angle * self.D2R),self.BaseRadius * np.sin(self.A6B_Angle * self.D2R),0], dtype=np.float64)

        # Compute the x, y and z location of the end of the actuators on the base
        self.A1C = np.array([self.BaseRadius * np.cos(self.A1B_Angle * self.D2R),self.BaseRadius * np.sin(self.A1B_Angle * self.D2R),0], dtype=np.float64)
        self.A2C = np.array([self.BaseRadius * np.cos(self.A2B_Angle * self.D2R),self.BaseRadius * np.sin(self.A2B_Angle * self.D2R),0], dtype=np.float64)
        self.A3C = np.array([self.BaseRadius * np.cos(self.A3B_Angle * self.D2R),self.BaseRadius * np.sin(self.A3B_Angle * self.D2R),0], dtype=np.float64)
        self.A4C = np.array([self.BaseRadius * np.cos(self.A4B_Angle * self.D2R),self.BaseRadius * np.sin(self.A4B_Angle * self.D2R),0], dtype=np.float64)
        self.A5C = np.array([self.BaseRadius * np.cos(self.A5B_Angle * self.D2R),self.BaseRadius * np.sin(self.A5B_Angle * self.D2R),0], dtype=np.float64)
        self.A6C = np.array([self.BaseRadius * np.cos(self.A6B_Angle * self.D2R),self.BaseRadius * np.sin(self.A6B_Angle * self.D2R),0], dtype=np.float64)

        # Compute the x, y, and z location of the control arm joints on the platform
        self.C1 = np.array([self.PlatformRadius * np.cos(self.C1P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C1P_Angle * self.D2R),0], dtype=np.float64)
        self.C2 = np.array([self.PlatformRadius * np.cos(self.C2P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C2P_Angle * self.D2R),0], dtype=np.float64)
        self.C3 = np.array([self.PlatformRadius * np.cos(self.C3P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C3P_Angle * self.D2R),0], dtype=np.float64)
        self.C4 = np.array([self.PlatformRadius * np.cos(self.C4P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C4P_Angle * self.D2R),0], dtype=np.float64)
        self.C5 = np.array([self.PlatformRadius * np.cos(self.C5P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C5P_Angle * self.D2R),0], dtype=np.float64)
        self.C6 = np.array([self.PlatformRadius * np.cos(self.C6P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C6P_Angle * self.D2R),0], dtype=np.float64)

        # Compute the x, y, and z location of the control arm joints on the platform in global space
        self.C1P = np.array([self.PlatformRadius * np.cos(self.C1P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C1P_Angle * self.D2R),np.nan], dtype=np.float64)
        self.C2P = np.array([self.PlatformRadius * np.cos(self.C2P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C2P_Angle * self.D2R),np.nan], dtype=np.float64)
        self.C3P = np.array([self.PlatformRadius * np.cos(self.C3P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C3P_Angle * self.D2R),np.nan], dtype=np.float64)
        self.C4P = np.array([self.PlatformRadius * np.cos(self.C4P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C4P_Angle * self.D2R),np.nan], dtype=np.float64)
        self.C5P = np.array([self.PlatformRadius * np.cos(self.C5P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C5P_Angle * self.D2R),np.nan], dtype=np.float64)
        self.C6P = np.array([self.PlatformRadius * np.cos(self.C6P_Angle * self.D2R),self.PlatformRadius * np.sin(self.C6P_Angle * self.D2R),np.nan], dtype=np.float64)


        if False:
            # Find the initial height of the platform if it is in its symetric position
            platZ = symbols('platZ')

            # Each of these solutions will produce the same result
            EQ1 = Eq(((self.A1B[0] - self.C1P[0]) ** 2 + (self.A1B[1] - self.C1P[1]) ** 2 + (0 - platZ) ** 2), ((self.ArmLen) ** 2))
            #EQ2 = Eq(((self.A2B[0] - self.C2P[0]) ** 2 + (self.A2B[1] - self.C2P[1]) ** 2 + (0 - platZ) ** 2), ((self.ArmLen) ** 2))
            #EQ3 = Eq(((self.A3B[0] - self.C3P[0]) ** 2 + (self.A3B[1] - self.C3P[1]) ** 2 + (0 - platZ) ** 2), ((self.ArmLen) ** 2))
            #EQ4 = Eq(((self.A4B[0] - self.C4P[0]) ** 2 + (self.A4B[1] - self.C4P[1]) ** 2 + (0 - platZ) ** 2), ((self.ArmLen) ** 2))
            #EQ5 = Eq(((self.A5B[0] - self.C5P[0]) ** 2 + (self.A5B[1] - self.C5P[1]) ** 2 + (0 - platZ) ** 2), ((self.ArmLen) ** 2))
            #EQ6 = Eq(((self.A6B[0] - self.C6P[0]) ** 2 + (self.A6B[1] - self.C6P[1]) ** 2 + (0 - platZ) ** 2), ((self.ArmLen) ** 2))

            # Solve the equation for the initial conditions
            platZ_sol = solve(EQ1)

            # There should only be two solutions, take the positive one
            if (len(platZ_sol) == 1):
                PlatformHeight = float(platZ_sol[0])
            else:
                if platZ_sol[0] >= platZ_sol[1]:
                    PlatformHeight = platZ_sol[0]
                else:
                    PlatformHeight = platZ_sol[1]

        PlatformHeight = sqrt(self.ArmLen**2 - (self.A1B[0] - self.C1P[0])**2 - (self.A1B[1] - self.C1P[1])**2)

        # Update the previously unknown heights
        self.C1P[2] = PlatformHeight
        self.C2P[2] = PlatformHeight
        self.C3P[2] = PlatformHeight
        self.C4P[2] = PlatformHeight
        self.C5P[2] = PlatformHeight
        self.C6P[2] = PlatformHeight

        # Create a coordinate reference for the platform
        self.PlatformPosition = np.array([0,0,PlatformHeight], dtype=np.float64)
        self.HomeHeight = PlatformHeight

        #End Effector Position and Orientation
        self.EndEffectorPosition = self.PlatformPosition + self.EndEffectorPositionOffset
        self.EndEffectorOrientation = Quaternion(np.array([1, 0, 0, 0], dtype=np.float64))

        # Return None
        return None

    def inverseKinematicSolution(self, EndEffectorPosition, EndEffectorQuaternion):
        '''
        Return the actuator positions given the end effector position
        '''

        # Compute the new position and orientation of the platform
        NewPlatformPosition = EndEffectorPosition - Quaternion(EndEffectorQuaternion).rotate(self.EndEffectorPositionOffset)

        # Compute the joint positions on the platform
        C1P = NewPlatformPosition + Quaternion(EndEffectorQuaternion).rotate(self.C1)
        C2P = NewPlatformPosition + Quaternion(EndEffectorQuaternion).rotate(self.C2)
        C3P = NewPlatformPosition + Quaternion(EndEffectorQuaternion).rotate(self.C3)
        C4P = NewPlatformPosition + Quaternion(EndEffectorQuaternion).rotate(self.C4)
        C5P = NewPlatformPosition + Quaternion(EndEffectorQuaternion).rotate(self.C5)
        C6P = NewPlatformPosition + Quaternion(EndEffectorQuaternion).rotate(self.C6)

        try:
            A1Zp = C1P[2] + np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A1B[0] - C1P[0]), 2) - np.power((self.A1B[1] - C1P[1]), 2))
            A1Zn = C1P[2] - np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A1B[0] - C1P[0]), 2) - np.power((self.A1B[1] - C1P[1]), 2))

            A2Zp = C2P[2] + np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A2B[0] - C2P[0]), 2) - np.power((self.A2B[1] - C2P[1]), 2))
            A2Zn = C2P[2] - np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A2B[0] - C2P[0]), 2) - np.power((self.A2B[1] - C2P[1]), 2))

            A3Zp = C3P[2] + np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A3B[0] - C3P[0]), 2) - np.power((self.A3B[1] - C3P[1]), 2))
            A3Zn = C3P[2] - np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A3B[0] - C3P[0]), 2) - np.power((self.A3B[1] - C3P[1]), 2))

            A4Zp = C4P[2] + np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A4B[0] - C4P[0]), 2) - np.power((self.A4B[1] - C4P[1]), 2))
            A4Zn = C4P[2] - np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A4B[0] - C4P[0]), 2) - np.power((self.A4B[1] - C4P[1]), 2))

            A5Zp = C5P[2] + np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A5B[0] - C5P[0]), 2) - np.power((self.A5B[1] - C5P[1]), 2))
            A5Zn = C5P[2] - np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A5B[0] - C5P[0]), 2) - np.power((self.A5B[1] - C5P[1]), 2))

            A6Zp = C6P[2] + np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A6B[0] - C6P[0]), 2) - np.power((self.A6B[1] - C6P[1]), 2))
            A6Zn = C6P[2] - np.sqrt(np.power(self.ArmLen, 2) - np.power((self.A6B[0] - C6P[0]), 2) - np.power((self.A6B[1] - C6P[1]), 2))

            solutions = [   [A1Zp, A1Zn],
                            [A2Zp, A2Zn],
                            [A3Zp, A3Zn],
                            [A4Zp, A4Zn],
                            [A5Zp, A5Zn],
                            [A6Zp, A6Zn]
                            ]

        except Exception as e:
            print("Could not solve inverse kinemetaic solution: {}".format(e))
            return (None, None)

        prevActs = [self.A1C, self.A2C, self.A3C, self.A4C, self.A5C, self.A6C]
        output = []
        for actNum, solution in enumerate(solutions):
           # Use the solution whos distance was closest to the previous
            if (np.abs(solution[0] - prevActs[actNum][2]) <= np.abs(solution[1] - prevActs[actNum][2])):
                output.append(solution[0])
            else:
                output.append(solution[1])

        # Return the actuator positions and the platform joint positions
        return (tuple(output), (C1P, C2P, C3P, C4P, C5P, C6P))

    def home(self):
        try:
            self.ser1.write(self.encodeHeight(0))
            self.ser2.write(self.encodeHeight(0))
            self.ser3.write(self.encodeHeight(0))
            self.ser4.write(self.encodeHeight(0))
            self.ser5.write(self.encodeHeight(0))
            self.ser6.write(self.encodeHeight(0))
        except Exception as e:
            print("Could not home: {}".format(e))

    def topHome(self):
        try:
            self.ser1.write(self.encodeHeight(6))
            self.ser2.write(self.encodeHeight(6))
            self.ser3.write(self.encodeHeight(6))
            self.ser4.write(self.encodeHeight(6))
            self.ser5.write(self.encodeHeight(6))
            self.ser6.write(self.encodeHeight(6))
        except Exception as e:
            print("Could not home: {}".format(e))


    def forwardKinematicSolution(self, ActuatorPositions):
        '''
        Return the end effector position given the actuator positions
        ~~~
        '''
        return

    def instantRelativeMove(self, NewPlatformPosition, NewPlatformQuaternion):
        '''
        Instantly apply relative movement to the system positions
        '''
        GlobPos = NewPlatformPosition + self.PlatformPosition
        GlobOrient = Quaternion(NewPlatformQuaternion) * self.PlatformQuaternion;

        return self.IntantGlobalMove(GlobPos, GlobOrient);


    def instantGlobalMove(self, EndEffectorPosition, EndEffectorQuaternion):
        '''
        Instanty apply global movement to the system positions
        '''
        actuators, joints = self.inverseKinematicSolution(EndEffectorPosition, EndEffectorQuaternion)

        if (actuators == None) or (joints == None):
            return False
        else:
            self.A1C[2], self.A2C[2], self.A3C[2], self.A4C[2], self.A5C[2], self.A6C[2] = actuators
            self.C1P, self.C2P, self.C3P, self.C4P, self.C5P, self.C6P = joints
            self.EndEffectorPosition = EndEffectorPosition
            self.EndEffectorOrientation = EndEffectorQuaternion
            return True

    def instantHome(self, actuate=False):
        '''
        Instantly home the system positions
        '''
        pos = self.instantGlobalMove([0, 0, self.HomeHeight], [1, 0, 0, 0])

        if actuate:
            self.ser1.write(self.encodeHeight(self.A1C[2]))
            self.ser2.write(self.encodeHeight(self.A2C[2]))
            self.ser3.write(self.encodeHeight(self.A3C[2]))
            self.ser4.write(self.encodeHeight(self.A4C[2]))
            self.ser5.write(self.encodeHeight(self.A5C[2]))
            self.ser6.write(self.encodeHeight(self.A6C[2]))

        return pos

    def isValidActuatorPositions(self):
        '''
        Returns true if actuator positions are valid
        ~~~
        '''
        return

    def isValidEndEffectorPositions(self):
        '''
        Returns true if the end effector position is in the work space
        '''
        actuators, joints = self.inverseKinematicSolution(EndEffectorPosition, EndEffectorQuaternion)

        if (actuators == None) or (joints == None):
            return False
        else:
            return True

    def planTrajectoryBetweenPoints(self, EndEffectorP1, EndEffectorQ1, EndEffectorP2, EndEffectorQ2):
        '''
        Return a trajectory from the position 1 to position 2
        '''

        # Arrays that hold the initial and final rotations
        q1 = Quaternion(EndEffectorQ1)
        q2 = Quaternion(EndEffectorQ2)

        # Arrays that hold the initial and final positions
        p1 = np.array(EndEffectorP1, dtype=np.float64)
        p2 = np.array(EndEffectorP2, dtype=np.float64)

        # Scale the discretization according to max step size
        NumberOfPositions = 10
        MaxActuatorDisplacement = 1.5 * self.MaxStepSize
        while MaxActuatorDisplacement > self.MaxStepSize:
            percentages = np.linspace(0, 1, NumberOfPositions, dtype=np.float64)

            # Calculate the position of the end effector at each percentage position p1 to p2
            pDiff = p2 - p1
            EEpositions = percentages.reshape(-1, 1) @ pDiff.reshape(1, -1) + p1

            # Calculate the quaternion rotation of the end effector at each percentage from q1 to q2
            g = np.vectorize(lambda i: Quaternion.slerp(q1, q2, i).elements, otypes=[np.ndarray])
            EErotations = np.vstack(g(percentages))

            actuatorPotision = np.zeros((NumberOfPositions, 6))
            platformJointPosition = np.zeros((NumberOfPositions, 6, 3))
            # Calculate the actuator positions at each iteration
            for row, position, rotation in zip(range(NumberOfPositions), EEpositions, EErotations):
                ActuatorHeights, JointPositions = self.inverseKinematicSolution(position, rotation)

                H1, H2, H3, H4, H5, H6 = ActuatorHeights
                C1, C2, C3, C4, C5, C6 = JointPositions

                actuatorPotision[row, 0] = H1
                actuatorPotision[row, 1] = H2
                actuatorPotision[row, 2] = H3
                actuatorPotision[row, 3] = H4
                actuatorPotision[row, 4] = H5
                actuatorPotision[row, 5] = H6

                platformJointPosition[row, 0, :] = C1
                platformJointPosition[row, 1, :] = C2
                platformJointPosition[row, 2, :] = C3
                platformJointPosition[row, 3, :] = C4
                platformJointPosition[row, 4, :] = C5
                platformJointPosition[row, 5, :] = C6

            # Calculate the difference in position from time steps
            ActuatorDisplacement = np.abs(np.diff(actuatorPotision, axis=0))

            # Calcualte the maximum difference in displacement from time steps across all actuators
            MaxActuatorDisplacement = np.amax(ActuatorDisplacement)

            # Determine a new best discretization based on the results
            NumberOfPositions *= ceiling(MaxActuatorDisplacement / self.MaxStepSize)

        # Assign time for each movement according to max speed
        MaxTimeStep = MaxActuatorDisplacement / self.MaxActuatorSpeed
        times = np.linspace(0, float(NumberOfPositions*MaxTimeStep), NumberOfPositions)

        # Return the end
        return (actuatorPotision, platformJointPosition, EEpositions, EErotations, times)

    def planTrajectory(self, EndEffectorPosition, EndEffectorQuaternion):
        '''
        Return a trajectory from the current position to the given position
        '''
        return self.planTrajectoryBetweenPoints(self.EndEffectorPosition, self.EndEffectorOrientation, EndEffectorPosition, EndEffectorQuaternion)


    def plotCurrentPosition(self):
        '''
        Show the current position of the stewart platform
        '''

        # Create the plot
        ax = plt.axes(projection="3d")

        # Set the plot limits
        ax.set_xlim(-2 * self.BaseRadius,2 * self.BaseRadius)
        ax.set_ylim(-2 * self.BaseRadius,2 * self.BaseRadius)
        ax.set_zlim(0,10 * self.BaseRadius)

        # Plot Actuator Bases
        ax.plot3D([self.A1B[0], self.A2B[0]], [self.A1B[1], self.A2B[1]], [self.A1B[2], self.A2B[2]], 'black')
        ax.plot3D([self.A2B[0], self.A3B[0]], [self.A2B[1], self.A3B[1]], [self.A2B[2], self.A3B[2]], 'black')
        ax.plot3D([self.A3B[0], self.A4B[0]], [self.A3B[1], self.A4B[1]], [self.A3B[2], self.A4B[2]], 'black')
        ax.plot3D([self.A4B[0], self.A5B[0]], [self.A4B[1], self.A5B[1]], [self.A4B[2], self.A5B[2]], 'black')
        ax.plot3D([self.A5B[0], self.A6B[0]], [self.A5B[1], self.A6B[1]], [self.A5B[2], self.A6B[2]], 'black')
        ax.plot3D([self.A6B[0], self.A1B[0]], [self.A6B[1], self.A1B[1]], [self.A6B[2], self.A1B[2]], 'black')

        # Plot Actuators
        ax.plot3D([self.A1B[0], self.A1C[0]], [self.A1B[1], self.A1C[1]], [self.A1B[2], self.A1C[2]], 'red')
        ax.plot3D([self.A2B[0], self.A2C[0]], [self.A2B[1], self.A2C[1]], [self.A2B[2], self.A2C[2]], 'red')
        ax.plot3D([self.A3B[0], self.A3C[0]], [self.A3B[1], self.A3C[1]], [self.A3B[2], self.A3C[2]], 'red')
        ax.plot3D([self.A4B[0], self.A4C[0]], [self.A4B[1], self.A4C[1]], [self.A4B[2], self.A4C[2]], 'red')
        ax.plot3D([self.A5B[0], self.A5C[0]], [self.A5B[1], self.A5C[1]], [self.A5B[2], self.A5C[2]], 'red')
        ax.plot3D([self.A6B[0], self.A6C[0]], [self.A6B[1], self.A6C[1]], [self.A6B[2], self.A6C[2]], 'red')

        # Plot Control Arms
        ax.plot3D([self.A1C[0], self.C1P[0]], [self.A1C[1], self.C1P[1]], [self.A1C[2], self.C1P[2]], 'black')
        ax.plot3D([self.A2C[0], self.C2P[0]], [self.A2C[1], self.C2P[1]], [self.A2C[2], self.C2P[2]], 'black')
        ax.plot3D([self.A3C[0], self.C3P[0]], [self.A3C[1], self.C3P[1]], [self.A3C[2], self.C3P[2]], 'black')
        ax.plot3D([self.A4C[0], self.C4P[0]], [self.A4C[1], self.C4P[1]], [self.A4C[2], self.C4P[2]], 'black')
        ax.plot3D([self.A5C[0], self.C5P[0]], [self.A5C[1], self.C5P[1]], [self.A5C[2], self.C5P[2]], 'black')
        ax.plot3D([self.A6C[0], self.C6P[0]], [self.A6C[1], self.C6P[1]], [self.A6C[2], self.C6P[2]], 'black')

        # Plot Platform Bases
        ax.plot3D([self.C2P[0], self.C1P[0]], [self.C2P[1], self.C1P[1]], [self.C2P[2], self.C1P[2]], 'black')
        ax.plot3D([self.C3P[0], self.C2P[0]], [self.C3P[1], self.C2P[1]], [self.C3P[2], self.C2P[2]], 'black')
        ax.plot3D([self.C4P[0], self.C3P[0]], [self.C4P[1], self.C3P[1]], [self.C4P[2], self.C3P[2]], 'black')
        ax.plot3D([self.C5P[0], self.C4P[0]], [self.C5P[1], self.C4P[1]], [self.C5P[2], self.C4P[2]], 'black')
        ax.plot3D([self.C6P[0], self.C5P[0]], [self.C6P[1], self.C5P[1]], [self.C6P[2], self.C5P[2]], 'black')
        ax.plot3D([self.C1P[0], self.C6P[0]], [self.C1P[1], self.C6P[1]], [self.C1P[2], self.C6P[2]], 'black')

        # Plot End Effector
        ax.scatter3D(self.EndEffectorPosition[0], self.EndEffectorPosition[1], self.EndEffectorPosition[2], 'purple', s = 80)

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

        # Show the plot
        plt.show()

        return

    def encodeHeight(self, height):
        binHeight = int(3400 * height*2)

        position = format(binHeight, "b").zfill(12)

        byte1 = int('1' + position[:6] + '1', 2)
        byte2 = int('0' + position[6:] + '1', 2)

        return bytearray([byte1, byte2])

    def executeTrajectory(self, Trajectory, show=False, actuate=False):
        '''
        Show a trajectory for the stewart platfrom
        '''

        # Decompose the elements of the trajectory

        ActHeights, JointPoss, EEPoss, EERots, Times = Trajectory

        if show:
            ax = plt.axes(projection="3d")
            plt.ion()
            plt.show()

        prevTime = 0
        for row, timeStamp in enumerate(Times):
            if actuate:
                if row == 0:
                    t1 = timeStamp
                else:
                    deltaTime = timeStamp - t1
                    t1 = timeStamp
                    time.sleep(deltaTime)


            print(row, timeStamp)
            # Update the position
            self.A1C[2], self.A2C[2], self.A3C[2], self.A4C[2], self.A5C[2], self.A6C[2] = ActHeights[row]
            self.C1P, self.C2P, self.C3P, self.C4P, self.C5P, self.C6P = JointPoss[row]
            self.EndEffectorPosition = EEPoss[row]
            self.EndEffectorOrientation = EERots[row]


            # Send the actuator trajectories to the MCU's
            if actuate:
                self.ser1.write(self.encodeHeight(self.A5C[2]))
                self.ser2.write(self.encodeHeight(self.A6C[2]))
                self.ser3.write(self.encodeHeight(self.A1C[2]))
                self.ser4.write(self.encodeHeight(self.A2C[2]))
                self.ser5.write(self.encodeHeight(self.A3C[2]))
                self.ser6.write(self.encodeHeight(self.A4C[2]))


            if show:
                plt.cla()

                # Set the plot limits
                ax.set_xlim(-2 * self.BaseRadius,2 * self.BaseRadius)
                ax.set_ylim(-2 * self.BaseRadius,2 * self.BaseRadius)
                ax.set_zlim(0,10 * self.BaseRadius)

                # Plot Actuator Bases
                ax.plot3D([self.A1B[0], self.A2B[0]], [self.A1B[1], self.A2B[1]], [self.A1B[2], self.A2B[2]], 'black')
                ax.plot3D([self.A2B[0], self.A3B[0]], [self.A2B[1], self.A3B[1]], [self.A2B[2], self.A3B[2]], 'black')
                ax.plot3D([self.A3B[0], self.A4B[0]], [self.A3B[1], self.A4B[1]], [self.A3B[2], self.A4B[2]], 'black')
                ax.plot3D([self.A4B[0], self.A5B[0]], [self.A4B[1], self.A5B[1]], [self.A4B[2], self.A5B[2]], 'black')
                ax.plot3D([self.A5B[0], self.A6B[0]], [self.A5B[1], self.A6B[1]], [self.A5B[2], self.A6B[2]], 'black')
                ax.plot3D([self.A6B[0], self.A1B[0]], [self.A6B[1], self.A1B[1]], [self.A6B[2], self.A1B[2]], 'black')

                # Plot Actuators
                ax.plot3D([self.A1B[0], self.A1C[0]], [self.A1B[1], self.A1C[1]], [self.A1B[2], self.A1C[2]], 'red')
                ax.plot3D([self.A2B[0], self.A2C[0]], [self.A2B[1], self.A2C[1]], [self.A2B[2], self.A2C[2]], 'red')
                ax.plot3D([self.A3B[0], self.A3C[0]], [self.A3B[1], self.A3C[1]], [self.A3B[2], self.A3C[2]], 'red')
                ax.plot3D([self.A4B[0], self.A4C[0]], [self.A4B[1], self.A4C[1]], [self.A4B[2], self.A4C[2]], 'red')
                ax.plot3D([self.A5B[0], self.A5C[0]], [self.A5B[1], self.A5C[1]], [self.A5B[2], self.A5C[2]], 'red')
                ax.plot3D([self.A6B[0], self.A6C[0]], [self.A6B[1], self.A6C[1]], [self.A6B[2], self.A6C[2]], 'red')

                # Plot Control Arms
                ax.plot3D([self.A1C[0], self.C1P[0]], [self.A1C[1], self.C1P[1]], [self.A1C[2], self.C1P[2]], 'black')
                ax.plot3D([self.A2C[0], self.C2P[0]], [self.A2C[1], self.C2P[1]], [self.A2C[2], self.C2P[2]], 'black')
                ax.plot3D([self.A3C[0], self.C3P[0]], [self.A3C[1], self.C3P[1]], [self.A3C[2], self.C3P[2]], 'black')
                ax.plot3D([self.A4C[0], self.C4P[0]], [self.A4C[1], self.C4P[1]], [self.A4C[2], self.C4P[2]], 'black')
                ax.plot3D([self.A5C[0], self.C5P[0]], [self.A5C[1], self.C5P[1]], [self.A5C[2], self.C5P[2]], 'black')
                ax.plot3D([self.A6C[0], self.C6P[0]], [self.A6C[1], self.C6P[1]], [self.A6C[2], self.C6P[2]], 'black')

                # Plot Platform Bases
                ax.plot3D([self.C2P[0], self.C1P[0]], [self.C2P[1], self.C1P[1]], [self.C2P[2], self.C1P[2]], 'black')
                ax.plot3D([self.C3P[0], self.C2P[0]], [self.C3P[1], self.C2P[1]], [self.C3P[2], self.C2P[2]], 'black')
                ax.plot3D([self.C4P[0], self.C3P[0]], [self.C4P[1], self.C3P[1]], [self.C4P[2], self.C3P[2]], 'black')
                ax.plot3D([self.C5P[0], self.C4P[0]], [self.C5P[1], self.C4P[1]], [self.C5P[2], self.C4P[2]], 'black')
                ax.plot3D([self.C6P[0], self.C5P[0]], [self.C6P[1], self.C5P[1]], [self.C6P[2], self.C5P[2]], 'black')
                ax.plot3D([self.C1P[0], self.C6P[0]], [self.C1P[1], self.C6P[1]], [self.C1P[2], self.C6P[2]], 'black')

                # Plot End Effector
                ax.scatter3D(self.EndEffectorPosition[0], self.EndEffectorPosition[1], self.EndEffectorPosition[2], 'purple', s = 80)


            # If this is not the first position
            if row != 0:
                # Calculate the time delay required to send the first signal

                # Likewise for show
                if show:
                    plt.draw()
                    plt.pause(timeStamp - prevTime)

                prevTime = timeStamp

            # Show the plot
            plt.show()
        return True

    def visPosition(self, EndEffectorPosition, EndEffectorQuaternion):
        '''
        Show a position for the stewart platform without executing it
        '''
        return

    def visTrajectory(self, Trajectory):
        '''
        Show a trajectory for the stewart platform without executing it
        '''
        return


def main():
    BaseAngleOffset = 76.4/2                                ##Actuator Angle Offset [deg]
    BaseRadius = 269.26 / 2 * 10**-3                        ##Actuator Base Radius [m]
    ActStroke = 6 * 0.0254                                  ##Actuator Stroke [m]
    ArmLen = 144 * 10**-3                                   ##Control Arm Length [m]
    PlatformAngleOffset = 43.58/2                           ##Control Arm Angle Offset[deg]
    PlatformRadius = 119.63/2 * 10**-3                      ##Control Arm Platform Radius [m]
    EndEffectorPositionOffset = np.array([0, 0, 0])         ##The offset of the beam from the platform [m]

    # Create an instance of the stewart mechanism
    mechanism = StewartPlatform(    BaseAngleOffset,
                                    BaseRadius,
                                    ActStroke,
                                    ArmLen,
                                    PlatformRadius,
                                    PlatformAngleOffset,
                                    EndEffectorPositionOffset
                                )

    mechanism.MaxStepSize = 0.01
    mechanism.MaxActuatorSpeed = 0.013368421052538 * 300


    #mechanism.plotCurrentPosition()

    preview = True
    actuate = True


    d = 0.04
    h2 = 0.7
    h1 = 0.6
    t1 = mechanism.planTrajectory([-d, d, h1], [1, 0, 0, 0])
    t2 = mechanism.planTrajectoryBetweenPoints([-d, d, h1], [1, 0, 0, 0], [d, d, h2], [1, 0, 0, 0])
    t3 = mechanism.planTrajectoryBetweenPoints([d, d, h2], [1, 0, 0, 0], [d, -d, h1], [1, 0, 0, 0])
    t4 = mechanism.planTrajectoryBetweenPoints([d, -d, h1], [1, 0, 0, 0], [-d, -d, h2], [1, 0, 0, 0])
    t5 = mechanism.planTrajectoryBetweenPoints([-d, -d, h2], [1, 0, 0, 0], [-d, d, h1], [1, 0, 0, 0])

    mechanism.topHome()
    time.sleep(5)

    mechanism.executeTrajectory(t1, show=preview, actuate=actuate)
    time.sleep(3)
    while True:
        mechanism.executeTrajectory(t2, show=preview, actuate=actuate)
        time.sleep(3)
        mechanism.executeTrajectory(t3, show=preview, actuate=actuate)
        time.sleep(3)
        mechanism.executeTrajectory(t4, show=preview, actuate=actuate)
        time.sleep(3)
        mechanism.executeTrajectory(t5, show=preview, actuate=actuate)
        time.sleep(3)

    #mechanism.executeTrajectory(t2, show=preview, actuate=actuate)
    #time.sleep(3)
    #mechanism.executeTrajectory(t3, show=preview, actuate=actuate)
    #time.sleep(3)
    #mechanism.executeTrajectory(t4, show=preview, actuate=actuate)
    #time.sleep(3)
    #mechanism.executeTrajectory(t5, show=preview, actuate=actuate)
    #time.sleep(3)
    #mechanism.executeTrajectory(t2, show=preview, actuate=actuate)
    #time.sleep(3)
    mechanism.topHome()
    time.sleep(5)
    #time.sleep(1)
    #mechanism.executeTrajectory(t3, show=preview, actuate=actuate)
    #time.sleep(1)
    #mechanism.executeTrajectory(t4, show=preview, actuate=actuate)
    #time.sleep(1)
    #mechanism.executeTrajectory(t5, show=preview, actuate=actuate)
    #time.sleep(1)
    #mechanism.executeTrajectory(t2, show=preview, actuate=actuate)
    #time.sleep(1)
    #mechanism.executeTrajectory(t3, show=preview, actuate=actuate)
    #time.sleep(1)
    #mechanism.executeTrajectory(t4, show=preview, actuate=actuate)
    #time.sleep(1)
    #mechanism.executeTrajectory(t5, show=preview, actuate=actuate)



if __name__ == "__main__":
    main()
