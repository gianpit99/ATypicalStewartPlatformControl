import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from sympy import *
from pyquaternion import Quaternion


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

        # Compute the actuator heights
        A1Z, A2Z, A3Z, A4Z, A5Z, A6Z = symbols("A1Z A2Z A3Z A4Z A5Z A6Z")

        A1Z_sol = solve(Eq((self.A1B[0] - C1P[0])**2 + (self.A1B[1] - C1P[1])**2 + (A1Z - C1P[2])**2, (self.ArmLen)**2))
        A2Z_sol = solve(Eq((self.A2B[0] - C2P[0])**2 + (self.A2B[1] - C2P[1])**2 + (A2Z - C2P[2])**2, (self.ArmLen)**2))
        A3Z_sol = solve(Eq((self.A3B[0] - C3P[0])**2 + (self.A3B[1] - C3P[1])**2 + (A3Z - C3P[2])**2, (self.ArmLen)**2))
        A4Z_sol = solve(Eq((self.A4B[0] - C4P[0])**2 + (self.A4B[1] - C4P[1])**2 + (A4Z - C4P[2])**2, (self.ArmLen)**2))
        A5Z_sol = solve(Eq((self.A5B[0] - C5P[0])**2 + (self.A5B[1] - C5P[1])**2 + (A5Z - C5P[2])**2, (self.ArmLen)**2))
        A6Z_sol = solve(Eq((self.A6B[0] - C6P[0])**2 + (self.A6B[1] - C6P[1])**2 + (A6Z - C6P[2])**2, (self.ArmLen)**2))

        solutions = [A1Z_sol, A2Z_sol, A3Z_sol, A4Z_sol, A5Z_sol, A6Z_sol]
        output = []

        # Return false if the solutions are complex, or return the postive solution
        prevActs = [self.A1C, self.A2C, self.A3C, self.A4C, self.A5C, self.A6C]
        for actNum, solution in enumerate(solutions):
            if (not solution[0].is_real) or (not solution[1].is_real):
                return (None, None)
            else:
                # Use the solution whos distance was closest to the previous
                if (np.abs(solution[0] - prevActs[actNum][2]) <= np.abs(solution[1] - prevActs[actNum][2])):
                    output.append(solution[0])
                else:
                    output.append(solution[0])

        # Return the actuator positions and the platform joint positions
        return (tuple(output), (C1P, C2P, C3P, C4P, C5P, C6P))


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

    def instantHome(self):
        '''
        Instantly home the system positions
        '''
        return self.instantGlobalMove([0, 0, self.HomeHeight], [1, 0, 0, 0])

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

        # Show the plot
        plt.show()

        return

    def executeTrajectory(self, Trajectory, show=False):
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
        for row, time in enumerate(Times):
            # Update the position
            self.A1C[2], self.A2C[2], self.A3C[2], self.A4C[2], self.A5C[2], self.A6C[2] = ActHeights[row]
            self.C1P, self.C2P, self.C3P, self.C4P, self.C5P, self.C6P = JointPoss[row]
            self.EndEffectorPosition = EEPoss[row]
            self.EndEffectorOrientation = EERots[row]

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
                    plt.pause(time - prevTime)

                prevTime = time

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
    BaseAngleOffset = 15                                    #Actuator Angle Offset [deg]
    BaseRadius = 0.55                                       #Actuator Base Radius [m]
    ActStroke = 2.5                                         #Actuator Stroke [m]
    ArmLen = 0.8                                            #Control Arm Length [m]
    PlatformAngleOffset = 25                                #Control Arm Angle Offset[deg]
    PlatformRadius = 0.3                                    #Control Arm Platform Radius [m]
    EndEffectorPositionOffset = np.array([0, 0, 0.5])       #The offset of the beam from the platform [m]

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
    mechanism.MaxActuatorSpeed = 1




    d = 0.2
    t1 = mechanism.planTrajectory([-d, d, 2.5], [1, 0, 0, 0])
    t2 = mechanism.planTrajectoryBetweenPoints([-d, d, 2.5], [1, 0, 0, 0], [d, d, 2.5], [1, 0, 0, 0])
    t3 = mechanism.planTrajectoryBetweenPoints([d, d, 2.5], [1, 0, 0, 0], [d, -d, 2.5], [1, 0, 0, 0])
    t4 = mechanism.planTrajectoryBetweenPoints([d, -d, 2.5], [1, 0, 0, 0], [-d, -d, 2.5], [1, 0, 0, 0])
    t5 = mechanism.planTrajectoryBetweenPoints([-d, -d, 2.5], [1, 0, 0, 0], [-d, d, 2.5], [1, 0, 0, 0])

    mechanism.executeTrajectory(t1, show=True)
    mechanism.executeTrajectory(t2, show=True)
    mechanism.executeTrajectory(t3, show=True)
    mechanism.executeTrajectory(t4, show=True)
    mechanism.executeTrajectory(t5, show=True)
    mechanism.executeTrajectory(t2, show=True)
    mechanism.executeTrajectory(t3, show=True)
    mechanism.executeTrajectory(t4, show=True)
    mechanism.executeTrajectory(t5, show=True)
    mechanism.executeTrajectory(t2, show=True)
    mechanism.executeTrajectory(t3, show=True)
    mechanism.executeTrajectory(t4, show=True)
    mechanism.executeTrajectory(t5, show=True)
    mechanism.executeTrajectory(t2, show=True)
    mechanism.executeTrajectory(t3, show=True)
    mechanism.executeTrajectory(t4, show=True)
    mechanism.executeTrajectory(t5, show=True)



if __name__ == "__main__":
    main()
