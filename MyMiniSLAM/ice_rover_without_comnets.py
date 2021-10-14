"""
 === Introduction ===
 
   The project is broken up into two parts.

   Part A:
        Create a SLAM implementation to process a series of landmark (beacon) measurements and movement updates.

        Hint: A planner with an unknown number of motions works well with an online version of SLAM.

    Part B:
        Here you will create the planner for the rover.  The rover does unfortunately has a series of limitations:

        - Start position
          - The rover will land somewhere within range of at least 3 or more landmarks (beacons or sites) for measurements.

        - Measurements
          - Measurements will come from beacons and test sites within range of the rover's antenna horizon (the horizon distance).
            * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'beacon'}, ...}
          - Beacons and test sites will always return a measurement if in range.

        - Movements
          - Action: 'move 1.570963 1.0'
            * The rover will turn counterclockwise 90 degrees and then move 1.0
          - stochastic due to the icy surface.
          - if max distance or steering is exceeded, the rover will not move.

        - Samples
          - Provided as list of x and y coordinates, [[0., 0.], [1., -3.5], ...]
          - Action: 'sample'
            * The rover will attempt to take a sample at the current location.
          - A rover can only take a sample once per requested site.
          - The rover MUst be within 0.25 distance to successfully take a sample.
            * Hint: Be sure to account for floating point limitations
          - The is a 100ms penalty if the robot is requested to sample a site not on the list or if the site has
            previously been sampled.
          - You may temporarily use sys.stdout = open('stdout.txt', 'w') to directly print data if necessary, HOWEVER, you MUST REMOVE these lines before submitting your code or it will keep our testing framework from giving you a grade (larger than a zero). 

        The rover will always execute a measurement first, followed by an action.

        The rover will have a time limit of 5 seconds to find and sample all required sites.
"""
#1------------------------------------------------------------------------------------------------------------
from matrix import matrix
import math
import random
import copy
#1------------------------------------------------------------------------------------------------------------
class SLAM:
    """Create a basic SLAM module.
    """
#2------------------------------------------------------------------------------------------------------------  
    def __init__(self, initialX = 0.0, initialY = 0.0, heading = 0.0):
        """Initialize SLAM components here.
        """
        self.mylocation = matrix()
        self.mylocation.zero(2, 2)
        self.mylocation.value[0][0] = 1.0
        self.mylocation.value[1][1] = 1.0
        self.myClocation = matrix()
        self.myClocation.zero(2, 1)
        self.myClocation.value[0][0] = initialX
        self.myClocation.value[1][0] = initialY
        self.knownLandmarks = []
        self.heading = heading
    def CheckKnownLandMarks(self, thisLandMark):
        index = []
        for i in range(len(self.knownLandmarks)):
            if self.knownLandmarks[i] == thisLandMark:
                index = i
        return index
    def truncateAngle(self, t):
        PI = math.pi
        return ((t + PI) % (2 * PI)) - PI
        # TODO
#2------------------------------------------------------------------------------------------------------------
#3------------------------------------------------------------------------------------------------------------
    def process_measurements(self, measurements):
        """Process a new series of measurements.

        Args:
            measurements(dict): Collection of measurements
                in the format {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'beacon'}, ...}

        Returns:
            x, y: current belief in location of the rover relative to initial location before movement
        """
        mylocation = self.mylocation
        myClocation = self.myClocation
        for key in measurements.keys():
            TheType = measurements[key]['type']
            if TheType == 'beacon':
                TheDist = measurements[key]['distance']
                TheBear = self.truncateAngle(measurements[key]['bearing'] + self.heading)
                SigmaDist = 0.05 * TheDist
                SigmaBear = 0.02 * TheBear
                BearNoise = SigmaBear
                DistNoise = SigmaDist
                dx = TheDist * math.cos(TheBear)
                dy = TheDist * math.sin(TheBear)
                noiseX = DistNoise * math.cos(BearNoise)
                noiseY = DistNoise * math.sin(BearNoise)
                measurement_noise = [1.0, 1.0]
                thismeasurement = [dx, dy]
                IDx = self.CheckKnownLandMarks(key)
                CurrentDimansion = len(myClocation.value)
                CurrentList = range(CurrentDimansion)
                if not IDx:
                    self.knownLandmarks.append(key)
                    IDx = self.CheckKnownLandMarks(key)
                    mylocation = mylocation.expand(dimx=CurrentDimansion+2, dimy=CurrentDimansion+2, list1=CurrentList, list2=CurrentList)
                    myClocation = myClocation.expand(dimx=CurrentDimansion+2, dimy=1, list1=CurrentList, list2=[0])
                mIDx = 2*IDx + 2
                for b in range(2):
                    mylocation.value[b][b] += 1.0 * (1.0+measurement_noise[b])
                    mylocation.value[mIDx + b][mIDx + b] += 1.0 * (1.0+measurement_noise[b])
                    mylocation.value[b][mIDx + b] -= 1.0 * (1.0+measurement_noise[b])
                    mylocation.value[mIDx + b][b] -= 1.0 * (1.0+measurement_noise[b])
                    myClocation.value[b][0] += -thismeasurement[b] * (1.0+measurement_noise[b])
                    myClocation.value[mIDx + b][0] += thismeasurement[b] * (1.0+measurement_noise[b])
        self.mylocation = mylocation
        self.myClocation = myClocation
        MU = mylocation.inverse() * myClocation
        x = MU.value[0][0]
        y = MU.value[1][0]
        # TODO
        
        #raise NotImplementedError

        #x = 0.0
        #y = 0.0

        return x, y
#3------------------------------------------------------------------------------------------------------------
#4------------------------------------------------------------------------------------------------------------
    def process_movement(self, steering, distance, motion_noise=0.001):
        """Process a new movement.

        Args:
            steering(float): amount to turn
            distance(float): distance to move
            motion_noise(float): movement noise

        Returns:
            x, y: current belief in location of the rover relative to initial location after movement
        """
        mylocation = self.mylocation
        myClocation = self.myClocation
        thisheading = self.truncateAngle(self.heading + steering)
        self.heading = thisheading
        dx = distance * math.cos(thisheading)
        dy = distance * math.sin(thisheading)
        thismotion = [dx, dy]
        dim = len(myClocation.value)
        data_pointer_list = [0, 1] + range(4, dim + 2)
        mylocation = mylocation.expand(dimx=dim+2, dimy=dim+2, list1=data_pointer_list, list2=data_pointer_list)
        myClocation = myClocation.expand(dimx=dim+2, dimy=1, list1=data_pointer_list, list2=[0])
        for b in range(4):
            mylocation.value[b][b] += 1.0 / motion_noise
        for b in range(2):
            mylocation.value[b][b+2] += -1.0 / motion_noise
            mylocation.value[b+2][b] += -1.0 / motion_noise
            myClocation.value[b][0] += -thismotion[b] / motion_noise
            myClocation.value[b+2][0] += thismotion[b] / motion_noise
        Alist = range(2, dim+2)
        A = mylocation.take(list1=[0, 1], list2=Alist)
        Blist = [0, 1]
        B = mylocation.take(list1=Blist, list2=Blist)
        Clist = [0, 1]
        C = myClocation.take(list1=Clist, list2=[0])
        mylocationPrime = mylocation.take(list1=Alist, list2=Alist)
        myClocationPrime = myClocation.take(list1=Alist, list2=[0])
        mylocation = mylocationPrime - A.transpose() * B.inverse() * A
        myClocation = myClocationPrime - A.transpose() * B.inverse() * C
        self.mylocation = mylocation
        self.myClocation = myClocation
        MU = mylocation.inverse() * myClocation
        x = MU.value[0][0]
        y = MU.value[1][0]
        # TODO
        #raise NotImplementedError

        #x = 0.0
        #y = 0.0

        return x, y
#4------------------------------------------------------------------------------------------------------------        


class WayPointPlanner:
    """Create a planner to navigate the rover to reach all the intended way points from an unknown start position.
    """
#5------------------------------------------------------------------------------------------------------------
    def __init__(self,  max_distance, max_steering):
        """Initialize your planner here.

        Args:
            max_distance(float): the max distance the robot can travel in a single move.
            max_steering(float): the max steering angle the robot can turn in a single move.
        """
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.heading = 0.0
        self.DoSLAM = SLAM()
        self.patrolTurnCount = 2
        self.patrolStraightMax = 1
        self.patrolStraightCount = 1
        self.foundSampleFlag = False
        self.foundSampleFlagonce = False
        self.SLAMMEASOffset = [0.0, 0.0, 0.0]
        self.sampleList = []
        self.initiateHeading = []
        self.oldmeasure = []
        self.countInitiateH = 0
    def truncateAngle(self, t):
        PI = math.pi
        return ((t + PI) % (2 * PI)) - PI
    def TruncateOfSteer(self, t):
        PI = math.pi
        if t > PI:
            NewTruncate = 2*PI - t
        elif t < -PI:
            NewTruncate = 2*PI + t
        elif t > 2*PI or t < -2*PI:
            NewTruncate = self.truncateAngle(t)
        else:
            NewTruncate = t
        return NewTruncate
    def CheckMeasurements(self, measurements):
        UsedKey = []
        UsedDist = []
        UsedBear = []
        MinDist = 9999999.0
        for key in measurements.keys():
            TheType = measurements[key]['type']
            if TheType == 'site':
                TheDist = measurements[key]['distance']
                TheBear = self.truncateAngle(measurements[key]['bearing'])
                if TheDist < MinDist:
                    UsedKey = key
                    UsedDist = TheDist
                    UsedBear = TheBear
                    MinDist = TheDist
        return UsedKey, UsedDist, UsedBear
    def generatePatrolAction(self):
        distance = self.max_distance * 1.0
        self.patrolStraightCount -= 1
        if self.patrolStraightCount == 0:
            steering = self.max_steering
            self.patrolTurnCount -= 1
            if self.patrolTurnCount == 0:
                self.patrolTurnCount = 2
                self.patrolStraightMax += 1
            self.patrolStraightCount = self.patrolStraightMax
        else:
            steering = 0.0
        action = 'move ' + str(steering) + ' ' + str(distance)
        return action, steering, distance
    def CheckSampled(self, NewDoingSample):
        FoundedSamples = []
        NoSampleFounded = True
        if not not self.sampleList:
            if len(self.sampleList) != len(NewDoingSample):
                until = len(NewDoingSample)
                i = 0
                while i < until and NoSampleFounded:
                    thisOld = self.sampleList[i]
                    thisNew = NewDoingSample[i]
                    if not (thisOld[0] == thisNew[0] and thisOld[1] == thisNew[1]):
                        FoundedSamples = thisOld
                        NoSampleFounded = False
                    i += 1
                if not FoundedSamples:
                    FoundedSamples = self.sampleList[-1]
        return FoundedSamples
    def FindNextS(self, currPos, samplelist):
        MinDist = 999999.0
        NextS = samplelist[0]
        for i in range(len(samplelist)):
            distance = math.sqrt((currPos[0] - samplelist[i][0])**2 + (currPos[1] - samplelist[i][1])**2)
            if distance < MinDist:
                NextS = samplelist[i]
                MinDist = distance
        return NextS
    def compute_distance(self, p, q):
        x1, y1 = p
        x2, y2 = q
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx ** 2 + dy ** 2)
    def compute_bearing(self, p, q):
        x1, y1 = p
        x2, y2 = q
        dx = x2 - x1
        dy = y2 - y1
        return math.atan2(dy, dx)
        # TODO
#5------------------------------------------------------------------------------------------------------------
    def next_move(self, sample_todo, measurements):
        """Next move based on the current set of measurements.

        Args:
            sample_todo(list): Set of locations remaining still needing a sample to be taken.
            measurements(dict): Collection of measurements from beacons and test sites in range.
                in the format {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'beacon'}, ...}

        Return:
            Next command to execute on the rover.
                allowed:
                    'move 1.570963 1.0' - turn left 90 degrees and move 1.0 distance
                    'sample' - take sample (will succeed if within tolerance of intended sample site)
        """
#6------------------------------------------------------------------------------------------------------------
        TheKey, TheDistance, TheBearing = self.CheckMeasurements(measurements)
        DoSLAM = self.DoSLAM
        measurex, measurey = DoSLAM.process_measurements(measurements)
        if self.countInitiateH > 0:
            dx = measurex - self.oldmeasure[0]
            dy = measurey - self.oldmeasure[1]
            self.heading = self.truncateAngle(math.atan2(dy, dx))
        self.oldmeasure = [measurex, measurey]
        self.countInitiateH += 1
        if not self.sampleList:
            self.sampleList = list(sample_todo)
        FoundedSamples = self.CheckSampled(sample_todo)
        if not not FoundedSamples:
            self.foundSampleFlagonce = True
            self.SLAMMEASOffset = FoundedSamples
            DoSLAM = self.DoSLAM
            measurex, measurey = DoSLAM.process_measurements(measurements)
            dx = FoundedSamples[0] - measurex
            dy = FoundedSamples[1] - measurey
            thismyClocation = self.DoSLAM.myClocation
            thismyClocation.value[0][0] += dx
            thismyClocation.value[1][0] += dy
            self.DoSLAM.myClocation = thismyClocation
            self.sampleList = list(sample_todo)
        if not self.foundSampleFlagonce:
            if not TheKey:
                action, steering, distance = self.generatePatrolAction()
                motionx, motiony = DoSLAM.process_movement(steering, distance)
            else:
                if TheDistance < 0.25:
                    action = 'sample'
                else:
                    UsedDistance = TheDistance
                    useSteer = self.TruncateOfSteer(TheBearing)
                    if math.fabs(useSteer) > self.max_steering:
                        useSteer = self.truncateAngle(self.max_steering * (useSteer/math.fabs(useSteer)))
                        UsedDistance = self.max_distance * 0.01
                    else:
                        if UsedDistance > self.max_distance:
                            UsedDistance = self.max_distance
                    action = 'move ' + str(useSteer) + ' ' + str(UsedDistance)
                    self.patrolTurnCount = 2
                    self.patrolStraightMax = 1
                    self.patrolStraightCount = 1
                    motionx, motiony = DoSLAM.process_movement(useSteer, UsedDistance)
            self.DoSLAM = DoSLAM
        else:
            if TheDistance < 0.25:
                action = 'sample'
            else:
                if not TheKey:
                    DoSLAM = self.DoSLAM
                    measurex, measurey = DoSLAM.process_measurements(measurements)
                    NextS = self.FindNextS([measurex, measurey], sample_todo)
                    dx = NextS[0] - measurex
                    dy = NextS[1] - measurey
                    TheBearing = self.truncateAngle(math.atan2(dy, dx))
                    distance = math.sqrt(dx**2 + dy**2)
                    steering = self.TruncateOfSteer(TheBearing - self.heading)
                    if math.fabs(steering) > self.max_steering:
                        UsedBearing = self.max_steering * (steering / math.fabs(steering))
                        UsedDistance = self.max_distance * 0.01
                    else:
                        UsedBearing = steering
                        if distance > self.max_distance:
                            UsedDistance = self.max_distance
                        elif distance < 0.001:
                            action, UsedBearing, UsedDistance = self.generatePatrolAction()
                            self.foundSampleFlagonce = False
                        else:
                            UsedDistance = distance
                    motionx, motiony = DoSLAM.process_movement(UsedBearing, UsedDistance)
                    action = 'move ' + str(UsedBearing) + ' ' + str(UsedDistance)
                    self.DoSLAM = DoSLAM
                else:
                    UsedDistance = TheDistance
                    useSteer = self.TruncateOfSteer(TheBearing)
                    if math.fabs(useSteer) > self.max_steering:
                        useSteer = self.truncateAngle(self.max_steering * (useSteer/math.fabs(useSteer)))
                        UsedDistance = self.max_distance * 0.01
                    else:
                        if UsedDistance > self.max_distance:
                            UsedDistance = self.max_distance
                    action = 'move ' + str(useSteer) + ' ' + str(UsedDistance)
                    self.patrolTurnCount = 2
                    self.patrolStraightMax = 1
                    self.patrolStraightCount = 1
                    motionx, motiony = DoSLAM.process_movement(useSteer, UsedDistance)
                    self.DoSLAM = DoSLAM
        # TODO
#6------------------------------------------------------------------------------------------------------------        
        #raise NotImplementedError

        #distance = 1.0
        #steering = 1.570963
        #action = 'move ' + str(steering) + ' ' + str(distance)

        return action
