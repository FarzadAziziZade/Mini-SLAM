#1------------------------------------------------------------------------------------------------------------
#Libraries
from matrix import matrix
import math
import random
import copy
#1------------------------------------------------------------------------------------------------------------
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
class SLAM:
    """Create a basic SLAM module.
    """
#2------------------------------------------------------------------------------------------------------------  
    def __init__(self, initialX = 0.0, initialY = 0.0, heading = 0.0):
        """Initialize SLAM components here.
        """      
        # initialize mylocation
        self.mylocation = matrix()
        # Can only initialize the initial x , y positions
        self.mylocation.zero(2, 2)
        self.mylocation.value[0][0] = 1.0
        self.mylocation.value[1][1] = 1.0
        # intialize myClocation
        self.myClocation = matrix()
        # Can only initialize myClocation with X and Y values of 0.0 , 0.0 --> initial believe of world is at center.
        self.myClocation.zero(2, 1)
        self.myClocation.value[0][0] = initialX
        self.myClocation.value[1][0] = initialY
        # initialize the list of collected list of landmark IDs and index in the myClocation/mylocation matrix. Or just a list of landmarks.
        self.knownLandmarks = []
        # keep an internal log for the heading.
        # This heading does not matter. It is not absolute... so the steering angle does not have an affect on it.
        # The first steering after initialization does not matter.
        self.heading = heading
    def CheckKnownLandMarks(self, thisLandMark):
        # This will look in the self:
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

        # Look at all the keys in the measurement dictionary.
        mylocation = self.mylocation
        myClocation = self.myClocation
        # print ' current list of landmarks --> ', self.knownLandmarks
        for key in measurements.keys():
            # Grab the Type, Bearing and Distance with the key.
            TheType = measurements[key]['type']
            # if Type is a beacon then calculate (landmark)
            # If this is not then ignore it??? --> We dont want to do anything with the sites. Maybe...
            if TheType == 'beacon':
                TheDist = measurements[key]['distance']
                TheBear = self.truncateAngle(measurements[key]['bearing'] + self.heading)
                SigmaDist = 0.05 * TheDist
                SigmaBear = 0.02 * TheBear
                # DistNoise = random.gauss(0, SigmaDist)
                # BearNoise = random.gauss(0, SigmaBear)
                BearNoise = SigmaBear
                DistNoise = SigmaDist
                # dx dy = what we actually update the myClocation with.
                dx = TheDist * math.cos(TheBear)
                dy = TheDist * math.sin(TheBear)
                # print ' bearingnoise = ', SigmaBear,
                noiseX = DistNoise * math.cos(BearNoise)
                noiseY = DistNoise * math.sin(BearNoise)
                # measurement_noise = [noiseX, noiseY]  #
                measurement_noise = [1.0, 1.0]  #
                # print ' noiseX = ', measurement_noise[0], ' | noiseY = ', measurement_noise[1]
                thismeasurement = [dx, dy]
                # thismeasurement = [1.0, 1.0]
                # Check if the key is in the self.knownlandmarks.
                IDx = self.CheckKnownLandMarks(key)
                CurrentDimansion = len(myClocation.value)
                # print 'printing out initial myClocation length ', CurrentDimansion
                CurrentList = range(CurrentDimansion)
                if not IDx:
                    # cant find the key? add it and expand the mylocation and myClocation
                    self.knownLandmarks.append(key)
                    IDx = self.CheckKnownLandMarks(key)
                    # Automatically expands it with 0.0's so we are good here.
                    mylocation = mylocation.expand(dimx=CurrentDimansion+2, dimy=CurrentDimansion+2, list1=CurrentList, list2=CurrentList)
                    myClocation = myClocation.expand(dimx=CurrentDimansion+2, dimy=1, list1=CurrentList, list2=[0])
                # Index to use is 2 + IDx (2 = x and y of current measurement)
                # print ' Planning to update this Index! --> ', IDx,
                mIDx = 2*IDx + 2
                # print ' in the mylocation and myClocation matrices it is this index! --> ', mIDx
                # Update the mylocation and myClocation matrices at the IDx with the measurements in the key.
                for b in range(2):
                    # Update mylocation
                    mylocation.value[b][b] += 1.0 * (1.0+measurement_noise[b])
                    mylocation.value[mIDx + b][mIDx + b] += 1.0 * (1.0+measurement_noise[b])
                    mylocation.value[b][mIDx + b] -= 1.0 * (1.0+measurement_noise[b])
                    mylocation.value[mIDx + b][b] -= 1.0 * (1.0+measurement_noise[b])
                    # Update myClocation
                    myClocation.value[b][0] += -thismeasurement[b] * (1.0+measurement_noise[b])
                    myClocation.value[mIDx + b][0] += thismeasurement[b] * (1.0+measurement_noise[b])
                # print 'Printing each instance of mylocation --> '
                # for i in range(len(mylocation.value)):
                #     print mylocation[i]
            # Finished updating mylocation and myClocation with all the measurements...
        # print ' FINAL list of landmarks --> ', self.knownLandmarks
        # Update the passthrough mylocation and myClocation with final mylocation and myClocation.
        self.mylocation = mylocation
        self.myClocation = myClocation
        # Find the final MU with these two.
        # for i in range(len(mylocation)):
        # print ' printing mylocation out! \n'
        # print mylocation
        MU = mylocation.inverse() * myClocation
        x = MU.value[0][0]
        y = MU.value[1][0]
        # print 'measurement update: x = ', x, ' and y = ', y
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
        # Expanding mylocation and myClocation for the motion update.
        mylocation = mylocation.expand(dimx=dim+2, dimy=dim+2, list1=data_pointer_list, list2=data_pointer_list)
        myClocation = myClocation.expand(dimx=dim+2, dimy=1, list1=data_pointer_list, list2=[0])
        # Updating mylocation and myClocation here now!
        for b in range(4):
            mylocation.value[b][b] += 1.0 / motion_noise
        for b in range(2):
            mylocation.value[b][b+2] += -1.0 / motion_noise
            mylocation.value[b+2][b] += -1.0 / motion_noise
            myClocation.value[b][0] += -thismotion[b] / motion_noise
            myClocation.value[b+2][0] += thismotion[b] / motion_noise
        # Obtain A B C mylocation' and myClocation'
        # A
        # dumb way to designate where the data is taken from
        Alist = range(2, dim+2)
        A = mylocation.take(list1=[0, 1], list2=Alist)
        # B
        # Always assume that we are taking the first two indices --> 2 dimensions. will screw up if otherwise.
        Blist = [0, 1]
        B = mylocation.take(list1=Blist, list2=Blist)
        # C
        # Pretty MUch always the first two of myClocation
        Clist = [0, 1]
        C = myClocation.take(list1=Clist, list2=[0])
        # mylocation Prime
        # This is the same data points indices as when obtaining A (OPlist = Alist)
        mylocationPrime = mylocation.take(list1=Alist, list2=Alist)
        # myClocation Prime
        # This is the same data points indices as when obtaining A (XPlist = Alist)
        myClocationPrime = myClocation.take(list1=Alist, list2=[0])
        # Including data from the position data points. and siMUtaneously updating the data to include only the most current data pos.
        # mylocation = mylocation' - A^T * B^-1 * A
        mylocation = mylocationPrime - A.transpose() * B.inverse() * A
        # myClocation = myClocation* - A^T * B^-1 * C
        myClocation = myClocationPrime - A.transpose() * B.inverse() * C
        # Update the passThrough mylocation and myClocation
        self.mylocation = mylocation
        self.myClocation = myClocation
        # compute best estimate
        MU = mylocation.inverse() * myClocation
        x = MU.value[0][0]
        y = MU.value[1][0]
        # print 'measurement update: x = ', x, ' and y = ', y
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
            # Grab the Type, Bearing and Distance with the key.
            TheType = measurements[key]['type']
            # if Type is a beacon then calculate (landmark)
            # If this is not then ignore it??? --> We dont want to do anything with the sites. Maybe...
            if TheType == 'site':
                TheDist = measurements[key]['distance']
                TheBear = self.truncateAngle(measurements[key]['bearing'])
                if TheDist < MinDist:
                    UsedKey = key
                    UsedDist = TheDist
                    UsedBear = TheBear
                    MinDist = TheDist
                # print ' for key (', key, ') the distance is ', TheDist
        # print 'final key used is ', UsedKey, ' with distance ', UsedDist
        return UsedKey, UsedDist, UsedBear
    def generatePatrolAction(self):
        # This will be the bulk code for finding the first sample.
        # Jump the furthest distance then look in all 4 direcitons
        # before every turn only need to look in 2 directions before
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
        # Check the new sample list versus the old one and see if there is anything new.
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
        # We need to sample all the sites in the todo list, the order doesnt matter.
        # However, not all sites will be visible in measurements due to the horizon distance.
        # If a site is visible from the very first measurement, then the task becomes easy.
        # You just navigate to the first site using the bearing and distance values from the measurement with type = site.
        # Once you sample that site, it disappears from the todo list (the todo list has the absolute coordinates of the sites to be sampled).
        # Therefore, we now know which site was sampled and we can determine the relative coordinates of all other sites in the todo list.
        # We can simply now beeline to these remaining sites and sample them.
        #
        # This becomes trickier when we dont see any site measurements from the beginning.
        # In this case, it looks like students are l moving the robot in a squared spiral until a site becomes visible( or performing a random walk).
        # Once the first site is visible, we can beeline to it, map other sites relative coordinates and sample them as well.
        #
        # LOOK HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Check the measurement --> if we see one... if we see two... take the smallest distance one.
        # print ' STARTING NEXT_MOVE CALC:'
        TheKey, TheDistance, TheBearing = self.CheckMeasurements(measurements)
        # print measurements
        # print ' found TheKey: ', TheKey, ' | TheDistance: ', TheDistance, ' | TheBearing: ', TheBearing
        #   activate DoSLAM
        DoSLAM = self.DoSLAM
        measurex, measurey = DoSLAM.process_measurements(measurements)
        if self.countInitiateH > 0:
            dx = measurex - self.oldmeasure[0]
            dy = measurey - self.oldmeasure[1]
            self.heading = self.truncateAngle(math.atan2(dy, dx))
        self.oldmeasure = [measurex, measurey]
        self.countInitiateH += 1
        # print 'heading = ', self.heading
        if not self.sampleList:
            # print ' adding the initial sample list to record. '
            self.sampleList = list(sample_todo)
        # if not self.foundSampleFlag:
        FoundedSamples = self.CheckSampled(sample_todo)
        if not not FoundedSamples:
            # print 'found Sample == ', FoundedSamples
            self.foundSampleFlagonce = True
            self.SLAMMEASOffset = FoundedSamples
            # self.DoSLAM = SLAM(initialX=FoundedSamples[0], initialY=FoundedSamples[1])
            # Update myClocation with the offset. --> initial position = [0.0, 0.0] now the new one should be + it .
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
            # We didn't find any samples yet! --> gotta just do patrol movement.
            # if we do not see any sites from measurement --> then do patrol movement.
            if not TheKey:
                # !!!!!!!!!!!!!!!!!!!!!!! IMPLEMENT PATROL HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!
                action, steering, distance = self.generatePatrolAction() # updates heading inside function.
                # print ' setting action to patrol. action = ', action
                # self.heading = self.truncateAngle(steering + self.heading)
                motionx, motiony = DoSLAM.process_movement(steering, distance)
            else:
                # if this is within distance to pick up the sample
                if TheDistance < 0.25:
                    action = 'sample'
                    # print ' taking sample because distance < 0.25 units. action = ', action
                else:
                    UsedDistance = TheDistance
                    # useSteer = self.TruncateOfSteer(TheBearing - self.heading)
                    useSteer = self.TruncateOfSteer(TheBearing)
                    # print ' BEE LINE PART: try to go steering: ', useSteer, ' | distance: ', UsedDistance, ' | current heading = ', self.heading, ' | trying to go to bearing = ', TheBearing
                    if math.fabs(useSteer) > self.max_steering:
                        useSteer = self.truncateAngle(self.max_steering * (useSteer/math.fabs(useSteer)))
                        UsedDistance = self.max_distance * 0.01
                    else:
                        if UsedDistance > self.max_distance:
                            UsedDistance = self.max_distance
                    action = 'move ' + str(useSteer) + ' ' + str(UsedDistance)
                    # self.heading = self.truncateAngle(self.heading + useSteer)
                    # resetting the patrol positions.
                    self.patrolTurnCount = 2
                    self.patrolStraightMax = 1
                    self.patrolStraightCount = 1
                    motionx, motiony = DoSLAM.process_movement(useSteer, UsedDistance)
                    # print ' we found the sample! gonna bee-line to it... action = ', action
            self.DoSLAM = DoSLAM
        else:
            # if this is within distance to pick up the sample
            if TheDistance < 0.25:
                action = 'sample'
                # print ' taking sample because distance < 0.25 units. action = ', action
            else:
                if not TheKey:
                    # We have already found a sample! --> will use SLAM with offset as initialx and initialy
                    #   activate DoSLAM
                    DoSLAM = self.DoSLAM
                    measurex, measurey = DoSLAM.process_measurements(measurements) # Comes out with x, y from initialx and initialy --> found form the first sample
                    # print ' we are going to figure out where we are... measurex = ', measurex, ' measurey = ', measurey, ' heading = ', self.heading
                    # find the next nearest available sample.
                    NextS = self.FindNextS([measurex, measurey], sample_todo)
                    # print ' we are finding the next nearest available sample: NextS = ', NextS
                    dx = NextS[0] - measurex
                    dy = NextS[1] - measurey
                    TheBearing = self.truncateAngle(math.atan2(dy, dx))
                    distance = math.sqrt(dx**2 + dy**2)
                    steering = self.TruncateOfSteer(TheBearing - self.heading)
                    # print ' try to go to steering = ', steering, ' | distance = ', distance
                    if math.fabs(steering) > self.max_steering:
                        UsedBearing = self.max_steering * (steering / math.fabs(steering))
                        UsedDistance = self.max_distance * 0.01
                    else:
                        UsedBearing = steering
                        # Cap the distance and bearing to
                        if distance > self.max_distance:
                            UsedDistance = self.max_distance
                        elif distance < 0.001:
                            # IF by chance our calculation does not get us close enough. take a random step.
                            action, UsedBearing, UsedDistance = self.generatePatrolAction()
                            self.foundSampleFlagonce = False
                            # randomstep = random.uniform(1, 3)
                            # UsedDistance = self.max_distance
                            # if randomstep == 1:
                            #     UsedBearing = self.max_steering
                            # elif randomstep == 2:
                            #     UsedBearing = 0.0
                            # else:
                            #     UsedBearing = -self.max_steering
                        else:
                            UsedDistance = distance
                    # get the position after movement
                    motionx, motiony = DoSLAM.process_movement(UsedBearing, UsedDistance)
                    # self.heading = self.truncateAngle(self.heading + UsedBearing)
                    action = 'move ' + str(UsedBearing) + ' ' + str(UsedDistance)
                    # print ' taking action: ', action
                    self.DoSLAM = DoSLAM
                else:
                    UsedDistance = TheDistance
                    # useSteer = self.TruncateOfSteer(TheBearing - self.heading)
                    useSteer = self.TruncateOfSteer(TheBearing)
                    # print ' BEE LINE PART: try to go steering: ', useSteer, ' | distance: ', UsedDistance, ' | current heading = ', self.heading, ' | trying to go to bearing = ', TheBearing
                    if math.fabs(useSteer) > self.max_steering:
                        useSteer = self.truncateAngle(self.max_steering * (useSteer/math.fabs(useSteer)))
                        UsedDistance = self.max_distance * 0.01
                    else:
                        if UsedDistance > self.max_distance:
                            UsedDistance = self.max_distance

                    action = 'move ' + str(useSteer) + ' ' + str(UsedDistance)
                    # self.heading = self.truncateAngle(self.heading + useSteer)
                    # resetting the patrol positions.
                    self.patrolTurnCount = 2
                    self.patrolStraightMax = 1
                    self.patrolStraightCount = 1
                    motionx, motiony = DoSLAM.process_movement(useSteer, UsedDistance)
                    self.DoSLAM = DoSLAM
        # print ' final action = ', action
        # TODO
#6------------------------------------------------------------------------------------------------------------        
        #raise NotImplementedError

        #distance = 1.0
        #steering = 1.570963
        #action = 'move ' + str(steering) + ' ' + str(distance)

        return action
