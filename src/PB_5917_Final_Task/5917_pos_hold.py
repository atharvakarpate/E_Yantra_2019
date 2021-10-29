#!/usr/bin/env python


"""
* Id : 5917
* Author : Atharva Karpate
* Filename: task_3_pos_hold.py
* Theme: Pollinator Bee
* Functions: __init__,arm,disarm,position_hold,calc_pid,pid_roll,pid_pitch,pid_throat,pid_yaw,limit,publish_plot_data,set_pid_alt,
            set_pid_pitch,set_pid_roll,set_pid_yaw,get_pose,set_yaw
* Global Variables: None 
* Description: Implements Position hold for multiple waypoints(pollinations). 
"""
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time


class DroneFly:
    """docstring for DroneFly"""

    """
        * Function Name: __init__
        * Input: None(self)
        * Output: Initialises nodes,PID constants and subscribes as well as publishes to diffrent topics.
        * Logic: Just Initialisation.
        * Example Call: As soon as object is decalared.
    """

    def __init__(self):

        rospy.init_node("pluto_fly", disable_signals=True)

        self.pluto_cmd = rospy.Publisher("/drone_command", PlutoMsg, queue_size=10)

        rospy.Subscriber("whycon/poses", PoseArray, self.get_pose)

        self.pub = rospy.Publisher(
            "throtError", Float64, queue_size=1000
        )  # Data published for throttle(altitude),yaw,pitch and roll error respectively.
        self.pub1 = rospy.Publisher("roll_error", Float64, queue_size=1000)
        self.pub2 = rospy.Publisher("pitch_error", Float64, queue_size=1000)
        self.pub3 = rospy.Publisher("yaw_error", Float64, queue_size=1000)
        self.pub4 = rospy.Publisher("zeroline", Float64, queue_size=1000)

        # To tune the drone during runtime
        rospy.Subscriber("/pid_tuning_altitude", PidTune, self.set_pid_alt)
        rospy.Subscriber("/pid_tuning_roll", PidTune, self.set_pid_roll)
        rospy.Subscriber("/pid_tuning_pitch", PidTune, self.set_pid_pitch)
        rospy.Subscriber("/pid_tuning_yaw", PidTune, self.set_pid_yaw)
        rospy.Subscriber(
            "/drone_yaw", Float64, self.set_yaw
        )  # To get data for drone Yaw from /drone_yaw

        rospy.Subscriber(
            "/red", Float64, self.set_red
        )  # To get number of red flowers pollinated
        rospy.Subscriber(
            "/blue", Float64, self.set_blue
        )  # To get number of blue flowers pollinated
        rospy.Subscriber(
            "/green", Float64, self.set_green
        )  # To get number of green flowers pollinated
        rospy.Subscriber(
            "/points", Float64, self.set_points
        )  # To get total number of flowers pollinated

        self.cmd = PlutoMsg()

        self.Tprevious_error = 0  # previous_error used to store the previous errors of various variables for PID
        self.TIterm = (
            0  # Iterm used to store the sum of errors of various variables for PID
        )
        self.Yprevious_error = 0
        self.YIterm = 0
        self.Rprevious_error = 0
        self.RIterm = 0
        self.Pprevious_error = 0
        self.PIterm = 0
        self.Yawvalue = 0
        self.Cyawvalue = 0
        self.redValue = 0

        # Position to hold.
        self.wp_x = 0
        self.wp_y = 0
        self.wp_z = 20

        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.cmd.plutoIndex = 0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        # PID constants for Roll
        self.kp_roll = 7.0
        self.ki_roll = 0.972
        self.kd_roll = 925.0
        # PID constants for Pitch
        self.kp_pitch = 7.0
        self.ki_pitch = 0.96
        self.kd_pitch = 920.0

        # PID constants for Yaw
        self.kp_yaw = 5.0
        self.ki_yaw = 0.0
        self.kd_yaw = 0.0

        # PID constants for Throttle
        self.kp_throt = 50.0
        self.ki_throt = 0.985
        self.kd_throt = 380.0

        # Correction values after PID is computed
        self.correct_roll = 0.0
        self.correct_pitch = 0.0
        self.correct_yaw = 0.0
        self.correct_throt = 0.0

        # Loop time for PID computation. You are free to experiment with this
        self.last_time = 0.0
        self.loop_time = 0.032

        self.flag = 0
        self.flag1 = 0

        self.redValue = 0
        self.greenValue = 0
        self.blueValue = 0
        self.points = 0

        rospy.sleep(0.1)

    """
        * Function Name: arm
        * Input: None(self)
        * Output: Publishes the rcAUX4 and rcThrottle values to arm the drone
        * Logic: Sets rcAUX4=1500 and rcThrottle = 1000 to arm the drone
        * Example Call: arm()
    """

    def arm(self):
        self.cmd.rcAUX4 = 1500
        self.cmd.rcThrottle = 1000
        self.pluto_cmd.publish(self.cmd)
        self.Yawvalue = 0
        rospy.sleep(0.1)

    """
        * Function Name: disarm
        * Input: None(self)
        * Output: Publishes the rcAUX4 value to disarm the drone
        * Logic: Sets rcAUX4=1100 to disarm the drone
        * Example Call: disarm()
    """

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(0.1)

    """
        * Function Name: initdrone
        * Input: None(self)
        * Output: Disarms and arms the drone
        * Logic: disarm() and arm() functions are called
        * Example Call: initdrone()
    """

    def initdrone(self):
        self.flag = 0
        rospy.sleep(2)
        print "disarm"
        self.disarm()
        rospy.sleep(0.2)
        print "arm"
        self.arm()
        rospy.sleep(0.1)
        pass

    """
        * Function Name: position_hold
        * Input: None(self)
        * Output: Disarms and arms the drone.Publishes rcPitch,rcRoll,rcThrottle,rcYaw for position hold
        * Logic: Calculates correction values for pitch,roll,throttle and yaw and publishes
        * Example Call: position_hold()
    """

    def position_hold(self):

        while True:
            self.publish_plot_data()  # Used to publish error data on various topics
            self.calc_pid()
            erLimit = 0.6
            erlLimit = 0.3

            if self.flag == 0:
                # flower 1
                # break the loop when position hold is within the specified limits.
                if (
                    ((self.Terror < erLimit) & (self.Terror > -erLimit))
                    & ((self.Perror < erLimit) & (self.Perror > -erLimit))
                    & ((self.Rerror < erLimit) & (self.Rerror > -erLimit))
                    & ((self.Yerror < erLimit) & (self.Yerror > -erLimit))
                ):
                    break
                # successful pollination of first flower
                if self.points == 1:
                    break

            if self.flag == 1:
                # flower 2
                # break the loop when position hold is within the specified limits.
                if (
                    ((self.Terror < erLimit) & (self.Terror > -erLimit))
                    & ((self.Perror < erLimit) & (self.Perror > -erLimit))
                    & ((self.Rerror < erLimit) & (self.Rerror > -erLimit))
                    & ((self.Yerror < erLimit) & (self.Yerror > -erLimit))
                ):
                    break
                # successful pollination of second flower
                if self.points == 2:
                    break

            if self.flag == 2:
                # flower 3
                # break the loop when position hold is within the specified limits.
                if (
                    ((self.Terror < erLimit) & (self.Terror > -erLimit))
                    & ((self.Perror < erLimit) & (self.Perror > -erLimit))
                    & ((self.Rerror < erLimit) & (self.Rerror > -erLimit))
                    & ((self.Yerror < erLimit) & (self.Yerror > -erLimit))
                ):
                    break
                # successful pollination of third flower
                if self.points == 3:
                    # print('Pollinated-3')
                    break

            if self.flag == 3:
                # beehive
                # break the loop when position hold is within the specified limits.
                if (
                    ((self.Terror < erlLimit) & (self.Terror > -erlLimit))
                    & ((self.Perror < erlLimit) & (self.Perror > -erlLimit))
                    & ((self.Rerror < erlLimit) & (self.Rerror > -erlLimit))
                    & ((self.Yerror < erLimit) & (self.Yerror > -erLimit))
                ):
                    break

            # Check your X and Y axis. You MAY have to change the + and the -.
            # We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
            pitch_value = int(1500 - self.correct_pitch)
            self.cmd.rcPitch = self.limit(pitch_value, 1700, 1300)

            roll_value = int(1500 + self.correct_roll)
            self.cmd.rcRoll = self.limit(roll_value, 1700, 1300)

            throt_value = int(1500 - self.correct_throt)
            self.cmd.rcThrottle = self.limit(throt_value, 1900, 1100)

            yaw_value = int(1500 + self.correct_yaw)  # To calculate correct yaw value
            self.cmd.rcYaw = self.limit(yaw_value, 1950, 1150)

            self.pluto_cmd.publish(self.cmd)

    """
        * Function Name: calc_pid
        * Input: None(self)
        * Output: Calls PID functions to calculate PID values in a loop time
        * Logic: Functions are called according to given Loop time for PID.
        * Example Call: calc_pid()
    """

    def calc_pid(self):
        self.seconds = time.time()
        current_time = self.seconds - self.last_time
        if current_time >= self.loop_time:
            self.pid_roll()
            self.pid_pitch()
            self.pid_throt()
            self.pid_yaw()

            self.last_time = self.seconds

    """
        * Function Name: pid_roll
        * Input: None(self)
        * Output: Sets the correct_roll value for the drone
        * Logic: Calculates propotional,derivative and integral terms and through these calculates the correct_roll through the PID equation
                correct_roll=kp_roll*error + ((error-Rprevious_error)*kd_roll) + RIterm        
        * Example Call: pid_roll()
    """

    def pid_roll(self):  # Calculates the correct value for Roll
        error = self.wp_x - self.drone_x
        self.RIterm = (self.RIterm + error) * self.ki_roll
        self.correct_roll = (
            self.kp_roll * error
            + ((error - self.Rprevious_error) * self.kd_roll)
            + self.RIterm
        )
        self.Rprevious_error = error

        # Compute Roll PID here

    """
        * Function Name: pid_pitch
        * Input: None(self)
        * Output: Sets the correct_pitch value for the drone
        * Logic: Calculates propotional,derivative and integral terms and through these calculates the correct_pithc through the PID equation
                correct_pitch=kp_pitch*error + ((error-Pprevious_error)*kd_pitch) + PIterm        
        * Example Call: pid_pithc()
    """

    def pid_pitch(self):  # Calculates the correct value for Pitch
        error = self.wp_y - self.drone_y
        self.PIterm = (self.PIterm + error) * self.ki_pitch
        self.correct_pitch = (
            self.kp_pitch * error
            + ((error - self.Pprevious_error) * self.kd_pitch)
            + self.PIterm
        )
        self.Pprevious_error = error

        # Compute Pitch PID here

    """
        * Function Name: pid_throt
        * Input: None(self)
        * Output: Sets the correct_throt value for the drone
        * Logic: Calculates propotional,derivative and integral terms and through these calculates the correct_throt through the PID equation
                correct_throt=kp_throt*error + ((error-Tprevious_error)*kd_throt) + TIterm        
        * Example Call: pid_throt()
    """

    def pid_throt(self):  # Calculates the correct value for Throttle(altitude)
        error = self.wp_z - self.drone_z

        self.TIterm = (self.TIterm + error) * self.ki_throt
        self.correct_throt = (
            (self.kp_throt * error)
            + ((error - self.Tprevious_error) * self.kd_throt)
            + self.TIterm
        )
        self.Tprevious_error = error

        # Compute Throttle PID here

    """
        * Function Name: pid_yaw
        * Input: None(self)
        * Output: Sets the correct_yaw value for the drone
        * Logic: Calculates propotional,derivative and integral terms and through these calculates the correct_yaw through the PID equation
                correct_yaw=kp_yaw*error + ((error-Yprevious_error)*kd_yaw) + YIterm        
        * Example Call: pid_yaw()
    """

    def pid_yaw(self):  # Calculates the correct value for Yaw
        error = self.Yawvalue - self.Cyawvalue

        self.YIterm = (self.YIterm + error) * self.ki_yaw
        self.correct_yaw = (
            (self.kp_yaw * error)
            + ((error - self.Yprevious_error) * self.kd_yaw)
            + self.YIterm
        )
        self.Yprevious_error = error

    """
        * Function Name: limit
        * Input: input_value, max_value, min_value
        * Output: Returns the  value within the max and min value limit.
        * Logic: If value is greater than max_value , max_value is given output.If value is less than min_value,min_value is given output.         
        * Example Call: limit(1800,1700,1300)
    """

    def limit(self, input_value, max_value, min_value):

        # Use this function to limit the maximum and minimum values you send to your drone

        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value

    """
        * Function Name: publish_plot_data()
        * Input: None(self)
        * Output: Publishes the throt,pitch,roll and yaw error.
        * Logic: Publishes values.         
        * Example Call: publish_plot_data()
    """

    def publish_plot_data(self):
        self.zeroline = 0
        self.Terror = self.wp_z - self.drone_z
        self.Rerror = self.wp_x - self.drone_x
        self.Perror = self.wp_y - self.drone_y
        self.Yerror = self.Yawvalue - self.Cyawvalue
        self.pub.publish(self.Terror)
        self.pub1.publish(self.Rerror)
        self.pub2.publish(self.Perror)
        self.pub3.publish(self.Yerror)
        self.pub4.publish(self.zeroline)  # Provides zeroline for reference

    """
        * Function Name: set_pid_alt
        * Input: pid_val
        * Output: Sets kp,ki and kd for alt.
        * Logic: Takes value from the PID trackbar and sets it to kp,ki and kd of alt.        
        * Example Call: set_pid_alt(pid_val)
    """

    def set_pid_alt(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

        self.kp_throt = pid_val.Kp
        self.ki_throt = pid_val.Ki
        self.kd_throt = pid_val.Kd

    """
        * Function Name: set_pid_roll
        * Input: pid_val
        * Output: Sets kp,ki and kd for roll.
        * Logic: Takes value from the PID trackbar and sets it to kp,ki and kd of roll.         
        * Example Call: set_pid_roll(pid_val)
    """

    def set_pid_roll(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

        self.kp_roll = pid_val.Kp
        self.ki_roll = pid_val.Ki
        self.kd_roll = pid_val.Kd

    """
        * Function Name: set_pid_pitch
        * Input: pid_val
        * Output: Sets kp,ki and kd for pitch.
        * Logic: Takes value from the PID trackbar and sets it to kp,ki and kd of pitch.         
        * Example Call: set_pid_pitch(pid_val)
    """

    def set_pid_pitch(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

        self.kp_pitch = pid_val.Kp
        self.ki_pitch = pid_val.Ki
        self.kd_pitch = pid_val.Kd

    """
        * Function Name: set_pid_yaw
        * Input: pid_val
        * Output: Sets kp,ki and kd for yaw.
        * Logic: Takes value from the PID trackbar and sets it to kp,ki and kd of yaw.         
        * Example Call: set_pid_yaw(pid_val)
    """

    def set_pid_yaw(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

        self.kp_yaw = pid_val.Kp
        self.ki_yaw = pid_val.Ki
        self.kd_yaw = pid_val.Kd

    """
        * Function Name: get_pose
        * Input: pose
        * Output: Sets drone_x,drone_y and drone_z.
        * Logic: Takes values for x,y,z from whycon coordinates.         
        * Example Call: get_pose(pose)
    """

    def get_pose(self, pose):

        # This is the subscriber function to get the whycon poses
        # The x, y and z values are stored within the drone_x, drone_y and the drone_z variables

        self.drone_x = pose.poses[0].position.x
        self.drone_y = pose.poses[0].position.y
        self.drone_z = pose.poses[0].position.z

    """
        * Function Name: set_yaw
        * Input: data
        * Output: Sets Cyawvalue for yaw of drone
        * Logic: Takes value from data.data and assigns it to Cyawvalue.         
        * Example Call: set_yaw(data)
    """

    def set_yaw(self, data):  # Provides the current yaw value from /drone_yaw
        self.Cyawvalue = data.data

    """
        * Function Name: set_red
        * Input: data
        * Output: Sets redValue equal to the number of red flowers pollinated
        * Logic: Takes value from data.data and assigns it to redValue.         
        * Example Call: set_red(data)
    """

    def set_red(self, data):
        self.redValue = data.data

    """
        * Function Name: set_blue
        * Input: data
        * Output: Sets blueValue equal to the number of blue flowers pollinated
        * Logic: Takes value from data.data and assigns it to blueValue.         
        * Example Call: set_blue(data)
    """

    def set_blue(self, data):
        self.blueValue = data.data

    """
        * Function Name: set_green
        * Input: data
        * Output: Sets greenValue equal to the number of green flowers pollinated
        * Logic: Takes value from data.data and assigns it to greenValue.         
        * Example Call: set_green(data)
    """

    def set_green(self, data):
        self.greenValue = data.data

    """
        * Function Name: set_points
        * Input: data
        * Output: Sets points equal to total number of flowers pollinated
        * Logic: Takes value from data.data and assigns it to points.         
        * Example Call: set_points(data)
    """

    def set_points(self, data):
        self.points = data.data


if __name__ == "__main__":
    temp = DroneFly()
    rospy.sleep(6)
    while not rospy.is_shutdown():

        print ("START")

        temp.initdrone()
        bee_hive = [-0.34, 6.5, 28.5]  # beehive location
        Hov_alt = 18.0  # max altitude for the bee to hover upon before pollinating the flowers
        Plant_Location = [
            (5.8, -1.64, 20.9),
            (0.1, -4.9, 24.7),
            (-5.4, 0.73, 24.7),
        ]  # co-ordinates where plants are placed

        temp.flag = 0
        temp.wp_x = bee_hive[0]
        temp.wp_y = (
            bee_hive[1] - 3
        )  # preventing bee from going out of the camera frame while it hovers above the beehive
        temp.wp_z = Hov_alt
        temp.position_hold()

        # position hold point between beehive and first plant for better stability
        temp.wp_x = (Plant_Location[0][0] + bee_hive[0]) / 2
        temp.wp_y = (Plant_Location[0][1] + bee_hive[1]) / 2
        temp.wp_z = Hov_alt
        temp.position_hold()

        for x in range(0, len(Plant_Location)):

            # assigning waypoints from Plant_Location Array
            temp.wp_x = Plant_Location[x][0]
            temp.wp_y = Plant_Location[x][1]
            temp.wp_z = (
                Plant_Location[x][2] - 3
            )  # -3 for making the bee hover above the plant
            n = 3

            # logic for maximizing the chances of pollination
            """n is set equal to 3 as bee is hovering at wp_z-3. The hovering altitude is lowered in steps by 1 and simultaneously 
            number of total detected flowers are checked. if (temp.points==(x+1)) i.e. if number of flowers detected increases by 1,
            this signifies detection of a new flower. For example in the first iteration of this loop, x=0 and temp.points=0 as at this
            point no flower is detected. As soon as a flower is detected, temp.points=1 which is equal to x+1 and this loop breaks, 
            signifying successful pollination. If even after decending the altitude exactly till the z co-ordinate of plant and n 
            becomes equal to 0, n is again set to 3, altitude is again increased to wp_z-3 and the same process continues till successful
            pollination is achieved."""
            while True:
                temp.position_hold()
                if temp.points == (x + 1):
                    break
                if n == 0:
                    n = 3
                    temp.wp_z = temp.wp_z - 3
                    continue
                n = n - 1
                temp.wp_z = temp.wp_z + 1

            # Next point between the current and the next plant is set for position hold for better stability.
            temp.flag = x + 1
            if (x + 1) < len(Plant_Location):
                temp.wp_x = (Plant_Location[x][0] + Plant_Location[x + 1][0]) / 2
                temp.wp_y = (Plant_Location[x][1] + Plant_Location[x + 1][1]) / 2
                temp.wp_z = Plant_Location[x][2] - 2.5
                temp.position_hold()

        print ("OUTPUT")
        print (
            "Pollination Done! Pollinated ",
            temp.redValue,
            " Red Daylily, ",
            temp.greenValue,
            " Green Carnation and ",
            temp.blueValue,
            " Blue Delphinium .",
        )
        print ("STOP")

        k = 7
        while k != 1:
            k = k - 2
            temp.wp_x = bee_hive[0]
            temp.wp_y = bee_hive[1]
            temp.wp_z = bee_hive[2] - k
            temp.position_hold()

        temp.disarm()
        print ("Landing successfully")

        rospy.spin()

