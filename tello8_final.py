from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import pygame
import numpy as np
import time

# Speed of the drone
S = 80
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
# pygame
FPS = 100
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

## Set True for webcamera testing without drone
inTest = True
cap = cv2.VideoCapture(0)  #set to -1 in Ubuntu OS


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.
            - Ν switch to auto navigate mode
    """

    def __init__(self):

        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([640, 480])
        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 80

        self.nav = "";
        self.nav_control = False;

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

        if not inTest:
            # Init Tello object that interacts with the Tello drone
            self.tello = Tello()
            self.send_rc_control = False
        else:
            cap = cv2.VideoCapture(-1)
            self.send_rc_control = True

    def run(self):
        print("run self")
        if not inTest:
            self.tello.connect()
            self.tello.set_speed(self.speed)

            # In case streaming is on. This happens when we quit this program without the escape key.
            self.tello.streamoff()
            self.tello.streamon()

            frame_read = self.tello.get_frame_read()
            frame = frame_read.frame
        else:
            ret, frame = cap.read()
            self.update()

        should_stop = False
        while not should_stop:
            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if not inTest:
                if frame_read.stopped:
                    break

            self.screen.fill([0, 0, 0])

            if not inTest:
                frame = frame_read.frame
                # battery n.
                textBat = "Bat: {}%".format(self.tello.get_battery())
            else:
                ret, frame = cap.read()
                textBat = "No drone 0";

            id, dis = self.recognize(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            font = pygame.font.SysFont("Arial", 36)
            txtsurf = font.render("Id:"+str(id)+" dist:"+str(dis), True, (255,255,255))
            txtsurf2 = font.render(str(textBat)+"- Nav:"+self.nav, True, (255,255,255))
            self.screen.blit(txtsurf, (200 - txtsurf.get_width() // 2, 150 - txtsurf.get_height() // 2))
            self.screen.blit(txtsurf2, (200 - txtsurf2.get_width() // 2, 200 - txtsurf2.get_height() // 2))
            pygame.display.update()
            time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        self.send_rc_control = True
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
            key：pygame
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_n:  # auto navigate
            self.nav_control = not self.nav_control
        elif key == pygame.K_l or key == pygame.K_SPACE:  # land
            not self.tello.land()
        self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello.

        """
        if self.send_rc_control:
            if not inTest:
                self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity, self.yaw_velocity)
                self.send_rc_control = False
            else:
                print(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity, self.yaw_velocity)
                self.send_rc_control = False

    def recognize(self,frame):
        # parameters for camera distortion
        dist = np.array(([[-0., 0., -0., -0., 0]]))
        mtx = np.array([[921, 0., 459], [0., 919, 351], [0., 0., 1]])

        font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
        h1, w1 = frame.shape[:2]

        # Read the camera picture
        # Correct distortion
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
        dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        x, y, w1, h1 = roi
        dst1 = dst1[y:y + h1, x:x + w1]
        frame = dst1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        #dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # Use aruco The detectmarkers() function can detect the marker and return the ID and the coordinates of the four corners of the sign board
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #   If you can't find it, type id
        if ids is not None:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)  # rotation vec + translation vec
            # Estimate the attitude of each marker and return the values rvet and tvec --- different
            # from camera coeficcients
            (rvec - tvec).any()  # get rid of that nasty numpy value array error

            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                aruco.drawDetectedMarkers(frame, corners)
                pos = corners[0][0][0]
                tvec[i, 0, 2] = tvec[i, 0, 2] * 1.8
                ftvec = [round(num, 2) for num in tvec[i, 0]]

                cv2.putText(frame, "Translation: " + str(ftvec[2]), (0, 244), font, .5, (0, 80, 80), 2, cv2.LINE_AA)

            # auto navigate enabled
            if self.nav_control:
                self.navigate(ftvec[2],pos, ids[0])
            return ids[0], ftvec[2]
        else:
            ##### DRAW "NO IDS" #####
            return -1, 0

    def navigate(self, dis, pos, id):
        # these commands are for specific position of Aruco markers in our playground
        self.nav = ""
        if id == 0:
            self.nav += "TakeOff and Up 60"
            if not inTest:
                self.tello.takeoff()
                self.tello.move_up(60)

        if id == 1:
            self.nav += "Land "
            if not inTest:
                self.tello.land()

        if id == 6:
            self.nav += "Rotate 180 and Fw 200"
            if not inTest:
                self.tello.rotate_clockwise(180)
                self.tello.move_forward(200)

        if id == 7:
            self.nav += "Rotate 90 "
            if not inTest:
                self.tello.rotate_clockwise(90)

        if id == 10:
            self.nav += "Rt 250 "
            if not inTest:
                self.tello.move_right(250)

        if id == 11:
            self.nav += "Flip + Rt 180 + Up50 "
            if not inTest:
                self.tello.flip_right()
                self.tello.move_right(180)
                self.tello.move_up(50)


def main():
    frontend = FrontEnd()
    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
