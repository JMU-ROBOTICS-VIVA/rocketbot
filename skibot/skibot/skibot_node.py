#!/usr/bin/env python
"""Skibot

A simulated sliding robot with a ROS interface.  Intended to be a more
physically realistic alternative to turtlesim.

Note that the physics here are crude.  Linear and angular friction are
modeled separely.  In real life they interact in complicated ways:

https://physics.aps.org/story/v11/st26

Subscribed topics:
  thrust    (geometry_msgs/Wrench)
  target_pose  (skibot/Pose)  Target pose (for visualization)
  target_point  (geometry_msgs/Point)  Target point (for visualization)


Published topics:
  pose  (skibot/Pose)  Current robot pose

Services:
  teleport (skibot/Teleport) teleport to the indicated location and stop

(0, 0) is at the lower left edge of the screen.

Author: Nathan Sprague

Upgrade to ROS2: Mridul Pareek, 14/JUN/2020

"""
# Some code taken from :
# http://www.gpwiki.org/index.php/Python:Pygame_basics


import time
import pygame
import numpy as np

# Import the new ros2 libraries
import rclpy
from rclpy.node import Node

# in ros2 custom interfaces are only supported by cpp so have to make a separate
# ros2 package for only the interface in cpp using ament_cmake
# https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/
from skibot_interfaces.msg import Pose
# there is no need for teleportResponse, instead you can return response.success from the callback
from skibot_interfaces.srv import Teleport

from pygame.locals import *
import pygame.transform
import pygame.image

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Point

# Physical constants.
GRAVITY = 9.8  # m/s^2
MU = .03  # Coefficient of Friction of the ice

PIXELS_PER_METER = 120
SCREEN_WIDTH_PX = 480
SCREEN_HEIGHT_PX = 480
SCREEN_WIDTH_M = float(SCREEN_WIDTH_PX) / PIXELS_PER_METER
SCREEN_HEIGHT_M = float(SCREEN_HEIGHT_PX) / PIXELS_PER_METER


def pos_to_pixels(pos):
    """ Convert from meters to screen position. """
    return (int(pos[0] * PIXELS_PER_METER),
            SCREEN_HEIGHT_PX - int(pos[1] * PIXELS_PER_METER))


class Skibot(object):
    """ Sliding robot. """

    # Robot attributes...
    MAX_FORCE = 5.0
    MAX_TORQUE = .2

    MASS = 1.0  # kg
    WIDTH = .3  # Meters

    # Calculated quantities...
    LINEAR_FRICTION = MU * MASS * GRAVITY
    MOMENT_OF_INERTIA = .5 * MASS * (WIDTH / 2.0) ** 2
    TORQUE_FRICTION = .66 * MU * MASS * GRAVITY * (WIDTH / 2.0)

    def __init__(self, screen, pos, theta):
        self._screen = screen
        self.set_pose(pos, theta)
        self.set_vel_zero()
        self.image = pygame.image.load('/home/mridul/Downloads/indigo.png')
        height_px = 48
        width_px = 48

        self.image = pygame.transform.smoothscale(self.image, (width_px,
                                                               height_px))
        self.image.convert()

    def set_pose(self, pos, theta):
        """ Set pose """
        self.pos = np.array(pos, dtype='float64')
        self.theta = theta

    def set_vel_zero(self):
        """ Set all velocities to zero. """
        self.vel = np.array([0, 0], dtype='float64')
        self.vel_rot = 0.0

    def update(self, wrench, dt):
        """Update the position and velocity based on the thrust.  Redraw the
        robot at the new position.

        """
        # https://physics.stackexchange.com/questions/349451/
        # expression-for-angular-friction

        # Angular Component
        # First, calculate angular velocity in the absense of friction

        torque = np.clip(wrench.torque.z, -self.MAX_TORQUE, self.MAX_TORQUE)
        angular_acc = torque / self.MOMENT_OF_INERTIA
        no_fric_vel = self.vel_rot + angular_acc * dt

        # Friction acts in the opposite direction...
        angular_acc_fric = (-np.sign(no_fric_vel) * self.TORQUE_FRICTION /
                            self.MOMENT_OF_INERTIA)
        angular_vel_fric = no_fric_vel + angular_acc_fric * dt

        # Friction can't turn something backwards...
        if np.sign(angular_vel_fric) != np.sign(no_fric_vel):
            angular_vel_fric = 0

        self.vel_rot = angular_vel_fric

        # Linear component

        # FIRST, calculate velocity in the absense of friction
        force = np.clip(wrench.force.x, -self.MAX_FORCE, self.MAX_FORCE)
        linear_acc = (force / self.MASS)
        x_acc = np.sin(self.theta + np.pi / 2) * linear_acc
        y_acc = np.cos(self.theta + np.pi / 2) * linear_acc
        acc = np.array([x_acc, y_acc], dtype='float32')
        no_fric_vel = self.vel + acc * dt

        # Friction acts in the opposite direction...
        if np.linalg.norm(no_fric_vel) > 0:
            force_friction = (-(no_fric_vel / np.linalg.norm(no_fric_vel)) *
                              self.LINEAR_FRICTION)
        else:
            force_friction = np.zeros(2)

        accel_fric = force_friction / self.MASS
        fric_vel = no_fric_vel + accel_fric * dt

        # Friction can't push something backwards...
        fric_vel[np.sign(fric_vel) != np.sign(no_fric_vel)] = 0
        self.vel = fric_vel

        # Finally... Update the pose.
        self.pos += self.vel * dt
        self.theta += self.vel_rot * dt

        # Handle bouncing.
        if self.pos[0] > float(self._screen.get_width()) / PIXELS_PER_METER:
            self.pos[0] = float(self._screen.get_width()) / PIXELS_PER_METER
            self.vel[0] = -self.vel[0]
        if self.pos[0] < 0:
            self.pos[0] = 0
            self.vel[0] = -self.vel[0]
        if self.pos[1] > float(self._screen.get_height()) / PIXELS_PER_METER:
            self.pos[1] = float(self._screen.get_height()) / PIXELS_PER_METER
            self.vel[1] = -self.vel[1]
        if self.pos[1] < 0:
            self.pos[1] = 0
            self.vel[1] = -self.vel[1]

        # Draw the robot.
        surf = pygame.transform.rotozoom(self.image,
                                         np.rad2deg(self.theta), 1.0)
        pixel_x = (self.pos[0] * PIXELS_PER_METER - surf.get_rect().width
                   * .5)
        pixel_y = (self.pos[1] * PIXELS_PER_METER - surf.get_rect().height
                   * .5)
        self._screen.blit(surf, (pixel_x, pixel_y))

# skibotnode inherits from Node class we imported earlier


class SkibotNode(Node):
    """ ROS Skibot node. """

    def __init__(self):

        # Initializing the node and setting up subscriptions
        super().__init__('skibot_node')
        self.subscription = self.create_subscription(
            Wrench, 'thrust', self.wrench_callback, 10
        )
        self.subscription = self.create_subscription(
            Pose, 'target_pose', self.target_pose_callback, 10
        )
        self.subscription = self.create_subscription(
            Point, 'target_point', self.target_point_callback, 10
        )
        self.loc_pub = self.create_publisher(Pose, 'pose', 10)

        self.target_pose = None
        self.target_point = None

        self.pub_rate = 35.0
        # creating the teleport service
        self.srv = self.create_service(
            Teleport, 'teleport', self.handle_teleport_service)

        # Start pygame...
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH_PX,
                                               SCREEN_HEIGHT_PX))
        self.refresh_rate = 100
        pygame.display.set_caption('Skibot 354')
        self.rocket = Skibot(self.screen, (SCREEN_WIDTH_M / 2,
                                           SCREEN_HEIGHT_M / 2), 0.0)

        # load and prep arrow image.
        # arrow_file = roslib.packages.resource_file('skibot', 'images',
        # 'arrow.png')

        # temporary solution... couldnt find any other way to load images
        self.arrow_img = pygame.image.load(
            '/home/mridul/Downloads/arrow_file.png')
        self.arrow_img = pygame.transform.smoothscale(self.arrow_img,
                                                      (38, 8))
        square = pygame.Surface((38, 38), flags=SRCALPHA)
        square.fill((255, 255, 255, 0))
        square.blit(self.arrow_img, (0, 15))
        self.arrow_img = square
        self.arrow_img.convert()

        self.cur_wrench = Wrench()
        self.thrust_start = 0
        # created a timer instead of having a while loop in run, it will invoke the run every 0.1 seconds
        self.timer = self.create_timer(1 / self.refresh_rate, self.run)

    def wrench_callback(self, wrench):
        print(wrench)
        self.thrust_start = time.time()
        self.cur_wrench = wrench

    def target_pose_callback(self, pose_msg):
        self.target_pose = pose_msg
        self.target_point = None

    def target_point_callback(self, point_msg):
        self.target_point = point_msg
        self.target_pose = None

    # service callback accepts an additional response parameter which is now returned instead
    def handle_teleport_service(self, teleport_srv, response):
        """ Move the skibot to the goal location. """

        if (teleport_srv.x < 0 or teleport_srv.x > SCREEN_WIDTH_M or
                teleport_srv.y < 0 or teleport_srv.y > SCREEN_HEIGHT_M):
            self.get_logger().info("Invalid teleport request: {}".format(teleport_srv))
            response.success = False
            return response
        while self.rocket is None:
            time.sleep(.1)
        self.rocket.set_pose((teleport_srv.x, teleport_srv.y),
                             teleport_srv.theta)
        self.rocket.set_vel_zero()
        self.wrench = Wrench()
        response.success = True

        return response

    # removed the while loop from here as it was not needed
    def run(self):
        last_pub = 0.0

        # destroy the timer when we get the quit from pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_timer(self.timer)

        self.screen.fill((255, 255, 255))

        if ((self.cur_wrench.force.x != 0 or
             self.cur_wrench.torque.z != 0) and
                time.time() > self.thrust_start + .6):
            # Stop obeying last wrench after .6 seconds.
            self.cur_wrench = Wrench()

        self.rocket.update(self.cur_wrench, 1.0 / self.refresh_rate)

        if self.target_pose is not None:
            pixel_pos = pos_to_pixels((self.target_pose.x,
                                       self.target_pose.y))
            angle = np.rad2deg(self.target_pose.theta)
            surf = pygame.transform.rotozoom(self.arrow_img,
                                             angle, 1.0)
            self.screen.blit(surf, (pixel_pos[0] -
                                    surf.get_rect().width * .5,
                                    (pixel_pos[1] -
                                     surf.get_rect().height * .5)))
        elif self.target_point is not None:
            pixel_pos = pos_to_pixels((self.target_point.x,
                                       self.target_point.y))
            pygame.draw.circle(self.screen, (0, 255, 0), pixel_pos, 5)

        pygame.display.flip()

        if (time.time() > (last_pub + 1.0 / self.pub_rate -
                           1 / self.refresh_rate)):
            angle = (self.rocket.theta + np.pi) % (2 * np.pi) - np.pi
            pose = Pose()
            pose.x = self.rocket.pos[0]
            pose.y = float(SCREEN_HEIGHT_PX) / \
                PIXELS_PER_METER - self.rocket.pos[1]
            pose.theta = angle
            pose.x_velocity = self.rocket.vel[0]
            pose.y_velocity = self.rocket.vel[1]
            pose.theta_velocity = self.rocket.vel_rot
            # publish the message
            self.loc_pub.publish(pose)
            last_pub = time.time()

# Adjusted the init sequence as per other ros2 examples.


def main(args=None):
    rclpy.init(args=args)
    node = SkibotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()
