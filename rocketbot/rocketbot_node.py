"""ROCKET BOT

A simulated Rocket with a ROS interface.  Just intended to serve as an
alternative to turtlesim for playing with topics and messages.

Publishes to:
  location      (geometry_msgs/Point)
  target        (geometry_msgs/Point)  Latest mouse click position
                                       (published continously)
  target_event  (geometry_msgs/Point)  Latest mouse click position
                                       (only published when click occurs)


Subscribes to:
  thrust    (geometry_msgs/Vector3)

(0, 0, 0) is at the lower left edge of the screen.  The x axis points
to the right and the y axis points up.  Thrust along the z axis will
be ignored.  Negative thrusts along the y axis will be ignored.

Author: Nathan Sprague

Upgrade to ROS2: Mridul Pareek

"""
# Some code taken from :
# http://www.gpwiki.org/index.php/Python:Pygame_basics


import pygame
import numpy as np

# imports for ROS2
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from pygame.locals import *

import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

ACC_MULTIPLIER = .003
GRAVITY = 9.8


class Rocket(pygame.sprite.Sprite):
    """ This is a rocket sprite. """

    HEIGHT = 30
    WIDTH = 10
    FLAME_HEIGHT = 20
    CONE_HEIGHT = 10

    def __init__(self, screen, pos, vel):
        pygame.sprite.Sprite.__init__(self)
        self._screen = screen
        self.pos = np.array(pos, dtype='float64')
        self.vel = np.array(vel, dtype='float64')

    def update(self, thrust):
        """Update the position and velocity based on the thrust.  Redraw the
        rocket at the new position.

        """
        self.vel[1] += GRAVITY * ACC_MULTIPLIER
        self.vel += thrust * ACC_MULTIPLIER
        self.pos += self.vel

        if self.pos[0] > self._screen.get_width():
            self.pos[0] = self._screen.get_width()
            self.vel[0] = -self.vel[0]
        if self.pos[0] < 0:
            self.pos[0] = 0
            self.vel[0] = -self.vel[0]
        if self.pos[1] > self._screen.get_height() - self.HEIGHT:
            self.pos[1] = self._screen.get_height() - self.HEIGHT
            self.vel[1] = 0
        if self.pos[1] < 0:
            self.pos[1] = 0
            self.vel[1] = 0

        if thrust[1] != 0:
            flame_height = -thrust[1] / GRAVITY * self.FLAME_HEIGHT
            flame_height = min(flame_height, 2 * self.FLAME_HEIGHT)
            p1 = (self.pos[0], self.pos[1] + self.HEIGHT)
            p2 = (self.pos[0] + self.WIDTH - 1, self.pos[1] + self.HEIGHT)
            p3 = (self.pos[0] + self.WIDTH / 2,
                  self.pos[1] + self.HEIGHT + flame_height)
            pygame.draw.polygon(self._screen, (255, 0, 0), (p1, p2, p3), 0)

        p1 = self.pos
        p2 = (self.pos[0] + self.WIDTH - 1, self.pos[1])
        p3 = (self.pos[0] + self.WIDTH / 2, self.pos[1] - self.CONE_HEIGHT)
        pygame.draw.polygon(self._screen, (0, 0, 0), (p1, p2, p3), 0)

        pygame.draw.rect(self._screen, (0, 0, 0), Rect(self.pos, (10, 30)))


class RocketNode(Node):

    def __init__(self):

        super().__init__('rocket_bot')
        self.node_done = False
        self.subscription = self.create_subscription(Vector3, 'thrust',
                                                     self.thrust_callback, 10)
        self.loc_pub = self.create_publisher(Point, 'location', 10)
        self.target_pub = self.create_publisher(Point, 'target', 10)
        self.target_event_pub = self.create_publisher(Point, 'target_event', 10)
        self.target = Point()
        self.target.x = 240.0
        self.target.y = 100.0
        self.target.z = 0.0

        self.cur_thrust = np.array([0.0, 0.0])
        self.thrust_start = 0
        pygame.init()

        self.refresh_rate = 100
        self.width = 480
        self.screen = pygame.display.set_mode((self.width, self.width))
        self.pub_rate = 10.0

        pygame.display.set_caption('RocketBot 354')

        self.rocket = Rocket(
            self.screen, (self.width / 2, self.width), [0.0, 0.])
        self.last_pub = 0.0

        self.timer = self.create_timer(1 / self.refresh_rate, self.run)
        self.future = Future()

    def thrust_callback(self, thrust):
        self.thrust_start = time.time()
        if thrust.y < 0:
            thrust.y = 0.0
        self.cur_thrust = np.array([thrust.x, -thrust.y])

    def run(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.future.set_result("exited")
            if event.type == pygame.MOUSEBUTTONUP:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                w, h = pygame.display.get_surface().get_size()
                self.target.x = float(mouse_x)
                self.target.y = float(h - mouse_y)
                self.target_event_pub.publish(self.target)

        self.screen.fill((255, 255, 255))
        if ((self.cur_thrust[0] != 0 or self.cur_thrust[1] != 0) and
                time.time() > self.thrust_start + .6):
            self.cur_thrust = np.array([0, 0])

        self.rocket.update(self.cur_thrust)

        pygame.display.flip()

        if time.time() > self.last_pub + 1.0 / self.pub_rate - 1 / self.refresh_rate:
            point = Point()
            point.x = self.rocket.pos[0]
            point.y = self.width - self.rocket.pos[1] - self.rocket.HEIGHT
            point.z = 0.0
            self.loc_pub.publish(point)
            self.target_pub.publish(self.target)
            self.last_pub = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = RocketNode()
    rclpy.spin_until_future_complete(node, node.future)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
