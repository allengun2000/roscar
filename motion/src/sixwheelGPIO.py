import pygame, sys
from pygame.locals import *
import ASUS.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode (GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.output(11,GPIO.LOW)
GPIO.setup(12, GPIO.OUT)
GPIO.output(12,GPIO.LOW)
GPIO.setup(13, GPIO.OUT)
GPIO.output(13,GPIO.LOW)
GPIO.setup(16, GPIO.OUT)
GPIO.output(16,GPIO.LOW)

pygame.init()
BLACK = (0,0,0)
WIDTH = 500
HEIGHT = 500
windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

windowSurface.fill(BLACK)

while True:
   for event in pygame.event.get():
      if event.type == QUIT:
            pygame.quit()
            sys.exit()
      if event.type == KEYDOWN:
         key = event.key
         if key == pygame.K_w:
            GPIO.output(11,GPIO.LOW)
            GPIO.output(12,GPIO.HIGH)
 	      GPIO.output(13,GPIO.LOW)
            GPIO.output(16,GPIO.HIGH)
            print("w")
         if key == pygame.K_d:
            GPIO.output(11,GPIO.LOW)
            GPIO.output(12,GPIO.HIGH)
 	      GPIO.output(13,GPIO.HIGH)
            GPIO.output(16,GPIO.LOW)
            print("d")
         if key == pygame.K_s:
            GPIO.output(11,GPIO.HIGH)
            GPIO.output(12,GPIO.LOW)
 	      GPIO.output(13,GPIO.HIGH)
            GPIO.output(16,GPIO.LOW)
            print("s")
         if key == pygame.K_a:
            GPIO.output(11,GPIO.HIGH)
            GPIO.output(12,GPIO.LOW)
 	      GPIO.output(13,GPIO.LOW)
            GPIO.output(16,GPIO.HIGH)
            print("a")
      if event.type == KEYUP:
            GPIO.output(11,GPIO.LOW)
            GPIO.output(12,GPIO.LOW)
 	      GPIO.output(13,GPIO.LOW)
            GPIO.output(16,GPIO.LOW)
            print("stop")
