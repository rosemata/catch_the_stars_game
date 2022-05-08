import serial
from pyfirmata import SERVO
from time import sleep
import time
from pyfirmata import Arduino, util, STRING_DATA
import pyfirmata
import turtle
import os
import random
import math

import serial.tools.list_ports

# print all the port name connected
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p)

# tries to connect to port and arduino
try:
  port = '/dev/cu.usbmodem101'
  board = Arduino(port)
  print("Successful")
except:
  print("Not successful")


# connects to pin from arduino to python
it = pyfirmata.util.Iterator(board)
it.start()
board.digital[10].mode = pyfirmata.INPUT
board.digital[9].mode = pyfirmata.INPUT
it = pyfirmata.util.Iterator(board)
it.start()



#----------------------------------------------------
#set up screen

screen = turtle.Screen()          #create screen
screen.setup(500,500)          #set up screen size
screen = turtle.getscreen()
screen.bgpic("bg_galaxy.gif")

#----------------------------------------------------
#main player
player = turtle.Turtle()
screen.addshape("spaceship.gif")
player.shape("spaceship.gif")
# player.penup()
player.speed(0)
player.setposition(0,-220)
player.setheading(90)
playerspeed = 15

#----------------------------------------------------
#player, move left and right
def move_right():
    x = player.xcor()
    x += playerspeed #takes current value of x subtract player speed
    if x > 250:
      x = -250
    player.setx(x)
  
def move_left():
  x = player.xcor()
  x -= playerspeed #takes current value of x subtract player speed
  if x < -250:
    x = 250
  player.setx(x)
  
#----------------------------------------------------
# collision function
def isCollision(t1,t2):
  distance = math.sqrt(math.pow(t1.xcor()-t2.xcor(),2)+math.pow(t1.ycor()-t2.ycor(),2))
  if distance < 15:
    return True
  else:
    return False
#----------------------------------------------------
# game over function   
def game_over():
  
  t1 = turtle.Turtle()
  t1.hideturtle()
  t1.penup()
  t1.color("white")
  t1.goto(-150,0)
  t1.write("GAME OVER", font=("Times", 40, "bold"))
  t1.hideturtle()

#----------------------------------------------------
# main 
def main():
  
#----------------------------------------------------
# harm
  enemy = turtle.Turtle()
  screen.addshape("asteroid.gif")
  enemy.shape("asteroid.gif")
  enemy.penup()
  enemy.speed(0)
  enemy.setposition(-150,300)
  enemyspeed = 10

#----------------------------------------------------
# benefit
  energy = turtle.Turtle()
  screen.addshape("star.gif")
  energy.shape("star.gif")
  energy.penup()
  energy.speed(0)
  energy.setposition(150,300)
  
  energyspeed = 10
#----------------------------------------------------
# set up score
  score=0
  score_pen = turtle.Turtle()
  score_pen.speed(0)
  score_pen.color("white")
  score_pen.penup()
  score_pen.setposition(-200, 200)
  scorestring = "Score: %s" %score 
  score_pen.write(scorestring, False, align="left", font=("Arial", 14, "normal"))
  score_pen.hideturtle()
#----------------------------------------------------
# set up life score  
  life=int(3)
  life_pen = turtle.Turtle()
  life_pen.speed(0)
  life_pen.color("white")
  life_pen.penup()
  life_pen.setposition(180, 200)
  lifestring = "Lives: %s" %life 
  life_pen.write(lifestring, False, align="left", font=("Arial", 14, "normal"))
  life_pen.hideturtle()
  
  scorestring = "Score: 0"
  lifestring = "Lives: 3"
  
  # send string to arduino program
  # to print to lcd
  data = scorestring + lifestring
  board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(data))
  
#----------------------------------------------------
# main loop  
  while True:
    data = scorestring
    # read input from pin 10 from arduino
    right = board.digital[10].read()
    # read input from pin 9 from arduino
    left = board.digital[9].read()
    
    # if theres input from pin 10, move the character right
    if right is True:
      move_right();
      # print(right)
      
    # if theres input from pin 10, move the character left
    if left is True:
      move_left();
      # print(left)
      time.sleep(0.1)
    
    # WHILE LOOP IS TRUE KEEP DECREMENTING Y VALUE OF STAR/ASTEROID
    # SO IT WILL SEEM LIKE FALLING
    # EVERYTIME THE LOOP REACHES BOTTOM OF SCREEN
    # IT WILL GO BACK TO TOP AND START AT RANDOM X VALUE USING RANDINT
    
    y = energy.ycor()
    y -= energyspeed
    energy.sety(y)
    
    y = enemy.ycor()
    y -= enemyspeed
    enemy.sety(y)
    
    
    if energy.ycor() == -250:
      x_random = random.randint(-250,250)
      energy.goto(x_random,250)
      
    if enemy.ycor() == -250:
      x_random = random.randint(-250,250)
      enemy.goto(x_random,250)
    
    # IF PLAYER COLLIDE WITH STAR
    if isCollision(player, energy):
      energy.hideturtle()
      score += 1
      
      scorestring= "Score: %s" %score
      # SEND DATA TO ARDUINO
      board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(" "))
      board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(" "))
      
      data = scorestring + lifestring
      # SEND DATA TO ARDUINO
      board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(data))
      score_pen.clear()
      score_pen.write(scorestring, False, align="left", font=("Arial", 14, "normal"))
      x_random = random.randint(-250,250)
      energy.goto(x_random,250)
      energy.showturtle()
      
    # IF PLAYER COLLIDE WITH  ASTEROID
    if isCollision(player, enemy):
      player.hideturtle()
      if life > 0:
        life -= 1
        lifestring= "Lives: %s" %life
        board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(" "))
        board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(" "))
        data = scorestring + lifestring
        board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(data))
        life_pen.clear()
        # board.send_sysex( STRING_DATA, util.str_to_two_byte_iter(data))
        life_pen.write(lifestring, False, align="left", font=("Arial", 14, "normal"))
        x_random = random.randint(-250,250)
        enemy.goto(x_random,250)
        energy.goto(x_random,250)
        energy.showturtle()
        enemy.showturtle()
        player.showturtle()
      if life == 0:
        game_over()
        enemy.hideturtle()
        energy.hideturtle()
        player.hideturtle()
        break
    
    
    


while True:
  main()