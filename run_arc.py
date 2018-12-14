# run instruction: python2 run_arc.py

import pygame
import numpy
import sys
import os

import time
import numpy as np
import random
import datetime

import scipy

from math import *

from aux import *
from experiment import *

import robot.robot as robot

import subprocess

from Tkinter import * # use for python2
import tkMessageBox
import tkSimpleDialog


import json

import mmap
import comedi
import array


# The little control window
CONTROL_WIDTH,CONTROL_HEIGHT= 500,500 #1000,800 #450,400 # control window dimensions
CONTROL_X,CONTROL_Y = 500,50 # controls where on the screen the control window appears


EXPERIMENT = "arc"



N_ROBOT_LOG = 13 # how many columns go in the robot log file


# This is a global dict that holds all the configuration options.
# Using a single variable for them makes it easier to keep track.
global conf
conf = {}



# The background of the screen
conf["BGCOLOR"] = (127,127,127)

# The size of the window (for pygame)
#displaySize = (1280,720)  # for 16:9 screens
#conf["SCREENSIZE"] = (1024,768)  # for 16:10 screens (e.g. Mac)
conf["SCREENSIZE"] = (1920,1080) # inmotion screen
conf["SCREENPOS"] = (1600,0) # the offset (allows you to place the subject screen "on" the second monitor)

# The size of the active screen area for the experiment (in pixels)
conf["MAXX"],conf["MAXY"] = conf["SCREENSIZE"]

# The center of the screen
conf["CENTERX"],conf["CENTERY"] = conf["MAXX"]/2,conf["MAXY"]/2


conf["M_TO_PIXEL_X"]= 8*conf["MAXX"]/0.4
conf["M_TO_PIXEL_Y"]=-8*conf["MAXX"]/0.4

# Read the screen-to-robot calibration from file
# _,conf["calib"] = pickle.load(open('calib.pickle27','rb')) ## Calib is for the inmotion robot
#conf["calib"]["screen.robot.ratio"] = (conf["calib"]["slope.x"]+conf["calib"]["slope.y"])/2 # compute the approximate pixel-to-screen correspondence
#conf["calib"]=1


# The sizes of various objects
conf["CURSOR_RADIUS"] = .00025 # in robot coordinates (m)

# this is the display size of the target, not the size of the target area used for determining whether subjects are long enough "within" the target.
#TARGET_SIZE = 10 # for Stewart Bio
conf["TARGET_RADIUS"] = .0015 # for UdeM, rounded    # 22 Feb

# The cursor colour
conf["CURSOR_COLOUR"] = (0,0,0)


# how close you need to be to the target in order to end the trial
#TARGET_AREA            = 10 # for Stewart Bio
#conf["TARGET_AREA"]            = 28.1 # for UdeM  # 22 Feb
conf["TARGET_AREA"] = conf["TARGET_RADIUS"]


# How long the cursor needs to be in the target in order for the trial to end (in seconds)
conf["IN_TARGET_TIME"] = .10


# How long the robot will take to place the subject in the start position
# from wherever they ended in the previous trial
conf["RETURN_TIME"] = 1.5

conf["FADE_TIME"] = .5 # how much time to let the forces fade at the target position

# This is for the no-feedback mode - since it's difficult for the subject to reach the target
# without visual feedback, we simply quit the trial after a fixed duration, which is specified
# by the parameter below.
conf["MAX_TRIAL_TIME"] = 2.5 # 1.7 # the maximum trial duration (in s) after which we abort the trial (if in no-feedback mode)


# The duration (in s) to show the final position, once the trial has ended (i.e. the participant
# has reached the target circle).
conf["FINAL_POSITION_SHOW_TIME"] = 1.500

#HISTORY_LINE_WIDTH = 1 # for Stewart Bio
conf["HISTORY_LINE_WIDTH"] = 4 # for UdeM, rounded  # 22 Feb


# The amount of pixels from the center that you have to have moved for us to
# call the movement started (and the clock starts ticking).
#MOVE_START_RADIUS = 3 # for Stewart Bio
conf["MOVE_START_RADIUS"] = conf["TARGET_RADIUS"]






# Colour to show the history of cursor positions (i.e. trajectory) (at the end of the trial)
conf["CURSOR_HISTORY_COLOUR"]       = (255,255,255)
conf["CURSOR_HISTORY_RADIUS"]       = 9
conf["CURSOR_HISTORY_WRONG_COLOUR"] = (255,0,0) # the colour of the cursor history if this was outside of the arc
conf["CURSOR_HISTORY_TIMESTEP"]     = .02 # the timestep (in s) to show cursor positions when we reveal the position at the end of the movement.
conf["CURSOR_HISTORY_N_POINTS"]     = 100 # how many points there should be in the normalised resampled trajectory

# The arc defines the path to be followed and will be drawn on the screen using lines.
#ARC_RADIUS     = 50 # use in Stewart Bio
#conf["ARC_RADIUS"]     = 149.28 # use at UdeM
conf["ARC_RADIUS_1"] = .005 # the radius of the arc in robot coordinates (m)
conf["ARC_RADIUS_2"] = conf["ARC_RADIUS_1"] #.005 # the radius of the arc in robot coordinates (m)

conf["ARC_HALF_WIDTH"] = .0001 # the width of the arc (in robot coordinates, i.e. m)

## NOTE -- the arc here is defined in (X,Y) plane, so that X and Y correspond to X and Y on the screen.
## However, in the robot coordinate systems, we have three dimensions (X,Y,Z). X is kept fixed (see X_PLANE),
## and then the screen X corresponds to the robot Y and the screen Y corresponds to the robot Z.
conf["ARC_BASE_Y"] = 0 # the y position of the targets (in robot coordinates)
conf["ARC_BASE_X"] = 0 # the x position of the center of the arc

conf["ARC_SEGMENTS"] = 50 # how many segments to use for drawing the arc

#ARC_HALF_WIDTH = .5 # use in Stewart Bio
#conf["ARC_HALF_WIDTH"] = 1.405 # use at UdeM ## 22 Feb
#conf["ARC_WIDTH"]      = conf["ARC_HALF_WIDTH"]*2
conf["ARC_COLOUR"]     = (255,255,255)


# We pre-calculate this because we use it in several places (and it doesn't change during the experiment)
conf["ARC_MIN_RADIUS_1"] = conf["ARC_RADIUS_1"]-conf["ARC_HALF_WIDTH"]
conf["ARC_MAX_RADIUS_1"] = conf["ARC_RADIUS_1"]+conf["ARC_HALF_WIDTH"]
conf["ARC_MIN_RADIUS_2"] = conf["ARC_RADIUS_2"]-conf["ARC_HALF_WIDTH"]
conf["ARC_MAX_RADIUS_2"] = conf["ARC_RADIUS_2"]+conf["ARC_HALF_WIDTH"]



# Mapping the timing judgements to a number so that Matlab can eat it
timing_number = {"ok":0, "too_slow":-1, "too_fast":1, "early_start":2, "incomplete.trial":99}




# This flag controls whether we give reward and show it on the screen.
conf["GIVE_REWARD"] = False
# The coordinates of the "coins: XX" message that is shown continuously on the screen
conf["REWARD_LABEL_X"] = 600
conf["REWARD_LABEL_Y"] = 460

# How long to show feedback (too slow, too fast, reward) (in seconds)
conf["REWARD_SHOW_TIME"] = 1.500

# Reward
conf["MAX_REWARD"]           = 100 # The maximum reward you can earn (in coins)

conf["MIN_MOVEMENT_TIME"]    = 1  # the minimum allowed movement time
# PREVIOUSLY this variable encoded: the movement time that gets the maximum time reward (any faster movement also gets the maximum reward, any slower movement gets linearly less reward until MAX_MOVEMENT_TIME which gives zero reward.
conf["MAX_MOVEMENT_TIME"]    = 1.4 # the maximum allowed movement time
# PREVIOUSLY this variable encoded: # movements longer than this will get zero time reward.

conf["MAX_DISTANCE_OUTSIDE"] = 10 # how far you can be (on average) outside of the arc to still get reinforcement


conf["FLIP_TEXT"] = True # whether to X-flip the text (mirror image) so that you can read it through a mirror.






## Whether to show the past trajectory while the subject is moving
conf["SHOW_ONLINE_TRAJECTORY"] = True

## Whether to show the trajectory after the trial has ended
conf["SHOW_OFFLINE_TRAJECTORY"] = True

## Whether to colour the offline trajectory according to whether subjects
## were too early or too late on that trial
conf["COLOUR_OFFLINE_TRAJECTORY"] = True

conf["TRAJECTORY_COLOR"] = (100,100,100) # the colour of the past trajectory



## The colours used for the trials where the subject moved too fast ("early") or too slow ("late").
## These then are the colours for the target and (if so configured) of the display of the trajectory.
conf["LATE_COLOUR"] = (50,50,255)
conf["EARLY_COLOUR"] = (255,0,0)
conf["TARGET_ON_TIME_COLOUR"] = (0,255,0)



# This is one of two values:
# starting.point means rotations happen around the starting point
# center.point means rotations happen around the center point (which is the location of the fixation cross).
# Of course, that makes a big difference. We should decide what we want to do.
#conf["ROTATION_PIVOT"] = "starting.point"

# Now {LEFT,RIGHT}_ORIGIN encodes the position of the starting and end points
conf["RIGHT_ORIGIN"] = (conf["ARC_BASE_X"]+2*conf["ARC_RADIUS_2"],conf["ARC_BASE_Y"]) #(conf["CENTERX"]+conf["ARC_RADIUS"],conf["CENTERY"])
conf["LEFT_ORIGIN"]  = (conf["ARC_BASE_X"]-2*conf["ARC_RADIUS_1"],conf["ARC_BASE_Y"]) #conf(conf["CENTERX"]-conf["ARC_RADIUS"],conf["CENTERY"])
conf["CENTER_ARC_ORIGIN"]=(conf["ARC_BASE_X"],conf["ARC_BASE_Y"])

# Demonstration = The ideal trajectory
conf["DEMO_FLAG"]=True
conf["DEMO_TIME"]=2.4           #The cursors stays 1.2 s then moves 1.2 s


conf["X_PLANE"]=0  # Defines the plane within which we restrict the robot




def draw_item(screen,item, (y,z)):   ##### DEPRECATED #######n
    """ Draw a particular item (target or cursor) on the screen at the given (y,z) position. """
    if item=="target-nogo": # The cursor prior to the go signal
        colour=(255,0,0)
        radius = conf["TARGET_SIZE"]
    if item=="target-go": # The cursor prior to the go signal
        colour=(255,255,255)
        radius = conf["TARGET_SIZE"]
    if item=="target-ended": # The target when the trial has ended
        colour = conf["TARGET_ON_TIME_COLOUR"]
        radius = conf["TARGET_SIZE"]
    if item=="target-ended-late": # The target when the trial has ended and subject was too late (too slow)
        colour = conf["LATE_COLOUR"]
        radius = conf["TARGET_SIZE"]
    if item=="target-ended-early": # The target when the trial has ended and subject was too early (too fast)
        colour = conf["EARLY_COLOUR"]
        radius = conf["TARGET_SIZE"]
    if item=="cursor":
        colour = conf["CURSOR_COLOUR"]
        radius = conf["CURSOR_SIZE"]

    (y,z) = (int(y),int(z)) # make sure the positions become ints
    pygame.draw.circle(screen,colour,(y,z),radius,0)



def draw_arc(surface,col=conf["ARC_COLOUR"]): #,horizontal_flip=False):
    # Draws the arc on to the surface (which should be the screen).
    # If horizontal_flip is True, then we flip the blitted surface horizontally
    # before blitting. This ensures that the feathering around the target area
    # really ends up around the target area and not the starting point.

    global conf


    # Draw the target arc between the target balls
    angs = np.linspace(0,np.pi,conf["ARC_SEGMENTS"])
    def get_arc_radius(rad, x_center):
        """ Determine the arc positions (y,z) for a given radius """
        return [ (x_center+rad*np.cos(a),
                  conf["ARC_BASE_Y"]+rad*np.sin(a)) for a in angs ]

    innerarc = get_arc_radius(conf["ARC_RADIUS_1"]-conf["ARC_HALF_WIDTH"],conf["CENTER_ARC_ORIGIN"][0]-conf["ARC_RADIUS_1"])
    outerarc = get_arc_radius(conf["ARC_RADIUS_1"]+conf["ARC_HALF_WIDTH"],conf["CENTER_ARC_ORIGIN"][0]-conf["ARC_RADIUS_1"])

    fullarc = innerarc+outerarc[::-1]+innerarc[0:1] # reverse order of outer arc and add to list
    #print(fullarc)

    poly = [ robot_to_screen(ry,rz,conf) for (ry,rz) in fullarc ]
    pygame.draw.polygon( surface,col,poly)
    #for (x,y) in poly:
    #    pygame.draw.circle(surface,conf["ARC_COLOUR"],(x,y),5)

    innerarc = get_arc_radius(-conf["ARC_RADIUS_2"]+conf["ARC_HALF_WIDTH"],conf["CENTER_ARC_ORIGIN"][0]+conf["ARC_RADIUS_2"])
    outerarc = get_arc_radius(-conf["ARC_RADIUS_2"]-conf["ARC_HALF_WIDTH"],conf["CENTER_ARC_ORIGIN"][0]+conf["ARC_RADIUS_2"])

    fullarc = innerarc+outerarc[::-1]+innerarc[0:1] # reverse order of outer arc and add to list
    #print(fullarc)

    poly = [ robot_to_screen(ry,rz,conf) for (ry,rz) in fullarc ]
    pygame.draw.polygon( surface,col,poly)





def draw_ball(surface,pos,colour):
    """
    Draw one of the two target "balls" (at the edges of the arc)

    Arguments
    sgn is whether this is the left or right direction.
    colour is the draw colour
    """

    # Draw the two target balls
    #for sgn in [-1,1]:
    #tx,ty = conf["ARC_BASE_X"]+sgn*conf["ARC_RADIUS"],conf["ARC_BASE_Y"]
    tx,ty=pos
    x1,y1 = robot_to_screen(tx-conf["TARGET_RADIUS"],ty-conf["TARGET_RADIUS"], conf)
    x2,y2 = robot_to_screen(tx+conf["TARGET_RADIUS"],ty+conf["TARGET_RADIUS"], conf)
    minx=min(x1,x2)
    maxx=max(x1,x2)
    miny=min(y1,y2)
    maxy=max(y1,y2)
    pygame.draw.ellipse(surface,colour,#conf["ARC_COLOUR"],
                        [minx,miny,maxx-minx,maxy-miny])



def draw_cursor(surface,pos):
    """ Draw the cursor at position y,z (in robot coordinates) """
    global conf

    (tx,ty) = pos # cursor position (in robot coordinates)
    x1,y1 = robot_to_screen(tx-conf["CURSOR_RADIUS"],ty-conf["CURSOR_RADIUS"],conf)
    x2,y2 = robot_to_screen(tx+conf["CURSOR_RADIUS"],ty+conf["CURSOR_RADIUS"],conf)
    minx=min(x1,x2)
    maxx=max(x1,x2)
    miny=min(y1,y2)
    maxy=max(y1,y2)
    pygame.draw.ellipse(surface,conf["CURSOR_COLOUR"],
                        [minx,miny,maxx-minx,maxy-miny])






def draw_sneak_peek(experiment,trialdata):
    # This function draws a schematic of the subject' s trajectory
    # even when it' s not shown on the subject' s screen,
    # so that the experimenter can be voyeur to the trajectory
    # and spot any irregularities.

    surf = pygame.Surface(conf["SCREENSIZE"])
    surf.fill((0,0,0))

    draw_arc(surf,(100,100,100))
    # Draw the target ball
    draw_ball(surf,trialdata["target.position"],(255,255,255))

    # Draw the starting ball
    draw_ball(surf,trialdata["start.position"],(125,125,125))

    # Now draw the history of previous positions
    if len(trialdata["cursor.history"])>1:
        #history = [ robot_to_screen(py,pz,conf) for (py,pz,_) in trialdata["position.history"] ]
        pygame.draw.lines(surf,(0,255,0),False,trialdata["cursor.history"],20)

    pygame.image.save(surf,'sneak_peek.bmp')






def draw_positions(experiment,trialdata):
    """ Update the positions of the various markers on the screen."""

    # Start with an empty screen
    # (like in Zen drawing, the most beautiful is an empty page)
    experiment.screen.fill(conf["BGCOLOR"])


    # Draw the arc that marks the path to be followed
    draw_arc(experiment.screen) #,trialdata["direction"]=="left")


    #if trialdata["phase"] in ["active","feedback"]:

    # Draw the target point (only if this is the "thin" condition)
    if True:

        if trialdata["phase"]=="feedback":
            target_color      = conf["TARGET_ON_TIME_COLOUR"]
            #if trialdata["timing"]=="too_slow":
            #    target_color  = conf["LATE_COLOUR"]
            #if trialdata["timing"]=="too_fast":
            #    target_color  = conf["EARLY_COLOUR"]
        else:
            target_color = conf["ARC_COLOUR"] #"target-go"

        target_sign = -1 if trialdata["direction"]=="left" else 1
        start_sign  = -target_sign # the "other" direction

        # Draw the target ball
        draw_ball(experiment.screen,trialdata["target.position"],target_color)

        # Draw the starting ball
        draw_ball(experiment.screen,trialdata["start.position"],conf["ARC_COLOUR"])


    if experiment.visualfb:

        # showing the trajectory so-far
        if trialdata["phase"] in ['active','feedback']:
            col = conf["TRAJECTORY_COLOR"]
            if trialdata["phase"]=="feedback" and conf["COLOUR_OFFLINE_TRAJECTORY"]:
                #if trialdata["timing"]=="too_slow":
                    #col = conf["LATE_COLOUR"]
                #elif trialdata["timing"]=="too_fast":
                    #col = conf["EARLY_COLOUR"]
                #else:
                col = conf["TARGET_ON_TIME_COLOUR"]

            # Now draw the history of previous positions
            if len(trialdata["cursor.history"])>1:
                #history = [ robot_to_screen(py,pz,conf) for (py,pz,_) in trialdata["position.history"] ]
                pygame.draw.lines(experiment.screen,col,False,trialdata["cursor.history"],conf["HISTORY_LINE_WIDTH"])

                #(oy,oz) = history[0] #,_) = trialdata["position.history"][0]
                #for (y,z) in history[1:]: #trialdata["position.history"][1:]:
                #    pygame.draw.line(experiment.screen,col,(oy,oz),(y,z),conf["HISTORY_LINE_WIDTH"])
                #    (oy,oz) = (y,z)


        # showing the cursor
        if trialdata["phase"] in ["active","feedback"]:

            if trialdata["t.current"]>trialdata["t.go"]:
                # Draw cursor at current position
                #if trialdata["t.current"]>=trialdata["t.start"]+trialdata["t.go"]:
                #draw_item(experiment.screen,"cursor",trialdata["cursor.position"])
                draw_cursor(experiment.screen,trialdata["cursor.position"])

    else: # i.e. if not experiment.visualfb

        if trialdata["phase"]=="active" and trialdata["in.start"]: #trialdata["t.movestart"]==None: # the go signal is there but the subject has not started moving
            draw_cursor(experiment.screen,trialdata["cursor.position"])



        if trialdata["phase"]=='feedback':
            # Draw cursor at current position
            #if trialdata["t.current"]>=trialdata["t.start"]+trialdata["t.go"]:
            #draw_item(experiment.screen,"cursor",trialdata["cursor.position"])
            #draw_cursor(experiment.screen,trialdata["cursor.position"])
            pass




def update_position(trialdata,(ry,rz),conf):
    # We have an (x,y) position from the robot. Here we need to
    # map that to screen coordinates.
    # This is also the place where we could insert all sorts of nasty
    # rotations or disturbances.

    trialdata["cursor.position"]=(ry,rz)

    if not np.isnan(ry) and not np.isnan(rz):
        if trialdata["phase"]=="active":

            sy,sz = robot_to_screen(ry,rz,conf)

            # First check whether we actually want to add anything - because we may
            # already have this position, no need to duplicate stuff.
            addthis = True
            if len(trialdata["cursor.history"])>0:
                oy,oz = trialdata["cursor.history"][-1]
                if oy==sy and oz==sz: addthis=False

            if addthis:
                trialdata["cursor.history"].append((sy,sz))

    return





def reset_trial(experiment,trialdata,t):
    """
    Sets everything up for the current trial to start ... now!
    """
    (ry,rz) = trialdata["start.position"]
    print("Initiate move to %f,%f"%(ry,rz))
    robot.move_to(conf["X_PLANE"],ry,rz,conf["RETURN_TIME"])

    trialdata['n.reset']        += 1
    trialdata["t.stay"]         = t+conf["RETURN_TIME"] # when to start staying and fading
    trialdata["t.go"]           = t+conf["RETURN_TIME"]+conf["FADE_TIME"] # when to give the go signal
    trialdata["t.trial.finish"] = np.inf #t+10*conf["MAX_TRIAL_TIME"] # when the trial will end

    trialdata["t.start"]        = t
    trialdata["t.current"]      = t
    trialdata["t.movestart"]    = None
    trialdata["t.target.enter"] = None
    trialdata["timing"]         = "ok"


    # The history of positions (x,y,t) in this trial, used to show the whole trajectory at the end of the trial
    trialdata["cursor.history"] = []

    trialdata["next.trial.t"]=np.nan








def finish_trial(experiment,trialdata):
    # Do all the things that we should do at the end of a trial,
    # such as logging the trial-level data.
    print("Trial ended. Timing %s"%trialdata["timing"])
    print("")
    traj = robot.stop_capture(True)

    experiment.captured.append({"trial"       :trialdata["trial.number"],
                                "trajectory"  :traj,
                                'n.reset'     :trialdata["n.reset"],
                                "capture.t"   :time.time()})

    comedi_unload()
    
    # Dump everything we have captured so far to a pickle
    #pickle.dump(traj,open('data/_tmp_trial%i.pickle'%trialdata["trial.number"],'wb'))
    #pickle.dump(experiment.captured,open(experiment.capturelog,'wb'))

    triallog(experiment,trialdata)






def start_new_trial(experiment,trialdata,dont_swap=False):
    # This function starts a new trial. It initiates a random target location and sets
    # the cursor to the starting position again.
    # If dont_swap is true, then we don't swap the target position (usually we do swap the target
    # position because we want to alternate left-to-right movement and right-to-left movement).
    # However, if the previous trial was early start, then we just want to revert to the previous target
    # position.


    if experiment.current_schedule>=len(experiment.schedule):
        # This should not happen in the current set up because when no next trial
        # is available we don't push it on to the schedule and therefore we don't
        # reach to this function.
        #experiment.keep_going = False
        #message_screen(experiment,"This part of the experiment is complete.")
        print("Finished schedule.") # Press ESC to close the interface.")
        trialdata["phase"]="null"

        robot.stay() # just fix the handle wherever it is
        print("Experiment schedule completed.")

        # Put all the data in the picle file
        pickle.dump(experiment.captured,open(experiment.capturelog,'wb'))

        close_logs() # at the end of a run

        # Now ask the experimenter for observations
        obsv = tkSimpleDialog.askstring('Please record any observations', 'Experimenter, please write down any observations.\nAny irregularities?\nDid the subject seem concentrated or not?\nWere things unclear or clear?\nAnything else that is worth noting?')
        with open(experiment.obsvlog,'w') as f:
            f.write(obsv)

        tkMessageBox.showinfo("Robot", "Block completed! Yay!")
        trialdata["t.trial.finish"]=np.inf # that's the end folks!
        experiment.keep_going = False
        gui["running"]=False
        update_ui()


    else:


        #conf["ARC_RADIUS_1"] = random.randint(1,2000)/100000
        #conf["ARC_RADIUS_2"] = random.randint(1,2000)/100000
        #conf["RIGHT_ORIGIN"] = (conf["ARC_BASE_X"]+2*conf["ARC_RADIUS_2"],conf["ARC_BASE_Y"])
        #conf["LEFT_ORIGIN"]  = (conf["ARC_BASE_X"]-2*conf["ARC_RADIUS_1"],conf["ARC_BASE_Y"])


        tcurrent  = trialdata["t.current"]
        tabs      = trialdata["t.absolute"]
        trialn    = experiment.schedule[ experiment.current_schedule ]["trial"]
        forcechan = experiment.schedule[ experiment.current_schedule ]["force.channel"] # whether this is a channel trial
        direction = trialdata["direction"]
        oldpos    = trialdata["cursor.position"]

        trialdata = {"trial.number"    :trialn,
                     "force.channel"   :forcechan,
                     "experiment"      :EXPERIMENT,
                     "phase"           :"return",
                     "t.current"       :tcurrent,
                     "t.absolute"      :tabs,
                     "cursor.position" :oldpos,
                     'saved'           :False,
                     "aborted"         :False,
                     "finished"        :False,
                     "in.start"        :True,
                     'n.reset'         :0 # keeps track of how often we had to reset the trial
                     }

        print ("Starting trial #%i"%trialdata["trial.number"])
        #robot.wshm('fvv_trial_no',trialn)

        init_comedi()

        # Decide the movement direction for this trial (should be opposite of that of the previous trial)
        if dont_swap:
            trialdata['direction']=direction
        else:
            if direction=="left":
                trialdata["direction"]="right"
            else:
                trialdata["direction"]="left"


        if trialdata["direction"]=="right":
            target_position,startpos = conf["RIGHT_ORIGIN"],conf["LEFT_ORIGIN"]
        else: # if direction==left
            target_position,startpos = conf["LEFT_ORIGIN"],conf["RIGHT_ORIGIN"]


        (ry,rz) = startpos
        print("Initiate move to %f,%f"%(ry,rz))
        robot.move_to(0,ry,rz,conf["RETURN_TIME"])


        trialdata["target.position"]=target_position


        # The starting position
        trialdata["start.position"]  =startpos

        # Apply the proper rotation
        #rotate_cursor(trialdata,experiment)
        trialdata["cursor.start"]    =startpos #trialdata["cursor.position"]

        # Decide after how long the GO signal is to be given (in seconds)
        #go_time = random.uniform(TRIAL_START_MIN,TRIAL_START_MAX)

        t = tcurrent
        reset_trial(experiment,trialdata,tcurrent)


    return trialdata









#
#
#  Stuff relating to logging
#
#



def init_logs(experiment,conf):
    #logfile = open("data/exampledata.txt",'w')
    timestamp = datetime.datetime.now().strftime("%d_%m.%Hh%Mm%S")
    basename = './data/%s_%s_%s_'%(experiment.participant,EXPERIMENT,timestamp)
    conf["basename"]=basename
    trajlog = '%strajectory.bin'%basename
    # robot.start_log(trajlog,N_ROBOT_LOG)      # This lign is only for the inmotion robot

    experiment.captured = []
    capturelog = '%scaptured.pickle27'%basename
    ##trajlog.write('participant experiment trial x y dx dy t t.absolute\n')

    obsvlog    = '%sobservations.txt'%basename

    triallog = open('%strials.txt'%basename,'w')
    triallog.write('participant experiment trial force.channel target.x target.y t.go t.movestart t.target.enter t.trial.end timing timing.numeric duration\n')


    conflog = open('%sparameters.json'%basename,'w')
    #conflog.write('parameter;value\n')
    params = {}
    for key,value in [ ('participant',  experiment.participant),
                       ('visualfb',     experiment.visualfb),
                       ('schedulefile', experiment.schedulefile),
                       ('calib',        experiment.calib),
                       #('trduration',   experiment.trduration),
                       ('fullscreen',   experiment.fullscreen),
                       #('ribbonwidth',  experiment.ribbonwidth),
                       ('rotation',     experiment.rotation),
                       #('input_device', input_device)
    ]:
        params[key]=value #.append((str(key),str(value)))
    for key in sorted(conf):
        if not key.startswith('comedi'):
            params[key]=conf[key]
            #nparams.append((str(key),str(conf[key])))
    #print(conf["calib"])
    #for (key,value) in params:
    #    conflog.write('%s;%s\n'%(key,value))
    json.dump(params,conflog)
    conflog.close()



    experiment.trajlog   = trajlog
    experiment.triallog  = triallog
    experiment.capturelog= capturelog
    experiment.obsvlog   = obsvlog
    #experiment.normlog  = normlog
    #experiment.trlog    = trlog










def triallog(experiment,trialdata):
    (tx,ty) = trialdata["target.position"]

    experiment.triallog.write( writeln( [
        (experiment.participant,            's'),
        (trialdata["experiment"],           's'),
        #(conf["ARC_WIDTH"],                 'f'),
        (trialdata["trial.number"],         'i'),
        #(experiment.rotation,               'f'),
        (trialdata["force.channel"],        'i'),
        (tx,'f'),(ty,'f'),
        (trialdata["t.go"],                 'f'),
        (trialdata["t.movestart"],          'f'),
        (trialdata["t.target.enter"],       'f'),
        (trialdata["t.trial.end"],          'f'),
        (trialdata["timing"],               's'),
        (timing_number[trialdata["timing"]],'i'),
        (trialdata["duration"],             'f'),
        ]))

    experiment.triallog.flush()


def close_logs():
    experiment.triallog.close()
    #robot.stop_log()




def read_schedule_file(experiment):
    ## Read a schedule file that tells us when to start which trial.
    ## Each time is divided by the TR duration to find the number of the TR.

    print("Reading trial schedule file %s"%experiment.schedulefile)
    f = open(experiment.schedulefile,'r')
    lns = f.readlines()
    f.close()

    # Extract the header
    header = [ h.strip() for h in lns[0].split() ]

    trials = []
    for l in lns[1:]:
        fields = [ f.strip() for f in l.split() ]
        dat = dict( zip(header,fields) )
        trials.append({"trial"      :int(dat["trial"]),
                       "force.channel":int(dat["force.channel"]),
                       #"TR"         :int(dat["TR"]),
                       #"t.offset"   :float(dat["t.offset"])
        })



    experiment.schedule = trials
    #experiment.ntrials = len([ tr for tr in trials if tr["type"]=="arctrace" ])
    print("Finished reading %i trials"%len(experiment.schedule))





def update_ui():
    global gui
    gui["runb"].configure(state=DISABLED)

    if gui["loaded"]:
        gui["loadb"].configure(state=DISABLED)

        if not gui["running"]:
            gui["runb"].configure(state=NORMAL)

    else: # not loaded
        gui["loadb"].configure(state=NORMAL)
        gui["runb"].configure(state=DISABLED)





def load_robot():
    """ Launches the robot process. """
    global gui
    tkMessageBox.showinfo("Robot", "We will now load the robot.\n\nLook at the terminal because you may have to enter your sudo password there.")

    robot.launch() # launches the robot C++ script
    robot.init()   # initialises shared memory
    # Then do zero FT
    #tkMessageBox.showinfo("Robot", "Now we will zero the force transducers.\nAsk the subject to let go of the handle." )
    #robot.release()
    gui["loaded"]=True
    update_ui()
    tkMessageBox.showinfo("Robot", "Experimenter, enable the forces on the robot!\n\nNow tell the subject the robot will move to the initial position\n\n Take your time to find the best position for your arm.")
    robot.move_to(conf["X_PLANE"],conf["ARC_BASE_X"],conf["ARC_BASE_Y"],conf["RETURN_TIME"])
    while (robot.move_is_done() == False):
        pass  #wait until the end of the movement
    robot.three_d_to_two_d(conf["X_PLANE"])

    #init_comedi()



def init_comedi():
    """ Initialises the COMEDI interface, which is for reading the force transducer."""
    
    ##
    ## Okay, now also load the sensor reading
    ##
    #open a comedi device
    conf["comedi.dev"] =comedi.comedi_open('/dev/comedi0')
    if not conf["comedi.dev"]: raise Exception("Error opening Comedi device") # This can happen if you do not have sufficient read privileges to /dev/comedi0, try running with sudo
    
    #get a file-descriptor for use later
    conf["comedi.fd"]    = comedi.comedi_fileno(conf["comedi.dev"])
    if conf["comedi.fd"]<=0: raise Exception("Error obtaining Comedi device file descriptor")

    print("\nCOMEDI loaded (for reading force sensors)")
    
    conf["comedi.freq"]=1000   # sampling frequency (Hz I hope)
    subdevice=0 # no idea
    conf["comedi.subdev"]=subdevice

    NCHANNEL=16  # how many channels we are reading (TODO)

    #three lists containing the chans, gains and referencing
    #the lists must all have the same length
    chans  = range(NCHANNEL);#[0,1,2,3]
    gains  = [0]*NCHANNEL #[0,0,0,0]
    aref   = [comedi.AREF_GROUND]*NCHANNEL #, c.AREF_GROUND, c.AREF_GROUND, c.AREF_GROUND]
    nchans = NCHANNEL #len(chans) #number of channels

    #wrappers include a "chanlist" object (just an Unsigned Int array) for holding the chanlist information
    mylist = comedi.chanlist(nchans) #create a chanlist of length nchans

    #now pack the channel, gain and reference information into the chanlist object
    #N.B. the CR_PACK and other comedi macros are now python functions
    for index in range(nchans):
        mylist[index]=comedi.cr_pack(chans[index], gains[index], aref[index])

    conf["comedi.size"] = comedi.comedi_get_buffer_size(conf["comedi.dev"], subdevice)
    print("Comedi buffer size is %d"% conf["comedi.size"])
    conf["comedi.map"]  = mmap.mmap(conf["comedi.fd"], conf["comedi.size"], mmap.MAP_SHARED, mmap.PROT_READ)
    ##print("map = ", map)

    cmd = comedi.comedi_cmd_struct()
    cmd.chanlist = mylist # adjust for our particular context
    cmd.chanlist_len = nchans
    cmd.scan_end_arg = nchans
    prepare_cmd(conf["comedi.dev"],subdevice,cmd,conf["comedi.freq"],nchans,mylist)
    conf["comedi.cmd"]=cmd




def dump_cmd(cmd):
    print("---------------------------")
    print("command structure contains:")
    print("cmd.subdev : ", cmd.subdev)
    print("cmd.flags : ", cmd.flags)
    print("cmd.start :\t", cmd.start_src, "\t", cmd.start_arg)
    print("cmd.scan_beg :\t", cmd.scan_begin_src, "\t", cmd.scan_begin_arg)
    print("cmd.convert :\t", cmd.convert_src, "\t", cmd.convert_arg)
    print("cmd.scan_end :\t", cmd.scan_end_src, "\t", cmd.scan_end_arg)
    print("cmd.stop :\t", cmd.stop_src, "\t", cmd.stop_arg)
    print("cmd.chanlist : ", cmd.chanlist)
    print("cmd.chanlist_len : ", cmd.chanlist_len)
    print("cmd.data : ", cmd.data)
    print("cmd.data_len : ", cmd.data_len)
    print("---------------------------")

    

def prepare_cmd(dev, subdev, C, freq, nchans, mylist):
    #global cmd

    C.subdev = subdev
    C.flags = 0
    C.start_src = comedi.TRIG_NOW
    C.start_arg = 0
    C.scan_begin_src = comedi.TRIG_TIMER
    C.scan_begin_arg = int(1e9/freq)
    C.convert_src = comedi.TRIG_TIMER
    C.convert_arg = 1
    C.scan_end_src = comedi.TRIG_COUNT
    C.scan_end_arg = nchans
    C.stop_src = comedi.TRIG_NONE
    #C.stop_src = c.TRIG_COUNT
    C.stop_arg = 0
    #C.stop_arg = 1000
    C.chanlist = mylist
    C.chanlist_len = nchans




def comedi_start_record():
    """ Start recording from the COMEDI device."""
    #test our comedi command a few times. 
    ret = comedi.comedi_command_test(conf["comedi.dev"],conf["comedi.cmd"])
    print("first cmd test returns ", ret)#, cmdtest_messages[ret]
    if ret<0:
        raise Exception("comedi_command_test failed")
    #dump_cmd(cmd)

    ret = comedi.comedi_command_test(conf["comedi.dev"],conf["comedi.cmd"])
    print("second test returns ", ret)#, cmdtest_messages[ret])
    if ret<0:
        raise Exception("comedi_command_test failed")
    if ret !=0:
        #dump_cmd(cmd)
        raise Exception("ERROR preparing command")
    #dump_cmd(cmd)
    
    ret = comedi.comedi_command(conf["comedi.dev"],conf["comedi.cmd"])
    if ret<0:
        print("### Error executing comedi_command!!! Not reading sensor")
        print(ret)

        
        
def comedi_stop_record(fname):
    """ Stop recording from the COMEDI device and save the output to the given filename fname. """
    back  = 0  # always read from the beginning of the buffer (assumes that we clear the buffer at the beginning of each trial)
    front = comedi.comedi_get_buffer_contents(conf["comedi.dev"],conf["comedi.subdev"])

    DATA = array.array("H") # reset array to empty    

    conf["comedi.map"].seek(back%conf["comedi.size"])
    for i in range(back,front,2):
	DATA.fromstring(conf["comedi.map"].read(2))
	if conf["comedi.map"].tell() == conf["comedi.size"]:
	    conf["comedi.map"].seek(0)
            
    of = open(fname,"wb")
    DATA.tofile(of) # append data to log file
    of.flush()
    of.close()

    ## 	time.sleep(.01)
    ret = comedi.comedi_mark_buffer_read(conf["comedi.dev"],conf["comedi.subdev"],front-back)
    print("Read %d bytes from COMEDI"%(front-back))
    return []


    


def comedi_unload():
    ret = comedi.comedi_close(conf["comedi.dev"])
    if ret<0:
	print("### ERROR executing comedi_close ### (but hey, we're done already anyway)")
    
    


def end_program():

    experiment.keep_going = False
    if gui['loaded']:
        robot.unload()
        comedi_unload()
        gui["loaded"]=False

    ending()
    sys.exit(0)





def decide_timing(trialdata):
    # Set the current time to be the end time of the trial.
    # Then determine whether the timing of the current trial was correct.
    trialdata["t.trial.end"]=trialdata["t.current"]

    ## Decide how long this trial's movement was.
    dtime = trialdata["t.trial.end"]-trialdata["t.movestart"]
    trialdata["duration"]=dtime
    #if dtime<conf["MIN_MOVEMENT_TIME"]:
    #    trialdata["timing"]="too_fast"
    if dtime>conf["MAX_MOVEMENT_TIME"]:
        trialdata["timing"]="too_slow"
    else:
        trialdata["timing"]="ok"




def channel_trial(x_plane,arc_base_x,arc_base_y,arc_radius):
    """ Play the movement which is in trajectory in the shared memory """
    robot.wshm('arc_x_plane', x_plane)
    robot.wshm('arc_base_y',  arc_base_x)
    robot.wshm('arc_base_z',  arc_base_y)
    robot.wshm('arc_radius',  arc_radius)
    robot.wshm('controller',  7)
    robot.wait_for_the_new_instruction(11) # update the variable group corresponding to this controller


        

def run():
    """ Do one run of the experiment. """
    if gui["running"]:
        return

    global conf

    participant=gui["subject.id"].get().strip()
    if participant=="":
        tkMessageBox.showinfo("Error", "You need to enter a participant ID.")
        return
    experiment.participant = participant


    schedulefile=gui["schedulef"].get().strip()
    if schedulefile=="":
        tkMessageBox.showinfo("Error", "You need to enter a schedule file name.")
        return
    if not os.path.exists(schedulefile):
        tkMessageBox.showinfo("Error", "The schedule file name you entered does not exist.")
        return
    experiment.schedulefile = schedulefile

    experiment.visualfb     = gui["visualfb"].get()!=0

    experiment.calib = None #conf["calib"]

    read_schedule_file(experiment)



    init_logs(experiment,conf)
    experiment.inputs = robot


    print("Running the experiment.")
    gui["running"]=True
    update_ui()


    trialdata = {"trial.number"    :0,
                 "experiment"      :EXPERIMENT,
                 "direction"       :"left",
                 "phase"           :"null",
                 "cursor.position" :(np.nan,np.nan),
                 "t.start"         :np.nan,
                 "t.trial.finish"  :-np.inf,
                 "finished"        :True,
                 "aborted"         :False,
                 "target.position" :conf["RIGHT_ORIGIN"],
                 "start.position"  :conf["LEFT_ORIGIN"]}


    ##center_trackball(experiment)
    # This means we are still waiting for the first trigger
    experiment.current_tr       = 1 # the number of the TR we have received. 0 means nothing yet received. 1 is the first TR.
    experiment.current_schedule = 0 # this is a pointer to where we are in the schedule. if zero, it means we haven't yet received the first trigger
    experiment.trigger_times    = [time.time()] # pretend that we receive a trigger

    draw_positions(experiment,trialdata)
    pygame.display.flip()
    #redraw_demo = False

    experiment.keep_going = True
    while experiment.keep_going:

        # Keep track of whether we will want to redraw
        redraw = False

        trialdata["t.absolute"] = time.time()

        # Get the current time (coded in seconds but with floating point precision)
        trialdata["t.current"] = trialdata["t.absolute"]-experiment.first_trigger_t()

        # Get current position from the robot
        pos = robot.rshm('y'),robot.rshm('z')
        update_position(trialdata,pos,conf)

        # Decide how far the subject has moved from the the starting position
        dfromstart = np.sqrt(sum((np.array(trialdata["cursor.position"])-np.array(trialdata["start.position"]))**2))
        # Determine whether we are still within the starting zone
        in_start  = dfromstart < conf["MOVE_START_RADIUS"] if not np.isnan(dfromstart) else True
        trialdata["in.start"]=in_start

        if trialdata["t.current"]>=trialdata["t.trial.finish"]: # this is where the trial should end --- NOTE that this is now deprecated, we no longer abort trials based on time

            # First of all, are we currently in the middle of a trial? Then we have to abort it.
            if trialdata["phase"]=="active":

                print("Forcing trial end.")
                print("Aborting trial because this takes too long.")
                redraw                            = True
                trialdata["aborted"]              = True

                trialdata["t.target.enter"]       = np.nan
                trialdata["t.trial.end"]          = trialdata["t.current"]
                trialdata["duration"]             = np.nan
                trialdata["timing"]               = "incomplete.trial"

                trialdata["phase"]="feedback"
                trialdata["show.feedback.until.t"] = trialdata["t.current"]+conf["FINAL_POSITION_SHOW_TIME"]

                robot.stay() # stop the subject in their tracks
                finish_trial(experiment,trialdata)


        # Is it time to start holding the robot at the center?
        if trialdata['phase']=='return' and trialdata['t.current']>trialdata['t.stay']:
            trialdata['phase']='stay'
            robot.start_capture()
            if trialdata["force.channel"]:
                # TODO: possibly we could "fade out" forces here gradually?
                print("--- Channel trial.")
                channel_trial(conf["X_PLANE"],
                              conf["ARC_BASE_X"],
                              conf["ARC_BASE_Y"],
                              conf["ARC_RADIUS_1"]) #arcradius)
            else:
                robot.active_to_null()
            comedi_start_record() # start recording, clear buffer


        if trialdata['phase']=='stay':

            #if trialdata['t.current']>trialdata['t.max.move.start']:
            #    print("Waiting too long in the starting location -- aborting trial.")
            #    trialdata["aborted"]=True

            if trialdata['t.current']>trialdata['t.go']:

                if not in_start: # if the subject is no longer in the start zone, it doesn't make sense to start the trial!
                    print("Resetting the trial - subject did not stay in the starting zone until the cursor appeared.")
                    print(dfromstart)
                    #print(trialdata['cursor.position'])
                    reset_trial(experiment,trialdata,trialdata["t.current"])
                else:
                    print("Go!")
                    trialdata['phase']='active' # go! start showing the cursor and let's move
                    if not trialdata["force.channel"]:
                        robot.three_d_to_two_d(conf["X_PLANE"])
                    redraw = True # because if no visual fb, we should at least show the cursor


        # If the feedback show time has completed...
        if trialdata["phase"]=="feedback" and trialdata["t.current"]>=trialdata.get("show.feedback.until.t",np.inf):
            # We have completed showing feedback
            print("Completed feedback show time.")
            trialdata["phase"]="null"
            trialdata["finished"]=True
            redraw = True



        if trialdata['phase']=='null' and trialdata["finished"]: # trialdata["t.current"]>=trialdata["t.trial.finish"]:

            # Start a new trial (or stop if there is nothing more to be done)
            trialdata = start_new_trial(experiment,trialdata)

            # Advance the schedule
            experiment.current_schedule += 1

            redraw = False # True







        if trialdata["phase"]=="active":
            redraw = True

            dtotarget = np.sqrt(sum((np.array(trialdata["cursor.position"])-np.array(trialdata["target.position"]))**2))
            # Determine whether we are inside the target area.
            # In the following, we set the in_target variable to TRUE if we are in the target area for this particular
            # experiment.
            in_target = dtotarget  < conf["TARGET_AREA"]

            if in_start:
                trialdata["t.movestart"]    = None; # This way, if you go back to the starting point, you effectively reset the trial
                trialdata["t.trial.finish"] = np.inf;

            else: # if not in start

                # If we haven't already left the starting position, mark the current time as
                # the first time that we did leave the starting position.
                if trialdata["t.movestart"]==None:

                    # Movement has started
                    trialdata["t.movestart"]    = trialdata["t.current"]
                    if experiment.visualfb:
                        trialdata["t.trial.finish"] = np.inf;
                    else:
                        trialdata["t.trial.finish"] = trialdata["t.movestart"] + conf["MAX_TRIAL_TIME"]
                    redraw = True



                if experiment.visualfb:
                    if in_target:
                        if trialdata["t.target.enter"]==None: # if we have already previously entered the target
                            trialdata["t.target.enter"]=trialdata["t.current"]
                        else: # if we were not in the target already
                            if trialdata["t.current"]-trialdata["t.target.enter"] > conf["IN_TARGET_TIME"]:

                                ##### Trial ending
                                decide_timing(trialdata)

                                trialdata["phase"]="feedback"
                                trialdata["show.feedback.until.t"] = trialdata["t.current"]+conf["FINAL_POSITION_SHOW_TIME"]

                                redraw = True

                                ### Wrap up the rest of the trial
                                robot.stay()

                                #if trialdata["force.channel"]:
                                comedi_stop_record("%s_force_trial%d.bin"%(conf["basename"],trialdata["trial.number"]))
                                
                                finish_trial(experiment,trialdata)


                    else:
                        # If we are outside of the target, we need to reset the target enter time
                        trialdata["t.target.enter"]=None



        """ # We're not actually aborting anything anymore
        if trialdata["aborted"]:
            experiment.screen.fill(conf["EARLY_COLOUR"])
            draw_arc(experiment.screen)
            draw_ball(experiment.screen,trialdata["target.position"],conf["ARC_COLOUR"])
            draw_ball(experiment.screen,trialdata["start.position"],conf["ARC_COLOUR"])
            calibri_font = pygame.font.SysFont("Calibri",100)
            text_surface = calibri_font.render("ABORTED",True,(255,255,255))
            experiment.screen.blit(text_surface, (900,200))
            pygame.display.flip()
        """


        if redraw:
            # Update the positions on the screen
            draw_positions(experiment,trialdata)

            if trialdata["timing"]=="too_slow":
                #experiment.screen.fill(conf["EARLY_COLOUR"])
                #draw_arc(experiment.screen)
                #draw_ball(experiment.screen,trialdata["target.position"],conf["ARC_COLOUR"])
                #draw_ball(experiment.screen,trialdata["start.position"],conf["ARC_COLOUR"])
                calibri_font = pygame.font.SysFont("Calibri",100)
                text_surface = calibri_font.render("TOO SLOW",True,(255,50,50))
                experiment.screen.blit(text_surface, (900,200))
                #pygame.display.flip()


            pygame.display.flip()

            if trialdata["timing"]=="too_slow":
                # play a sound to indicate that this is too slow
                #conf["honk"].play()
                cmd =["aplay","materials/carhorn.wav"]
                subprocess.call(cmd)

            if trialdata['phase']=='feedback' and not trialdata['saved']:

                # Save to a file too
                if True:
                    print("Saving to file")
                    draw_sneak_peek(experiment,trialdata)
                    #pygame.image.save(experiment.screen,'screenshot.bmp')
                    subprocess.call(['convert','sneak_peek.bmp',
                                     #'-crop','800x500+600+200',
                                     #'-flip',
                                     '-resize','400x250','screenshot.gif'])
                    trialdata['saved']=True

                    gui["photo"]=PhotoImage(file='screenshot.gif')
                    gui["photolabel"].configure(image=gui["photo"])


        #if redraw_demo :
        #    draw_demonstration(experiment, trialdata)


        master.update_idletasks()
        master.update()

    gui["running"]=False






def init_tk():
    global gui
    gui = {}

    master = Tk()
    master.geometry('%dx%d+%d+%d' % (CONTROL_WIDTH, CONTROL_HEIGHT, CONTROL_X, CONTROL_Y))
    master.configure(background='black')

    f = Frame(master,background='black')
    loadb   = Button(f, text="Load robot",                background="green",foreground="black", command=load_robot)
    runb    = Button(f, text="Run",                       background="blue", foreground="white", command=run)
    quitb   = Button(f, text="Quit",                      background="red",foreground="black", command=end_program)

    gui["subject.id"] = StringVar()
    l      = Label(f, text="subject ID",             fg="white", bg="black")
    subjid = Entry(f, textvariable=gui["subject.id"],fg="white", bg="black",insertbackground='yellow')

    row  = 0
    f.grid         (row=row,padx=10,pady=10)
    row += 1
    loadb.grid     (row=row,column=0,sticky=W,padx=10,pady=10)

    row += 1
    l.grid         (row=row,column=0,sticky=W,pady=10)
    subjid.grid    (row=row,column=1,sticky=W,padx=10)

    row +=1
    gui["schedulef"]  = StringVar()
    gui["schedulef"].set("simpleschedule.txt")
    l      = Label(f, text="schedule file",          fg="white", bg="black")
    e      = Entry(f, textvariable=gui["schedulef"], fg="white", bg="black",insertbackground='yellow')
    l.grid(row=row,column=0,sticky=W,pady=10)
    e.grid(row=row,column=1,sticky=W,padx=10)


    row +=1
    gui["visualfb"] = IntVar()
    gui["visualfb"].set(1)
    c = Checkbutton(f,text="Show visual feedback",variable=gui["visualfb"],fg='red',bg='black') #,fg='white',bg='black')
    c.grid(row=row,column=0,sticky=W,padx=10,pady=10)
    #c.configure(state=NORMAL)

    row += 1
    runb.grid      (row=row,column=0,sticky=W,padx=10)

    row += 1
    quitb.grid     (row=row,sticky=W,padx=10,pady=10)


    row += 1
    gui["photo"]=PhotoImage(file='screenshot_base.gif')
    l = Label(f,image=gui['photo'])
    l.grid(row=row,column=0,columnspan=5,sticky=W)
    gui["photolabel"]=l


    # Make some elements available for the future
    gui["loadb"]   =loadb
    gui["runb"]    =runb
    gui["quitb"]   =quitb
    gui["keep_going"]=False
    gui["loaded"]    =False
    gui["running"] =False

    master.bind()

    return master





## Make a new experiment object that allows us to conveniently access all data
experiment = Experiment(EXPERIMENT)

experiment.fullscreen=False





rotation = 0
experiment.rotation = 0 #2*np.pi*(float(rotation)/360) # convert rotation to radians





## Initialise everything
experiment.screen,experiment.mainfont = init_interface(experiment,conf)
#draw_arc(experiment.screen) # so that something is there for the subject to gaze at
experiment.visualfb = False
draw_positions(experiment,{"phase":"null","direction":"left","target.position":conf["RIGHT_ORIGIN"],"start.position":conf["LEFT_ORIGIN"]})
pygame.display.flip()

# This prepares the surface that contains the arc, so that we don't have to
# fully redraw it every time we refresh the screen. So they are prepared once and
# then cached.
#global arcsurf_toright
#global arcsurf_toleft
#arcsurf,arclocation = prepare_arc(experiment)
#arcsurf_toleft  = pygame.transform.flip(arcsurf_toright,True,False)



master = init_tk()
update_ui()
master.mainloop()
