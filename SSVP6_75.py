#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy2 Experiment Builder (v1.80.06), June 27, 2014, at 21:05
If you publish work using this script please cite the relevant PsychoPy publications
  Peirce, JW (2007) PsychoPy - Psychophysics software in Python. Journal of Neuroscience Methods, 162(1-2), 8-13.
  Peirce, JW (2009) Generating stimuli for neuroscience using PsychoPy. Frontiers in Neuroinformatics, 2:10. doi: 10.3389/neuro.11.010.2008
"""

from __future__ import division  # so that 1/3=0.333 instead of 1/3=0
from psychopy import visual, core, data, event, logging, sound, gui
from psychopy.constants import *  # things like STARTED, FINISHED
import numpy as np  # whole numpy lib is available, prepend 'np.'
from numpy import sin, cos, tan, log, log10, pi, average, sqrt, std, deg2rad, rad2deg, linspace, asarray
from numpy.random import random, randint, normal, shuffle
import os  # handy system and path functions

# Store info about the experiment session
expName = 'SSVP5_75'  # from the Builder filename that created this script
expInfo = {u'session': u'001', u'participant': u'001'}
dlg = gui.DlgFromDict(dictionary=expInfo, title=expName)
if dlg.OK == False: core.quit()  # user pressed cancel
expInfo['date'] = data.getDateStr()  # add a simple timestamp
expInfo['expName'] = expName

# Setup filename for saving
filename = 'data/%s_%s_%s' %(expInfo['participant'], expName, expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath=None,
    savePickle=True, saveWideText=True,
    dataFileName=filename)
#save a log file for detail verbose info
logFile = logging.LogFile(filename+'.log', level=logging.EXP)
logging.console.setLevel(logging.WARNING)  # this outputs to the screen, not a file

endExpNow = False  # flag for 'escape' or other condition => quit the exp

# Start Code - component code to be run before the window creation

# Setup the Window
win = visual.Window(size=(1280, 800), fullscr=True, screen=0, allowGUI=False, allowStencil=False,
    monitor='Desktop external monitor', color=[0,0,0], colorSpace='rgb',
    blendMode='avg', useFBO=True,
    units='cm')
# store frame rate of monitor if we can measure it successfully
expInfo['frameRate']=win.getActualFrameRate()
if expInfo['frameRate']!=None:
    frameDur = 1.0/round(expInfo['frameRate'])
else:
    frameDur = 1.0/60.0 # couldn't get a reliable measure so guess

# Initialize components for Routine "Instructions"
InstructionsClock = core.Clock()
instructions = visual.TextStim(win=win, ori=0, name='instructions',
    text='Start to save file\n\nPress space bar to begin',    font='Arial',
    pos=[0, 0], height=1., wrapWidth=None,
    color='white', colorSpace='rgb', opacity=1,
    depth=0.0)

# Initialize components for Routine "Blank"
BlankClock = core.Clock()
text = visual.TextStim(win=win, ori=0, name='text',
    text=None,    font='Arial',
    pos=[0, 0], height=0.1, wrapWidth=None,
    color='white', colorSpace='rgb', opacity=1,
    depth=0.0)
fixation = visual.GratingStim(win=win, name='fixation',
    tex='gauss', mask=None,
    ori=0, pos=[0, 0], size=[0.05, 0.05], sf=None, phase=0.0,
    color=[1,1,1], colorSpace='rgb', opacity=1,
    texRes=128, interpolate=True, depth=-1.0)

# Initialize components for Routine "InterTrial"
InterTrialClock = core.Clock()
Fixation = visual.GratingStim(win=win, name='Fixation',
    tex='gauss', mask=None,
    ori=0, pos=[0, 0], size=[0.05, 0.05], sf=None, phase=0.0,
    color=[1,1,1], colorSpace='rgb', opacity=1,
    texRes=128, interpolate=True, depth=0.0)
import serial #BP Serial functions
#-------BP serial port-------
ser = serial.Serial('/dev/cu.usbserial-FTGQVV1C', baudrate=57600, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=1) #xonxoff was 1

import time #BP to use time.time

# Initialize components for Routine "trial"
trialClock = core.Clock()

pattern1 = visual.GratingStim(win=win, name='pattern1',units='cm', 
    tex=None, mask=None,
    ori=0, pos=[0, 0], size=3, sf=1, phase=0.0,
    color=[1,1,1], colorSpace='rgb', opacity=1,
    texRes=256, interpolate=True, depth=-1.0)
pattern2 = visual.GratingStim(win=win, name='pattern2',units='cm', 
    tex=None, mask=None,
    ori=0, pos=[0, 0], size=3, sf=1, phase=0,
    color=[-1,-1,-1], colorSpace='rgb', opacity=1,
    texRes=256, interpolate=True, depth=-2.0)

# Initialize components for Routine "endOfExperiment"
endOfExperimentClock = core.Clock()
ThankYou = visual.TextStim(win=win, ori=0, name='ThankYou',
    text='thank you',    font='Arial',
    pos=[0, 0], height=1.5, wrapWidth=None,
    color='white', colorSpace='rgb', opacity=1,
    depth=0.0)
sound_1 = sound.Sound('A', secs=1)
sound_1.setVolume(1)

# Create some handy timers
globalClock = core.Clock()  # to track the time since experiment started
routineTimer = core.CountdownTimer()  # to track time remaining of each (non-slip) routine 

#------Prepare to start Routine "Instructions"-------
t = 0
InstructionsClock.reset()  # clock 
frameN = -1
# update component parameters for each repeat
key_resp_start = event.BuilderKeyResponse()  # create an object of type KeyResponse
key_resp_start.status = NOT_STARTED
# keep track of which components have finished
InstructionsComponents = []
InstructionsComponents.append(instructions)
InstructionsComponents.append(key_resp_start)
for thisComponent in InstructionsComponents:
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED

#-------Start Routine "Instructions"-------
continueRoutine = True
while continueRoutine:
    # get current time
    t = InstructionsClock.getTime()
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *instructions* updates
    if t >= 0.0 and instructions.status == NOT_STARTED:
        # keep track of start time/frame for later
        instructions.tStart = t  # underestimates by a little under one frame
        instructions.frameNStart = frameN  # exact frame index
        instructions.setAutoDraw(True)
    
    # *key_resp_start* updates
    if t >= 0.0 and key_resp_start.status == NOT_STARTED:
        # keep track of start time/frame for later
        key_resp_start.tStart = t  # underestimates by a little under one frame
        key_resp_start.frameNStart = frameN  # exact frame index
        key_resp_start.status = STARTED
        # keyboard checking is just starting
        event.clearEvents(eventType='keyboard')
    if key_resp_start.status == STARTED:
        theseKeys = event.getKeys(keyList=['space'])
        
        # check for quit:
        if "escape" in theseKeys:
            endExpNow = True
        if len(theseKeys) > 0:  # at least one key was pressed
            # a response ends the routine
            continueRoutine = False
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineTimer.reset()  # if we abort early the non-slip timer needs reset
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in InstructionsComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # check for quit (the Esc key)
    if endExpNow or event.getKeys(keyList=["escape"]):
        core.quit()
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()
    else:  # this Routine was not non-slip safe so reset non-slip timer
        routineTimer.reset()

#-------Ending Routine "Instructions"-------
for thisComponent in InstructionsComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)

#------Prepare to start Routine "Blank"-------
t = 0
BlankClock.reset()  # clock 
frameN = -1
# update component parameters for each repeat
# keep track of which components have finished
BlankComponents = []
BlankComponents.append(text)
BlankComponents.append(fixation)
for thisComponent in BlankComponents:
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED

#-------Start Routine "Blank"-------
continueRoutine = True
while continueRoutine:
    # get current time
    t = BlankClock.getTime()
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *text* updates
    if t >= 0.0 and text.status == NOT_STARTED:
        # keep track of start time/frame for later
        text.tStart = t  # underestimates by a little under one frame
        text.frameNStart = frameN  # exact frame index
        text.setAutoDraw(True)
    elif text.status == STARTED and frameN >= (text.frameNStart + 120):
        text.setAutoDraw(False)
    
    # *fixation* updates
    if t >= 0.0 and fixation.status == NOT_STARTED:
        # keep track of start time/frame for later
        fixation.tStart = t  # underestimates by a little under one frame
        fixation.frameNStart = frameN  # exact frame index
        fixation.setAutoDraw(True)
    elif fixation.status == STARTED and frameN >= (fixation.frameNStart + 120):
        fixation.setAutoDraw(False)
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineTimer.reset()  # if we abort early the non-slip timer needs reset
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in BlankComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # check for quit (the Esc key)
    if endExpNow or event.getKeys(keyList=["escape"]):
        core.quit()
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()
    else:  # this Routine was not non-slip safe so reset non-slip timer
        routineTimer.reset()

#-------Ending Routine "Blank"-------
for thisComponent in BlankComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)

# set up handler to look after randomisation of conditions etc
trials_120 = data.TrialHandler(nReps=120, method='random', 
    extraInfo=expInfo, originPath=None,
    trialList=[None],
    seed=None, name='trials_120')
thisExp.addLoop(trials_120)  # add the loop to the experiment
thisTrial_120 = trials_120.trialList[0]  # so we can initialise stimuli with some values
# abbreviate parameter names if possible (e.g. rgb=thisTrial_120.rgb)
if thisTrial_120 != None:
    for paramName in thisTrial_120.keys():
        exec(paramName + '= thisTrial_120.' + paramName)

for thisTrial_120 in trials_120:
    currentLoop = trials_120
    # abbreviate parameter names if possible (e.g. rgb = thisTrial_120.rgb)
    if thisTrial_120 != None:
        for paramName in thisTrial_120.keys():
            exec(paramName + '= thisTrial_120.' + paramName)
    
    #------Prepare to start Routine "InterTrial"-------
    t = 0
    InterTrialClock.reset()  # clock 
    frameN = -1
    # update component parameters for each repeat
    
    # keep track of which components have finished
    InterTrialComponents = []
    InterTrialComponents.append(Fixation)
    for thisComponent in InterTrialComponents:
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    
    #-------Start Routine "InterTrial"-------
    continueRoutine = True
    while continueRoutine:
        # get current time
        t = InterTrialClock.getTime()
        frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
        # update/draw components on each frame
        
        # *Fixation* updates
        if t >= 0.0 and Fixation.status == NOT_STARTED:
            # keep track of start time/frame for later
            Fixation.tStart = t  # underestimates by a little under one frame
            Fixation.frameNStart = frameN  # exact frame index
            Fixation.setAutoDraw(True)
        elif Fixation.status == STARTED and frameN >= (Fixation.frameNStart + 120):
            Fixation.setAutoDraw(False)
        
        
        # check if all components have finished
        if not continueRoutine:  # a component has requested a forced-end of Routine
            routineTimer.reset()  # if we abort early the non-slip timer needs reset
            break
        continueRoutine = False  # will revert to True if at least one component still running
        for thisComponent in InterTrialComponents:
            if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # at least one component has not yet finished
        
        # check for quit (the Esc key)
        if endExpNow or event.getKeys(keyList=["escape"]):
            core.quit()
        
        # refresh the screen
        if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
            win.flip()
        else:  # this Routine was not non-slip safe so reset non-slip timer
            routineTimer.reset()
    
    #-------Ending Routine "InterTrial"-------
    for thisComponent in InterTrialComponents:
        if hasattr(thisComponent, "setAutoDraw"):
            thisComponent.setAutoDraw(False)
    firstLoop=True
    
    
    # set up handler to look after randomisation of conditions etc
    withinATrial_fill_4_sec = data.TrialHandler(nReps=30, method='sequential', 
        extraInfo=expInfo, originPath=None,
        trialList=[None],
        seed=None, name='withinATrial_fill_4_sec')
    thisExp.addLoop(withinATrial_fill_4_sec)  # add the loop to the experiment
    thisWithinATrial_fill_4_sec = withinATrial_fill_4_sec.trialList[0]  # so we can initialise stimuli with some values
    # abbreviate parameter names if possible (e.g. rgb=thisWithinATrial_fill_4_sec.rgb)
    if thisWithinATrial_fill_4_sec != None:
        for paramName in thisWithinATrial_fill_4_sec.keys():
            exec(paramName + '= thisWithinATrial_fill_4_sec.' + paramName)
    
    for thisWithinATrial_fill_4_sec in withinATrial_fill_4_sec:
        currentLoop = withinATrial_fill_4_sec
        # abbreviate parameter names if possible (e.g. rgb = thisWithinATrial_fill_4_sec.rgb)
        if thisWithinATrial_fill_4_sec != None:
            for paramName in thisWithinATrial_fill_4_sec.keys():
                exec(paramName + '= thisWithinATrial_fill_4_sec.' + paramName)
        
        #------Prepare to start Routine "trial"-------
        t = 0
        trialClock.reset()  # clock 
        frameN = -1
        # update component parameters for each repeat
        # send time stamp
        #if firstLoop==True:
        #    t=time.time()#t=time.clock() # what is the difference?
        #    timeString= "%.5f" %t
        #    thisExp.addData('timestamp',timeString)
        # send code
        if firstLoop==True:
            ser.write(chr(70)) # BP send code 70
            firstLoop=False
            thisExp.addData('CodeSent',70)
        
        
        # keep track of which components have finished
        trialComponents = []
        trialComponents.append(pattern1)
        trialComponents.append(pattern2)
        for thisComponent in trialComponents:
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        
        #-------Start Routine "trial"-------
        continueRoutine = True
        while continueRoutine:
            # get current time
            t = trialClock.getTime()
            frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
            # update/draw components on each frame
            # Its going to write something for each frame, might as well put time stame
            t=time.time()#t=time.clock() # what is the difference?
            timeString= "%.5f" %t
            thisExp.addData('FrameTimeStamps',timeString)
            
            
            # *pattern1* updates
            if frameN >= 0.0 and pattern1.status == NOT_STARTED:
                # keep track of start time/frame for later
                pattern1.tStart = t  # underestimates by a little under one frame
                pattern1.frameNStart = frameN  # exact frame index
                pattern1.setAutoDraw(True)
            elif pattern1.status == STARTED and frameN >= (pattern1.frameNStart + 4):
                pattern1.setAutoDraw(False)
            
            # *pattern2* updates
            if frameN >= 4 and pattern2.status == NOT_STARTED:
                # keep track of start time/frame for later
                pattern2.tStart = t  # underestimates by a little under one frame
                pattern2.frameNStart = frameN  # exact frame index
                pattern2.setAutoDraw(True)
            elif pattern2.status == STARTED and frameN >= (pattern2.frameNStart + 4):
                pattern2.setAutoDraw(False)
            
            # check if all components have finished
            if not continueRoutine:  # a component has requested a forced-end of Routine
                routineTimer.reset()  # if we abort early the non-slip timer needs reset
                break
            continueRoutine = False  # will revert to True if at least one component still running
            for thisComponent in trialComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break  # at least one component has not yet finished
            
            # check for quit (the Esc key)
            if endExpNow or event.getKeys(keyList=["escape"]):
                core.quit()
            
            # refresh the screen
            if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
                win.flip()
            else:  # this Routine was not non-slip safe so reset non-slip timer
                routineTimer.reset()
        
        #-------Ending Routine "trial"-------
        for thisComponent in trialComponents:
            if hasattr(thisComponent, "setAutoDraw"):
                thisComponent.setAutoDraw(False)
        thisExp.addData('frameNumber',frameN)
        thisExp.nextEntry()
        
    # completed 30 repeats of 'withinATrial_fill_4_sec'
    
    # get names of stimulus parameters
    if withinATrial_fill_4_sec.trialList in ([], [None], None):  params = []
    else:  params = withinATrial_fill_4_sec.trialList[0].keys()
    # save data for this loop
    withinATrial_fill_4_sec.saveAsText(filename + 'withinATrial_fill_4_sec.csv', delim=',',
        stimOut=params,
        dataOut=['n','all_mean','all_std', 'all_raw'])
    thisExp.nextEntry()
    
# completed 120 repeats of 'trials_120'

# get names of stimulus parameters
if trials_120.trialList in ([], [None], None):  params = []
else:  params = trials_120.trialList[0].keys()
# save data for this loop
trials_120.saveAsText(filename + 'trials_120.csv', delim=',',
    stimOut=params,
    dataOut=['n','all_mean','all_std', 'all_raw'])

#------Prepare to start Routine "endOfExperiment"-------
t = 0
endOfExperimentClock.reset()  # clock 
frameN = -1
routineTimer.add(7.000000)
# update component parameters for each repeat
# keep track of which components have finished
endOfExperimentComponents = []
endOfExperimentComponents.append(ThankYou)
endOfExperimentComponents.append(sound_1)
for thisComponent in endOfExperimentComponents:
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED

#-------Start Routine "endOfExperiment"-------
continueRoutine = True
while continueRoutine and routineTimer.getTime() > 0:
    # get current time
    t = endOfExperimentClock.getTime()
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *ThankYou* updates
    if t >= 3 and ThankYou.status == NOT_STARTED:
        # keep track of start time/frame for later
        ThankYou.tStart = t  # underestimates by a little under one frame
        ThankYou.frameNStart = frameN  # exact frame index
        ThankYou.setAutoDraw(True)
    elif ThankYou.status == STARTED and t >= (3 + (4-win.monitorFramePeriod*0.75)): #most of one frame period left
        ThankYou.setAutoDraw(False)
    # start/stop sound_1
    if t >= 3.5 and sound_1.status == NOT_STARTED:
        # keep track of start time/frame for later
        sound_1.tStart = t  # underestimates by a little under one frame
        sound_1.frameNStart = frameN  # exact frame index
        sound_1.play()  # start the sound (it finishes automatically)
    elif sound_1.status == STARTED and t >= (3.5 + (1-win.monitorFramePeriod*0.75)): #most of one frame period left
        sound_1.stop()  # stop the sound (if longer than duration)
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineTimer.reset()  # if we abort early the non-slip timer needs reset
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in endOfExperimentComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # check for quit (the Esc key)
    if endExpNow or event.getKeys(keyList=["escape"]):
        core.quit()
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

#-------Ending Routine "endOfExperiment"-------
for thisComponent in endOfExperimentComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
ser.close() #BP

win.close()
core.quit()
