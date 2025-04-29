# Microspine-Robot-2025

Codebase for the operation of the 2025 Microspine Robot

## Contents
 - `dynamixel.py` - dynamixel driver implementing serial comms
 - `servo.py` - servo class for each dynamixel
 - `gui.py` - graphical user interface implementation file
 - `main.py` - main file for running anchoring system
 - `port.py` - Port class file, used to connect to microcontroller

# Getting Started
## Software Setup
 - Clone this repository using `git clone https://github.com/pmof13323/Microspine-Robot-2025.git`
    - Ensure you have set up an SSH key using the instructions here: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
 - Clone the Dynamixel SDK into the same folder using `git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git`
    - More info here: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/#dynamixel-sdk
 - Download the Dynamixel Wizard 2.0 from here: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/ 

## Hardware Setup
 - Connect Dynamixel to Open RB via 3 pin connector
 - Connect Open RB to computer using a USB-C cable this will power the servo
 - Open the Dynamixel Wizard application

## Dynamixel Wizard Setup
 - Click 'Scan' from the Device drop down menu
 - Configure the scan settings:
    - Select 'Protocol 2.0'
    - Select the COM port connected to the Open RB
    - Select all baud rates (Dynamixel should be 57600)
    - Select 0-10 from the bottom slider
 - Execute the scan, the Dynamixel should be discovered and a control menu should pop up

# Git Workflow
## Cloning
 - `git clone` - clone the remote repository to your local machine
 - `git pull` - update your local repository with any remote changes (needed to avoid conflicts)

## Branching
 - `git checkout main` - checkout main branch before making changes
 - `git branch <branch name>` - create local branch off main to make changes

## Committing changes
 - `git add -p` - add changes from your local
 - `git commit -m <insert commit message>` - commit changes
 - `git push` - push changes to remote repository

# Software Setup Cont.
## Firmware Update
- SDK Instructions (https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) (5.5 in Basic Features)
- On Dynamixel Wizard check once connected if it has asterisk over the device name on left hand side tab 
- If there is go to Tools->Firmware Update

## Code
- Ensure to install setup.py from python folder from DynamixelSDK 