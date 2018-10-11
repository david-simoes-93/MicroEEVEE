# MicroEEVEE

A global repository for MicroMouse, CiberMouse, and a DIY EEVEE agent.

## EEVEE

We built the 3D Model in [TinkerCad](https://www.tinkercad.com/), [SolidWorks](https://www.solidworks.com/), and [MeshMixer](http://www.meshmixer.com/), and published it in [ThingiVerse](https://www.thingiverse.com/thing:3148948). We based our design on Jukka Seppänen's great [design](https://www.yobi3d.com/v/4kIfHKWdJ5/eevee-kijaidesign-01.stl).

![eevee model](https://user-images.githubusercontent.com/9117323/46804341-4600ac00-cd5a-11e8-8fcd-86e9f92aa558.png)

Controlled with a [RaspberryPi 3b](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) with a [camera](https://uk.pi-supply.com/products/raspberry-pi-camera-board-v1-3-5mp-1080p), an [Arduino Pro Mini](https://store.arduino.cc/arduino-pro-mini) connected through a [TTL/USB adapter](https://www.sunrom.com/p/cp2102-usb-ttl-uart-module), 2 [6v 210RPM DC Motors](https://www.tosave.com/p/New-6V-210RPM-Encoder-Gear-Motor-4mm-Shaft-With-Mounting-Bracket-65mm-Wheel-Kit-134140.html) and their [motor driver](https://www.bananarobotics.com/shop/How-to-use-the-L298N-Dual-H-Bridge-Motor-Driver), 5 [ground sensors](https://www.pololu.com/file/0J117/QRE1113GR.pdf), 2 [InfraRed sensors](https://www.pololu.com/product/136), 4 [UltraSound sensors](https://www.mouser.com/ds/2/813/HCSR04-1022824.pdf), 2 buttons, an on/off switch, and 2 LEDs. We use a [2S to 5V 3A UBEC](https://www.aliexpress.com/item/Hot-Sale-DC-DC-Converter-Step-Down-Module-UBEC-3A-5V-12V-BEC-For-RC-Airplane/32824894112.html?spm=a2g0s.13010208.99999999.265.63b73c009dZqCQ) to have the same [2-cell 1800mAh LiPo](https://www.botnroll.com/en/batteries/1053-hacker-lipobattery74-v-900mah.html) to power everything, a [buzzer monitor](https://www.aliexpress.com/item/Voltage-monitor-1-8S-Lipo-Li-ion-Fe-Battery-Voltage-2IN1-Tester-Low-Voltage-Buzzer-Alarm/32847601843.html?spm=a2g0s.13010208.99999999.259.67d43c0030BAoU), charged with a [Graupner Li-charger](https://www.graupner.de/mediaroot/files/6462_Li_Charger_4_Plus_de_en_fr.pdf) and the corresponding [12V 3A charger](https://www.botnroll.com/en/power-supply-ac-dc-12v/1101-power-supply-acdc-12v-3a.html). 

Regarding the DC/DC Step-Down Converter (a.k.a UBEC in the RC community), as per this [issue](https://raspberrypi.stackexchange.com/questions/89731/can-this-dc-motor-driver-feed-my-rpi) and this [solution](https://electronics.stackexchange.com/questions/400000/powering-a-rpi-and-a-motor-driver-with-the-same-lipo-battery): the LiPo has 25C discharge rate, so 45A peak current; the peak stall current of our motors is 2x3.2A, RPiB is 1A, ArduinoPM is 0.1A + 0.04A per pin, camera is 0.25A, UltraSound are 4x0.015A, InfraRed are 2x0.03A, ground sensors are 5x0.05A, for a total of about 2.32A for the RPi and 6.2A for the motors, well below our limit. We want to provide 5.1v and 2.5A to our RPi (recommended charger values), so we require 12.75W output for the UBEC. Since our LiPo can range from 6.6V to 8.4V output, the UBEC's theoretical maximum current draw will be 1.93A (if we had 100% efficiency), so somewhat over 10A will be drawn from the LiPo in peak times. Since everything seems to be within limits, we then need a Step-Down converter with 6-9V input, 5V and 2.5A output, like this [one](https://www.pololu.com/product/2850/specs).

Here's the circuitry, made with [draw.io](https://drive.google.com/file/d/1LcyPtO9vEOZJ04b2OpFpYvRo7CwWos43/view?usp=sharing).

![circuits](https://raw.githubusercontent.com/bluemoon93/MicroEEVEE/master/EEVEE/Circuits.png)

Install [Raspbian Lite](https://www.raspberrypi.org/downloads/raspbian/), enable [ssh](https://www.raspberrypi.org/documentation/remote-access/ssh/README.md), and make your RPi an [AP](https://www.raspberrypi.org/documentation/configuration/wireless/access-point.md) so you can remotely control things. Make sure to ssh into the RPi with the -X option if you want to see the GUI. Then update everything, install required packages, get the repo, and ready things.

    sudo apt update
    sudo apt upgrade
    nano .bashrc
        export LANG=en_GB.UTF-8
        export LC_ALL=en_GB.UTF-8
        export LC_CTYPE=en_GB.UTF-8
    source .bashrc
    sudo apt install tmux git python3-serial 
    virtualenv -p /usr/bin/python3 py3env
    source py3env/bin/activate
    pip install RPi.GPIO pygame
    git clone https://github.com/bluemoon93/MicroEEVEE.git
    cd MicroEEVEE
    chmod +x startMEevee.sh
    
To run, simply 

    ./startMEevee.sh
    
Not everything is complete! Our TODO list:

    software
        fazer PID
        odometria
    soldar as placas
    testar os botoes
    testar os LEDs
    InfraRed: Average current consumption: 30 mA (note: this sensor draws current in large, short bursts, and the manufacturer recommends putting a 10 µF capacitor or larger across power and ground close to the sensor to stabilize the power supply line)
    testar UBEC
    

## MazeRunner (Micro-Rato)

More info [here](http://microrato.ua.pt/microrato).

## Explorer (CiberEEVEE)

More info [here](http://microrato.ua.pt/ciberrato). To run CiberEEVEE, just activate your environment and

    ./startEevee.sh
    
Install dependencies with

    python3 -m venv ./venv
    source venv/bin/activate
    sudo pip install --upgrade pip
    sudo pip3 install pygame
    (cd CiberSim; make)
    (cd CiberViewer; make)
