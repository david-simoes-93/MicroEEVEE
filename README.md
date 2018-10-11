# MicroEEVEE

A global repository for MicroMouse, CiberMouse, and a DIY EEVEE agent.

## EEVEE

Controlled with a [RaspberryPi 3b](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/), an [Arduino Pro Mini](https://store.arduino.cc/arduino-pro-mini), 2 DC Motors, an [L298N Dual H-Bridge Motor Driver](https://www.bananarobotics.com/shop/How-to-use-the-L298N-Dual-H-Bridge-Motor-Driver), 5 [QRE1113GR sensors](https://www.pololu.com/file/0J117/QRE1113GR.pdf),

Make your RPi an [AP](https://www.raspberrypi.org/documentation/configuration/wireless/access-point.md) so you can remotely control things.

We use a UBEC to have the same 2-cell LiPo to power everything (as per this [issue](https://raspberrypi.stackexchange.com/questions/89731/can-this-dc-motor-driver-feed-my-rpi) and this [solution](https://electronics.stackexchange.com/questions/400000/powering-a-rpi-and-a-motor-driver-with-the-same-lipo-battery)).

TODO:

    software
        fazer PID
        odometria
    soldar as placas
    testar os botoes
    espaço pa bateria(s) tb
    UBEC https://electronics.stackexchange.com/questions/400000/powering-a-rpi-and-a-motor-driver-with-the-same-lipo-battery/400210#400210

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
