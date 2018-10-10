# MicroEEVEE
MicroMouse and CiberMouse repo for the EEVEE agent!

To run CiberEEVEE, just activate your environment and

    ./startEevee.sh
    
Install dependencies with

    python3 -m venv ./venv
    source venv/bin/activate
    sudo pip install --upgrade pip
    sudo pip3 install pygame
    (cd CiberSim; make)
    (cd CiberViewer; make)
    
Make your RPi an [AP](https://www.raspberrypi.org/documentation/configuration/wireless/access-point.md) so you can remotely control things.

TODO:

    software
        fazer PID
        odometria
    soldar as placas
    testar os botoes
    mudar o design 3d
        mudar botoes
        buracos pos LEDs
        buracos para os pilares tb
    espaço pa bateria(s) tb
    UBEC https://electronics.stackexchange.com/questions/400000/powering-a-rpi-and-a-motor-driver-with-the-same-lipo-battery/400210#400210
