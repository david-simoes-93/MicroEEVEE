#/bin/bash

source venvCiber/bin/activate 

(cd CiberSim; ./simulator)&
sleep 3

(cd CiberViewer; ./Viewer)&
sleep 3

(export PYTHONPATH=$(pwd); cd CiberEEVEE; python3 eevee.py)

killall simulator Viewer

