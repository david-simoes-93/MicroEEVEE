#/bin/bash

source venv/bin/activate

(cd CiberSim; ./simulator)&
sleep 1 

(cd CiberViewer; ./Viewer)&
sleep 1

(export PYTHONPATH=$(pwd); cd CiberEEVEE; python3 eevee.py)

killall simulator Viewer

