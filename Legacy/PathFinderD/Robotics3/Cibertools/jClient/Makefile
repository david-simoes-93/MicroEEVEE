
all: jClient

SOURCES = Engine/jClient.java ciberIF/ciberIF.java ciberIF/beaconMeasure.java ciberIF/gpsMeasure.java Engine/MapGUI.java Engine/Map.java Engine/Behaviour.java \
		Engine/Sensors.java Engine/Comm.java Engine/FollowTheBeacon.java Engine/FollowTheWall.java	Engine/DodgeObstacle.java Engine/Wander.java \
		Engine/EncodeMessage.java Engine/DecodeMessage.java   Engine/MapLoaderThread.java


JARFILES = ciberIF/beaconMeasure.class  ciberIF/gpsMeasure.class\
	    ciberIF/Parameters.class  ciberIF/Measures.class\
	    ciberIF/ReplyHandler.class  ciberIF/SensorHandler.class  ciberIF/xmlParser.class\
        ciberIF/ciberIF.class    
JARFILES2 = Engine/Behaviour.class Engine/Sensors.class Engine/Comm.class Engine/MapGUI.class Engine/Map.class Engine/FollowTheBeacon.class \
		Engine/FollowTheWall.class	Engine/DodgeObstacle.class Engine/Wander.class  Engine/EncodeMessage.class Engine/DecodeMessage.class  \
		Engine/MapLoaderThread.class

jClient:  
	javac $(SOURCES)
	jar -cf ciberIF.jar $(JARFILES)
	jar -cf Engine.jar $(JARFILES2)
	#rm -f $(JARFILES)

javadoc:
	javadoc -d doc -public $(SOURCES)

clean:
	rm -rf Engine/*.class ciberIF/*.class *.jar *~ ciberIF/*~ core* doc
