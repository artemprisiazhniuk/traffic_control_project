from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 120  # number of time steps
    # demand per second from different directions
    pWE = 1. / 15
    pEW = 1. / 15
    pNS = 1. / 15
    pSN = 1. / 15
    with open("./cross.route.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="18"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="18"/>

        <route id="right" edges="left0A0 A0right0" />
        <route id="left" edges="right0A0 A0left0" />
        <route id="down" edges="top0A0 A0bottom0" />
        <route id="up" edges="bottom0A0 A0top0" /> """, file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pSN:
                print('    <vehicle id="up_%i" type="typeNS" route="up" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1

        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


def run():
    from numpy import argmax
    """execute the TraCI control loop"""
    step = 0
    # we start with phase 2 where EW has green
    traci.trafficlight.setPhase("A0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        occupancy_list = [
                traci.lane.getLastStepVehicleNumber('bottom0A0_0') + traci.lane.getLastStepVehicleNumber('top0A0_0'),
                traci.lane.getLastStepVehicleNumber('right0A0_0') + traci.lane.getLastStepVehicleNumber('left0A0_0'),
                ]
        max_id = argmax(occupancy_list)
        
        phase = traci.trafficlight.getPhase("A0")
        if phase != 2 and max_id == 1:
                traci.trafficlight.setPhase("A0", 2)
        elif phase == 2 and max_id == 0:
                traci.trafficlight.setPhase("A0", 0)
        step += 1
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "fstcrossing.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()

