#!/usr/bin/env python

import asyncio, argparse
from mavsdk import System, telemetry
from drone import Drone


UDP_NUM_START = 14540
PORT_NUM_START = 50050


async def main(config):
    # Create drone objects for desired number of drones
    drones_list = []

    for i in range(int(config['num_drones'])):
        drone_new = await Drone.create(system_address=f"udp://:{UDP_NUM_START+i}", port=(50050+i))
        drones_list.append(drone_new)

    # Fly drones simulatenously - hardcoded
    drones_fly_coroutines = []

    for count, drone in enumerate(drones_list):
        drones_fly_coroutines.append(asyncio.ensure_future(drone.run_hardcoded_mavlink()))

    # Wait for all drones to be finished
    for drone_fly_couroutine in drones_fly_coroutines:
        await drone_fly_couroutine


# Ensure multiple mavsdk_servers are running first, using e.g. ./multi_mavsdk_server.sh -n 3
if __name__ == '__main__':
    # Accept number of drones to control
    parser = argparse.ArgumentParser(description="Launch multiple drone control nodes", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-n","--num_drones", default=1, help="number of drones to control")
    args = parser.parse_args()
    config = vars(args)
    print("about to loop")

    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(config))

    
