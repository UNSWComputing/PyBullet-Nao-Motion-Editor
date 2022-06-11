import sys
from qibullet import SimulationManager

if __name__ == "__main__":
    print("Starting simulation ...")
    simulation_manager = SimulationManager()

    client_id = simulation_manager.launchSimulation(gui=True)
    # Note: only 1 GUI can be launched at a time.

    print("Spawning Nao ...")
    nao = simulation_manager.spawnNao(
        client_id,
        spawn_ground_plane=True)
    
    # A blocking call to keep the simulation open
    input("- Press any key to terminate simulation -")

    print("Terminating simulation ...")
    simulation_manager.stopSimulation(client_id)
