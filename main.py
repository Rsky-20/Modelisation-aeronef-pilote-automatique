import lib.utils as utils
from lib.utils import ControlAircraftModel
import lib.sisopy31 as spy31
import numpy as np
import lib.atm_std as atm_std


def main():
    # Initialize aircraft model
    aircraft = ControlAircraftModel()
    
    aircraft.display_time_process(aircraft.equilibrium_conditions)
    aircraft.display_time_process(aircraft.compute_dynamic_matrix)
    aircraft.display_time_process(aircraft.compute_control_matrix)
    #aircraft.display_equilibrium_condition_values()
    #aircraft.display_matrix()
    aircraft.display_time_process(aircraft.open_loop)
    aircraft.display_state_space_info()



if __name__ == '__main__':
    main()