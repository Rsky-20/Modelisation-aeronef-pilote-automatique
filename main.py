#import lib.utils as utils
#from lib.utils import ControlAircraftModel

from lib.test_control_aircraft_model import ControlAircraftModel
from lib.test_display_CAM import Display_CAM, display_time_process
import lib.sisopy31 as spy31
import lib.atm_std as atm_std


def main_old():
    # Initialize aircraft model
    aircraft = ControlAircraftModel()
    
    
    aircraft.display_time_process(aircraft.equilibrium_conditions)
    aircraft.display_time_process(aircraft.compute_dynamic_matrix)
    aircraft.display_time_process(aircraft.compute_control_matrix)
    # aircraft.display_aircraft_parameters()
    # aircraft.display_equilibrium_condition_values()
    aircraft.display_matrix()
    aircraft.display_time_process(aircraft.open_loop)
    #aircraft.display_state_space_info()
    aircraft.compute_transient_phase()
    #aircraft.display_transient_phase()
    aircraft.plot_step_response()

def main():
    aircraft = ControlAircraftModel()
    display_cam = Display_CAM(aircraft)
    
    display_cam.process('Aircraft Characteristics')
    display_cam.aircraft_parameters()
    
    display_cam.process('Find : Eq. Point')
    display_time_process(aircraft.equilibrium_conditions)
    display_cam.equilibrium_condition_values()
    
    display_cam.process('State Space arround Eq Point')
    display_time_process(aircraft.compute_dynamic_matrix)
    display_time_process(aircraft.compute_control_matrix)
    display_cam.matrix()
    
    display_cam.process('Study : Study of open loop modes')
    display_time_process(aircraft.open_loop)
    display_cam.open_loop()

    
    display_cam.process('Study : Transient phase of the uncontrolled aircraft')
    display_time_process(aircraft.compute_transient_phase)
    display_cam.transient_phase()
    display_cam.plot_step_response()


if __name__ == '__main__':
    main()