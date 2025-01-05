from lib.control_aircraft_model import ControlAircraftModel
from lib.display_CAM import Display_CAM, display_time_process
import lib.sisopy31 as spy31
import lib.atm_std as atm_std
import inquirer
import os

# Configuration globale
display_process_time = True

# Function to clear the console
def clear_console():
    os.system('cls' if os.name == 'nt' else 'clear')

# Function to pause and wait for user input
def pause():
    input("\nPress Enter to continue...")


def execute_all_steps(aircraft, display_cam):
    """Execute all predefined steps for the selected aircraft."""
    if display_process_time:
        display_time_process(aircraft.equilibrium_conditions)
        display_time_process(aircraft.compute_dynamic_matrix)
        display_time_process(aircraft.compute_control_matrix)
        display_time_process(aircraft.open_loop)
        display_time_process(aircraft.compute_transient_phase)
        display_time_process(lambda: aircraft.compute_feedback_loop('q'))
        display_time_process(lambda: aircraft.compute_feedback_loop('gamma'))
        display_time_process(lambda: aircraft.compute_feedback_loop('z'))
        display_time_process(lambda: aircraft.saturation())
    else:
        aircraft.equilibrium_conditions()
        aircraft.compute_dynamic_matrix()
        aircraft.compute_control_matrix()
        aircraft.open_loop()
        aircraft.compute_transient_phase()
        aircraft.compute_feedback_loop('q')
        aircraft.compute_feedback_loop('gamma')
        aircraft.compute_feedback_loop('z')
        aircraft.saturation()
    
    pause()

def main_menu():
    """Interactive menu for choosing an aircraft and processing functions."""
    # Aircraft choices
    plane_choices = ['MIRAGE III class with c=0.52', 'MIRAGE III class with new COG', 'Quit']

    # Function choices
    function_choices = [
        'Aircraft Characteristics',
        'Find: Eq. Point',
        'State Space around Eq Point',
        'Study: Open loop modes',
        'Study: Transient phase of the uncontrolled aircraft',
        'Build: q FEEDBACK LOOP',
        'Build: gamma FEEDBACK LOOP',
        'Build: z FEEDBACK LOOP',
        'SISOPY31.SISOTOOL GAIN',
        'ADDITION OF A SATURATION',
        'Quit'
    ]

    while True:
        # Select aircraft
        clear_console()
        plane_question = [
            inquirer.List(
                'plane',
                message="Select a plane to process:",
                choices=plane_choices
            )
        ]
        plane_answer = inquirer.prompt(plane_question)['plane']

        # Initialize aircraft and display_cam
        if plane_answer == 'MIRAGE III class with c=0.52':
            aircraft = ControlAircraftModel()
        elif plane_answer == 'MIRAGE III class with new COG':
            aircraft = ControlAircraftModel(name='MIRAGE_III_class_with_new_COG')
            aircraft.c = 1.1 * aircraft.f  # Adjust center of gravity
            aircraft.compute_new_X_Y()
        elif plane_answer == 'Quit':
            clear_console()
            print("Exiting the menu. Goodbye!")
            break
            
        aircraft.save_fig = True
        display_cam = Display_CAM(aircraft)
        
        execute_all_steps(aircraft, display_cam)

        while True:
            clear_console()
            # Select function
            questions = [
                inquirer.List(
                    'menu',
                    message="Select a part to process:",
                    choices=function_choices
                )
            ]
            answer = inquirer.prompt(questions)['menu']

            # Execute selected function
            if answer == 'Aircraft Characteristics':
                clear_console()
                display_cam.process('Aircraft Characteristics')
                display_cam.aircraft_parameters()
                pause()

            elif answer == 'Find: Eq. Point':
                clear_console()
                display_cam.process('Find: Eq. Point')
                if display_process_time:
                    display_time_process(aircraft.equilibrium_conditions)
                else:
                    aircraft.equilibrium_conditions()
                display_cam.equilibrium_condition_values()
                pause()

            elif answer == 'State Space around Eq Point':
                clear_console()
                display_cam.process('State Space around Eq Point')
                if display_process_time:
                    display_time_process(aircraft.compute_dynamic_matrix)
                    display_time_process(aircraft.compute_control_matrix)
                else:
                    aircraft.compute_dynamic_matrix()
                    aircraft.compute_control_matrix()
                display_cam.matrix()
                pause()

            elif answer == 'Study: Open loop modes':
                clear_console()
                display_cam.process('Study: Open loop modes')
                if display_process_time:
                    display_time_process(aircraft.open_loop)
                else:
                    aircraft.open_loop()
                display_cam.open_loop()
                pause()

            elif answer == 'Study: Transient phase of the uncontrolled aircraft':
                clear_console()
                display_cam.process('Study: Transient phase of the uncontrolled aircraft')
                if display_process_time:
                    display_time_process(aircraft.compute_transient_phase)
                else:
                    aircraft.compute_transient_phase()
                display_cam.transient_phase()
                pause()

            elif answer == 'Build: q FEEDBACK LOOP':
                clear_console()
                display_cam.process('Build: q FEEDBACK LOOP')
                if display_process_time:
                    display_time_process(lambda: aircraft.compute_feedback_loop('q'))
                else:
                    aircraft.compute_feedback_loop('q')
                display_cam.closedloop_TF('q')
                pause()

            elif answer == 'Build: gamma FEEDBACK LOOP':
                clear_console()
                display_cam.process('Build: gamma FEEDBACK LOOP')
                if display_process_time:
                    display_time_process(lambda: aircraft.compute_feedback_loop('gamma'))
                else:
                    aircraft.compute_feedback_loop('gamma')
                display_cam.closedloop_TF('gamma')
                pause()

            elif answer == 'Build: z FEEDBACK LOOP':
                clear_console()
                display_cam.process('Build: z FEEDBACK LOOP')
                if display_process_time:
                    display_time_process(lambda: aircraft.compute_feedback_loop('z'))
                else:
                    aircraft.compute_feedback_loop('z')
                display_cam.closedloop_TF('z')
                pause()

            elif answer == 'SISOPY31.SISOTOOL GAIN':
                clear_console()
                display_cam.process('SISOPY31.SISOTOOL GAIN')
                display_cam.gain_k()
                pause()

            elif answer == 'ADDITION OF A SATURATION':
                clear_console()
                display_cam.process('ADDITION OF A SATURATION')
                if display_process_time:
                    display_time_process(lambda: aircraft.saturation())
                else:
                    aircraft.saturation()
                display_cam.saturation()
                pause()

            elif answer == 'Quit':
                clear_console()
                print("Exiting the menu. Goodbye!")
                break

def main(plane):
    """Execute all steps for the MIRAGE III with c=0.52."""
    if plane =='MIRAGE III with c=0.52':
        aircraft = ControlAircraftModel()
    if plane =='MIRAGE III with new COG':
        aircraft = ControlAircraftModel(name='MIRAGE_III_class_with_new_COG')
        aircraft.c = 1.1 * aircraft.f  # Adjust center of gravity
    aircraft.save_fig = True
    display_cam = Display_CAM(aircraft)
    execute_all_steps(aircraft, display_cam)

if __name__ == '__main__':
    # Comment/uncomment to choose main or menu
    main_menu()
    #main("MIRAGE III with c=0.52") #/'MIRAGE III with new COG'