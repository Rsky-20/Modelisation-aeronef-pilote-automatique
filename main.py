from lib.control_aircraft_model import ControlAircraftModel
from lib.display_CAM import Display_CAM, display_time_process
import lib.sisopy31 as spy31
import lib.atm_std as atm_std
import inquirer
import os


aircraft = ControlAircraftModel()
aircraft.save_fig = True
display_cam = Display_CAM(aircraft)
display_process_time = True


# Function to clear the console
def clear_console():
    os.system('cls' if os.name == 'nt' else 'clear')

# Function to pause and wait for user input
def pause():
    input("\nPress Enter to continue...")

def main_menu():
    """Menu interactif pour naviguer entre les parties."""
    choices = [
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
        pause()
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


    while True:
        clear_console()
        questions = [
            inquirer.List(
                'menu',
                message="Select a part to process:",
                choices=choices
            )
        ]
        answer = inquirer.prompt(questions)

        if answer['menu'] == 'Aircraft Characteristics':
            clear_console()
            display_cam.process('Aircraft Characteristics')
            display_cam.aircraft_parameters()
            pause()

        elif answer['menu'] == 'Find: Eq. Point':
            clear_console()
            display_cam.process('Find : Eq. Point')
            display_cam.equilibrium_condition_values()
            pause()

        elif answer['menu'] == 'State Space around Eq Point':
            clear_console()
            display_cam.process('State Space around Eq Point')
            display_cam.matrix()
            pause()

        elif answer['menu'] == 'Study: Open loop modes':
            clear_console()
            display_cam.process('Study : Study of open loop modes')
            display_cam.open_loop()
            pause()

        elif answer['menu'] == 'Study: Transient phase of the uncontrolled aircraft':
            clear_console()
            display_cam.process('Study : Transient phase of the uncontrolled aircraft')
            display_cam.transient_phase()
            pause()

        elif answer['menu'] == 'Build: q FEEDBACK LOOP':
            clear_console()
            display_cam.process('Build : q FEEDBACK LOOP')  
            display_cam.closedloop_TF('q')
            pause()
            
        elif answer['menu'] == 'Build: gamma FEEDBACK LOOP':
            clear_console()
            display_cam.process('Build : gamma FEEDBACK LOOP')
            display_cam.closedloop_TF('gamma')
            pause()

        elif answer['menu'] == 'Build: z FEEDBACK LOOP':
            clear_console()
            display_cam.process('Build : z FEEDBACK LOOP')
            display_cam.closedloop_TF('z')
            pause()
            
        elif answer['menu'] == 'SISOPY31.SISOTOOL GAIN':
            clear_console()
            display_cam.process('SISOPY31.SISOTOOL GAIN')
            display_cam.gain_k()
            pause()
            
        elif answer['menu'] == 'ADDITION OF A SATURATION':
            clear_console()
            display_cam.process('ADDITION OF A SATURATION')
            display_cam.saturation()
            pause()
        
        elif answer['menu'] == 'Quit':
            clear_console()
            print("Exiting the menu. Goodbye!")
            break

def main():
    
    display_cam.process('Aircraft Characteristics')
    #display_cam.aircraft_parameters()
    
    display_cam.process('Find : Eq. Point')
    if display_process_time:
        display_time_process(aircraft.equilibrium_conditions)
    else:aircraft.equilibrium_conditions()
        
    #display_cam.equilibrium_condition_values()
    
    display_cam.process('State Space arround Eq Point')
    if display_process_time:
        display_time_process(aircraft.compute_dynamic_matrix)
        display_time_process(aircraft.compute_control_matrix)
    else:
        aircraft.compute_dynamic_matrix()
        aircraft.compute_control_matrix()
    #display_cam.matrix()
    
    display_cam.process('Study : Study of open loop modes')
    if display_process_time:
        display_time_process(aircraft.open_loop)
    else:aircraft.open_loop()
    #display_cam.open_loop()

    
    display_cam.process('Study : Transient phase of the uncontrolled aircraft')
    if display_process_time:
        display_time_process(aircraft.compute_transient_phase)
    else:aircraft.compute_transient_phase()
    #display_cam.transient_phase()
    #display_cam.plot_step_response()
    
    display_cam.process('Build : q FEEDBACK LOOP')
    if display_process_time:
        display_time_process(lambda:aircraft.compute_feedback_loop('q'))
    else:aircraft.compute_feedback_loop('q')
    #display_cam.closedloop_TF('q')
    #display_cam.display_damp(aircraft.damp_closeloop_q[2], aircraft.damp_closeloop_q[1], aircraft.damp_closeloop_q[0])
    
    display_cam.process('Build : gamma FEEDBACK LOOP')
    if display_process_time:
        display_time_process(lambda:aircraft.compute_feedback_loop('gamma'))
    else:aircraft.compute_feedback_loop('gamma')
    #display_cam.closedloop_TF('gamma')
    
    display_cam.process('Build : z FEEDBACK LOOP')
    if display_process_time:
        display_time_process(lambda:aircraft.compute_feedback_loop('z'))
    else:aircraft.compute_feedback_loop('z')
    #display_cam.closedloop_TF('z')    
    #display_cam.gain_k()
    
    
    aircraft.saturation()


if __name__ == '__main__':
    main_menu()
    #main()