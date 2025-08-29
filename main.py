import numpy as np
import control
import pandas as pd
import customtkinter as ctk
from tkinter import filedialog
from visualizer import Visualizer


# Parameters defining the system
default_ks =50750   # Stiffness of the spring
kt = 190000  # Stiffness of the tire
default_cs =1750    # Damping constant
ms = 285.3    # Mass sprung
mus = 60    # Mass unsprung

# Global Variables
displ_list, displ_pid_list, acc_list, acc_pid_list = [], [], [], []
z0, z1, z2, road_x, road_z, x = [], [], [], [], [], []
umf, road, displ, displ_pid, acc, acc_pid = 0, 0, 0, 0, 0, 0
yr = []

# Simulation Parameters
tstart = 0
tstop = 9
increment = 0.001
t = np.arange(tstart, tstop + 1, increment)

Ar = 0.025
Pag = 90
Fap = Pag * Ar

# Function to browse and load a CSV file
def browse_file():
    global yr  # Use the global yr variable
    file_path = filedialog.askopenfilename(filetypes=[("CSV Files", "*.csv")])
    if file_path:
        road = pd.read_csv(file_path, header=None, names=['Value'])
        yr = road['Value'].tolist()
    # Update the label with the selected file path
    selected_file_label.configure(text=f"Selected File: {file_path}")

# Function to handle dropdown selection
def on_dropdown_change(event):
    global yr
    selected_item = dropdown_var.get()
    selected_index = dropdown_options.index(selected_item)
    selected_path = dropdown_paths[selected_index]
    
    if selected_item == "None(Browse)":
        browse_button.configure(state=ctk.NORMAL)  # Enable the button
    else:
        browse_button.configure(state=ctk.DISABLED)  # Disable the button
        
    if selected_path:  # Check if the selected option has a non-empty path
        print("Selected:", selected_item)
        road = pd.read_csv(selected_path, header=None, names=['Value'])
        yr = road['Value'].tolist()  # Call your simulation function with the selected path

# Function to update parameters based on selected mode
def update_parameters():
    selected_mode = mode_var.get()

    if selected_mode == "Comfort\nks=53000\ncs=7340":
        cs_value.set(7340)
        ks_value.set(53000)
    elif selected_mode == "Normal\nks=50750\ncs=1750":
        cs_value.set(default_cs)
        ks_value.set(default_ks)
    elif selected_mode == "Sport\nks=47466\ncs=815":
        cs_value.set(815)
        ks_value.set(47466)
    elif selected_mode == "Track\nks=43569\ncs=456":
        cs_value.set(456)
        ks_value.set(43569)

        
def start_simulation():
    global ks_value,kt,cs_value,ms,mus,t,Fap,yr,tstop,increment,displ_list,acc_list,displ_pid_list,acc_pid_list,z0,z1,z2,x,road_x,road_z,x,umf
    cs=cs_value.get()
    ks=ks_value.get()
    road_x = np.linspace(0, 100, 10000)
    v=10
    if yr is not None:
        A = [[0, 1, 0, 0], [-ks / ms, -cs / ms, ks / ms, cs / ms], [0, 0, 0, 1], [ks / mus, cs / mus, -(ks + kt) / mus, -cs / mus]]
        B = [[0, 0], [1 / ms, 0], [0, 0], [-1 / mus, kt / mus]]
        C = [[-ks / ms, -cs / ms, ks / ms, cs / ms], [0, 0, 1, 0], [1, 0, -1, 0]]
        D = [[1 / ms, 0], [0, -1], [0, 0]]
        sys = control.ss(A, B, C, D)

        # Initial conditions: [displacement, velocity, unsprung mass displacement, unsprung mass velocity] = [0, 0, 0, 0]
        x0 = [0, 0, 0, 0]

        # Stack yr and Fap for the input to forced_response
        inputs = np.vstack((Fap * np.ones_like(t), yr))

        # Step response for the system with the two inputs
        t, y, x = control.forced_response(sys, t, inputs, X0=x0, return_x=True)

        # Use the Ziegler-Nichols tuning formulas to calculate PID gains
        c = 500
        Ti = (4*cs+15)/75
        Kp =(6+0.5*cs)/(9+8.4*c)
        Ki = 1/Ti
        Kd = 0

        # PID Controller Variables
        integral_sum = 0
        prev_error = 0
        # Arrays to store PID control inputs and outputs
        pid_inputs = np.zeros_like(t)
        pid_outputs = np.zeros_like(t)

        # Loop through each time step and calculate PID control effort
        for i in range(1, len(t)):
            # Calculate the error (difference between setpoint and current displacement)
            error = pid_inputs[i] - y[2, i]

            # Calculate the integral term (sum of errors over time)
            integral_sum += error * increment

            # Calculate the derivative term (change in error over time)
            derivative = (error - prev_error) / increment
            prev_error = error

            # Calculate the PID control effort (control signal)
            pid_control_effort = Kp * error +Ki*integral_sum + Kd * derivative

            # Store the PID control effort in the input array for the system
            inputs[1, i] = pid_control_effort
            pid_inputs[i] = inputs[1, i]
            pid_outputs[i] = pid_control_effort

        # Perform the simulation of the system with PID control using the modified inputs
        t_sim, y_pid, x_pid = control.forced_response(sys, t, inputs, X0=x0, return_x=True)
        
        road_z=yr
        dx = road_x[1] - road_x[0]
        diff_road_z=np.concatenate(([0], np.diff(road_z)))
        z0dot=np.zeros((len(road_z), 2))
        z0dot[:, 1] = diff_road_z / increment
        z0dot=z0dot[:,1]
        x=v*t
        umf=1
        u=np.interp(x, road_x, z0dot)
        u= np.vstack((Fap * np.ones_like(t), u))
        z0=np.interp(x, road_x, road_z) * umf
        print(z0)
        z1 = z0 + y_pid[1]
        z2 = z1 + y_pid[2]
        # Plot the results with PID control
        displ_list=y[2,:].tolist()
        displ_pid_list=y_pid[2,:].tolist()
        acc_list=y[0,:].tolist()
        acc_pid_list=y_pid[0,:].tolist()
        
        def set(arg,i):
            global yr,displ_list,displ_pid_list,acc_list,acc_pid_list,road,displ,displ_pid,acc,acc_pid,z0,z1,z2,t,x,road_x,road_z,x,umf # Get global variables
            road = yr[i]
            displ = displ_list[i] 
            displ_pid =displ_pid_list[i] 
            acc=acc_list[i]
            acc_pid=acc_pid_list[i]
            z0_i=z0[i]
            z1_i=z1[i]
            z2_i=z2[i]
            t_i=t[i]
            x_i=x[i]
            return (0, road,displ,displ_pid,acc,acc_pid,z0_i,z1_i,z2_i,t_i,x_i,road_x,road_z,umf) # Return position
        Visualizer(callback=set, interval=increment*1000, simulation_time=tstop, initial=(0, road, displ, 0, acc, 0),displ_list=displ_list,yr=yr,acc_list=acc_list)

def _quit():
    root.quit()
    root.destroy() 

ctk.set_appearance_mode("light")
ctk.set_default_color_theme("green")
root = ctk.CTk()
root.title("Modeling Dynamic Behaviour Of A Car Suspension")
root.geometry("850x450")
root.grid_columnconfigure(1, weight=2)
root.grid_columnconfigure((2, 3), weight=0)
root.grid_rowconfigure((0, 1, 2), weight=0)

ks_value=ctk.IntVar()
cs_value=ctk.IntVar()

dropdown_frame=ctk.CTkFrame(root)
dropdown_frame.grid(row=1,column=2,pady=20,sticky="ew")

label_select=ctk.CTkLabel(master=dropdown_frame,text="Select The Type Of Road/Browse A CSV File")
label_select.grid(row=0, column=0,columnspan=3, padx=20, pady=20, sticky="ew")

dropdown_options = ["None(Browse)", "Hill", "Downhill", "Gravel", "Hilly Road", "Speed Bumps"]
dropdown_paths = ["", "src/LinearRise.csv", "src/LinearFall.csv", "src/RandomSamples.csv", "src/SinTenWaves.csv", "src/TriangleFiveWaves.csv"]
dropdown_var = ctk.StringVar()
dropdown_var.set(dropdown_options[0])  # Set the default option
dropdown = ctk.CTkComboBox(master=dropdown_frame, variable=dropdown_var, values=dropdown_options,command=on_dropdown_change)
dropdown.bind("<<CTkComboBoxSelected>>", on_dropdown_change)
dropdown.grid(row=1, column=0,columnspan=6,padx=10, sticky="ew")

browse_button = ctk.CTkButton(root, text="Browse CSV File", command=browse_file)
browse_button.grid(row=2, column=3, padx=20, pady=(20, 20), sticky="nsew")

selected_file_frame=ctk.CTkFrame(root,border_width=200,border_color="black",width=600,height=30)
selected_file_frame.grid(row=2,column=2,pady=20,sticky="w")
selected_file_frame.pack_propagate(False)
selected_file_label = ctk.CTkLabel(master=selected_file_frame, text="", justify="left",bg_color="transparent",fg_color="transparent",)
selected_file_label.grid(row=2, column=2, padx=20, sticky="w")
selected_file_label.pack(fill="both", expand=True)

radio_frame=ctk.CTkFrame(root)
radio_frame.grid(row=0,column=2,sticky='ew')

label = ctk.CTkLabel(master=radio_frame, text="Select Suspension Mode:\n\n(Stiffness of the spring=ks\nDamping constant=cs)")
label.grid(row=0, column=1, padx=10, pady=10, sticky="")

modes = ["Comfort\nks=53000\ncs=7340", "Normal\nks=50750\ncs=1750", "Sport\nks=47466\ncs=815", "Track\nks=43569\ncs=456"]
mode_var = ctk.StringVar(value="Normal\nks=50750\ncs=1750")  # Set "Normal" as the default mode

for i, mode in enumerate(modes):
     ctk.CTkRadioButton(master=radio_frame, text=mode, variable=mode_var, value=mode).grid(row=1, column=1+i, padx=10, pady=20, sticky="n")

apply_button = ctk.CTkButton(root, text="Start Simulation", command=lambda: [update_parameters(), start_simulation()])
apply_button.grid(row=3, column=1, pady=(20, 0), columnspan=3, sticky="ew")

root.protocol("WM_DELETE_WINDOW", _quit)
root.mainloop()


