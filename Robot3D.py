import tinyik
import numpy as np
import tkinter as tk

# Function to update the variables based on the values entered in the entries
def update_variables():
    global var1, var2, var3, var4, var5, var6
    
    try:
        var1 = int(entry_var1.get())
        var2 = int(entry_var2.get())
        var3 = int(entry_var3.get())
        var4 = int(entry_var4.get())
        var5 = int(entry_var5.get())
        var6 = int(entry_var6.get())
        
        # Update the labels to show the new values
        label_var1.config(text=f"Var 1: {var1}")
        label_var2.config(text=f"Var 2: {var2}")
        label_var3.config(text=f"Var 3: {var3}")
        label_var4.config(text=f"Var 4: {var4}")
        label_var5.config(text=f"Var 5: {var5}")
        label_var6.config(text=f"Var 6: {var6}")
        
    except ValueError:
        # Handle invalid input (non-integer values)
        error_label.config(text="Please enter valid integer values!")
    arm.angles = np.deg2rad([var1, var2, var3 ,var4,var5,var6])

# Create the main window
root = tk.Tk()
root.title("Multiple Variables Modifier")

# Initial values for the variables
var1 = var2 = var3 = var4 = var5 = var6 = 0

# Create labels for the variables
label_var1 = tk.Label(root, text=f"Var 1: {var1}", font=('Helvetica', 15))
label_var1.pack(padx=20, pady=5)

label_var2 = tk.Label(root, text=f"Var 2: {var2}", font=('Helvetica', 15))
label_var2.pack(padx=20, pady=5)

label_var3 = tk.Label(root, text=f"Var 3: {var3}", font=('Helvetica', 15))
label_var3.pack(padx=20, pady=5)

label_var4 = tk.Label(root, text=f"Var 4: {var4}", font=('Helvetica', 15))
label_var4.pack(padx=20, pady=5)

label_var5 = tk.Label(root, text=f"Var 5: {var5}", font=('Helvetica', 15))
label_var5.pack(padx=20, pady=5)

label_var6 = tk.Label(root, text=f"Var 6: {var6}", font=('Helvetica', 15))
label_var6.pack(padx=20, pady=5)

# Create entry widgets for each variable
entry_var1 = tk.Entry(root, font=('Helvetica', 15), width=10)
entry_var1.pack(padx=20, pady=5)

entry_var2 = tk.Entry(root, font=('Helvetica', 15), width=10)
entry_var2.pack(padx=20, pady=5)

entry_var3 = tk.Entry(root, font=('Helvetica', 15), width=10)
entry_var3.pack(padx=20, pady=5)

entry_var4 = tk.Entry(root, font=('Helvetica', 15), width=10)
entry_var4.pack(padx=20, pady=5)

entry_var5 = tk.Entry(root, font=('Helvetica', 15), width=10)
entry_var5.pack(padx=20, pady=5)

entry_var6 = tk.Entry(root, font=('Helvetica', 15), width=10)
entry_var6.pack(padx=20, pady=5)

# Button to apply the entered values to the variables
apply_button = tk.Button(root, text="Apply Values", font=('Helvetica', 15), command=update_variables)
apply_button.pack(padx=20, pady=20)

# Label for error messages
error_label = tk.Label(root, text="", font=('Helvetica', 12), fg="red")
error_label.pack()

arm = tinyik.Actuator(['z', [0., 0., 0.5], 'y', [0., 0., 1.], 'y', [0., 0., 0.5], 'z', [0., 0., 0.25], 'y', [0., 0., 0.20], 'z', [0., 0., 0.1]])



visualize_button = tk.Button(root, text="Visualize Arm", font=('Helvetica', 15), command=lambda:tinyik.visualize(arm))
visualize_button.pack(padx=20, pady=20)

# Start the Tkinter event loop
root.mainloop()

