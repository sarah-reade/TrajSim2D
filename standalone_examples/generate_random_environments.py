import matplotlib.pyplot as plt
import os
from pathlib import Path
from time import sleep
from trajsim2d_core.environment import generate_random_border, generate_random_objects
from trajsim2d_core.visualisation import initialise_visualisation
from trajsim2d_core.twodmanip import PlanarManipulator

# Directory to save snapshots
SAVE_DIR = Path("~/Downloads").expanduser()
SAVE_DIR.mkdir(exist_ok=True)

NUM_ITERATIONS = 5  # How many random visualisations to generate

for i in range(NUM_ITERATIONS):
    print(f"Iteration {i+1}/{NUM_ITERATIONS}")

    # --- Generate random test data ---
    border = generate_random_border(border_size=10, smoothness=0.1)
    objs = generate_random_objects(object_size=0.5, num_objs=5, smoothness=0.1)
    arm = PlanarManipulator()

    # --- Initialise the visualisation ---
    canvas, base_tf, border_id, object_ids, arm_ids = initialise_visualisation(border=border, objs=objs, arm=arm)

    # --- Save a snapshot of the current figure ---
    img_path = SAVE_DIR / f"visualisation_{i+1:03d}.png"
    canvas.fig.savefig(img_path)
    print(f"Saved snapshot: {img_path}")

    # --- Wait until the user closes the figure ---
    print("Close the visualisation window to continue...")
    plt.show(block=True)  # Blocks until the window is closed

    # Optional: small pause between iterations
    sleep(0.5)

print("All iterations completed!")