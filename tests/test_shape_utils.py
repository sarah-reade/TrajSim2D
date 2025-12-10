import unittest
import numpy as np
import matplotlib.pyplot as plt
from trajsim2d_core.shape_utils  import decompose_to_convex_shapes

def generate_person_outline(
    head_radius=0.2,
    torso_width=0.35,
    torso_height=0.6,
    arm_length=0.35,
    leg_length=0.45,
    resolution=60
):
    """
    Generates a simplified silhouette outline of a generic person.
    Returns a list of (x, y) coordinates in order.
    """

    coords = []

    # --- Head (circle) ---
    theta = np.linspace(0, 2*np.pi, resolution)
    head = [(head_radius * np.cos(t), head_radius * np.sin(t) + torso_height/2 + head_radius)
            for t in theta]

    # --- Torso (rectangle) ---
    half_w = torso_width / 2
    half_h = torso_height / 2
    torso = [
        (-half_w, -half_h),  # left-bottom
        (-half_w,  half_h),  # left-top
        ( half_w,  half_h),  # right-top
        ( half_w, -half_h)   # right-bottom
    ]

    # --- Arms (two simple segments extending from torso midpoint) ---
    arm_y = half_h * 0.7
    left_arm = [(-half_w, arm_y), (-half_w - arm_length, arm_y)]
    right_arm = [(half_w, arm_y), (half_w + arm_length, arm_y)]

    # --- Legs (two straight segments angled outward) ---
    leg_y = -half_h
    left_leg = [( -torso_width * 0.2, leg_y), ( -torso_width * 0.2 - 0.2, leg_y - leg_length )]
    right_leg = [(  torso_width * 0.2, leg_y), (  torso_width * 0.2 + 0.2, leg_y - leg_length )]

    # Combine shapes into outline chain (clockwise)
    # Start at bottom of right leg, move around full silhouette
    outline = []
    outline.extend([right_leg[1], right_leg[0]])
    outline.extend(reversed(torso))  # go around torso
    outline.extend(right_arm[::-1])  # arm to torso
    outline.extend(left_arm)         # across chest to left arm
    outline.extend(torso[:1])        # reconnect to torso corner
    outline.extend(head)             # head circle
    outline.extend(left_leg)         # down to legs


    outline = [
        # Head (top at y ~1.9)
        (0.00, 1.90),
        (0.30, 1.95),
        (0.45, 1.80),
        (0.45, 1.55),
        (0.30, 1.40),
        (0.00, 1.35),
        (-0.30, 1.40),
        (-0.45, 1.55),
        (-0.45, 1.80),
        (-0.30, 1.95),
        (0.00, 1.90),

        # Shoulders and arms
        (-0.60, 1.20),
        (-1.00, 0.80),
        (-1.05, 0.50),
        (-0.95, 0.20),

        (-0.55, 0.10),  # torso left
        (-0.50, -0.60),
        (-0.45, -1.40),
        (-0.35, -1.70),

        # Legs
        (0.00, -1.80),
        (0.35, -1.70),
        (0.45, -1.40),
        (0.50, -0.60),
        (0.55, 0.10),

        # Right arm
        (0.95, 0.20),
        (1.05, 0.50),
        (1.00, 0.80),
        (0.60, 1.20)
        ]
        
    return outline

class TestConvexDecomposition(unittest.TestCase):

    def test_decomposition_and_plot(self):
        outline = generate_person_outline()
        outline_np = np.array(outline)

        # Call the decomposition function
        convex_parts = decompose_to_convex_shapes(outline_np)

        # Basic validation
        self.assertTrue(len(convex_parts) > 0)
        for part in convex_parts:
            self.assertIsInstance(part, np.ndarray)
            self.assertGreater(part.shape[0], 0)

        # ------------------------------------------------------------
        # Visualization with filled convex parts
        # ------------------------------------------------------------
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors
        import random

        fig, ax = plt.subplots(figsize=(6, 10))

        # Plot original outline (wireframe for reference)
        ax.plot(outline_np[:, 0], outline_np[:, 1], color="black", linewidth=1.5)

        # Distinct colors for each convex region
        color_list = list(mcolors.TABLEAU_COLORS.values())

        for i, part in enumerate(convex_parts):
            xs, ys = part[:, 0], part[:, 1]

            # Choose color deterministically to avoid randomness during testing
            color = color_list[i % len(color_list)]

            ax.fill(xs, ys, alpha=0.5, color=color, edgecolor="black")

        ax.set_aspect("equal", "box")
        ax.set_title("Filled Convex Decomposition of Person Outline")

        plt.show()

if __name__ == "__main__":
    unittest.main()


