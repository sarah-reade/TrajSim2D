##
# @file geometry_canvas.py
# @brief GeometryCanvas class for managing and visualizing 2D geometric primitives using Matplotlib.
# @details
# This module defines the GeometryCanvas class, which wraps a Matplotlib figure and provides
# methods to create, manipulate, and render geometric objects such as polygons, rectangles,
# circles, and NumPy array-based images or coordinate data.
#
# Features:
# - Add, move, recolor, and remove geometric shapes.
# - Add and visualize NumPy arrays as images or point sets.
# - Simple architecture suitable for extension or integration with GUIs.
#
# Example usage:
# @code{.py}
# from geometry_canvas import GeometryCanvas
# import numpy as np
#
# canvas = GeometryCanvas()
# rect_id = canvas.add_rectangle((1, 1), 2, 3)
# poly_id = canvas.add_polygon(np.random.rand(5, 2) * 10)
# canvas.move_shape(rect_id, dx=1.0, dy=0.5)
# canvas.update_color(poly_id, "orange")
# @endcode
#
# @authors
# Sarah Reade (28378329@students.lincoln.ac.uk)  
# ChatGPT (OpenAI Assistant)
#
# @date
# 2025-10-30
#
# @version
# 1.0
#
# @license
# MIT License
# @par License Text
# Copyright (c) 2025 Sarah Reade and ChatGPT (OpenAI Assistant)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
##

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, Circle,FancyArrowPatch
import numpy as np
import uuid

class GeometryCanvas:
    """
    @class GeometryCanvas
    @brief A simple geometry and visualization canvas built on top of Matplotlib.

    The GeometryCanvas class manages a matplotlib figure and allows users to
    create, modify, move, and delete basic geometric primitives such as polygons,
    rectangles, and circles. It also supports adding NumPy arrays either as images
    or as collections of 2D points.

    Example:
    @code{.py}
    canvas = GeometryCanvas()
    rect_id = canvas.add_rectangle((1, 1), 2, 3)
    canvas.move_shape(rect_id, dx=1, dy=0.5)
    @endcode
    """

    def __init__(self):
        """
        @brief Constructor for GeometryCanvas.
        
        Initializes a matplotlib figure and axis for drawing. 
        Also enables interactive mode for live updates.
        """
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.shapes = {}  # Dictionary mapping UUID -> patch or image
        self.ax.set_aspect('equal', adjustable='datalim')
        self.ax.autoscale(enable=True, tight=True)
        self.ax.autoscale_view()
        plt.ion()
        plt.show()

    def add_shape(self,obj,color='blue',alpha=0.5):
        
        if isinstance(obj,list):
            id = []
            for obj_ind in obj:
                id.append(self.add_shape(obj_ind,color,alpha))

        elif isinstance(obj,np.ndarray):
            id = self.add_array(arr=obj,color=color,alpha=alpha)

        elif isinstance(obj,Rectangle):
            id = self.add_rectangle(obj=obj,color=color,alpha=alpha)

        elif isinstance(obj,Circle):
            id = self.add_circle(obj=obj,color=color,alpha=alpha)
        
        else:
            return None

        return id

    def add_polygon(self, points, color='blue', alpha=0.5):
        """
        @brief Add a polygon to the canvas.

        @param points Array-like of shape (N, 2) containing vertex coordinates.
        @param color Color of the polygon.
        @param alpha Transparency value between 0 and 1.
        @return Unique shape identifier (UUID string).
        """
        poly = Polygon(points, closed=True, color=color, alpha=alpha)
        shape_id = str(uuid.uuid4())
        self.ax.add_patch(poly)
        self.shapes[shape_id] = poly
        self._refresh()
        return shape_id

    def add_rectangle(self, obj=None,xy=[0.0,0.0], width=0.0, height=0.0, color='red', alpha=0.5):
        """
        @brief Add a rectangle to the canvas.

        @param obj representing a matplotlib rectangle
        @param xy Tuple (x, y) representing the lower-left corner of the rectangle OR 3x3 transform matrix.
        @param width Rectangle width.
        @param height Rectangle height.
        @param color Fill color of the rectangle.
        @param alpha Transparency value between 0 and 1.
        @return Unique shape identifier (UUID string).
        """
        if isinstance(obj,Rectangle):
            obj.set_color(color)
            obj.set_alpha(alpha)
            shape_id = str(uuid.uuid4())
            self.ax.add_patch(obj)
            self.shapes[shape_id] = obj
            self._refresh()
            return shape_id
        
        elif isinstance(xy, np.ndarray) and xy.shape == (3, 3):
            # rectangle corners relative to local origin
            corners = np.array([
                [-width / 2,  height / 2, 1],
                [ width / 2,  height / 2, 1],
                [ width / 2, -height / 2, 1],
                [-width / 2, -height / 2, 1]
            ])

            # Apply full 3x3 transform
            transformed = (xy @ corners.T).T[:, :2]
            
            return self.add_array(transformed, color=color, alpha=alpha)
        else:
            rect = Rectangle(xy, width, height, color=color, alpha=alpha)
            shape_id = str(uuid.uuid4())
            self.ax.add_patch(rect)
            self.shapes[shape_id] = rect
            self._refresh()
            return shape_id


    def add_circle(self, obj=None,center=[0.0,0.0], radius=0.0, color='green', alpha=0.5):
        """
        @brief Add a circle to the canvas.

        @param center Tuple (x, y) representing the circle center, OR 3x3 transform matrix.
        @param radius Radius of the circle.
        @param color Fill color of the circle.
        @param alpha Transparency value between 0 and 1.
        @return Unique shape identifier (UUID string).
        """
        if isinstance(obj,Circle):
            obj.set_edgecolor(color)
            obj.set_facecolor(color)
            obj.set_alpha(alpha)
            shape_id = str(uuid.uuid4())
            self.ax.add_patch(obj)
            self.shapes[shape_id] = obj
            self._refresh()

        elif isinstance(center, np.ndarray) and center.shape == (3, 3):
            # Only translation matters for a circle
            center = tuple(center[:2, 2])

            circ = Circle(center, radius, color=color, alpha=alpha)
            shape_id = str(uuid.uuid4())
            self.ax.add_patch(circ)
            self.shapes[shape_id] = circ
            self._refresh()
            return shape_id

    def add_array(self, arr, color='cyan', alpha=0.5, closed=True):
        """
        @brief Add a NumPy array as a geometric shape (polygon or polyline).
        
        Treats arr as a collection of points. Each row should be [x, y].

        @param arr NumPy array of shape (N, 2)
        @param color Fill color of the polygon
        @param alpha Transparency value
        @param closed If True, the polygon is closed; otherwise it is a polyline
        @return Unique shape identifier (UUID string)
        @throws ValueError If array shape is invalid
        """
        if arr.ndim != 2 or arr.shape[1] != 2:
            raise ValueError("Array must have shape (N,2) for geometric points")

        poly = Polygon(arr, closed=closed, color=color, alpha=alpha)
        shape_id = str(uuid.uuid4())
        self.ax.add_patch(poly)
        self.shapes[shape_id] = poly
        self._refresh()
        return shape_id

    def move_shape(self, shape_id, dx, dy):
        """
        @brief Translate a shape by the given offsets.

        @param shape_id Unique identifier of the shape to move.
        @param dx Offset in the X direction.
        @param dy Offset in the Y direction.
        @throws ValueError If the shape is not found.
        """
        shape = self.shapes.get(shape_id)
        if shape is None:
            raise ValueError("Shape not found")
        if isinstance(shape, Polygon):
            shape.set_xy(shape.get_xy() + np.array([dx, dy]))
        elif isinstance(shape, Rectangle):
            shape.set_xy((shape.get_x() + dx, shape.get_y() + dy))
        elif isinstance(shape, Circle):
            shape.center = (shape.center[0] + dx, shape.center[1] + dy)
        self._refresh()

    def update_color(self, shape_id, color):
        """
        @brief Update the color of a shape.

        @param shape_id Unique identifier of the shape.
        @param color New color to apply.
        """
        shape = self.shapes.get(shape_id)
        if hasattr(shape, "set_color"):
            shape.set_color(color)
            self._refresh()

    def remove_shape(self, shape_id):
        """
        @brief Remove a shape from the canvas.

        @param shape_id Unique identifier of the shape to remove.
        """
        shape = self.shapes.pop(shape_id, None)
        if shape is not None and hasattr(shape, "remove"):
            shape.remove()
        self._refresh()


    def _refresh(self):
        """
        @brief Redraw the canvas and auto-adjust the view to include all shapes.
        @details
        Computes the bounding box of all shapes and adjusts axis limits once,
        avoiding cumulative zoom-out effects. Works for rectangles, polygons,
        circles, and image arrays.
        """
        all_x, all_y = [], []

        for shape in self.shapes.values():

            # --- Rectangle (special case) ---
            if isinstance(shape, Rectangle):
                x, y = shape.get_xy()
                w, h = shape.get_width(), shape.get_height()
                corners = np.array([
                    [x, y],
                    [x + w, y],
                    [x + w, y + h],
                    [x, y + h]
                ])
                all_x.extend(corners[:, 0])
                all_y.extend(corners[:, 1])
                continue

            # --- Polygon ---
            if hasattr(shape, 'get_xy'):
                xy = np.array(shape.get_xy())
                if xy.ndim == 2 and xy.shape[1] >= 2:
                    all_x.extend(xy[:, 0])
                    all_y.extend(xy[:, 1])
                    continue

            # --- Circle ---
            if hasattr(shape, 'get_center') and hasattr(shape, 'get_radius'):
                cx, cy = shape.get_center()
                r = shape.get_radius()
                all_x.extend([cx - r, cx + r])
                all_y.extend([cy - r, cy + r])
                continue

            # --- Image (AxesImage) ---
            if hasattr(shape, 'get_extent'):
                xmin, xmax, ymin, ymax = shape.get_extent()
                all_x.extend([xmin, xmax])
                all_y.extend([ymin, ymax])
                continue

        if all_x and all_y:
            xmin, xmax = min(all_x), max(all_x)
            ymin, ymax = min(all_y), max(all_y)
            dx, dy = xmax - xmin, ymax - ymin

            # Add a small margin
            margin_x = dx * 0.05 if dx else 0.1
            margin_y = dy * 0.05 if dy else 0.1

            self.ax.set_xlim(xmin - margin_x, xmax + margin_x)
            self.ax.set_ylim(ymin - margin_y, ymax + margin_y)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def add_tf(self, tf: np.ndarray):
        """
        @brief Add a 2D transform (3x3 homogeneous matrix) as a coordinate frame to the canvas.
        
        @param tf 3x3 np.ndarray representing a 2D homogeneous transform.
        @return Unique shape identifier (UUID string).
        
        @details
        Draws a small coordinate frame representing the transform:
        - Red arrow: local +X direction
        - Blue arrow: local +Y direction
        - Black dot: transform origin
        
        The transform is stored internally with its UUID for later updates or removal.
        """
        if tf.shape != (3, 3):
            raise ValueError("Transform must be a 3x3 homogeneous matrix")

        # Extract position and orientation
        origin = tf[:2, 2]
        x_axis = tf[:2, 0]
        y_axis = tf[:2, 1]
        scale = 0.1  # visual length

        # Draw coordinate axes using FancyArrowPatch
        x_arrow = FancyArrowPatch(
            posA=origin,
            posB=origin + x_axis * scale,
            color='r',
            arrowstyle='->',
            mutation_scale=10
        )
        y_arrow = FancyArrowPatch(
            posA=origin,
            posB=origin + y_axis * scale,
            color='b',
            arrowstyle='->',
            mutation_scale=10
        )

        # Add to axes
        self.ax.add_patch(x_arrow)
        self.ax.add_patch(y_arrow)
        origin_dot, = self.ax.plot(origin[0], origin[1], 'ko', markersize=3)

        # Register object
        tf_id = str(uuid.uuid4())
        self.shapes[tf_id] = {
            'origin': origin_dot,
            'x_arrow': x_arrow,
            'y_arrow': y_arrow,
            'tf': tf
        }

        # Refresh canvas
        self._refresh()
        return tf_id