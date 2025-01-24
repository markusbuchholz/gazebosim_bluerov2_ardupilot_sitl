import tkinter as tk
from tkinter import messagebox
import numpy as np
import random
import math
import subprocess
import heapq

class CoordinateApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CoralGuide Path Planner")
        self.grid_size = 50
        self.step_size = 10
        self.node_radius = 5
        self.obstacle_radius = 1.0  # Diameter of 3 units
        self.canvas_size = 500
        self.center_x = self.canvas_size // 2
        self.center_y = self.canvas_size // 2
        self.start = None
        self.goal = None
        self.waypoints = []
        self.obstacles = []

        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=5, sticky='nsew')

        self.start_button = tk.Button(root, text="Set Start Point", command=self.set_start)
        self.start_button.grid(row=1, column=0, sticky='nsew')

        self.goal_button = tk.Button(root, text="Set Goal Point", command=self.set_goal)
        self.goal_button.grid(row=1, column=1, sticky='nsew')

        self.waypoint_button = tk.Button(root, text="Add Waypoint", command=self.add_waypoint)
        self.waypoint_button.grid(row=1, column=2, sticky='nsew')

        self.obstacle_button = tk.Button(root, text="Add Obstacle", command=self.add_obstacle)
        self.obstacle_button.grid(row=1, column=3, sticky='nsew')

        self.plan_button = tk.Button(root, text="RRT* Path", command=self.plan_path)
        self.plan_button.grid(row=2, column=0, sticky='nsew')

        self.run_ros_button = tk.Button(root, text="Run ROS", command=self.run_ros_program)
        self.run_ros_button.grid(row=2, column=3, sticky='nsew')

        self.simple_path_button = tk.Button(root, text="Simple Path", command=self.save_simple_path)
        self.simple_path_button.grid(row=2, column=2, sticky='nsew')

        self.a_star_button = tk.Button(root, text="A* Path", command=self.plan_a_star_path)
        self.a_star_button.grid(row=2, column=1, sticky='nsew')

        self.clear_button = tk.Button(root, text="Clear", command=self.clear_canvas)
        self.clear_button.grid(row=2, column=4, sticky='nsew')

        # Add padding to the canvas
        self.canvas.grid_configure(padx=5, pady=5)

        self.canvas.bind("<Button-1>", self.get_coordinates)
        self.mode = None

    def set_start(self):
        self.mode = "start"

    def set_goal(self):
        self.mode = "goal"

    def add_waypoint(self):
        self.mode = "waypoint"

    def add_obstacle(self):
        self.mode = "obstacle"

    def canvas_to_grid(self, x, y):
        grid_x = (x - self.center_x) / self.grid_size
        grid_y = (self.center_y - y) / self.grid_size
        return grid_x, grid_y

    def grid_to_canvas(self, grid_x, grid_y):
        x = grid_x * self.grid_size + self.center_x
        y = self.center_y - grid_y * self.grid_size
        return x, y

    def get_coordinates(self, event):
        x, y = event.x, event.y
        grid_x, grid_y = self.canvas_to_grid(x, y)
        coord = (x, y)

        if self.mode == "start":
            self.start = (grid_x, grid_y)
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="green")
        elif self.mode == "goal":
            self.goal = (grid_x, grid_y)
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="red")
        elif self.mode == "waypoint":
            self.waypoints.append((grid_x, grid_y))
            self.canvas.create_oval(coord[0]-self.node_radius, coord[1]-self.node_radius,
                                    coord[0]+self.node_radius, coord[1]+self.node_radius, fill="blue")
        elif self.mode == "obstacle":
            self.obstacles.append((grid_x, grid_y))
            self.canvas.create_oval(coord[0]-self.obstacle_radius*self.grid_size, coord[1]-self.obstacle_radius*self.grid_size,
                                    coord[0]+self.obstacle_radius*self.grid_size, coord[1]+self.obstacle_radius*self.grid_size, fill="black")

    def plan_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        all_points = [self.start] + self.waypoints + [self.goal]
        complete_path = []

        for i in range(len(all_points) - 1):
            segment_path = self.rrt_star(all_points[i], all_points[i + 1])
            if segment_path is None:
                messagebox.showerror("Error", f"No path found between points {i} and {i+1}.")
                return
            if i != 0:
                segment_path = segment_path[1:]
            complete_path.extend(segment_path)

        self.draw_path(complete_path)
        self.save_path(complete_path, "mission_path.txt")
        messagebox.showinfo("Success!", "Path found and saved to mission_path.txt")

    def plan_a_star_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        all_points = [self.start] + self.waypoints + [self.goal]
        complete_path = []

        for i in range(len(all_points) - 1):
            segment_path = self.a_star(all_points[i], all_points[i + 1])
            if segment_path is None:
                messagebox.showerror("Error", f"No path found between points {i} and {i+1}.")
                return
            if i != 0:
                segment_path = segment_path[1:]
            complete_path.extend(segment_path)

        self.draw_path(complete_path)
        self.save_path(complete_path, "mission_path.txt")
        messagebox.showinfo("Success!", "A* path found and saved to mission_path.txt")

    def draw_path(self, path):
        for i in range(len(path) - 1):
            x1, y1 = self.grid_to_canvas(*path[i])
            x2, y2 = self.grid_to_canvas(*path[i+1])
            self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=2)

    def save_path(self, complete_path, filename):
        with open(filename, "w") as file:
            for point in complete_path:
                x, y = point
                file.write(f"{x}, {y}\n")

    def save_simple_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Please set both start and goal points.")
            return

        with open("mission_path.txt", "w") as file:
            file.write(f"{self.start[0]}, {self.start[1]}\n")
            for waypoint in self.waypoints:
                file.write(f"{waypoint[0]}, {waypoint[1]}\n")
            file.write(f"{self.goal[0]}, {self.goal[1]}\n")
        messagebox.showinfo("Success!", "Simple path saved to mission_path.txt")

    def rrt_star(self, start, goal):
        nodes = [start]
        parent = {start: None}
        for i in range(10000):
            random_point = (random.uniform(-5, 5), random.uniform(-5, 5))
            nearest_node = self.get_nearest_node(nodes, random_point)
            new_node = self.steer(nearest_node, random_point)
            if not self.is_colliding(nearest_node, new_node):
                nodes.append(new_node)
                parent[new_node] = nearest_node
                if self.distance(new_node, goal) < self.step_size / self.grid_size:
                    parent[goal] = new_node
                    return self.extract_path(parent, goal)
        return None

    def get_nearest_node(self, nodes, point):
        return min(nodes, key=lambda node: self.distance(node, point))

    def steer(self, from_node, to_point):
        angle = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
        new_point = (from_node[0] + (self.step_size / self.grid_size) * math.cos(angle), from_node[1] + (self.step_size / self.grid_size) * math.sin(angle))
        return new_point

    def is_colliding(self, from_node, to_node):
        for obstacle in self.obstacles:
            if self.distance_to_circle(obstacle, from_node, to_node) < self.obstacle_radius:
                return True
        return False

    def distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    def distance_to_circle(self, circle_center, line_start, line_end):
        px, py = circle_center
        ax, ay = line_start
        bx, by = line_end
        line_mag = self.distance(line_start, line_end)
        if line_mag == 0:
            return self.distance(circle_center, line_start)
        u = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / float(line_mag ** 2)
        closest_point = (ax + u * (bx - ax), ay + u * (by - ay))
        return self.distance(circle_center, closest_point)

    def extract_path(self, parent, goal):
        path = [goal]
        current_node = goal
        while parent[current_node] is not None:
            current_node = parent[current_node]
            path.append(current_node)
        path.reverse()
        return path

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.distance(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if self.distance(current, goal) < self.step_size / self.grid_size:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def get_neighbors(self, node):
        neighbors = []
        for dx in [-self.step_size / self.grid_size, 0, self.step_size / self.grid_size]:
            for dy in [-self.step_size / self.grid_size, 0, self.step_size / self.grid_size]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (node[0] + dx, node[1] + dy)
                if not self.is_colliding(node, neighbor):
                    neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def run_ros_program(self):
        try:
            subprocess.run(["ros2", "run", "move_blueboat", "run_mission"], check=True)
            messagebox.showinfo("Success", "ROS 2 program executed successfully.")
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Error", f"ROS 2 program failed: {e}")

    def clear_canvas(self):
        self.canvas.delete("all")
        self.start = None
        self.goal = None
        self.waypoints = []
        self.obstacles = []

if __name__ == "__main__":
    root = tk.Tk()
    app = CoordinateApp(root)
    root.mainloop()
