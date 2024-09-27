import math
import tkinter as tk

# Class for nodes. Each grid will represent 1 node
class Node:
    def __init__(self, coordinates=None, node_type=None, neighbour=None, parent=None, total_cost=float('inf'), heuristic = 0, teleported_parent= None):
        self.coordinates = coordinates
        self.node_type = node_type
        self.parent = parent
        self.total_cost = total_cost
        self.heuristic = heuristic

# Create node object for each grid
def createMap(height, width):
    map_grid = []

    for x in range(width):
        for y in range(height):
            grid = Node((x, y), "normal")
            map_grid.append(grid)

    assignNodeType(map_grid)
    return map_grid


#Assign Node type to grid
def assignNodeType(map):
    trap1 = [(8, 2)]
    trap2 = [(1, 1), (2, 4)]
    trap3 = [(6, 1), (5, 3)]
    trap4 = [(3, 1)]
    reward1 = [(4, 0), (1, 3)]
    reward2 = [(5, 5), (7, 2)]
    treasure = [(4, 1), (3, 4), (7, 3), (9, 3)]
    obstacle = [(0, 3), (2, 2), (3, 3), (4, 2), (8, 1), (4, 4), (6, 4), (7, 4), (6, 3)]
    #Assign them their respective type if they are in any of the list
    for x in map:
        if x.coordinates in trap1:
            x.node_type = "trap1"
        elif x.coordinates in trap2:
            x.node_type = "trap2"
        elif x.coordinates in trap3:
            x.node_type = "trap3"
        elif x.coordinates in trap4:
            x.node_type = "trap4"
        elif x.coordinates in reward1:
            x.node_type = "reward1"
        elif x.coordinates in reward2:
            x.node_type = "reward2"
        elif x.coordinates in treasure:
            x.node_type = "treasure"
        elif x.coordinates in obstacle:
            x.node_type = "obstacle"
        else:
            x.node_type = "normal"

# Returns the heuristic of the node. The heuristic of the node will be the
# treasure that is closest to the current node. The distance formula is referenced
# from: https://www.redblobgames.com/grids/hexagons/. The coordinate system used is
# offset even-q coordinate system. It is first converted into axial in order to use
# a formula to get the distance
def heuristic(current_node, treasure_available):
    if len(treasure_available) > 0:
        current_coordinates = current_node.coordinates
        treasure_distances = []
        # convert the current node coordinates to axial coordinate
        x, y = current_coordinates
        y = (x + (x & 1)) / 2

        # Iterate through each treasure to get their distance from current node. Distance used is manhattan distance
        for treasure in treasure_available:
            # convert treasure node coordinates to axial coordinate
            a , b = treasure.coordinates
            b = (a + (a & 1)) / 2

            distance = (abs(x - a) + abs(y + b) + abs(x + y - a - b)) /2
            treasure_distances.append((treasure, distance))
        # Sort treasure and return the closest treausre
        treasure_distances.sort(key=lambda x: x[1])
        return treasure_distances[0]
    else:
        return None

# Heuristic function to calculate get the estimated cost of reaching the nearest treausre
def heuristicFunction(node, treasure_available):
    # the node's total cost so far
    g = node.total_cost
    # heuristic calculated
    h = heuristic(node, treasure_available)[1]
    if h is None:
        return 0
    return g + h

# Return the list of neighbours of a node that are not obstacles
def assignNeighbour(map_grid, node):
    final_neighbours = []
    (x, y) = node.coordinates

    # Direction of each neighbour node. Has different calculation depending on the depending on whether the column
    # is even or odd. Idea for this was referenced from: https://www.redblobgames.com/grids/hexagons/
    # Coordinate system used is offset even-q
    neighbours_odd = [(x + 1, y), (x + 1, y - 1), (x, y - 1), (x - 1, y - 1), (x - 1, y), (x, y + 1)]
    neighbours_even = [(x + 1, y + 1), (x + 1, y), (x, y - 1), (x - 1, y), (x - 1, y + 1), (x, y + 1)]

    neighbours = neighbours_even if x % 2 == 0 else neighbours_odd

    # for loop to get neighbours that are within the map and are not obstacles
    for (a, b) in neighbours:
        for z in map_grid:
            if (a, b) == z.coordinates:
                if z.node_type != "obstacle":
                    final_neighbours.append(z)
    return final_neighbours

# Calculate total cost of a node by checking its path for rewards and trap in order to get a proper
# cost and energy multiplier to get the true total cost. Once trap/reward has been activated, their respective
# multiplier will double for every node the algorithm has entered.
def calculateCost(map,path,current_node):
    temp_total_cost = 0
    multiplier_step = 1
    multiplier_energy = 1
    trap1_counter = 0
    trap2_counter = 0
    reward1_counter = 0
    reward2_counter = 0
    path.insert(-1,current_node)
    # for loop to check whether the path contains any reward or trap
    for i in path:
        cost = multiplier_step * multiplier_energy
        if i.node_type == "trap1":
            i.node_type = "normal"
            trap1_counter += 1
        if i.node_type == "trap2":
            i.node_type = "normal"
            trap2_counter += 1
        if i.node_type == "reward1":
            i.node_type = "normal"
            reward1_counter += 1
        if i.node_type == "reward2":
            i.node_type = "normal"
            reward2_counter += 1
        if trap1_counter > 0:
            multiplier_energy *= 2
            if trap1_counter == 2:
                multiplier_energy *= 2
        if trap2_counter > 0:
            multiplier_step *= 2
            if trap2_counter == 2:
                multiplier_step *= 2
        if reward1_counter > 0:
            multiplier_energy /= 2
            if reward1_counter == 2:
                multiplier_energy /= 2
        if reward2_counter > 0:
            multiplier_step /= 2
            if reward2_counter == 2:
                multiplier_step /= 2
        temp_total_cost += cost
    # Ensure that the node type are back to their respective node type after the calculation
    assignNodeType(map)
    return temp_total_cost

# A* Algorithm function
def a_star(stateSpace, initialCoordinate, goalState):
    # function to reset the every node so that the algorithm can continue functioning after reaching one of the treasure
    def reset_nodes():
        for node in stateSpace:
            node.parent = None
            node.total_cost = float('inf')

    frontier = []
    explored = []
    treasure_available = goalState
    goals = []
    solution = []
    previous_path =[]
    final_solution = []
    cost_total = 0
    parent = {}

    # Get the node object of the initial coordinate and append it to frontier
    for x in map_grid:
        if x.coordinates == initialCoordinate:
            frontier.append(x)
            x.total_cost = 0
            break
    # Get node object of treasure and append to goals list
    for y in treasure_available:
        for x in map_grid:
            if x.coordinates == y:
                goals.append(x)
                break
    counter_lol = 0
    while frontier:
        # sort the frontier based on heuristic
        frontier.sort(key=lambda node: node.heuristic)
        # pop frontier and assign the first node of frontier as current_node
        current_node = frontier.pop(0)
        trap3VisitedBefore = {(5,3): 0,
                           (6,1): 0}

        # Add 1 to the coordinate's value in trap3VisitedBefore dictionary if it is in the path
        for i in path(final_solution,current_node):
            for key,value in trap3VisitedBefore.items():
                if i.coordinates == key:
                    trap3VisitedBefore[key] += 1
        # If statement for the current node is trap 3 and it has only appeared once in the path
        if current_node.node_type == "trap3" and trap3VisitedBefore[current_node.coordinates] == 1:
            if current_node.parent is None:
                break
            x = 0
            y = 0
            t = 0
            # Calculation for moving in the 6 directions based on whether the column is odd and even
            neighbours_even = [(x + 1, y + 1), (x + 1, y), (x, y - 1), (x - 1, y), (x - 1, y + 1), (x, y + 1)]
            neighbours_odd = [(x + 1, y), (x + 1, y - 1), (x, y - 1), (x - 1, y - 1), (x - 1, y), (x, y + 1)]
            # Get the direction which the node has previously moved
            previous_node_direction = (current_node.coordinates[0] - current_node.parent.coordinates[0],current_node.coordinates[1] - current_node.parent.coordinates[1])
            # Check whether the parent is in odd or even column to get the right formula to move in the right direction
            if current_node.parent.coordinates[0] % 2 == 0:
                for a in neighbours_even:
                    if a == previous_node_direction:
                        t = neighbours_even.index(a)
            else:
                for b in neighbours_odd:
                    if b == previous_node_direction:
                        t = neighbours_odd.index(b)
            # Get the coordinates of the nodes once it has move 2 grid away following the previous direction
            current_node_coordinates = (current_node.coordinates[0] + neighbours_even[t][0]+ neighbours_odd[t][0],
                                        current_node.coordinates[1] + neighbours_even[t][1]+ neighbours_odd[t][1])
            # Hardcode code block to ensure that the coordinates does not move into an obstacle or out of map
            if current_node_coordinates == (4,2):
                current_node_coordinates = (5,2)
            if current_node_coordinates == (6,3):
                current_node_coordinates = (6,2)
            if current_node_coordinates == (6,-1):
                current_node_coordinates = (6,0)
            # Find the node in map that correspond to the calculated coordinates and assign that node as the current node
            for c in map_grid:
                if c.coordinates == current_node_coordinates:
                    c.parent = current_node
                    c.total_cost = current_node.total_cost
                    c.heuristic = heuristic(current_node,goals)
                    current_node = c
                    break

        # If current trap is trap 4, all goals will be cleared
        if current_node.node_type == "trap4":
            temp_goals = goals.copy()
            goals.clear()

        # if the current node is part of the goals list, it will reset the algorithm to focus on the next treasure
        if current_node in goals:
            # Append the path to the goal into a list
            previous_path.append(construct_path_nodes(current_node))
            solution.append(construct_path_nodes(current_node))
            final_solution.clear()
            counter_lol += len(explored)
            for b in solution:
                for item in b:
                    final_solution.append(item)
            cost_total = final_solution[-1].total_cost
            # Remove the treasure node from the goal list has it has already been collected
            goals.remove(current_node)
            # Clear frontier and explored list and reset all nodes so it can function normally to the next treasure
            frontier.clear()
            explored.clear()
            reset_nodes()
            frontier.append(current_node)
            current_node.total_cost = 0
        # Once all treasure has been collected (which is 4 in this case), the algorithm will end and final_solution
        # list and cost_total will be returned
        if len(solution) == 4:
            for i in range(len(final_solution)):
                final_solution[i] = final_solution[i].coordinates
            final_solution.insert(0,initialCoordinate)
            print("Number of explored nodes:", counter_lol)
            return final_solution, cost_total

        # Append current node to explored list
        explored.append(current_node)
        # If there are still treasure in goals list, the nodes will continue expanding
        if len(goals) > 0:
            # get the children of the current node
            children = assignNeighbour(stateSpace, current_node)
            for child in children:
                # Calculate the total cost of the child
                tentative_cost = calculateCost(stateSpace, path(final_solution,current_node),  child)
                # If the child is not in explored and frontier, the current node will be assigned as this node's parent.
                # Heuristic and total_cost will be assigned to this node and it will be appended
                if not any(explored_node.coordinates == child.coordinates for explored_node in explored) and not any(
                        frontier_node.coordinates == child.coordinates for frontier_node in frontier):
                    parent[child] = current_node
                    child.parent = current_node
                    child.total_cost = tentative_cost
                    child.heuristic = heuristicFunction(child,goals)
                    frontier.append(child)
                # if any node in frontier is more than the tentative cost, the node will be removed and replaced
                # with the same node but with new parent, total_cost and heuristic
                elif any(frontier_node.coordinates == child.coordinates for frontier_node in frontier):
                    existing_node = next((n for n in frontier if n.coordinates == child.coordinates), None)
                    if existing_node and existing_node.total_cost > tentative_cost:
                        frontier.remove(existing_node)
                        child.parent = current_node
                        child.total_cost = tentative_cost
                        child.heuristic = heuristicFunction(child, goals)
                        frontier.append(child)
        else:
            goals = temp_goals
    return None

# Combine given path with the path traced from the goal node to the start node.
def path(path, goalNode):
    solution = []
    final_solution = []
    traceNode = goalNode
    while traceNode is not None:
        solution.insert(0, traceNode)
        traceNode = traceNode.parent
    solution.pop(0)
    for x in path:
        final_solution.append(x)
    for y in solution:
        final_solution.append(y)
    return final_solution

# Constructs the path of nodes from the goal node to the start node by tracing parent nodes.
def construct_path_nodes(goalNode):
    solution = []
    traceNode = goalNode
    while traceNode is not None:
        solution.insert(0, traceNode)
        traceNode = traceNode.parent
    solution.pop(0)
    return solution

# Draws hexagon on the canvas, color attributes are also created here and assigned in draw_grid.
def draw_hexagon(canvas, x, y, size, color, outline_color="black"):
    points = []
    for i in range(6):
        angle = math.pi / 3 * i
        x_i = x + size * math.cos(angle)
        y_i = y + size * math.sin(angle)
        points.append((x_i, y_i))
    canvas.create_polygon(points, fill=color, outline=outline_color)

# Draws the grid of hexagons on the canvas based on the map_grid, color for each hexagon is determined based on node type.
def draw_grid(canvas, map_grid):
    size = 30
    for node in map_grid:
        x, y = node.coordinates
        x_pos = x * 1.5 * size + 50
        y_pos = y * math.sqrt(3) * size + 50
        if x % 2 == 0:
            y_pos += (math.sqrt(3) / 2) * size

        color = "white"
        text_color = "black"
        outline_color = "black"
        if node.node_type == "trap1":
            color = "purple"
        elif node.node_type == "trap2":
            color = "violet"
        elif node.node_type == "trap3":
            color = "blue"
        elif node.node_type == "trap4":
            color = "indigo"
        elif node.node_type == "reward1":
            color = "green"
        elif node.node_type == "reward2":
            color = "cyan"
        elif node.node_type == "treasure":
            color = "gold"
        elif node.node_type == "obstacle":
            color = "black"
            text_color = "white"
            outline_color = "white"

        draw_hexagon(canvas, x_pos, y_pos, size, color, outline_color)
        canvas.create_text(x_pos, y_pos, text=f"({x},{y})", fill=text_color)

# Draws the solution path on the canvas with red underlines.
def draw_path(canvas, path):
    size = 30
    x , y = path
    x_pos = x * 1.5 * size + 50
    y_pos = y * math.sqrt(3) * size + 50
    if x % 2 == 0:
            y_pos += (math.sqrt(3) / 2) * size

    #for displaying red path in circle
    #canvas.create_oval(x_pos - size / 3, y_pos - size / 3, x_pos + size / 3, y_pos + size / 3, fill="red")

    text_id = canvas.find_closest(x_pos, y_pos)[0]
    bbox = canvas.bbox(text_id)
    if bbox:
        x1, y1, x2, y2 = bbox
        canvas.create_line(x1, y2, x2, y2, fill="red", width=4)
    canvas.master.title(f"Path Visualization")

# Displays the solution and total cost in a new window in text form, total energy cost and steps taken are displayed here.
def display_solution(solution, cost_total):
    solution_window = tk.Toplevel()
    solution_window.title("Solution")
    solution_window.geometry("220x400")

    for i, step in enumerate(solution):
        path_label = tk.Label(solution_window, text=f"Step {i + 1}: {step}")
        path_label.pack()

    total_cost_label = tk.Label(solution_window, text=f"Total Cost: {cost_total}")
    total_cost_label.pack()

# Displays the legends for different node types in a new window, color mapping is done according to node type.
def display_legends():
    legend_window = tk.Toplevel()
    legend_window.title("Legends")
    legend_window.geometry("250x210")

    legends = [
        ("Trap 1", "purple"),
        ("Trap 2", "violet"),
        ("Trap 3", "blue"),
        ("Trap 4", "indigo"),
        ("Reward 1", "green"),
        ("Reward 2", "cyan"),
        ("Treasure", "gold"),
        ("Obstacle", "black", "white"),
        ("Normal", "white"),
        ("Solution", "red")
    ]

    for legend in legends:
        text_color = "black"
        if len(legend) == 3:
            legend_name, bg_color, text_color = legend
        else:
            legend_name, bg_color = legend

        legend_label = tk.Label(legend_window, text=legend_name, bg=bg_color, fg=text_color, width=20)
        legend_label.pack()

# Main function to run algorithm and display results in different windows.
if __name__ == "__main__":
    initialCoordinate = (0, 0)
    goalState = [(4, 1), (3, 4), (7, 3), (9,3)]
    map_grid = createMap(6, 10)
    solution, cost_total = a_star(map_grid, initialCoordinate, goalState)

    # Print the solution to debug
    print("Solution:", solution)

    root = tk.Tk()
    canvas = tk.Canvas(root, width=500, height=400)
    canvas.pack()

    draw_grid(canvas, map_grid)

    # Print the paths in the solution
    step_counter = 0
    for step in solution:
        step_counter += 1
        print("Step ",step_counter,":", step)
        draw_path(canvas, step)

    # Visualize output
    display_solution(solution, cost_total)
    display_legends()
    root.mainloop()