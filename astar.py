#Import important libraries 
import time
import copy
import cv2
# uncomment below line if running in colab
# from google.colab.patches import cv2_imshow
import heapq as hq
import numpy as np
import math

# function to draw the obstacles on a canvas map 
def obstacles_map(canvas):
    # rectangle 1 obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(100,150), (150,150), (150,250), (100,250)])], color = (255, 0, 0))
    # rectangle 2 obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(100,0), (150,0), (150,100), (100,100)])], color = (255, 0, 0))
    # heaxgon obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(235,87),(300,50),(365,87),(365,162),(300,200),(235,162)])], color = (255, 0, 0))
    # traingle obstacle with the given dimensions, the thickness of 5mm is considered inwards
    cv2.fillPoly(canvas, pts = [np.array([(460,225), (510,125), (460,25)])], color = (255, 0, 0))
    # cv2.imshow('canvas', canvas)
    # cv2.waitKey(0)
    return canvas


# TO DO: Add 'offset' that would be including the robot radius and clearance, you can use your previous function as well


def coord_input(canvas, manual_input=False):
    if(manual_input):
    # initialize empty lists
        start_position = []
        goal_position = [] 
        # Get X and Y coordinates for the start node/position
        while True:
            state = input("Enter the X Coordinate of Start position: ")
            try:
                x = int(state)
            except ValueError:
                # if the coordinate entered is not integer, ask again
                print("Enter a valid integer for X Coordinate instead")
                continue
            if not (0 <= x < canvas.shape[1]):
                # to check if the coordinate value entered is greater than the canvas dimensions
                print("X Coordinate is out of bounds")
                continue
            state = input("Enter the Y Coordinate of Start position: ")
            try:
                y = int(state)
            except ValueError:
            # if the coordinate entered is not integer, ask again
                print("Please enter a valid integer for Y Coordinate instead")
                continue
            if not (0 <= y < canvas.shape[0]):
            # to check if the coordinate value entered is greater than the canvas dimensions
                print("Y Coordinate is out of bounds")
                continue        
            if(canvas[canvas.shape[0]-1-y][x][0]==255 or 0<=x<5 or 0<=y<5):
            # to check if the entered coordinates of x and y lie inside the obstacle space
                print("The entered start position is in the obstacle space, enter again")
                continue      
            start_position = [x, y]
            break   
        # angle for start position
        while True:
            start_position_angle = input("Please enter the angle for start position as a multiple of 30 degrees: ")
            try:
                angle = int(start_position_angle)
                if angle % 30 != 0:
                    raise ValueError
            except ValueError:
                print("Please enter a valid head angle that is a multiple of 30 degrees.")
                continue
            if angle < 0:
                angle += 360
            start_position.append(angle)
            break
        # Get X and Y coordinates for the final state
        while True:
            state = input("Enter the X Coordinate of Goal position: ")
            try:
                x = int(state)
            except ValueError:
            # if the coordinate entered is not integer, ask again
                print("Please enter a valid integer for X Coordinate instead")
                continue
            if not (0 <= x < canvas.shape[1]):
            # to check if the coordinate value entered is greater than the canvas dimensions
                print("X Coordinate is out of bounds")
                continue     
            state = input("Enter the Y Coordinate of Goal position: ")
            try:
                y = int(state)
            except ValueError:
            # if the coordinate entered is not integer, ask again
                print("Please enter a valid integer for Y Coordinate instead")
                continue
            if not (0 <= y < canvas.shape[0]):
                # to check if the coordinate value entered is greater than the canvas dimensions
                print("Y Coordinate is out of bounds")
                continue    
            if(canvas[canvas.shape[0]-1-y][x][0]==255 or 0<=x<5 or 0<=y<5):
            # to check if the entered coordinates of x and y lie inside the obstacle space
                print("The entered goal node is in the obstacle space, enter again")
                continue
            goal_position = [x, y]
            break  
        # angle for final state
        while True:
            goal_position_angle = input("Please enter the angle for goal position as a multiple of 30 degrees: ")
            try:
                angle = int(goal_position_angle)
                if angle % 30 != 0:
                    raise ValueError
            except ValueError:
                print("Please enter a valid head angle that is a multiple of 30 degrees.")
                continue
            if angle < 0:
                angle += 360
            goal_position.append(angle)
            break
        # step size
        while True:
            step_size = input("Enter the robot's step size (1 to 10): ")
            if(int(step_size)<1 and int(step_size)>10):
                print("Invalid, enter again!")
            else:
                break
        # return start_position, goal_position
    else:
        start_position = [50, 110, 30]
        goal_position = [220, 220, 30]
        step_size = 5
    return start_position, goal_position, int(step_size)

def radius_clearance_robot(manual_input=False):
    if(manual_input):
        while True:
            try:
                clearance = int(input("Enter the robot's clearance: "))
                if clearance < 0:
                    print("Invalid, please try again!")
                else:
                    break
            except ValueError:
                print("Enter interger value instead!")

        while True:
            try:
                robot_radius = int(input("Enter the robot's radius: "))
                if robot_radius < 0:
                    print("Invalid, please try again!")
                else:
                    break
            except ValueError:
                print("Enter interger value instead!")
    else:
        robot_radius = 5
        clearance = 5    
    return robot_radius, clearance

def obstacle_checkpoint(next_node_width, next_node_height, canvas):    
    # check if the node is in the obstacle region
    if canvas[int(round(next_node_height))][int(round(next_node_width))][0]==255:
        return False
    else:
        return True

def cost_to_goal(node, final):
    # Cost To Goal between present node and goal nodes using a Euclidean distance.
    x1, y1, _ = node
    x2, y2, _ = final
    euclidean_dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return euclidean_dist

def goal_node_check(node, final):
    # returns bool, false if present node in not the goal node, checks within the threshold distance of 1.5
    return np.linalg.norm(np.array(node[:2]) - np.array(final[:2])) < 1.5 and node[2] == final[2]


def zero_deg_action(node, canvas, visited_node, step_size): 
    # Moves the robot at 0 degree angle (wrt robot's frame) by the step size
    x, y, theta = node  
    # Calculate new angle
    new_angle = (theta + 0) % 360   
    # Calculate new x and y coordinates after the new angle 
    next_x = x + step_size * np.cos(np.deg2rad(new_angle))
    next_y = y + step_size * np.sin(np.deg2rad(new_angle))    
    # Check if new coordinates are within the canvas boundaries and not colliding with any obstacles
    if not (0 <= next_x < canvas.shape[1] and 0 <= next_y < canvas.shape[0] and obstacle_checkpoint(next_x, next_y, canvas)):
        return False, node, False
    
    # Check if new node is a seen before or is duplicate
    idx = (int(next_y*2), int(next_x*2), int(new_angle/30))
    if visited_node[idx] == 1:
        return True, node, True    
    # Update visited matrix and return new node
    visited_node[idx] = 1
    next_node = [next_x, next_y, new_angle]
    # bool, which is true if child node can be generated
    # list of next node is child nodes generated after the action
    # bool if the generates node is visible
    return True, next_node, False
    

def plus_thirty_deg_action(node, canvas, visited_node, step_size):    # Local angles
    # Moves the robot at +30 degree angle (wrt robot's frame) by the step size
    x, y, theta = node  
    # Calculate new angle
    new_angle = (theta - 30) % 360   
    # Calculate new x and y coordinates after the new angle 
    next_x = x + step_size * np.cos(np.deg2rad(new_angle))
    next_y = y + step_size * np.sin(np.deg2rad(new_angle))
    
    # Check if new coordinates are within the canvas boundaries and not colliding with any obstacles
    if not (0 <= next_x < canvas.shape[1] and 0 <= next_y < canvas.shape[0] and obstacle_checkpoint(next_x, next_y, canvas)):
        return False, node, False
    
    # Check if new node is a seen before or is duplicate
    idx = (int(next_y*2), int(next_x*2), int(new_angle/30))
    if visited_node[idx] == 1:
        return True, node, True    
    # Update visited matrix and return new node
    visited_node[idx] = 1
    next_node = [next_x, next_y, new_angle]
    # bool, which is true if child node can be generated
    # list of next node is child nodes generated after the action
    # bool if the generates node is visible
    return True, next_node, False


def minus_thirty_deg_action(node, canvas, visited_node, step_size): 
    # Moves the robot at -30 degree angle (wrt robot's frame) by the step size
    x, y, theta = node  
    # Calculate new angle
    new_angle = (theta + 30) % 360   
    # Calculate new x and y coordinates after the new angle 
    next_x = x + step_size * np.cos(np.deg2rad(new_angle))
    next_y = y + step_size * np.sin(np.deg2rad(new_angle))
    
    # Check if new coordinates are within the canvas boundaries and not colliding with any obstacles
    if not (0 <= next_x < canvas.shape[1] and 0 <= next_y < canvas.shape[0] and obstacle_checkpoint(next_x, next_y, canvas)):
        return False, node, False
    
    # Check if new node is a seen before or is duplicate
    idx = (int(next_y*2), int(next_x*2), int(new_angle/30))
    if visited_node[idx] == 1:
        return True, node, True    
    # Update visited matrix and return new node
    visited_node[idx] = 1
    next_node = [next_x, next_y, new_angle]
    # bool, which is true if child node can be generated
    # list of next node is child nodes generated after the action
    # bool if the generates node is visible
    return True, next_node, False


def plus_sixty_deg_action(node, canvas, visited_node, step_size):    # Local angles
    # Moves the robot at -60 degree angle (wrt robot's frame) by the step size
    x, y, theta = node  
    # Calculate new angle
    new_angle = (theta + 60) % 360   
    # Calculate new x and y coordinates after the new angle 
    next_x = x + step_size * np.cos(np.deg2rad(new_angle))
    next_y = y + step_size * np.sin(np.deg2rad(new_angle))
    
    # Check if new coordinates are within the canvas boundaries and not colliding with any obstacles
    if not (0 <= next_x < canvas.shape[1] and 0 <= next_y < canvas.shape[0] and obstacle_checkpoint(next_x, next_y, canvas)):
        return False, node, False
    
    # Check if new node is a seen before or is duplicate
    idx = (int(next_y*2), int(next_x*2), int(new_angle/30))
    if visited_node[idx] == 1:
        return True, node, True    
    # Update visited matrix and return new node
    visited_node[idx] = 1
    next_node = [next_x, next_y, new_angle]
    # bool, which is true if child node can be generated
    # list of next node is child nodes generated after the action
    # bool if the generates node is visible
    return True, next_node, False


def minus_sixty_deg_action(node, canvas, visited_node, step_size):    # Local angles
    # Moves the robot at -60 degree angle (wrt robot's frame) by the step size
    x, y, theta = node  
    # Calculate new angle
    new_angle = (theta + 60) % 360   
    # Calculate new x and y coordinates after the new angle 
    next_x = x + step_size * np.cos(np.deg2rad(new_angle))
    next_y = y + step_size * np.sin(np.deg2rad(new_angle))
    
    # Check if new coordinates are within the canvas boundaries and not colliding with any obstacles
    if not (0 <= next_x < canvas.shape[1] and 0 <= next_y < canvas.shape[0] and obstacle_checkpoint(next_x, next_y, canvas)):
        return False, node, False
    
    # Check if new node is a seen before or is duplicate
    idx = (int(next_y*2), int(next_x*2), int(new_angle/30))
    if visited_node[idx] == 1:
        return True, node, True    
    # Update visited matrix and return new node
    visited_node[idx] = 1
    next_node = [next_x, next_y, new_angle]
    # bool, which is true if child node can be generated
    # list of next node is child nodes generated after the action
    # bool if the generates node is visible
    return True, next_node, False


def plus_sixty_deg_action(node, canvas, visited_node, step_size):    # Local angles
    # Moves the robot at -60 degree angle (wrt robot's frame) by the step size
    x, y, theta = node  
    # Calculate new angle
    new_angle = (theta + 60) % 360   
    # Calculate new x and y coordinates after the new angle 
    next_x = x + step_size * np.cos(np.deg2rad(new_angle))
    next_y = y + step_size * np.sin(np.deg2rad(new_angle))
    
    # Check if new coordinates are within the canvas boundaries and not colliding with any obstacles
    if not (0 <= next_x < canvas.shape[1] and 0 <= next_y < canvas.shape[0] and obstacle_checkpoint(next_x, next_y, canvas)):
        return False, node, False
    
    # Check if new node is a seen before or is duplicate
    idx = (int(next_y*2), int(next_x*2), int(new_angle/30))
    if visited_node[idx] == 1:
        return True, node, True    
    # Update visited matrix and return new node
    visited_node[idx] = 1
    next_node = [next_x, next_y, new_angle]
    # bool, which is true if child node can be generated
    # list of next node is child nodes generated after the action
    # bool if the generates node is visible
    return True, next_node, False

# heurastic function considers the euclidean distance
def h_cost_calc(node, goal_node):
    print(node)
    print(goal_node)
    x1,y1=node[0:2]
    x2,y2=goal_node[0:2]
    print(x1, y1, x2, y2)
    dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
    return dist

def astar(start_position, final_position, canvas, visited_node, step_size):
  # TO FILL
    h_cost = h_cost_calc(start_position, final_position)
    g_cost = 0
    f_cost = h_cost+g_cost
    # f_cost g_cost h_cost node parent_node
    open_list = [(f_cost, h_cost, g_cost, start_position, start_position)]#heap data structure of tuples 
    closed_list = {}#dict
    hq.heapify(open_list)
    while(len(open_list)):
        # pop new node
        new_closed_list_element = open_list.pop()
        print(new_closed_list_element)
        closed_list = dict({(new_closed_list_element[3][0], new_closed_list_element[3][1]):new_closed_list_element[4]})
        print(closed_list)
        node = new_closed_list_element[3]
        # add node in the closed list

        for flaga, node, flagb in [
        zero_deg_action(node, canvas, visited_node, step_size),
        plus_thirty_deg_action(node, canvas, visited_node, step_size),
        minus_thirty_deg_action(node, canvas, visited_node, step_size),
        minus_sixty_deg_action(node, canvas, visited_node, step_size),
        plus_sixty_deg_action(node, canvas, visited_node, step_size),]:
            if flaga:
                pass
            # check if node is in obstacle spaxe
            # check if it is in canvas
            # check if it is in closed list
            # if it is in OL then update the value
            # if it is not in OL update the OL
            # hq.heapify(open_list)

# def back_track(start_position, final_position, all_list, canvas):
  # TO FILL

if __name__ == '__main__':
    
    canvas = np.ones((250,600,3), dtype="uint8")    # Creating a blank canvas/map
    # radiaus and clearnce input
    robot_radius, clearance = radius_clearance_robot(manual_input=False)
    # draw the obstacles
    # canvas = obstacles_map(canvas,offset = (clearance + robot_radius))
    canvas = obstacles_map(canvas)
    # start and goal node input with angles
    start_position, final_position, step = coord_input(canvas, manual_input=False) 
    print(start_position)
    # Changing the input Cartesian Coordinates of the Map to Image Coordinates:
    start_position[1] = canvas.shape[0]-1 - start_position[1]
    final_position[1] = canvas.shape[0]-1 - final_position[1]
    # print(initial_state, final_state)
    # Converting to image coordinates
    if start_position[2] != 0:
        start_position[2] = 360 - start_position[2]
    if final_position[2] != 0:
        final_position[2] = 360 - final_position[2]

    print("\nCoordinates  of start position: ", start_position)
    print("Coordinates of goal position: ", final_position)
    start_time = time.time()
    cv2.circle(canvas,(int(start_position[0]),int(start_position[1])),2,(0,0,255),-1)
    cv2.circle(canvas,(int(final_position[0]),int(final_position[1])),2,(0,0,255),-1)
    # call the main astar algo   
    visited_node = []
    astar(start_position,final_position,canvas, visited_node, step)
    # total time taken    
    end_time = time.time()  
    print("\nExecution time is ", end_time-start_time, "seconds" )
    # cv2.imshow("A* Path Visualization", canvas)
    cv2.imshow('canvas', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()