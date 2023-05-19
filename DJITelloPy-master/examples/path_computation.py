from PIL import Image
import heapq
from queue import PriorityQueue
import numpy as np
import cv2
from image_distance import compute_distance
# Load image




def pixel_assignments(image_path, rover_marker_ID, wall_marker_ID, destination_marker_ID):
    # Load the image
    image = cv2.imread(image_path)

    # Define the aruco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters_create()

    detector = cv2.aruco.Detector()

    # Detect the aruco markers in the image
    corners, ids, _ = detector.detectMarkers(image, aruco_dict, parameters=aruco_params)

    # Get the corners of the rover, wall, and destination markers
    rover_corners = corners[np.where(ids == rover_marker_ID)[0]]
    wall_corners = corners[np.where(ids == wall_marker_ID)[0]]
    destination_corners = corners[np.where(ids == destination_marker_ID)[0]]

    # Convert the corners to pixel coordinates
    wall_pixels = [np.int32(corner.squeeze()) for corner in wall_corners]
    rover_pixels = [np.int32(corner.squeeze()) for corner in rover_corners]
    destination_pixels = [np.int32(corner.squeeze()) for corner in destination_corners]

    # Return the pixel coordinates for each marker type
    return wall_pixels, rover_pixels, destination_pixels




def compress_image(image, n, rover_marker_ID, wall_marker_ID, destination_marker_ID):
    # divide the image into n x n blocks and compute the colour of each block
    block_width = image.width // n
    block_height = image.height // n

    pixel_matrix = np.asarray(image)

    blocks = []
    destination_block_pixels = 0
    rover_block_pixels = 0
    break_out_flag = False
    wall_pixels, rover_pixels, destination_pixels = pixel_assignments(image, rover_marker_ID, wall_marker_ID, destination_marker_ID)
    #traverse each block in the grid, and determine which sections are walls, rover or destination
    for i in range(n):
        for j in range(n):
            block = pixel_matrix[j*block_height:(j+1)*block_height,i*block_width:(i+1)*block_width]

            rover_pixels_counter = 0
            destination_pixels_counter
            for row in block:             
                for pixel in row:
                    
                    #handle each colour appropriately
                    if pixel in rover_pixels:
                        rover = (j, i)
                        rover_pixels_counter += 1
                    elif pixel in destination_pixels:
                        destination_pixels_counter += 1
                    elif pixel in wall_pixels:
                        walls.append((j, i))
                        break_out_flag = True
                        break

                if break_out_flag:
                    break_out_flag = False
                    break
  
            #determine block that is most covered by green
            if destination_pixels_counter >= destination_pixels_counter:
                destination = (j, i)
                destination_block_pixels = destination_pixels_counter

            #determine block most covered by red
            if rover_pixels_counter >= rover_block_pixels:
                rover = (j, i)
                rover_block_pixels = rover_pixels_counter



    


    compressed_image = np.zeros((n,n))
    #draw objects on compressed image
    for wall in walls:
        compressed_image[wall[0]][wall[1]] = 1
    
    compressed_image[rover[0]][rover[1]] = 2

    compressed_image[destination[0]][destination[1]] = 3
 
    return compressed_image, rover, destination

def heuristic(a, b):
    # calculate the Manhattan distance between two points
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def astar(array, start, dest):
    # initialize the priority queue
    pq = PriorityQueue()
    pq.put((0, start))

    # initialize the distance and visited arrays
    dist = {start: 0}
    visited = {start: None}

    while not pq.empty():
        # get the cell with the lowest f-score
        curr_cost, curr = pq.get()

        # if we've reached the destination, reconstruct the path and return it
        if curr == dest:
            path = []
            while curr is not None:
                path.append(curr)
                curr = visited[curr]
            path.reverse()
            return path

        # check the neighbors of the current cell
        for next in [(0, 1), (0, -1), (1, 0), (-1, 0), (-1,-1), (1, 1)]:
            neighbor = (curr[0] + next[0], curr[1] + next[1])
            # check if the neighbor is inside the array bounds
            if neighbor[0] < 0 or neighbor[0] >= array.shape[0] or neighbor[1] < 0 or neighbor[1] >= array.shape[1]:
                continue
            # check if the neighbor is a wall cell
            if array[neighbor] == 1:
                continue
            # calculate the tentative distance from the start to the neighbor through the current cell
            tentative_dist = dist[curr] + 1
            # update the neighbor's distance if it's lower than the current distance
            if neighbor not in dist or tentative_dist < dist[neighbor]:
                dist[neighbor] = tentative_dist
                priority = tentative_dist + heuristic(dest, neighbor)
                pq.put((priority, neighbor))
                visited[neighbor] = curr

    # if we haven't found the destination, return None
    return None



def find_path(image,height, r_ID, d_ID, w_ID, compress_factor=20):

    image = Image.open('C:/Users/peter/Documents/Bio inspired robotics/telloside/Pi/input.png')
    if image.mode == 'CMYK':
        image = image.convert('RGB')
    pixels = image.load()


    # Find walls, rover and destination
    walls = []
    rover = None
    destination = None

    #compress image
    compressed_image, start, dest = compress_image(image, compress_factor, r_ID, d_ID, w_ID)

    #find path  
    path = astar(compressed_image, start, dest)

    #draw computed path on original image
    block_width = image.width // compress_factor
    block_height = image.height // compress_factor
    for j, i in path:
        for y in range(j*block_height,(j+1)*block_height):
            for x in range(i*block_width, (i+1)*block_width):
                pixels[x,y] = (255, 255, 0)


    path_distances = []
    for j, i in path:
        block_centre_y = (j*block_height + (j+1) * block_height  - 1) // 2
        block_centre_x = (i*block_width + (i+1) * block_width - 1) // 2
        path_distances.append(compute_distance(block_centre_x, block_centre_y, height))

    image.save('output.png')

    return path_distances

