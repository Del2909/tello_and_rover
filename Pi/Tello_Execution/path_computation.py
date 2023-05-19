from PIL import Image
import heapq
from queue import PriorityQueue
import numpy as np
import cv2
# Load image
import cv2.aruco as aruco
import math
#from droneFunctions2 import detectAruco

def get_pixels_inside_corners(corners):
    """
    Given a list of four tuples representing pixel coordinates of the four corners of a rectangle,
    return a list of all pixel coordinates inside that rectangle.
    """
    # Get the minimum and maximum x and y coordinates of the 
    pixels = []

    if len(corners) == 0:
        print("Empty")
    else:
        min_x = int(min([corner[0] for corner in corners]))
        max_x = int(max([corner[0] for corner in corners]))
        min_y = int(min([corner[1] for corner in corners]))
        max_y = int(max([corner[1] for corner in corners]))

        # Create a list of all pixel coordinates inside the rectangle
        for x in range(min_x, max_x+1):
            for y in range(min_y, max_y+1):
                pixels.append((x, y))

    return pixels

def compute_distance(pixel_x, pixel_y, camera_matrix, dist_coeffs, z):
    # Define the image coordinates
    img_coords = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)

    # Undistort the image coordinates
    undistorted_coords = cv2.undistortPoints(img_coords, camera_matrix, dist_coeffs)
    
    # Convert the undistorted image coordinates to normalized image coordinates
    normalized_coords = undistorted_coords[0][0]
    #normalized_coords = np.dot(np.linalg.inv(camrea_matrix), np.array([undistorted

    # Calculate the real-world x and y coordinates in the camera coordinx`ate system
    x = normalized_coords[0] * z
    y = normalized_coords[1] * z

    return (x, y)




# assigns rover, wall or destination or neither to blocks of the segmented grid of the picture
def pixel_assignments(image_path, rover_marker_ID, wall_marker_ID,
                      destination_marker_ID, camera_matrix, distortion_coefficients):
    
    img = cv2.imread(image_path)
    # Define the ArUco dictionary
    marker_size = 150
    print("Pixel entered")


    # Modify the parameters to make detection more lenient
 
    #set cv2 detection parameters
    
    
    
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    counter = 0
    notAll = True
    #detect aruco markers
    while notAll:
        counter += 1
        tvec, ids, corners, z = detectAruco(img, camera_matrix, distortion_coefficients)
        if ids is not None:
            if 1 in ids and 2 in ids and 3 in ids and len(ids) >= 7:
                notAll = False
        if counter == 20:
            notAll = False
    #detect aruco marker
    #tvec, ids, corners = detectAruco(img, camera_matrix, distortion_coefficients)

    print("IDs found: ", ids)   


    wall_corners = []
    # Get the corners of the rover, wall, and destination markers
    if rover_marker_ID in ids:
        rover_corners = corners[np.where(ids == rover_marker_ID)[0][0]]
    else:
        print("Rover marker ID not found")
        rover_corners = []

    if wall_marker_ID in ids:
        for id in (np.where(ids == int(wall_marker_ID))[0]):
           wall_corners.append(corners[id][0])


    else:
        print("wall marker ID not found")

        wall_corners = []

    if destination_marker_ID in ids:
        destination_corners = corners[np.where(ids == int(destination_marker_ID))[0][0]]
    else:
 
        print("destination marker ID not found")
        destination_corners = []

    # Convert the corners to pixel coordinates

    
    wall_pixels = []
    for x in range(len(wall_corners)):
        wall_pixels += get_pixels_inside_corners(wall_corners[x])



    rover_pixels = get_pixels_inside_corners(rover_corners[0])
    destination_pixels = get_pixels_inside_corners(destination_corners[0])
    # Return the pixel coordinates for each marker type
    return wall_pixels, rover_pixels, destination_pixels, z  

def detectAruco( frame, camera_matrix, camera_distortion):
    """
    For future work try to reinitialise the frame for the downward camera and front camera seperately
    """
    
    ### --- Detecting aruco marker and calculating distance
    #marker width and height 
    marker_size = 150 #mm        

    #initialise camera calibration matrix

    #define the aruco marker spec 
    #it is 4x4_250 series
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    #convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    #detect aruco markers
    corners, ids , rejected = aruco.detectMarkers(frame,aruco_dict, camera_matrix, camera_distortion)  

    # Draw the detected markers on the original image
    image_with_markers = frame.copy()

    detected=0
    tvec = [0,0,0]
    print("CHECK IDS: ", ids)
    if ids is not None:
        detected=1
        #print("ids found",ids)
        #draw box around the aruco markers
        #aruco.drawDetectedMarkers(frame,corners)
        #aruco.drawDetectedMarkers(frame, rejected)
        #get 3D pose of each and every aruco marker
        #get rotational and translational vectors
        rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners,marker_size,camera_matrix,camera_distortion)
        
        rvec = rvec_list_all[0][0]
        z = 0
        counter = 0
        for i_d in ids:
            if i_d == 1:
                index = np.where(ids == i_d)
                print("height: ", tvec_list_all[index][0][2])
                z += tvec_list_all[index][0][2] + 430
                counter += 1
        z /= counter
            
            
        tvec = tvec_list_all[0][0]
    
        #aruco.drawAxis(frame,camera_matrix,camera_distortion,rvec,tvec,30)
        tvec_str= "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
        rvec_str= "x_r=%4.0f y_r=%4.0f z_r=%4.0f"%(rvec[0],rvec[1],rvec[2])
        #print("rvec",rvec_str)
        cv2.putText(frame,tvec_str,(10,20),cv2.FONT_HERSHEY_PLAIN,1.5,(0,0,255),2,cv2.LINE_AA)

    return tvec, ids, corners, z


#reduces image to grid with size n x n
def compress_image(image, image_path, n,m,  rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients):
    high_compress_factor = True
    wall_avoidance_value = 9

    # divide the image into n x n blocks and compute the colour of each block
    block_width = image.width // n  
    block_height = image.height // m
    print(image.height)
    print(image.width)
    # create 2D array of pixels
    pixel_matrix = np.asarray(image)

    blocks = []
    destination_block_pixels = 0
    rover_block_pixels = 0

    #if wall is seen in block, entire block is defined as wall and exists loop of block
    break_out_flag = False
    
    #assigns pixels as walls, rover or destination
    wall_pixels, rover_pixels, destination_pixels, z = pixel_assignments(image_path, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients)
    
    # traverse each block in the grid, and determine which sections are walls, rover or destination
    walls = []


    for i in range(n):
        for j in range(m):
            
            block = pixel_matrix[j*block_height:(j+1)*block_height,i*block_width:(i+1)*block_width]



            # Counts number of pixels in the block
            rover_pixels_counter = 0
            destination_pixels_counter = 0
            
                

            # For each pixel in the block:
            for y in range(j*block_height,(j+1)*block_height):
                for x in range(i*block_width, (i+1)*block_width):
                    #if j == 4 and i == 3:
                       # print(pixel)
                    pixel = (x, y)

                    # counts number of rover pixels
                    if pixel in rover_pixels:
                        rover_pixels_counter += 1

                    # counts number of destination pixels
                    elif pixel in destination_pixels:
                        destination_pixels_counter += 1

                    # if wall pixel, block is avoided
                    elif len(wall_pixels) > 0:
                        if pixel in wall_pixels:

                            for x in range(wall_avoidance_value + 1):
                                walls.append((j-x, i))
                                walls.append((j+x, i))
                                walls.append((j,i+x))
                                walls.append((j,i-x))

                            break_out_flag = True
                            break
                            
                # breaks when wall is detected
                if break_out_flag:
                    break_out_flag = False
                    break
  
            #determine block that is most covered by destination
            if destination_pixels_counter > destination_block_pixels:

                destination = (j, i)
                destination_block_pixels = destination_pixels_counter

            #determine block most covered by rover
            if rover_pixels_counter > rover_block_pixels:
                rover = (j, i)
                rover_block_pixels = rover_pixels_counter

    compressed_image = np.zeros((m,n))

    #draw objects on compressed image
    for wall in walls:
        compressed_image[wall[0]][wall[1]] = 1
    
    compressed_image[rover[0]][rover[1]] = 2

    compressed_image[destination[0]][destination[1]] = 3
 
    #returns image as a 2D array of size n x n, with 1 representing walls, 2 representing rover, 3 representing destination, and 0 representing space.
    return compressed_image, rover, destination, z

# Define the heuristic function
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* pathfinding algorithm given a 2D array
def astar(array, start, dest):
    # Initialize the priority queue
    pq = PriorityQueue()
    pq.put((0, start, None))

    # Initialize the distance and visited arrays
    dist = {start: 0}
    visited = {start: None}

    while not pq.empty():
        # Get the cell with the lowest f-score
        curr_cost, curr, prev_dir = pq.get()

        # If we've reached the destination, reconstruct the path and return it
        if curr == dest:
            path = []
            while curr is not None:
                path.append(curr)
                curr = visited[curr]
            path.reverse()
            return path

        # Check the neighbours of the current cell
        for next in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbour = (curr[0] + next[0], curr[1] + next[1])
            # Check if the neighbour is inside the array bounds
            if neighbour[0] < 0 or neighbour[0] >= array.shape[0] or neighbour[1] < 0 or neighbour[1] >= array.shape[1]:
                continue
            # Check if the neighbour is a wall cell
            if array[neighbour] == 1:
                continue

            # Add a small penalty for changing direction
            direction_penalty = 10000 if prev_dir is None or prev_dir != next else 0

            # Calculate the tentative distance from the start to the neighbour through the current cell
            tentative_dist = dist[curr] + 1 + direction_penalty
            # Update the neighbour's distance if it's lower than the current distance
            if neighbour not in dist or tentative_dist < dist[neighbour]:
                dist[neighbour] = tentative_dist
                priority = tentative_dist + heuristic(dest, neighbour)
                pq.put((priority, neighbour, next))
                visited[neighbour] = curr

    # If we haven't found the destination, return None
    return None

def calculate_direction(x, y, x0, y0):
    # Calculate the angle in radians
    
    
    new_x = x - x0
    new_y = y - y0
    
    if new_x > 30 and new_y > 30:
        return -90
    elif new_x < -30 and new_y > 30:
        return -90
    
    elif new_x > 30 and new_y < -30:
        return 90
    elif new_x < -30 and new_y < -30:
        return 90
    else:
        return 0
    # Convert the angle to degrees
    

def reformat_path(movements):
    formatted_movements = []
    formatted_movements.append((0,0,1))
    turn = False

    direction_change = 0
    distance = 0
    direction = 0
    print("ORIGINAL MOVEMENTS: ", movements)
    last_movement_x = -80
    last_movement_y = 0
    
    
    for i, (movement_x, movement_y) in enumerate(movements):
        
        new_direction =  -calculate_direction(movement_x, movement_y, last_movement_x, last_movement_y)
        direction_change = new_direction
        
        
        
        print(direction_change)
        if direction_change > 70:
            if i != 0:
                distance += 200
                formatted_movements.append((distance, 0, 0))
            formatted_movements.append((0,90,0))
            distance = 200
                #distance = 0
        elif direction_change < -70:
            if i!= 0:
                distance -= 200
                formatted_movements.append((distance, 0, 0))
            formatted_movements.append((0,-90,0))
            distance = -200
                #distance = 0
        last_movement_x = movement_x
        last_movement_y = movement_y
        
        
        distance += math.sqrt(movement_x**2 + movement_y**2)
        #print(distance)
    formatted_movements.append((distance, 0, 0))


    formatted_movements.append((0,0,2))

        
    return formatted_movements




# finds real world distance of series of movements from rover to destination avoiding walls
def find_path(image_path, height, r_ID,
w_ID, d_ID, camera_matrix, dist_coefficients,compress_factor_height=64,
              compress_factor_width=48):

    # open image 
    image = Image.open(image_path)
    
    img = cv2.imread(image_path)
    undist = cv2.undistort(img, camera_matrix, dist_coefficients)
    cv2.imwrite('undistorted_image.jpg', undist)
    #if image.mode == 'CMYK':
    #    image = image.convert('RGB')
    pixels = image.load()

    walls = []
    rover = None
    destination = None
    # compress image

    compressed_image, start, dest, z = compress_image(image, image_path, compress_factor_height, compress_factor_width, r_ID, w_ID, d_ID, camera_matrix, dist_coefficients)
    


    # find path  
    path = astar(compressed_image, start, dest)
    print("Path : ", path,start,dest)
    # draw computed path on original image

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    #convert the frame to grayscale
    #gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    #detect aruco markers


    block_width = image.width // compress_factor_height
    block_height = image.height // compress_factor_width
    for j, i in path:
        for y in range(j*block_height,(j+1)*block_height):
            for x in range(i*block_width, (i+1)*block_width):
                pixels[x,y] = (255, 255, 0)
    
    
    # find distance away from camera of starting block - set to initial distance
    block_centre_y = (path[0][0]*block_height + (path[0][0]+1) * block_height  - 1) // 2
    block_centre_x = (path[0][1]*block_width + (path[0][1]+1) * block_width - 1) // 2
    
    dist_from = compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, z)

    path_distances = []

    # finds difference in real world distance of center of next block in path from previous block in path.
    for j, i in path[1:]:
        block_centre_y = (j*block_height + (j+1) * block_height  - 1) // 2
        block_centre_x = (i*block_width + (i+1) * block_width - 1) // 2

        # computes real world distance of center of block - real world distance of center of last block
        # appends to path_distances, containing all distances to travel
        path_distances.append((np.array(compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, z)) - np.array(dist_from)).tolist())
        dist_from = compute_distance(block_centre_x,block_centre_y, camera_matrix, dist_coefficients, z)

    new_path = reformat_path(path_distances)
    image.save('output.png')

    return new_path



def get_aruco_distance(img_path, camera_matrix, dist_coeffs):
    
    img = cv2.imread(img_path)

    # Define the ArUco dictionary
    marker_size = 8

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #img_blurred = cv2.GaussianBlur(img_gray, (3, 3), 0)

    #_, gray_frame = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY)

    #gray_frame = cv2.adaptiveThreshold(img_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # Detect the ArUco markers in the input image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, aruco_dict, camera_matrix, dist_coeffs)   
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)


    print("IDs found: ", ids)
    for marker in range(len(ids)):

        # Convert the rotation vector to a rotation matrix
        
   
        rvec_marker = rvec[marker][0]
        tvec_marker = tvec[marker][0]
        tvec_marker[1] -= 40
        R, _ = cv2.Rodrigues(rvec_marker)

        # Convert the marker position from camera coordinates to world coordinates
    
        marker_pos_world = compute_distance(242,141, camera_matrix, dist_coeffs, tvec_marker[2])
        marker_pos_world2 = compute_distance(68,144, camera_matrix, dist_coeffs, tvec_marker[2])
        print("marker 1: ", marker_pos_world)
        print("marker 2: ", marker_pos_world2)

        #marker_pos_camera = np.reshape(marker_pos_camera, (1, 3))
        print(tvec_marker[0])
        print(tvec_marker[1])

        marker_pos_world = "bohs"

        # Print the position of the marker in world coordinates
        print('Marker position (x, y, z):', marker_pos_world)


if __name__ == "__main__":
    fx = 160
    cx = 161
    fy = 159
    cy = 105.656

    dist_coeffs = np.array([ 0.02252689, -0.32137224, -0.00607589, -0.00284081,  0.19866972])

    

    fx = 433.44868
    fy = 939.895
    cx = 107
    cy = 318.9

    dist_coeffs = np.array([0.8333, 0.699, 0.455, 0.00159, -0.94509282], dtype=np.float32)
    
    fx = 130
    cx = 160.7
    fy = 131.9
    cy = 106

    dist_coeffs = np.array([ 0.53889391, -0.85682489, -0.01124153,  0.01029454,  0.41007014])
    
    '''
    #get_aruco_distance("bottom_camera.jpg", camera_matrix, dist_coeffs)
    
    fx = 160
    cx = 161
    fy = 159
    cy = 105.656

    dist_coeffs = np.array([ 0.02252689, -0.32137224, -0.00607589, -0.00284081,  0.19866972])
    
    fx = 141.15
    cx = 163.55
    fy = 141.5
    cy = 107.034

    dist_coeffs = np.array([ 0.32329846, -0.57971221,  0.00214554,  0.0186935 ,  0.27343211])
    '''
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    

    
    path = find_path("bottom_camera.jpg", 200, 3, 1, 2, camera_matrix, dist_coeffs)
    print(path)
