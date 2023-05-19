from PIL import Image
import heapq
from queue import PriorityQueue
import numpy as np
import cv2
# Load image
import cv2.aruco as aruco

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

    # Calculate the real-world x and y coordinates in the camera coordinate system
    x = normalized_coords[0] * z
    y = normalized_coords[1] * z

    return x, y

# assigns rover, wall or destination or neither to blocks of the segmented grid of the picture
def pixel_assignments(image_path, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients):
    
    img = cv2.imread(img_path)

    # Define the ArUco dictionary
    marker_size = 80 



    # Modify the parameters to make detection more lenient
    '''parameters.adaptiveThreshConstant = 7
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minMarkerPerimeterRate = 0.01
    parameters.maxMarkerPerimeterRate = 0.5
    parameters.minCornerDistanceRate = 0.05

    parameters.errorCorrectionRate = 0.5
    '''
    #set cv2 detection parameters
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 500
    parameters.adaptiveThreshConstant = 100
    parameters.polygonalApproxAccuracyRate = 0.1
    parameters.maxMarkerPerimeterRate = 4
    parameters.minMarkerDistanceRate = 0.05
    parameters.minOtsuStdDev =  1
    parameters.perspectiveRemovePixelPerCell = 1


    parameters.minMarkerPerimeterRate = 0.01   # Decrease the minimum perimeter rate
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE   # Skip corner refinement
    parameters.polygonalApproxAccuracyRate = 0.01   # Decrease the polygonal approximation accuracy rate

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    
    #image distortion
    gray_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (3, 3), 0)
    gray_frame = cv2.adaptiveThreshold(gray_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 71, 0.5)

    cv2.imshow("Pre-processed Image", gray_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Detect the ArUco markers in the input image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, dist_coeffs, parameters = parameters)   
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)


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
        print("wall marker ID not found")
        destination_corners = []

    # Convert the corners to pixel coordinates

    
    wall_pixels = []
    for x in range(len(wall_corners)):
        wall_pixels += get_pixels_inside_corners(wall_corners[x])



    rover_pixels = get_pixels_inside_corners(rover_corners[0])
    destination_pixels = get_pixels_inside_corners(destination_corners[0])
    # Return the pixel coordinates for each marker type
    return wall_pixels, rover_pixels, destination_pixels    




#reduces image to grid with size n x n
def compress_image(image, image_path, n, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients):
    
    # divide the image into n x n blocks and compute the colour of each block
    block_width = image.width // n
    block_height = image.height // n

    # create 2D array of pixels
    pixel_matrix = np.asarray(image)

    blocks = []
    destination_block_pixels = 0
    rover_block_pixels = 0

    #if wall is seen in block, entire block is defined as wall and exists loop of block
    break_out_flag = False
    
    #assigns pixels as walls, rover or destination
    wall_pixels, rover_pixels, destination_pixels = pixel_assignments(image_path, rover_marker_ID, wall_marker_ID, destination_marker_ID, camera_matrix, distortion_coefficients)
    
    # traverse each block in the grid, and determine which sections are walls, rover or destination
    walls = []
    for i in range(n):
        for j in range(n):
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
                            walls.append((j, i))
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

    compressed_image = np.zeros((n,n))

    #draw objects on compressed image
    for wall in walls:
        compressed_image[wall[0]][wall[1]] = 1
    
    compressed_image[rover[0]][rover[1]] = 2

    compressed_image[destination[0]][destination[1]] = 3
 
    #returns image as a 2D array of size n x n, with 1 representing walls, 2 representing rover, 3 representing destination, and 0 representing space.
    return compressed_image, rover, destination


# calculate the Manhattan distance between two points
def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

#path finding algorithm given 2D array of 
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
        for next in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
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



# finds real world distance of series of movements from rover to destination avoiding walls
def find_path(image_path, height, r_ID, w_ID, d_ID, camera_matrix, dist_coefficients,compress_factor=20):

    # open image 
    image = Image.open(image_path)
    if image.mode == 'CMYK':
        image = image.convert('RGB')
    pixels = image.load()

    walls = []
    rover = None
    destination = None

    # compress image
    compressed_image, start, dest = compress_image(image, image_path, compress_factor, r_ID, w_ID, d_ID, camera_matrix, dist_coefficients)
    
    print(compressed_image)
    # find path  
    path = astar(compressed_image, start, dest)

    # draw computed path on original image
    block_width = image.width // compress_factor
    block_height = image.height // compress_factor
    for j, i in path:
        for y in range(j*block_height,(j+1)*block_height):
            for x in range(i*block_width, (i+1)*block_width):
                pixels[x,y] = (255, 255, 0)
    
    
    # find distance away from camera of starting block - set to initial distance
    block_centre_y = (path[0][0]*block_height + (path[0][0]+1) * block_height  - 1) // 2
    block_centre_x = (path[0][1]*block_width + (path[0][1]+1) * block_width - 1) // 2
    dist_from = compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, height)

    path_distances = []

    # finds difference in real world distance of center of next block in path from previous block in path.
    for j, i in path:
        block_centre_y = (j*block_height + (j+1) * block_height  - 1) // 2
        block_centre_x = (i*block_width + (i+1) * block_width - 1) // 2

        # computes real world distance of center of block - real world distance of center of last block
        # appends to path_distances, containing all distances to travel
        path_distances.append(np.array(compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, height)) - np.array(dist_from))
        dist_from = compute_distance(block_centre_x, block_centre_y, camera_matrix, dist_coefficients, height)

    image.save('output.png')

    return path_distances



def get_aruco_distance(img_path, camera_matrix, dist_coeffs):
    
    img = cv2.imread(img_path)

    # Define the ArUco dictionary
    marker_size = 80 

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #img_blurred = cv2.GaussianBlur(img_gray, (3, 3), 0)

    _, gray_frame = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY)

    #gray_frame = cv2.adaptiveThreshold(img_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    #cv2.imshow("Pre-processed Image", gray_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Detect the ArUco markers in the input image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, dist_coeffs)   
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)



    print(np.shape(rejectedImgPoints))
    print("IDs found: ", ids)
    for marker in range(len(ids)):

        # Convert the rotation vector to a rotation matrix
        
   
        rvec_marker = rvec[marker][0]
        tvec_marker = tvec[marker][0]
        
        R, _ = cv2.Rodrigues(rvec_marker)

        # Convert the marker position from camera coordinates to world coordinates
    

        marker_pos_camera = np.array([0, 0, 0], dtype=np.float32)

        #marker_pos_camera = np.reshape(marker_pos_camera, (1, 3))

        marker_pos_world = np.linalg.inv(R) @ tvec_marker.reshape((3,1)) - np.linalg.inv(R) @ marker_pos_camera.reshape((3,1))





        # Print the position of the marker in world coordinates
        print('Marker position (x, y, z):', marker_pos_world)




print("START")
img_path = "bottom_camera.jpg"

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



fx = 135.805
cx = 157.0827
fy = 151.654
cy = 124.869

dist_coeffs = np.array([ 0.38930746, -0.62153127,  0.01333144  ,0.02126241 , 0.27916012])


camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

#distance = compute_distance(159, 61, camera_matrix, dist_coeffs, 628)

#print("distance 1: ", distance)


#distance = compute_distance(201, 114, camera_matrix, dist_coeffs, 628)

#print("distance 2: ", distance)

#get_aruco_distance(img_path, camera_matrix, dist_coeffs)
find_path(img_path, 600, 0, 1, 2, camera_matrix, dist_coeffs,compress_factor=30)
