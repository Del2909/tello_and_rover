# Symbiotic Power and Data Sharing: DJI Tello Drone and Rover

This project is designed to create a symbiotic relationship between a [DJI Tello Edu drone](https://store.dji.com/uk/product/tello-edu)(The Tello EDU has more access to the SDK (software Developers Kit), than the consumer range Tello, for more advanced coding features) and a custom rover. The Tello drone captures aerial images to compute an optimal path for the rover, while the rover not only performs path computation and completion tasks but also serves as a charging platform for the drone. This integration is made possible through the development of unique features like path computation and autonomous landing algorithms. 

## Demo Video

To see our project in action, watch our demo video on YouTube:

[![Demo Video](http://img.youtube.com/vi/U3Ds2H3q8cQ/0.jpg)](http://www.youtube.com/watch?v=U3Ds2H3q8cQ "Project Demo")


## Features

- **Aerial Image Capture:** The DJI Tello drone is equipped to capture high-resolution aerial images for path computation.
- **Optimal Path Computation:** Our custom algorithm processes the aerial images to compute the most efficient path for the rover.
- **Autonomous Operation:** The rover has been designed to autonomously navigate the computed path.
- **Charging Platform:** The rover features a built-in charging platform compatible with the DJI Tello drone.
- **Autonomous Landing:** Post aerial image capture, the drone autonomously lands on the rover for recharging.

## Setup and Execution

Follow the steps below to set up a virtual environment and install necessary dependencies(Note: most libraries used on Raspberry Pi 4 (4GB ram) so some libraries might not install for mac or windows):

1. First, ensure that you have Python installed on your system. We recommend using Python 3.6 or above. You can download the latest version of Python from the [official website](https://www.python.org/downloads/).

2. Install `virtualenv` to create a virtual environment. You can install it using pip:

    ```
    pip install virtualenv
    ```

3. Create a new virtual environment:

    ```
    python -m venv roverenv 
    ```

4. Activate the virtual environment:

    - On Windows, run:

        ```
        .\venv\Scripts\activate
        ```

    - On Unix or MacOS, run:

        ```
        source roverenv/bin/activate
        ```

5. Install the necessary packages using the `submissionrequirements.txt` file:

    ```
    pip install -r submissionrequirements.txt
    ```

After setting up the virtual environment and installing the necessary packages, you can run the Python script:

1. Navigate to the `Pi/Tello_execution` directory:

    ```
    cd Pi/Tello_execution
    ```

2. Run the `main.py` script:

    ```
    python3 main.py
    ```

Remember to deactivate the virtual environment after you're done:


## Critical Notes
Please note that the DJI Tello drone requires at least 40 percent battery for one round of autonomous takeoff, path computation, and landing. Ensure sufficient battery before initiating the operation.

We are continuously working to improve the efficiency and functionality of our system. Feel free to report any issues or contribute to our project.

