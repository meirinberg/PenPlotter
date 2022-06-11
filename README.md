# Pen Plotter
<p align="center">
Mechanical Engineering 405 - Mechatronics <br> 
Term Project <br> 
Created by Mike Eirinberg, Ben Bradley, and Ryan Dean
</p>
<br>
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Whole Setup.jpg" width="500">
</p>

## Demonstration
A demonstration of our pen plotter can be found using the following links. The first link demonstrates all features of our pen plotter and its attempt to draw a triangle. As discussed in the video, the motors provided by our mechatronics course were old and occasionally lock up. As a result, sometimes the drawing can't be finished. However, the second video shows a successful demonstration of the plotter drawing a triangle.

Pen Plotter Demonstration and Triangle-Drawing Attempt: <br>
https://youtu.be/bnkg2Q-DLaM

Succesful Triangle-Drawing Attempt:<br>
https://youtu.be/nqa20NQLPtA


## Project Background

In mechatronics, we were tasked with creating a 2.5 degree-of-freedom non-Cartesian pen plotter with the following rules. The pen plotter: 

1. Must be able to draw something
2. Must move in two different degrees of freedom (non-Cartesian)
3. Must use stepper motors for 2 axes
4. Must run off of 24V benchtop power supplies drawing no more than 6A
5. Must be safe (generally no lasers, fire, or hazarderous systems)

## Proposal
This project uses basic polar coordinates to accomplish drawing. The following image displays our initial sketch of our project’s hardware. Even though the structure of each component has been substantially refined, the core function can still be demonstrated in the sketch. It consists of a pivot point in the ‘Angular Subsystem’ sketch serving as the origin about which all major drawing components rotate. The ‘Radial Subsystem’ is an arm that extends overhead across the drawing area and provides structure for moving the pen. At the end, there is a T-shaped support structure with a wheel on the bottom. The wheel minimizes the friction required to start rotating the radial subsystem about the angular subsystem. We decided to use a support structure, because without it there would be a moment experienced by the angular motor rotor when the pen holder would be greatly extended in the radial direction. This could ultimately make it more difficult to move the motors and add drooping to the system. The next subsystem is the pen holder. This holds and activates the pen as it, using a lead screw, moves along the radial subsystem. The pen is pressed down by a solenoid upon activation.


<p align="center">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/OriginalPenPlotterSketch.png" width="500">
</p>
<p align="center">
Figure 1. Initial sketch of pen plotter system
</p>



## First Iteration Challenges
We decided to use a 3D printer to develop the customized parts that interact with the motor. Our first prints revealed critical flaws in our design. The first iteration prints can be seen below.
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/FirstIteration.jpg" width="300">
</p>
<p align="center">
Figure 2. First iteration 3D prints with radial and angular motor
</p>
We discovered that the pen plotter's resolution would face significant issues. Upon considering the dimensions of our drawing space, we realized that the angular motor would need to have very high accuracy. This is because resolution decreases as the pen moves farther out radially with each angular change. The provided motors only have 48 steps per revolution (7.5 degrees/step). This was far too low for our design, so we found a NEMA 17 stepper motor with 200 steps (1.8 degrees/step). This was a critical design challenge, because it caused us to change most radial motor design. The radial motor had to be properly secured. However, we did not change the radial motor to a NEMA 17 because our lead screw had a pitch of 2mm. Because of this, even with 48 steps, this small pitch allowed for higher resolution radial advancement. The pitch forced the radial motor to spin a lot to move linearly along the lead screw.

## Construction and Final Design
### Mechanical Design
A CAD model of our final design can be seen below. All part files and assemblies can be found in the 'parts' folder. See the images below for more details.
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/CAD_Model.png" width="500" />
</p>
<p align="center">
Figure 3. CAD mode of final design
</p>

Our final design closely follows the core concept of our initial sketch and accounts for the different geometry of using a NEMA 17 motor in our angular subsystem. This angular system is mounted in the center of a semicircle drawing canvas, and angularly rotates the radial system at its base to sweep angularly across the drawing area. We do not make use of the entire semicircle, since part of it will be used as space for the electronics. 
<br>

Our final design still consists of a solenoid to actuate the pen as well as a set screw for finding optimal pen height. It is critical for the design of this system to minimize friction, which may prevent the pen from actuating. This is because a solenoid has a starting force when it first begins to move and a holding force when it is fully extended. The starting force is the smallest since the solenoid is just beginning to actuate and can't experience the full effect of the induced magnetic field. Providing adequate clearance and guide structures allow for the pen to be rigidly actuated under the starting force. Our angular (theta) motor’s mount has a wide base to provide as much support as possible. Thrust bearings are used between the theta and radial motor assembly to further minimize angular friction during drawing. 
<br>

Because of the shaft-support structure, we don’t have to worry about the large moment created on the theta motor when the pen is far out along the lead screw, but this also increases the moment of inertia, making it harder for the theta motor to start rotating. A big design consideration was to make the system as adjustable as possible. There are many set screws around the assembly to make it easier to adjust lengths, heights, and replace components. The rest of the system is labeled below. 


<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Mechanical Overview.png" width="500" />
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Pen Actuator.png" width="250" /> 
</p>
<p align="center">
Figure 4. Labeled components of mechanical design
</p>

### Electrical Design
Looking at the electrical components of our project, we were provided a Nucleo L476RG and a custom-made power board with a TMC4210 and TMC2208 motor driver per stepper motor. The TMC4210 interprets position, velocity, and acceleration commands, then sends step and direction signals to the TMC2208. The TMC2208 then takes this information and sends the appropriate current to the stepper motors. For each TMC2208, the voltage reference (Vref) needs to be set to create an internal current limit for the electrical components and stepper motors. We adjusted this by twisting a screw on the TMC2208 until we measured 0.7V. Datasheets for these components can be found in the folder above.

A 24V power supply is hooked up to the power board to supply power to the motors. For the solenoid, we used a solid-state relay and a separate power source, because of the solenoid's different power requirements. As seen in the image below, a joystick and LCD were also used as a user interface. As will be discussed later, these work together to select drawing files and control the free-draw feature.
<p align="center" float="left">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Electronics Overview.png" width="250" />
</p>
<p align="center">
Figure 5. Labeled components of electrical design
</p>

## Wiring Diagram
We used pinhole soldier boards for our hardware with trimmed leads. This ensured that we would not have a faulty connection, due to poor breadboard conductivity, while minimizing our resistances through smaller wires. This resulted in a wiring that never faultered. This wiring is modeled below. We communicate with our hardware using [Thonny](https://thonny.org/) connected to a laptop through a usb to micro usb connection. This powered our microcontroller, the STM32, which was the brains of our project. The STM32 had an attached "Shoe of Brian" to allow us to use micropython for this project. We utilized micropython to poll our joystick and send data to our LCD. The STM32 also controlled the pen through a solenoid that was attached to a relay. The power board contained two TMC4210s and two TMC2208s.  

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Microcontroller.JPG" align="center" width="750">
</p>
<p align="center">
Figure 6. Detailed wiring diagram
</p>

## Software Design and Implementation
Our code was structured as a finite state machine (FSM) using cooperative multitasking. We use five states, with the main four controlling the user interface, image processing, and sending motor commands in various modes. The FSM can be seen below with each state showing its task period and priority. The scheduler works by running based on frequency and priority, running the highest priority task at each call. Each state’s implementation will be discussed in further detail below.
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/State_Diagram.jpg" align="center" width="750">
</p>
<p align="center">
Figure 7. State transition diagram
</p>

### S1 – User Interface
We designed our program to include an LCD-based user interface. This took some thoughtful programming. The user can scroll through the list of options by using the joystick axes and clicking the joystick to make a selection. Using the custom LCD class that we programmed, we sent one menu item to each line of the screen. We added a ">" character to prefix one menu option as a way for the user to highlight the item they wished to select. The program uses indexing to know which choice the user has highlighted and selected. To implement the scroll look, the program chooses which items to present in the list, one for each line. If the user has reached the top or bottom of the list, the scrolling ability stops.

The first menu item is always "Free Draw," which will put the program into that mode when selected. The next menu items are loaded from a basic file explorer function we programmed. The file explorer opens the current directory and finds the files with a .hpgl extension. It then lists those HP-GL files as the next menu items. Clicking an HP-GL file will cause the program to run file-draw mode using that filename.

The blue button on the STM32 microcontroller acts as our in-print cancellation button. By clicking this button during an active print, the pen plotter will return to its starting position and display the main menu user interface. When pressed during the free-draw mode, it will only cancel the mode, but not return to any position.

### S2 – Image Processing
This state receives the file name selected in the user interface to read the appropriate HP-GL data from a file stored on the board. The image-processing state can take a long time depending on the complexity of the image. Therefore, it was given the largest period in our task manager. This state calculates all required motor position data in one go and stores it as a global variable to be accessed from the file-draw state. 

All image-processing code was derived using the [Newton Raphson](#newton-raphson) method based on [Kinematic](#Kinematics) analysis. This analysis is discussed later. A key part of image processing was using linear interpolation. Due to the requirements of the system, the motors will spin at different rates. This means that over long distances, the motors can reach their target location at different times. This warps the image, so interpolation between the data points can be used to offset the effect. We first looked at the distance between points and saw if they passed certain thresholds. Various thresholds require different degrees of interpolation, with larger distances requiring more data points. This is accomplished using the linspace() function inherent to Python. Once interpolated and with the values added back to the original set, the data could then be sent to the file-draw state.

### S3 – File Draw
Once image processing is complete, this state will index through all the motor angle data and send the commands to the power board's drivers to run the stepper motors. Once the command is sent, our code reads the current position of the motors and will only continue indexing once they have reached their desired positions. The stepper motors are initialized in ramp mode for this state, since we need to send position data.

### S4 – Free Draw
The free-draw mode allows for manual drawing using a joystick. The user can control the pen-plotter directly, drawing anything they desire. X-direction joystick motion controls the theta motor while the Y-direction joystick motion controls the radial motor. Clicking the joystick will actuate the solenoid to force the pen up or down onto the paper. The stepper motors are initialized in velocity mode for this state and a virtual threshold for the joystick was hardcoded. This means that when the joystick passes a certain point, the motors will move at constant velocity. The threshold also helps with noise, as a resting joystick without a threshold could sporadically send signals.

## Kinematics

We used the following image to determine the calculations for our system. The image is viewed from the top looking down onto the drawing board. The ${\theta_1}$ represents the angular change of the arm while ${\theta_2}$ represents the angular change of the radial motor required to move the pen in the radial direction. This analysis should align with any polar-based system. 

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/kinematicsWebsiteImage.png" width="400">
</p>
<p align="center">
Figure 8. Diagram of primary variables used in kinematic analysis
</p>

The design of our robot follows the standard polar coordinate system. The two parameters are a radius, $r$, and an angle, $\theta$. Our robot was centered at the bottom of the semicircle drawing canvas.

Two stepper motors are used to drive both these variables. Thus, the proper stepping angles must be calculated to operate the system correctly. 

Our first parameter, $\theta$, is used by the first stepper motor to move the system angularly (denoted as $\theta_1$).

The radius parameter, $r$, moves along a lead screw with a 2mm pitch. By using the second stepper motor to rotate the lead screw, we can move to our desired radius. As such, we must calculate the stepper motor rotation needed to move a desired distance along the lead screw (denoted as $\theta_2$). We know that there are 25.4mm per inch, so that implies there are 12.7 threads per inch. Furthermore, $\theta_2 \cdot \frac{2mm}{2\pi} \cdot \frac{1 in}{25.4mm} = \frac{\theta_2}{\pi 25.4 in}$. This constant, $\pi \cdot 25.4$, (denoted as $\alpha$), is used below.

Recall the trigonometric relationship between Cartesian and polar coordinates: $x = rcos(\theta_1)$ and $y = rsin(\theta_1)$

Initial Matrix:

$\begin{bmatrix}x \newline y \end{bmatrix}$= $\begin{bmatrix}  \frac{\theta_2}{\alpha} & cos(\theta_1) \newline  \frac{\theta_2}{\alpha} & sin(\theta_1)\end{bmatrix}$ 

$x = f(\theta)$

Jacobian Matrix:

$\frac{\partial f}{\partial \theta}$ = $\begin{bmatrix}  \frac{\partial f_1}{\partial \theta_1} & \frac{\partial f_1}{\partial \theta_2}\newline  \frac{\partial f_2}{\partial \theta_1} & \frac{\partial f_2}{\partial \theta_2}\end{bmatrix}$  

$\frac{\partial f_1}{\partial \theta_1} = -\frac{\theta_2}{\alpha}sin(\theta_1)$ &emsp; &emsp; $\frac{\partial f_1}{\partial \theta_2} = \frac{1}{\alpha}cos(\theta_1)$  

$\frac{\partial f_2}{\partial \theta_1} = \frac{\theta_2}{\alpha}cos(\theta_1)$ &emsp; &emsp; $\frac{\partial f_2}{\partial \theta_2} = \frac{1}{\alpha}sin(\theta_1)$

Find the velocity kinematics by differentiating x = f(θ) with respect to time:

$x = \frac{d}{dt}(f(\theta))$

$ \frac{df_1}{dt} = \frac{d}{dt}[\frac{\theta_2}{\alpha}cos(\theta_1)] = \frac{1}{\alpha}cos(\theta_1) - \frac{\theta_2}{\alpha}sin(\theta_1)$ &emsp; &emsp; &emsp; &emsp; $ \frac{df_2}{dt} = \frac{d}{dt}[\frac{\theta_2}{\alpha}sin(\theta_1)] = \frac{1}{\alpha}sin(\theta_1) + \frac{\theta_2}{\alpha}cos(\theta_1)$

$x$ = $\begin{bmatrix}  -\frac{\theta_2}{\alpha}sin(\theta_1) & \frac{1}{\alpha}cos(\theta_1)\newline  \frac{\theta_2}{\alpha}cos(\theta_1) & \frac{1}{\alpha}sin(\theta_1)\end{bmatrix}$ 
$\begin{bmatrix}  \frac{1}{\alpha}(cos(\theta_1) - \theta_2sin(\theta_1) \newline  \frac{1}{\alpha}(sin(\theta_1) + \theta_2cos(\theta_1)\end{bmatrix}$ 

$x$ = $\begin{bmatrix}  -\frac{\theta_2}{\alpha}sin(\theta_1)[\frac{1}{\alpha}(cos(\theta_1) - \theta_2sin(\theta_1)] +  \frac{1}{\alpha}cos(\theta_1[\frac{1}{\alpha}(sin(\theta_1)+\theta_2cos(\theta_1)]\newline  \frac{\theta_2}{\alpha}cos(\theta_1)[\frac{1}{\alpha}(cos(\theta_1) -\theta_2sin(\theta_1)] +  \frac{1}{\alpha}sin(\theta_1)[\frac{1}{\alpha}(sin(\theta_1)+\theta_2cos(\theta_1)]\end{bmatrix}$ 


Inverse Kinematics:

$y = x - f(\theta) = g(\theta)$

$\frac{\partial g(\theta))}{\partial \theta} = \frac{\partial}{\partial \theta}[0-f(\theta)]$

$\frac{\partial g(\theta))}{\partial \theta} = -\frac{\partial}{\partial \theta}f(\theta)$

## Newton Raphson

The Newton Raphson method was used to find the proper motor angular position data to produce desired rectilinear coordinates. This method is typically used to find the roots of a function. Rather than use the function directly derived from our kinematic analysis, we needed to slightly modify our position matrices to allow for solving for specific points. This is done by taking our kinematic analysis output and subtracting it from the desired position. This function can then have its roots solved for where each root corresponds to the desired and actual motor positions converging. The output results in angular data we can send to the motors. More information on this method can be found [here](https://en.wikipedia.org/wiki/Newton%27s_method). 

Once the image was processed, we verified our analysis by plotting motor angular data and final plotted positions. As can be seen below, there are no discontinuities in the outputs and our final plot matches the desired triangular image exactly. A GIF was also constructed to show the plotter’s motion as it plots, with the blue line representing the arm of the system and the red lines representing the drawn pen lines. The final image shows the triangle that our pen plotter was able to accomplish.

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Radial Motor Desired Position Plot.png" width="300">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Theta Motor Desired Position Plot.png" width="300">
</p>
<p align="center">
Figure 9. Plots of radial (left) and angular (right) motor output position
</p>
  
<p align="center" float="left">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Triangle Desired Position Plot.png" width="300">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle.gif" width="320">
</p>
<p align="center">
Figure 10. Desired pen plotter plots
</p>

## Results and Final Thoughts
Our project was successful in creating a functional pen plotter. The triangle, as discussed in the Newton Raphson section above, can be seen below. Clearly, the lines are not as smooth as we had hoped but would improve with more interpolated data points. However, the provided Nucleo does not have a large amount of storage available, so it can be difficult to find the right balance of image quality. 
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/TriangleDraw.jpg" align="center" width="500">
</p>
<p align="center">
Figure 11. Pen plotter output from triangle input image
</p>
Unfortunately for the free drawing state, there is some delay between an input from the joystick and motion of the motor. This can be mostly attributed to the added weight of all mechanical components. It can be difficult to angularly accelerate a mass with a high moment of inertia, leading to a delayed start and stop reaction time. A future design could involve optimizing the weight of the radial arm and eliminating the shaft support. Finally, the biggest issue we kept encountering was the reliability of the provided motors. We found that after some time, the motors would lock up and refuse to rotate, only to start rotating when trying moments later. This made it difficult in the beginning to know if there was a software or hardware issue, so we recommend that anyone starting a project like this ensure their motors can handle the required load.


## Component Walkthrough

### Microcontroller

We used the STM32L476RG as our microcontroller. The microcontroller has 128KB ram and 1Mb flash. We are able to fully control our system with our microcontroller. We used micropython as our language of choice for this project with the Shoe of Brian attachment, in order to interface native C in the STM32 to micropython. The Shoe of Brian allows us to "flash" our micropython code to the STM32 board through a simple drag-and-drop on our personal computer.

### Stepper Motors

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/StepperMotor.JPG" align="right" width="250">
</p>
<p align="right">
Figure 12. Stepper motor wiring configuration
</p>

We used two 4-wire stepper motors with internals (shown below). By inducing a current, we were able to rotate our arm motor 48 individual steps for our radial stepper motor and 200 steps for our theta stepper motor. This allowed us to make fairly precise drawings, but did come with some troubles. We destroyed three 48-step motors, while our 200-step NEMA 17 motor survived. The 48-step motor tended to produce results that varied despite the code being the same at times. This repeatedly turned out to be a motor issue and not a code issue. We would recommend purchasing reliable stepper motors to save time and expense in future projects.

### Solenoid and Relay

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Solenoid.JPG" align="right" width="150">
</p>

We utilized a non-polar solenoid to control our pen-up and pen-down motions. We mounted the [solenoid to a rail](https://github.com/meirinberg/PenPlotter/blob/main/README.md#mechanical-design), so we were able to do pen-up and pen-down motions in any position in our radial and theta axes. We connected our solenoid to a relay, which worked flawlessly.

### Power Supply 

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Power_Board.png" align="right" width="250">
</p>

We used a power board to control our stepper motors from one SPI configuration. We were able to do this by manually enabling which stepper was active, toggling the corresponding chip select. Additionally, our power board has an input of a 24V rail and supplies 12V to each motor. This aided in our wiring. The power board has two TMC4210 chips and two TMC2208 chips. TMC4210s were used for enabling and calculating TMC2208 commands. The TMC2208 chips are used to send the calculated commands directly to each motor. For more information on setting the TMC2208s current limit, [look here](https://wiki.fysetc.com/TMC2208/).

### LCD

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/LCD .jpg" align="right" width="250">
</p>

The LCD board is configured in 4-bit mode. The LCD class was written manually, by using delays and bit-banging pins. Initially, we thought the use of delays would slow down our pen plotter's other functions. However, we were able to reduce the delay to 0.0005 seconds, which made the delay negligable. Unfortunately, 4-bit mode requires more wiring connections than an LCD with an I2C bus module. To reduce the large wiring footprint without an I2C configuration, we soldered our pin connections to a solder-board. This reduction in size allowed the LCD to fit within the 3D-printed housing we designed for it. We attached a potentiometer to the LCD's Vo pin to enable the user to manually adjust the contrast of the LCD.


### Joystick

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/JoyStick.JPG" align="right" height="250">
</p>

The joystick employs two ADCs to read the X and Y axes from the joystick potentiometer. We used the ADC code that we built in our first mechatronics lab. We used 3.3V to operate the joystick, even though it is rated for 5V. We found that operation at 3.3V was optimal because that voltage matched the internal reference voltage of the microcontroller's ADC. This allows for smooth movement tracking with the joystick. However, we did remove noisy inaccuracies at the joystick's center position by introducing a threshold. We configured three GPIO pins as inputs, two for the ADCs, and one for the joystick's button. We flipped some of the axes through programming, because our 3D-printed housing better accommodated the joystick upside down.
