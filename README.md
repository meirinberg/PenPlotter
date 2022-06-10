# Pen Plotter
<p align="center">
Mechanical Engineering 405 - Mechatronics <br> 
Term Project <br> 
Created by Mike Eirinberg, Ben Bradley, and Ryan Dean.
</p>
<br>
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Whole Setup.jpg" width="500">
</p>

## Demonstration
A demonstration of our pen plotter can be found using the following links. The first link demonstrates all features of our pen plotter and attempts to draw a triangle. As discussed in the video, the class provided motors are old and lock up occasionally and sometimes the drawing can't be finished. However, the second video shows a complete demonstration of the plotter drawing a triangle.

Pen Plotter Demonstration and Triangle Drawing Attempt: <br>
https://youtu.be/bnkg2Q-DLaM

Complete Triangle Drawing Attempt:<br>
https://youtu.be/nqa20NQLPtA


## Project Background

In ME 405 we were tasked with creating a 2.5 degree-of-freedom non-Cartesian pen plotter with the following rules:

1. The project must be able to draw something
2. The device must move in two different degrees of freedom (non-Cartesian)
3. Must use stepper motors for 2 axes
4. Must run off of 24V benchtop power supplies drawing no more than 6A
5. Must be safe (generally no lasers, fire, or hazarderous systems)

## Proposal
This project uses basic polar coordinates to accomplish drawing.The following image displays our initial sketch of our project’s hardware. Even though the structure of each component has been substantially refined, the core function can still be demonstrated in the sketch. It consists of a pivot point in the ‘Angular Subsystem’ sketch serving as the origin about which all major drawing components rotate around. The ‘Radial Subsystem’ is an arm that extends overhead across the drawing area and will provide structure for moving the pen. At the end is a T-shaped support structure with a wheel on the bottom. This minimizes the friction required to start rotating the radial subsystem about the angular subsystem. The next subsystem is the pen holder. This holds and activates the pen as well as moves along the radial subsystem using a lead screw. The pen is envisioned to be pressed down by a solenoid upon activation.
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/OriginalPenPlotterSketch.png" width="500">
</p>

## First Iteration Challenges
3D printing was the obvious first choice for the more customized parts that need to interact with the motor. Our first prints revealed critical information on the flaws in our design. The first iteration prints can be seen below with the angular and radial motors.
[Image of first green print]
Our discovered problem was a matter of resolution. Upon considering the dimensions of our drawing space, we realized that the angular motor would need to have very high resolution. This is because resolution decreases as the pen moves farther out radially with each angular change. The provided motors only have 48 steps per revolution (7.5 degrees/step). This was far too low for our design so we found a NEMA 17 stepper motor with 200 steps (1.8 degrees/step). This was a critical design challenge because it changed most of our design in how the radial motor was properly secured. However, the radial motor was not changed to a NEMA 17 because our lead screw had a pitch of 2mm. Even with 48 steps this small pitch allows for higher resolution radial advancement.

## Construction and Final Design
### Mechanical Design
Our final design closely follows the core concept of our initial sketch and accounts for the different geometry of using a NEMA 17 motor in our angular subsystem. This angular system is mounted in the center of a half circle drawing canvas, and angularly rotates the radial system at its base to sweep angularly across the drawing area. We do not make use of the entire half circle, since part of it will be used as space for the electronics. 
<br>

Our final design still consists of a solenoid to actuate the pen. It was critical for the design of this system to minimize friction preventing the pen from actuating. This is because a solenoid has a starting force and holding force, with the starting force being the smallest since the solenoid is just beginning to actuate. Providing adequate clearance and guide structures allow for the pen to be rigidly actuated under the starting force. Our angular (theta) motor’s mount has a wide base to provide as much support as possible. Thrust bearings are used between the theta and radial motor assembly to further minimize angular friction during drawing. 
<br>

Because of the shaft support structure, we don’t have to worry about the large moment created on the theta motor when the pen is far out along the lead screw, but this also increases the moment of inertia, making it harder for the theta motor to start rotating. A big design consideration was to make the system as adjustable as possible. There are many set screws around the assembly to make it easier to adjust lengths, heights, and replace components. The rest of the system is labeled below. 


<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Mechanical Overview.png" width="500" />
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Pen Actuator.png" width="250" /> 
</p>

### Electrical Design
Looking at the electrical components of our project, we were provided a Nucleo L476RG and a custom-made power board with a TMC4210 and TMC2208 motor driver per stepper motor. Datasheets for these components can be found a the folder above. A 24V power supply is hooked up to the power board to supply power to the motors. For the solenoid, we used a solid-state relay and a separate power source because of its different power requirements. As seen in the image below, a joystick and LCD were also used as a user interface. As will be discussed later, these work together to select drawing files and control the free draw feature.
<p align="center" float="left">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Electronics Overview.png" width="250" />
</p>

## Wiring Diagram
We used pinhole soldier boards for our hardware with trimmed leads. This ensured that we would not have a faultly connect due to poor breadboard conductivity while minimizing our resistances through smaller wires. This resulted in a wiring that never faultered. This wiring is modeled below. We interfaces with our hardware using [Thonny](https://thonny.org/) connected to a laptop through a usb to micro usb connection. This powered our microcontroller, the STM32, which was the brains for our project. The STM32 had an attached Shoe of Brian to allow us to use micropython for this project. We utalized micropython to poll our Joy Stick and send data to our LCD. The STM32 also controlled the pen being up or down through a Solenoid that was attached to a relay and interfaces with our power board. The Power Board contained two TMC4210s and two TMC2208s. The TMC4210s controlled our system being on and the TMC2208s interfaced with our stepper motors as well as limitted their current draw to 0.8 amps in order to prevent the components from getting too hot and get damaged. 

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Microcontroller.JPG" align="center" width="750">
</p>

## Software Design and Implementation
Our code was structured as a finite state machine (FSM) using cooperative multitasking. We use five states, with the main four controlling the user interface, image processing, and sending motor commands in various modes. The FSM can be seen below with each state showing its task period and priority. The scheduler works by running based on priority, finding the highest priority task which is ready to run and calls it. Each state’s implementation will be discussed in further detail below.
<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/State_Diagram.jpg" align="center" width="750">
</p>

### S1 – User Interface
We designed our program to include a LCD-based user interface. This took some thoughtful programming. The user can scroll through the list of options using the joystick axes and click the joystick to make a selection. Using the custom LCD class we wrote, we sent one menu item to each line of the screen. We added a ">" character to prefix one menu option as a way for the user to highlight which item they wished to select.  The program uses indexing to know which choice they have highlighted and selected. To implement the scroll look, the program chooses which items to present in the list, one for each line. If the user has reached the top or bottom of the list, the scrolling ability stops.

The first menu item is always "Free Draw" which will put the program into that mode when selected. The next menu items are loaded from a basic file explorer function we programmed. The file explorer opens the current directory and finds the files with a .hpgl extension. It then lists those HP-GL files as the next menu items. Clicking an HP-GL file will cause the program to run "File Draw" mode using that filename.

The blue button on the STM32 microcontroller acts as our in-print cancellation button. By clicking this button during an active print, the pen plotter will home back to its starting position and return to the main menu user interface. When pressed during the free draw mode, it will only cancel the mode, but not home back to any position.

### S2 – Image Processing
This state receives the file name selected in the user interface to read the appropriate HPGL data from a file stored on the board. The image processing state can take a long time depending on the complexity of the image. Therefore, it was given the largest period in our task manager. This state calculates all required motor position data in one go and stores it as a global variable to be accessed from the File Draw state. 

All image processing code was derived using the [Newton Raphson](##Newton_Raphson) method based on [Kinematic](##Kinematics) analysis. These sections are discussed later. 

### S3 – File Draw
Once image processing is complete, this state will index through each motor angle data and send the commands to the power boards drivers to run the stepper motors. Once the command is sent, our code reads the current position of the motors and will only continue indexing once they have reached their desired positions. The stepper motors are initialized in ramp mode for this state since we need to send position data.

### S4 – Free Draw
The Free Draw mode allows for manual drawing using a joystick. The user can control the pen-plotter directly, drawing anything they desire. X-direction joystick motion controls the theta motor while the Y-direction joystick motion controls the radial motor. Clicking the joystick will actuate the solenoid to force the pen up or down onto the paper. The stepper motors are initialized in velocity mode for this state and virtual threshold for the joystick was hardcoded. This means that when the joystick passes a certain point, the motors will move at constant velocity. The threshold also helps with noise, as a resting joystick can sporadically send signals.

## Kinematics

We used the following to determine the calculations for our system (shown below). This analysis should align with any polar-based system. 

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/kinematicsWebsiteImage.png" width="400">
</p>

The design of our robot follows the standard polar coordinate system. The two parameters are a radius, $r$, and an angle, $\theta$. Our robot will be mounted at the bottom left corner of the drawing paper.

Two stepper motors will be used to drive both these variables. Thus, the proper stepping angles must be calculated to operate the system correctly. 

Our first parameter, $\theta$, will be used by the first stepper motor to move the system angularly (denoted as $\theta_1$).

The radius parameter, $r$, moves along a lead screw with a 2mm pitch. By using the second stepper motor to rotate the lead screw, we can move to our desired radius. As such, we must calculate the stepper motor rotation needed to move a desired distance along the lead screw (denoted as $\theta_2$). We know that there are 25.4mm per inch, so that implies there are 12.7 threads per inch. Furthermore, $\theta_2 \cdot \frac{2mm}{2\pi} \cdot \frac{1 in}{25.4mm} = \frac{\theta_2}{\pi 25.4 in}$. This constant, $\pi \cdot 25.4$, (denoted as $\alpha$), is used below.

Recall the trigonometric relationship between cartesian and polar coordinates: $x = rcos(\theta_1)$ and $y = rsin(\theta_1)$

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

The Newton Raphson method was used to find the proper motor angular position data to produce desired rectilinear coordinates. This method is typically used to find the roots of a function. Rather than use the function directly derives from our kinematic analysis, we needed to slightly modify our function to allow for solving for specific points. This is done by taking our kinematic analysis output and subtracting it from the desired position. This function can then have its roots solved for where each root corresponds to the desired and actual motor positions converging. The output is then the motor angular data we can send to the motors. More information on this method can be found [here](https://en.wikipedia.org/wiki/Newton%27s_method). 

Once the image was processed, we verified our analysis by plotting motor angular data and final plotted positions. As can be seen below, there are no discontinuities in the outputs and our final plot matches the desired triangular image exactly. A GIF was also constructed to show the plotter’s motion as it plots, with the blue line representing the arm of the system, and the red lines presenting the drawn pen lines.

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle.gif" align="center" width="500">
</p>
  
<p align="center" float="left">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Radial Motor Desired Position Plot.png" width="280">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Theta Motor Desired Position Plot.png" width="280">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Triangle Desired Position Plot.png" width="280">
</p>

## Component Walkthrough

Our board is the surface our program draws on. It is made out of wood.

We used the STM32 as our microcontroller. The Microcontroller has < x ram > and < x storage >. We are able to fully control our system with our microcontroller. We used micropython as our language of choice for this project. 


The shoe of brian is neccessary as an adaptor to utilize micropython for our project. This is due to limittation present on our stm32 hardware. 


Used for tmc2208s and tmc4210s, which control our stepper motors in this project. Contains all of our power connections except for the solonoid due to hardware limitations. 


Used to show the user real time location info of where our pen is.


Used for user controlled movement. Utilizes velocity mode as in practice it is easier to draw by hand with.

### Microcontroller

### Stepper Motors

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/StepperMotor.JPG" align="right" width="250">
</p>

We had two 4-wire stepper motors with internals shown below. By inducing a current we were able to rotate our arm motor 48 individual steps for our radial stepper motor and 200 steps for our theta stepper motor. This allowed us to make fairly precise drawings, but did come with some troubles. We went through three 48 stepper motors while our 200 stepper did not die at all. Our motor tended to preduces results that varied despite the code being the same at times. This turned out to be a motor issue, repeatably, and not a code issue. We would recommend picking up reliably steppers in order to save yourself a headache for future projects.

### Solenoid and Relay

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Solenoid.JPG" align="right" width="150">
</p>

We utalized a non-polar solenoid to control our pen up and pen down motion. We mounted the [solenoid to a rail](https://github.com/meirinberg/PenPlotter/blob/main/README.md#mechanical-design), so we were able to do pen up and pen down motions in any position in our radial and theta axis. We connected our Solenoid to a relay, which was a little overkill for the project but worked flawlessly. We used this relay because it happened to be laying around and was more easily mounted as we did not have another pin board on hand. Without a relay one could use another switch mechanism such as a mosfet or bjt to control when the solenoid turns on (pen down) or off (pen up).

### Power Supply 

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Power_Board.png" align="right" width="250">
</p>

We used a power board in order to control our stepper motors from one spi signal. We were able to do this by manually enabling which stepper was on as well as sending commands to the corresponding chip select. Additionally, our power board had an input of a 24V rail and supplied 12V to each motor, which aided in our wiring. The Power Board had two TMC4210 chips and two TMC2208 chips. TMC4210s were used for enabling the TMC2208s as well as translating the spi for the TMC2208. The TMC2208 chips were used to control maximum current from the power rail, approximately 0.8 amps, as well as being the main driver we wrote code for. For more info on setting the TMC2208s current limit [look here](https://wiki.fysetc.com/TMC2208/).

### LCD

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/LCD .jpg" align="right" width="250">
</p>

The LCD board is configured in 4-bit mode. The LCD class was written manually by using delays and bit-banging pins. Initially, we thought the use of delays would slow down our pen plotter's other functions. However, we were able to reduce the delay to 0.0005 seconds which proved to be so fast that the delays were negligable. Unfortunately, 4-bit requires more wires than that of an I2C configured LCD. To tackle the large wiring footprint, we soldered our pin connections to a solder-board. This reduction in size allowed the LCD to fit within the 3D-printed housing we designed for it. We attached a potentiometer for the ability to manually adjust the contrast of the LCD.


### Joystick

<p align="center">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/JoyStick.JPG" align="right" height="250">
</p>

The joystick employs two ADCs to read the X and Y axes from the joystick potentiometer. We used the ADC code that we built in our first mechatronics lab. We used 3.3V to operate the joystick even though it is rated for 5V. We found that operation at 3.3V was optimal becuase that voltage matched the internal reference voltage of the microcontroller's ADC. This allowed for smooth movement tracking with the joystick. However, we did remove noisy inaccuracies at the joystick's center position by introducing a threshold. We configured three GPIO pins as inputs, two for the ADCs and one for the joystick's button. We flipped some of the axes via programming because our 3D-printed housing prefered the joystick oriention to have the wires facing the top.
