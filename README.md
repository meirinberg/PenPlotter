# Pen Plotter
<p align="center">
Mechanical Engineering 405 - Mechatronics <br> 
Term Project <br> 
Created by Mike Eirinberg, Ben Bradley, and Ryan Dean.
</p>
<br>
<p align="center">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Whole Setup.jpg" width="600">
</p>

## Project Background

In ME 405 we were tasked with creating a 2.5 degree-of-freedom non-Cartesian pen plotter with the following rules:

1. The project must be able to draw something
2. The device must move in two different degrees of freedom (non-Cartesian)
3. Must use stepper motors for 2 axes
4. Must run off of 24V benchtop power supplies drawing no more than 6A
5. Must be safe (generally no lasers, fire, or hazarderous systems)

## Proposal
The following image displays our initial sketch of our project’s hardware. Even though the structure of each component has been substantially refined, the core function can still be demonstrated in the sketch. It consists of a pivot point in the ‘Angular Subsystem’ sketch serving as the origin about which all major drawing components rotate around. The ‘Radial Subsystem’ is an arm that extends overhead across the drawing area. At the end is a T-shaped support structure with a wheel on the bottom. This minimizes the friction required to start rotating the radial subsystem about the angular subsystem. The next subsystem is the pen holder. This holds and activates the pen as well as moves along the radial subsystem using a lead screw. The pen is envisioned to be pressed down by a solenoid upon activation.
<p align="center">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/OriginalPenPlotterSketch.png" width="600">
</p>

## Construction and Final Design
Our device consists of a solenoid that moves linearly along a long arm. It uses polar coordinates to operate. This system had critical design features to minimize the negative effects of the pen-actuator payload. Our “theta” motor’s mount is bulky, a lead screw-arm with an additional support rod, and a wheel for the end of the arm to reduce downward weight. This arm is mounted to one corner of the drawing canvas, and rotates angularly at its base to sweep angularly across the page. For our radial axis, we use a stepper motor connected to a lead screw to move the pen actuator forward and backward. The pen actuator is made of a solenoid.

Our theta stepper motor controls our angle and our radius stepper motor controls how far out the pen is from the pivot. Our solenoid makes our pen move up and down.

<p align="center" float="left">
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Mechanical Overview.png" width="300" />
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Electronics Overview.png" width="300" />
  <img src="https://github.com/meirinberg/PenPlotter/blob/main/images/06042022 Pictures/Pen Actuator.png" width="300" /> 
</p>

## Wiring Diagram - high level


## Code Structure and Implementation
Our term project combines two operating modes, “Free Draw” and “File Draw”. 

The File Draw mode operates the pen plotter to draw a vector image loaded onto our microcontroller (as the term project requires). 

The Free Draw mode allows for manual drawing using a joystick. The user can control the pen-plotter directly, drawing anything they desire. Clicking the joystick down will make the pen touch the paper for drawing. Clicking the joystick again will lift the pen. The movement of the joystick will control the movement of the pen-plotter. We plan to switch between the two modes using a programmatic method, or by using a real hardware switch.

## Kinematics

We used the following to determine the calculations for our system (shown below). This analysis should align with any polar-based system. 

The design of our robot follows the standard polar coordinate system. The two parameters are a radius, $r$, and an angle, $\theta$. Our robot will be mounted at the bottom left corner of the drawing paper.

Two stepper motors will be used to drive both these variables. Thus, the proper stepping angles must be calculated to operate the system correctly. 

Our first parameter, $\theta$, will be used by the first stepper motor to move the system angularly (denoted as $\theta_1$).

The radius parameter, $r$, moves along a lead screw with a 2mm pitch. By using the second stepper motor to rotate the lead screw, we can move to our desired radius. As such, we must calculate the stepper motor rotation needed to move a desired distance along the lead screw (denoted as $\theta_2$). We know that there are 25.4mm per inch, so that implies there are 12.7 threads per inch. Furthermore, $\theta_2 \cdot \frac{2mm}{2\pi} \cdot \frac{1 in}{25.4mm} = \frac{\theta_2}{\pi 25.4 in}$. This constant, $\pi \cdot 25.4$, (denoted as $\alpha$), is used below.

Recall the trigonometric relationship between cartesian and polar coordinates: $x = rcos(\theta_1)$ and $y = rsin(\theta_1)$

Initial Matrix:
$\begin{bmatrix}x \\ y \end{bmatrix}$= $\begin{bmatrix}  \frac{\theta_2}{\alpha} & cos(\theta_1) \\  \frac{\theta_2}{\alpha} & sin(\theta_1)\end{bmatrix}$ 

$x = f(\theta)$

Jacobian Matrix:
$\frac{\partial f}{\partial \theta}$ = $\begin{bmatrix}  \frac{\partial f_1}{\partial \theta_1} & \frac{\partial f_1}{\partial \theta_2}\\  \frac{\partial f_2}{\partial \theta_1} & \frac{\partial f_2}{\partial \theta_2}\end{bmatrix}$

$\frac{\partial f_1}{\partial \theta_1} = -\frac{\theta_2}{\alpha}sin(\theta_1)$

$\frac{\partial f_1}{\partial \theta_2} = \frac{1}{\alpha}cos(\theta_1)$

$\frac{\partial f_2}{\partial \theta_1} = \frac{\theta_2}{\alpha}cos(\theta_1)$

$\frac{\partial f_2}{\partial \theta_2} = \frac{1}{\alpha}sin(\theta_1)$

Find the velocity kinematics by differentiating x = f(θ) with respect to time:

$x = \frac{d}{dt}(f(\theta))$

$ \frac{df_1}{dt} = \frac{d}{dt}[\frac{\theta_2}{\alpha}cos(\theta_1)] = \frac{1}{\alpha}cos(\theta_1) - \frac{\theta_2}{\alpha}sin(\theta_1)$

$ \frac{df_2}{dt} = \frac{d}{dt}[\frac{\theta_2}{\alpha}sin(\theta_1)] = \frac{1}{\alpha}sin(\theta_1) + \frac{\theta_2}{\alpha}cos(\theta_1)$

$x$ = $\begin{bmatrix}  -\frac{\theta_2}{\alpha}sin(\theta_1) & \frac{1}{\alpha}cos(\theta_1)\\  \frac{\theta_2}{\alpha}cos(\theta_1) & \frac{1{\alpha}sin(\theta_1)\end{bmatrix}$ $\begin{bmatrix}  \frac{1}{\alpha}(cos(\theta_1) - \theta_2sin(\theta_1)\\  \frac{1}{\alpha}(sin(\theta_1) + \theta_2cos(\theta_1)\end{bmatrix}$ 

$x$ = $\begin{bmatrix}  -\frac{\theta_2}{\alpha}sin(\theta_1)[\frac{1}{\alpha}(cos(\theta_1) - \theta_2sin(\theta_1)] +  \frac{1}{\alpha}cos(\theta_1[\frac{1}{\alpha}(sin(\theta_1)+\theta_2cos(\theta_1)]\\  \frac{\theta_2}{\alpha}cos(\theta_1)[\frac{1}{\alpha}(cos(\theta_1) -\theta_2sin(\theta_1)] +  \frac{1}{\alpha}sin(\theta_1)[\frac{1}{\alpha}(sin(\theta_1)+\theta_2cos(\theta_1)]\end{bmatrix}$ 


Inverse Kinematics:
$y = x - f(\theta) = g(\theta)$

$\frac{\partial g(\theta))}{\partial \theta} = \frac{\partial}{\partial \theta}[0-f(\theta)]$

$\frac{\partial g(\theta))}{\partial \theta} = -\frac{\partial}{\partial \theta}f(\theta)$

## Newton Raphson

Newton Raphson is a method to find roots to a mechanical system. We used the [Kinematics](##Kinematics) to derive our equations for the radius and theta of our system. We then took the derivative of our main function and used a method of successive approximation to come up with a solution within a specified threshold. This solution is graphed and showed visually below.

<p align="center">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Newton_Ralphson.gif" align="center" width="300">
</p>
  
<p align="center" float="left">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Radial Motor Desired Position Plot.png" width="300">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Theta Motor Desired Position Plot.png" width="300">
<img src="https://github.com/meirinberg/PenPlotter/blob/main/images/Triangle Plotting/Triangle Desired Position Plot.png" width="300">
</p>

A video of our working system is shown below.

[Video demo of our system](youtube.com)

## Components


Our board is the surface our program draws on. It is made out of wood.

Our lead screw moves < 2mm I think? > mm per lead. Used for slow but accurate movements.

We used the STM32 as our microcontroller. The Microcontroller has < x ram > and < x storage >. We are able to fully control our system with our microcontroller. We used micropython as our language of choice for this project.


The shoe of brian is neccessary as an adaptor to utalize micropython for our project. This is due to limittation present on our stm32 hardware. 


Used for tmc2208s and tmc4210s, which control our stepper motors in this project. Contains all of our power connections except for the solonoid due to hardware limitations. 


Used to show the user real time location info of where our pen is.


Used for user controlled movement. Utalizes velocity mode as in practice it is easier to draw by hand with.



## Component Walkthrough
### Microcontroller
### Stepper Motors
### Solenoid
### Power Supply and Relays
### LCD
### Joystick


