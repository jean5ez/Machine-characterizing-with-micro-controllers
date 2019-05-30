# Machine-characterizing-with-micro-controllers
Machine learning does not have to mean you have to use neural networks.  If you could have solved the issue with a few multi-meters and a 
notepad, chances are that a micro-controller can do it too.  
Least square method maths does somewhat resemble neural network maths though, but it doesn't require more than a second to train.  
Luckily we also already know how many parameters go into a DC machine state equation.  It is still cool to think that there is less work 
needed in implementing motor control, just because the micro-controller can do a lot of the hard work for you.
It is not heavy in ram or rom, so you can shove it in your bigger control task somewhere.
It is not in a library yet.
It is in the middle of a simulation.

The simulation is that of a DC Motor.
The characteristics of the DC Motor is adjusted by adjusting the constants in the start part of the c++ file.

Then there is an inner part where the machine characteristics are trasnfered to variables where the control part can get them.  
These are speed and current.  
These are suppose to be measurements taken by the micro controller.

The thing that "learns" is right in the middle.  I will add parts of or reference to the least square method from my text book to help you
get what is happening there, but it is just matrices being inverted, transposed and multiplied, etc.  Very quickly because they are small 
matrices, because a DC machine is not that complicated.

A very interesting thing is happening right there and that is that you find yourself in a sort of unitless world.  The measurements samples 
are just added together and the maths just sorts it out.  This is also makes it good for using it anywhere.  
The model is then used to find what voltage to apply in order to achieve a certain speed in the future.  (One sample ahead in the simulation.)
Speed control works wonderfully.
I didn't spend much thinking on torque control.  Maybe just current limit.

What is maybe not so smart is that when applying the model to predict a sample ahead and provide an output voltage to have position control,
something else must go in between: Speed profile.

What must the speed be at what position to eventually reach a prefered position and have a speed of zero when you do.  The need for this is
unfortunate.  But not to tough on whoever wants to use it.

At least you don't have to fight overshoot and steady state errors, and you don't have to worry about using your application on any DC machine
that you want to.

And that is it.

So it is the part in the middle and all the matrix functions you want to take out and put in your PIC, or Arduino.  Put it in a function that 
returns a future voltage value for a given speed and current measurement and a required speed in the future.  I will assist in that later.
