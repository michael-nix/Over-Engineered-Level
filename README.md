# Over Engineering a Level
Or, the unreasonable effectiveness of low pass filters?

**Michael Nix**, Toronto, Canada, 2026

---

Did you know: humans walk in a completely different manner than airplanes fly?  Did you know: the environment within which humans walk is completely different from the environment within which airplanes fly?  Frankly, I don't like to think too much about it.  Parachute joke.

A few months ago I found these cool little project kits from [Freenove](https://github.com/freenove), and since I have experience with Espressif Systems products, I thought it would be neat to play around with one (I bought the ultimate one for like, fifty bucks off of Amazon, pretty good deal); maybe use it as a proving ground for some of my state estimation ideas, or even as something of a portfolio for professional work.  So I wire up the IMU with some LEDs, getting them to go green when the IMU is pointing down.  A good first step.  Then I was thinking about how to add more and more math stuff in to it; however, everything that involves linear algebra requires some really boring math code.  I looked around for some decent C linear algebra libraries but it was very boring work.  *Then* I remembered that if you keep everything in three dimensions, you can do all the linear algebra by hand.  Put on some tunes, code some terrible functions, and the compiler will probably take care of performance for you.

Which brings us back to humans walking and planes flying.  A while ago I was doing some work analysing how humans move; specifically while moving their arms.  That is, with an IMU strapped to their wrist.  It turns out that waving your arms around requires a pretty hefty amount of acceleration, even when walking.  However, when using acceleration measurements to estimate things, using the force of gravity as a reference for, "down," is a pretty good idea, unless you're distorting it with $\pm 2 g$ of arm waving.  If you were flying a plane way up in the sky, you could ignore acceleration measurements when you know they're wonky, instead using a reference like magnetic north to help out.  But if your IMU doesn't have a magnetometer, or you're inside next to a bunch of electromagnetic interference, you're gonna have some issues.  

So can we adapt normal tools to this kind of situation?

Given the comments above, the most obvious normal tool would be the Mahoney filter.  Specifically the, "explicit complementary filter with bias correction," that doesn't use quaternions.  This gives us a way of combining acceleration measurements, magnetometer measurements, and gyroscope measurements to estimate the rate of change of the attitude of a system with respect to some initial reference frame while compensating for errors in gyroscope measurements.

Here's what the math looks like:

```math
\dot{\hat{\mathbf{R}}} = \hat{\mathbf{R}} \left ( (\mathbf{\Omega}^y - \hat{\mathbf{b}})_\times + k_P \, (\mathbf{\omega}_\mathrm{mes})_\times \right )
```

```math
\dot{\hat{\mathbf{b}}} = -k_I \, \mathbf{\omega}_\mathrm{mes} \\
```

```math
\mathbf{\omega}_\mathrm{mes} = \sum_{i=1}^n k_i \, \mathbf{v}_i \times \hat{\mathbf{v}}_i
```

Where a hat, $\hat{}$, over a letter indicates an estimate; no hat means measurement; each $v_i$ is a unit vector, and they're all orthogonal to one another; b is the bias in the gyroscope, whose rate of change is estimated by the sum of error in unit vector estimates and current measurements (given by their cross product; a rotation); $\Omega^y$ are the gyroscope measurements; a skew, $_\times$, turns a rotation vector into a skew-symmetric rotation matrix (so you can add, multiply, and invert them); $\mathbf{R}$ is a rotation matrix that translates current measurements into your original frame of reference; and all the $k\mathrm{s}$ are gain constants (integral, proportional, whatever).

*Theoretically* the gain constants for $\omega_\mathrm{mes}$ are determined by the existence of eigenvalues of a matrix whose columns are given by your choice of unit vectors, and it only works if you have two or more unit vectors (which we don't), so we'll set those all to one for now.

These unit vectors, $v_i$, are what will define your frame of reference (e.g. north, east, down).  So when you start, you orient yourself in a known good direction, setting your frame of reference. Then $\hat{R}$ will let you change any measurements you collect back to that original frame of reference via multiplication $\hat{R} \, v_i$.  To take measurements in the original frame of reference (e.g. if you need to know where down is *now*), you just multiply them by the transpose, i.e. $\hat{R}^\mathrm{T} \, v_i$.

In order to make this work inside of our computers, we will need to discretize it.  We'll start with the hard part first, and then leave the rest up as an exercise for the reader.  In order to maintain stability as sample rates are reduced, I prefer to use implicit methods for dealing with differential equations, in particular trapezoidal integration or the Crank-Nicolson method.  This means turning our differential into a simple forward finite difference, but the rate of change is equal to the average of the starting point and the ending point.  Similar to the area under a trapezoid when approximating integrals.  This looks like:

```math
\frac{\hat{\mathbf{R}}^{n+1} - \hat{\mathbf{R}}^{n}}{\Delta t} = \frac{1}{2} \left ( \hat{\mathbf{R}}^{n+1} + \hat{\mathbf{R}}^n \right ) \left ( (\mathbf{\Omega}^y - \hat{\mathbf{b}})_\times + k_P \, (\mathbf{\omega}_\mathrm{mes})_\times \right )
```

We then do a quick substitution to make things easier to follow:

```math
\mathbf{R}_p = \frac{\Delta t}{2} \left ( (\mathbf{\Omega}^y - \hat{\mathbf{b}})_\times + k_P \, (\mathbf{\omega}_\mathrm{mes})_\times \right )
```

Re-arrange some matrices:

```math
\hat{\mathbf{R}}^{n+1} \left (\mathbf{I} - \mathbf{R}_p \right ) = \hat{\mathbf{R}^n} \left(\mathbf{I} + \mathbf{R}_p \right)
```

Do a quick inversion, multiplying from the left:

```math
\hat{\mathbf{R}}^{n+1} = \hat{\mathbf{R}^n} \left(\mathbf{I} + \mathbf{R}_p \right) \left (\mathbf{I} - \mathbf{R}_p \right )^{-1}
```

And there you have it, a way of estimating your change in reference frame using existing measurements.  The nice thing about this approach is that the result is still an orthogonal matrix with a determinant of one, so there's no need to then also approximate something else and then normalize it.  This also, if you squint real hard, looks like something of a rotation.  

In this final formulation for estimtaing $\mathbf{R}$, if you look at the definition of $\mathbf{R}_p$, you can kinda see that this is a complementary filter that combines gyroscope measurements (with bias compensation) with an estimate of rotation that comes from accelerometer measurements (since I don't have a magnetometer).  That means that your rotation matrix will always be updated based on a fraction of acceleration measurements determined by $k_P / (1 + k_p)$, and a fraction of bias compensated gyroscope measurements determined by $1 / (1 + k_p)$.  Looking at how the bias is estimated from its rate of change, you can also tell that $k_I$ determines how quickly bias estimates are updated based on that same acceleration measurement (i.e. the comparison between original down in the current reference frame and the current measurement of down).

These are the parameters we're left with having to think about: how fast we want to update our bias estimate, and, how much we want to trust our accelerometer measurement vs. gyroscope measurement when updating our rotation matrix.  It is more complicated than that, but that's an ok starting heuristic.  Thinking this way can also give us some insight into how we can further reduce error with minimal futzin' about.

You can do something similar with the gyroscope bias, $b$, but that requires turning your vectors into matrices, then following the same method.  Instead it's easier to just do the regular explicit Euler method and pray you're sampling fast enough for stability's sake (also it's easy to check for stability).  I suppose you could also do a piecewise implicit integration on the bias vector, which could be a good compromise.  

However, if you want to start slowing down your sample rate, knowing that you have an unconditionally stable way to integrate values through time is incredibly useful.  You'll still suffer from errors and aliasing, of course, but you'll at least be able to collect data over time, analyze it, and perhaps figure out some compensating controls.  Interestingly, since it's possible to have an unstable bias vector whose only use is in an unconditionally stable system, we don't see what we would expect with an unstable system, i.e. exponential growth in error; however, you do see some wacky extreme oscillations, which is fun.

Somewhere in this repository is a file, probably called `attitude.c/.h`, that gives a sample implementation of this approach in a function most likely called `mahoney_filter`.  Also in the filters component, there is a test folder that contains a MATLAB mex function that can be used to demonostrate how this works.

NOTE: I don't use quaternions, but the Mahoney filter paper does discuss them, and they are recommended as they avoid defects like gimbal lock, and can be more performant.  I find the rotation matrix approach more intuitive to discuss, so that's what I use.  For my use case gimbal lock won't be a problem, nor will performance.

Now, gyroscope bias, $b$, is typically understood as a constant reading of angular velocity when the gyroscope is stationary.  That is, even if the reading should be zero, there is some constant reading--as well as noise, of course.  This is mostly due to mechanical stress, and does happen with accelerometers and magnetometers too; however, those are largely overshadowed by the force of gravity and Earth's magnetic field.  Gyroscopes measure a velocity, so their, "default," values should be zero; i.e. when nothing is moving.  For the MPU-6050 IMU that I'm using, gyroscope readings look like:

<p align=center>
    <img src="./figures/gyroscope_bias.svg" width=75%><br>
    <i>Figure 1: Gyroscope Bias</i>
</p>

This isn't terrible, but error will very quickly accumulate so that you can't really tell which way you're facing.  And that's what the Mahoney filter aims to correct: using two known good references (down and north), it should be possible to remove bias from gyroscope measurements so that you can figure out which way you're facing as you move around.

<p align=center>
    <img src="./figures/mahoney_input_002.svg" width=75%><br>
    <i>Figure 2: Accelerometer and Gyroscope Input Data: dt = 0.02</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_0_0.svg" width=75%><br>
    <i>Figure 3: Mahoney Filter Output: dt = 0.02, kp = 0, ki = 0</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_1_0.svg" width=75%><br>
    <i>Figure 4: Mahoney Filter Output: dt = 0.02, kp = 1, ki = 0</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_0_1.svg" width=75%><br>
    <i>Figure 5: Mahoney Filter Output: dt = 0.02, kp = 0, ki = 1</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_1_1.svg" width=75%><br>
    <i>Figure 6: Mahoney Filter Output: dt = 0.02, kp = 1, ki = 1</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_1_10.svg" width=75%><br>
    <i>Figure 7: Mahoney Filter Output: dt = 0.02, kp = 1, ki = 10</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_10_1.svg" width=75%><br>
    <i>Figure 8: Mahoney Filter Output: dt = 0.02, kp = 10, ki = 1</i>
</p>

<p align=center>
    <img src="./figures/mahoney_input_002_filtered_2.svg" width=75%><br>
    <i>Figure 9: Filtered Accelerometer and Gyroscope Input Data: dt = 0.02, fc = 2</i>
</p>

<p align=center>
    <img src="./figures/mahoney_output_002_10_1_2.svg" width=75%><br>
    <i>Figure 10: Mahoney Filter Output: dt = 0.02, kp = 10, ki = 1, fc = 2</i>
</p>

...

## Appendix A: README.md

I forgot the actual README.
