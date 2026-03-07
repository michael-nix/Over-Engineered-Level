# Over Engineering a Level
Why use a bubble when you can use calculus incorrectly?

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

*Theoretically* those gain constants are determined by the eigenvalues of a matrix whose columns are given by your choice of unit vectors, and it only works if you have two or more unit vectors (which we don't), so we'll set those all to one for now.

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

And there you have it, a way of estimating your change in reference frame using existing measurements.  The nice thing about this approach is that the result is still an orthogonal matrix with a determinant of one, so there's no need to then also approximate something else and then normalize it.  This also, if you squint real hard, looks like something of a rotation.  You can do something similar with the gyroscope bias, $b$, but that requires turning your vectors into matrices, then following the same method.  Instead it's easier to just do the regular explicit Euler method and pray you're sampling fast enough for stability's sake (also it's easy to check).

Somewhere in this repository is a file, probably called `attitude.c/.h`, that gives a sample implementation of this approach in a function most likely called `mahony_filter`.

Now, gyroscope bias is typically understood 

<p align=center>
    <img src="./figures/gyroscope_bias.svg" width=75%><br>
    <i>Figure 1: Gyroscope Bias</i>
</p>

we're gonna get rid of it somewhere else.......

remove b from these equations; use only acceleration.........

trapezoidal integration of R........

obviously just a complementary filter, combining accel and gyroscope.......

...
