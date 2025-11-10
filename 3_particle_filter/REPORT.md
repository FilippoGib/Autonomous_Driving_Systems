Academic Year: 2025/2026

# Report on Particle Filter assignment development

### Step 1: Basic requirements implementation

I was able to implement the basic requirements of the assignment. The result is acceptable however it is really really noisy compared to the "best output" we were given by the professor.
The parameters are the following: 

``` c++
// #define RANDOM_INIT // comment this line to turn random initialization OFF
double sigma_init [3] = {0.80, 0.80, 0.10};  //[x,y,theta] initialization noise. -> lets try 0.8 meters on x,y and 0.10 rads on theta
double sigma_pos [3]  = {.50, .50, degrees_to_radiants(20)}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double sigma_landmark [2] = {0.30, 0.30};     //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
```

and the result is this:
![first prototype result](/first_prototype.png)

My next steps are:

- Try to squeese all the performance from the basic PF  

  1. compare non random init to random init performance
  2. understand how to make the PF less "shaky"
  3. play around with parameters such as: more particles vs less particles

- Try to implement algorithmic improvements in order to surpass the basic implementation of the PF

  1. implement a better distance metric
  2. implement a stocastic resampling -> resampling wheel is stocastic when picking the particles but does not add noise to the particle itself. That step is delegated to the prediction step but it would be interesting to try to add extra noise to a small percentage of particles.

