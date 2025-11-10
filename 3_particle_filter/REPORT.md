Academic Year: 2025/2026

# Report on Particle Filter assignment development

## Step 1: Basic requirements implementation

- **Description**:

  I was able to implement the basic requirements of the assignment. The result is acceptable however it is really really noisy compared to the "best output" we were given by the professor.
The parameters are the following: 

``` c++
#define NPARTICLES 100
// #define RANDOM_INIT // comment this line to turn random initialization OFF
double sigma_init [3] = {0.80, 0.80, degrees_to_radiants(6)};  //[x,y,theta] initialization noise. -> lets try 0.8 meters on x,y and 0.10 rads on theta
double sigma_pos [3]  = {.50, .50, degrees_to_radiants(20)}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double sigma_landmark [2] = {0.30, 0.30};     //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
```

- **Results**:

![first prototype result](/first_prototype.png)

- **Comments**: The result appears to be really noisy both on the localization and the orientation graph.



- **Next steps**:

  - Try to squeese all the performance from the basic PF  

    1. compare non random init to random init performance
    2. understand how to make the PF less "shaky"
    3. play around with parameters such as: more particles vs less particles

  - Try to implement algorithmic improvements in order to surpass the basic implementation of the PF

    1. implement a better distance metric
    2. implement a stocastic resampling + number of particles decay-> resampling wheel is stocastic when picking the particles but does not add noise to the particle itself. That step is delegated to the prediction step but it would be interesting to try to add extra noise to a small percentage of particles.

## Step 2: Try to squeese all the performance out of the basic PF  

### Scenario 1:

**IDEA:**
See how the filter behaves with random initialization

**First try**

- **Descirption**: I will keep all the parameters like before and use random initilization
``` c++
#define NPARTICLES 100
#define RANDOM_INIT
double sigma_init [3] = {0.80, 0.80, degrees_to_radiants(6)};  // does not matter in this scenario
double sigma_pos [3]  = {.50, .50, degrees_to_radiants(20)}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double sigma_landmark [2] = {0.30, 0.30};     //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
```
- **Results**: The robot is completely unable to localize itself
- **Comments**: The number of particles might be not enough -> no particles land near the ground truth position.

**Second try**

- **Description**: Try to initialize x50 particles in oder to have more probability of landing in a position near the ground truth
``` c++
#define NPARTICLES 5000
// same as before
```

- **Results:**: The robot is still completely unable to localize itself
- **Comments**: Since increasing the number of particles did not work I think we need to also work on the other parameters. The problem is that the difference between `initial_guess_init` and `random_init` is bigger than I thought. When we initialize with a solid starting guess we want:

  - Some exploration guided towards the initial guess
  - Small update uncertainty
  - Strong trust in sensors
  - Fast convergence (exploitation)

  On the other hand when we initialize randomly we want:
    - Much more exploration in all directions
    - Big update uncertainty in order to keep exploring
    - Less trust in sensors (not because sensors are worse but to give "bad" particles a chance to survive next iteration)
    - Slow convergence
  

**Third try**
- **Description**: Try to keep a high number of particles and tune the prediction uncertainty and the landmark uncertainty
``` c++
#define NPARTICLES 5000
#define RANDOM_INIT
double sigma_init [3] = {0.80, 0.80, degrees_to_radiants(6)};  // does not matter in this scenario
double sigma_pos [3]  = {.50, .50, degrees_to_radiants(30)}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double sigma_landmark [2] = {1.0, 1.0};     //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
```

- **Results**: The robot is now able to localize itself
![first prototype result](/scenario_1_random_init.png)

- **Comments**: The result is even better than the base implementation as it shows much less noise on both the localization and the orentation graph, as well as more precise guesses compared to the ground truth graph


---
### Scenario 2:

**IDEA:**
I want to see what happens if I keep the same parameters and go back to non-random init

**First try**

- **Description**:

``` c++
#define NPARTICLES 5000
// #define RANDOM_INIT
double sigma_init [3] = {0.80, 0.80, degrees_to_radiants(6)};
double sigma_pos [3]  = {.50, .50, degrees_to_radiants(30)}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double sigma_landmark [2] = {1.0, 1.0};     //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
```

- **Results**: The robot is able to localize itself 
![first prototype result](/scenario_2_non_random_init.png)

- **Comments**: The result is really similar to the result of the random init. It is worth noting that the two orientation graphs appear more different, however I think it is due to the fact that the random init graph has a big spike at the beginning due to the fact that the initial orientation is random and it needs some time to adjust. 

  I wonder if what would happen if I brought the number of particles back down to 100.

- **Observations**: Compared to the basic PF implementation, the results of the last two scenarios appear to be much less noisy. I wounder if they only appear to be less noisy because I have less samples (due to the fact that using more particles is more computationally expensive)

**Second try**

- **Description**: I want to see what happens when I keep all the parameters as they are now, and I just lower the number of particles back to 100. I expect to go back to a much more noisy graph.

``` c++
#define NPARTICLES 100
// same as before
```
- **Results**: To no-one surprise the result is back to beign super noisy

![first prototype result](/scenario_2_non_random_init_less_particles.png)

- **Comments**:
Reducing the numer of particles allows the algorithm to run much faster -> more odometry samples. The reason why the result is so noisy is because we don't trust the landmarks positions but we also introduce a lot of noise in the prediction.

- **Conclusions**:
The random init model benefits from having low "trust" in what it is doing and is able to compensate when a large number of particles are used.
In the non-random init model it makes no sense to have low trust in the landmarks and in the model. 

- **Next**: I want to go back to non-random init, raise the number of particles a bit and try to tighten the parameters.
---
### Scenario 3:

**Description:**
- hi 

**Results:**
- res

---