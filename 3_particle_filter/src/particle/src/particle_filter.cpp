#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <iterator>
#include <cstdlib>

#include "particle/particle_filter.h"
using namespace std;

static default_random_engine gen;

// #define RESAMPLING_WHEEL
#define RESAMPLING_STRATIFIED

const double IM_LOST_THRESHOLD = 1e-3; 

/*
* TODO
* This function initialize randomly the particles
* Input:
*  std - noise that might be added to the position
*  nParticles - number of particles
*/
void ParticleFilter::init_random(double std[], int nParticles, double min_x, double max_x, double min_y, double max_y) 
{
    // I don't see the reason to use std at this point

    if(nParticles == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("pf node logger"), "Number of initialization particles == 0");
        this->particles.clear();
        return;
    }

    this->particles.clear();
    this->particles.reserve(nParticles);

    double safety_margin = 2.0; // just to make sure to spawn inside the garage and not outside/inside the walls

    uniform_real_distribution<double> x_dist(min_x + safety_margin, max_x - safety_margin);
    uniform_real_distribution<double> y_dist(min_y + safety_margin, max_y - safety_margin);
    uniform_real_distribution<double> theta_dist(-M_PI, M_PI);

    for(int i = 0; i < nParticles; i++)
    {
        Particle p (x_dist(gen), y_dist(gen), theta_dist(gen));
        p.id = i;
        p.weight = 1.0/nParticles;
        this->particles.push_back(p);
    }
    this->num_particles = nParticles;
    is_initialized=true;
}

/*
* TODO
* This function initialize the particles using an initial guess
* Input:
*  x,y,theta - position and orientation
*  std - noise that might be added to the position
*  nParticles - number of particles
*/ 
void ParticleFilter::init(double x, double y, double theta, double std[],int nParticles) {
    if(nParticles == 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("pf node logger"), "Number of initialization particles == 0");
        this->particles.clear();
        return;
    }
    normal_distribution<double> dist_x(x, std[0]); // normal_distribution takes (mean, std)
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    //TODO
    this->particles.clear();
    this->particles.reserve(nParticles);
    for(int i = 0; i< nParticles; i++)
    {
        double gaussian_theta = dist_theta(gen);
        double theta_wrapped = atan2(sin(gaussian_theta), cos(gaussian_theta)); // wrap in -pi, pi
        Particle p (dist_x(gen), dist_y(gen),theta_wrapped);
        p.id = i;
        p.weight = 1.0/nParticles;
        this->particles.push_back(p);
    }
    this->num_particles = nParticles; 
    is_initialized=true;
}

/*
* TODO
* The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep
* Input:
*  delta_t  - time elapsed beetween measurements
*  std_pos  - noise that might be added to the position
*  velocity - velocity of the vehicle
*  yaw_rate - current orientation
* Output:
*  Updated x,y,theta position
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    for(auto& p : this->particles){
    //for each particle
        double x_f, y_f, theta_f;
        if (fabs(yaw_rate) < 0.00001) { // if yaw_rate is near zero we have an equation (see slided)
            x_f = p.x + velocity*delta_t*cos(p.theta);
            y_f = p.y + velocity*delta_t*sin(p.theta);
            theta_f = p.theta;
        }else{ // if angulare velocity is not neglegible we need to use a different set of equations
            x_f = p.x + (velocity/yaw_rate)*(sin(p.theta+yaw_rate*delta_t)-sin(p.theta));
            y_f = p.y + (velocity/yaw_rate)*(cos(p.theta)-cos(p.theta+yaw_rate*delta_t));
            theta_f = p.theta+yaw_rate*delta_t;
        }   
        normal_distribution<double> dist_x(0, std_pos[0]); //the random noise cannot be negative in this case
        normal_distribution<double> dist_y(0, std_pos[1]);
        normal_distribution<double> dist_theta(0, std_pos[2]);
        //TODO: add the computed noise to the current particles position (x,y,theta)
        x_f+=dist_x(gen);
        y_f+=dist_y(gen);
        theta_f+=dist_theta(gen);
        double theta_wrapped = atan2(sin(theta_f), cos(theta_f)); // wrap in -pi, pi
        p.x = x_f;
        p.y = y_f;
        p.theta = theta_wrapped;
	}
}

static inline double squared_euclidean_distance2D(double x1, double y1, double x2, double y2)
{
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

/*
* TODO
* This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
* Input:
*  mapLandmark   - landmarks of the map
*  observations  - observations of the car
* Output:
*  Associated observations to mapLandmarks (perform the association using the ids)
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs>& observations) {
   //TODO
   //TIP: Assign to observations[i].id the id of the landmark with the smallest euclidean distance

   // for each observation find the nearest landmark
    for(auto& observation : observations)
    {
        double mindist = 1e10;
        int min_id = -1;
        for(const auto& landmark : mapLandmark)
        {
            double curr_dist = squared_euclidean_distance2D(observation.x, observation.y, landmark.x, landmark.y);
            if(curr_dist<mindist)
            {
                mindist = curr_dist;
                min_id = landmark.id;
            }
        }
        observation.id = min_id;
    }
    // OSS: at the end there could be a landmark not associated to any observation but all observations are associated with a landmark
}

/*
* TODO
* This function transform a local (vehicle) observation into a global (map) coordinates
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
*/
// a noi arrivano le coordinate delle osservazioni dei sensori, le proiettiamo per ogni particella in coordinate globali e poi dopo facciamo l'association
static inline LandmarkObs transformation(const LandmarkObs observation, const Particle p){
    // I decided to use Eigen for a more general solution   
    LandmarkObs global;
    // make the observation in p_ref_frame a 2d vector
    Eigen::Vector2d observation_v(observation.x, observation.y);

    // translation * rotation = rototranslation
    Eigen::Transform<double, 2, Eigen::Affine> from_particle_to_global = Eigen::Translation2d(Eigen::Vector2d(p.x,p.y)) * Eigen::Rotation2Dd(p.theta);

    Eigen::Vector2d global_v = from_particle_to_global * observation_v;

    global.id = observation.id;
    global.x = global_v.x();
    global.y = global_v.y();

    return global;
}

/*
* TODO
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)
*/
void ParticleFilter::updateWeights(double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {

    //Creates a vector that stores the map (this part can be improved)
    std::vector<LandmarkObs> mapLandmark;
    for(int j=0;j<map_landmarks.landmark_list.size();j++){
        mapLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
    }
    // for each particle perform the following steps
    for(int i=0;i<particles.size();i++){

        // Before applying the association we have to transform the observations in the global coordinates
        std::vector<LandmarkObs> transformed_observations;
        //TODO: for each observation transform it (transformation function)
        for(const auto& observation : observations)
        {
           LandmarkObs transformed_observation = transformation(observation, particles[i]);
           transformed_observations.push_back(transformed_observation);
        }
        
        //TODO: perform the data association (associate the landmarks to the observations)
        dataAssociation(mapLandmark, transformed_observations);
        
        particles[i].weight = 1.0;
        // Compute the probability
		//The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
		//We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;
            //get the associated landmark 
            for (int p = 0; p < mapLandmark.size(); p++) {
                if (transformed_observations[k].id == mapLandmark[p].id) {
                    l_x = mapLandmark[p].x;
                    l_y = mapLandmark[p].y;
                }
            }	
			// How likely a set of landmarks measurements are, given a prediction state of the car 
            double w = exp( -( pow(l_x-obs_x,2)/(2*pow(std_landmark[0],2)) + pow(l_y-obs_y,2)/(2*pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );
            particles[i].weight *= w;
        }
    }    
}

// Update signature
void resamplig_wheel(vector<Particle> &particles, int N_old, int N_new)
{
    uniform_int_distribution<int> dist_distribution(0, N_old - 1); // Use N_old
    double beta  = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> new_particles;

    // Use N_old to populate weights vector
    weights.reserve(N_old); 
    for(int i = 0; i < N_old; i++) 
        weights.push_back(particles[i].weight);
                                                                
    float max_w = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> uni_dist(0.0, max_w);

    // Use N_new to create the new particle set
    new_particles.reserve(N_new);
    for(size_t i = 0; i < N_new; i++) // <-- Use N_new
    {
        beta = beta + uni_dist(gen) * 2.0;
        while(weights[index] < beta)
        {
            beta = beta-weights[index];
            index = (index + 1) % N_old; // <-- Use N_old
        }
        new_particles.push_back(particles[index]);
    }

    particles.swap(new_particles);
    RCLCPP_INFO(rclcpp::get_logger("pf_logger_wheel"),"New particle count: %zu\n", new_particles.size());
}

void stratified_resampling(vector<Particle> &particles, int N_old, int N_new)
{
    // stratify using the old number of particles
    vector<Particle> new_particles;
    new_particles.reserve(N_new); 
    
    vector<double> weights; 
    weights.reserve(N_old);

    for(const auto &p : particles) 
    {
        weights.push_back(p.weight);
    }

    double weights_total = std::accumulate(weights.begin(), weights.end(), 0.0);

    vector<double> weights_cumulative(N_old);
    if (weights_total > 0.0) 
    {
        weights_cumulative[0] = weights[0] / weights_total;
        for (int i = 1; i < N_old; ++i) 
        {
            weights_cumulative[i] = weights_cumulative[i-1] + (weights[i] / weights_total);
        }
    } else {
        for (int i = 0; i < N_old; ++i) 
        {
            weights_cumulative[i] = (double)(i+1) / N_old;
        }
    }
    weights_cumulative[N_old - 1] = 1.0; 

    // resample using the new number of particles
    double substrate_width = 1.0 / N_new; 

    std::uniform_real_distribution<double> distribution(0.0, substrate_width);
    double draw = distribution(gen);
    
    
    int index = 0;
    for (int i = 0; i < N_new; ++i) 
    { 
        while (draw > weights_cumulative[index]) 
        {
            index++;
            if (index >= N_old) index = N_old - 1;
        }
        
        new_particles.push_back(particles[index]);
        draw += substrate_width;
    }

    particles.swap(new_particles);
    RCLCPP_INFO(rclcpp::get_logger("pf_logger_stratified"),"New particle count: %zu\n", new_particles.size());
}

int ParticleFilter::adaptive_resize_number_of_particles() // adaptive particle filter fashion
{
    vector<double> weights;
    weights.reserve(this->num_particles);
    double weights_total = 0.0;
    for(const auto &p : this->particles) {
        weights.push_back(p.weight);
        weights_total += p.weight;
    }

    if (weights_total < IM_LOST_THRESHOLD)
    {
        RCLCPP_WARN(rclcpp::get_logger("pf_logger_resize"), "THE FILTER GOT LOST -> SPAWNING MAX NUMBER OF PARTICLES\n");
        return this->max_particles;
    }

    double sum_squared_w = 0.0;
    if (weights_total > 0.0) {
        for(double &w : weights) {
            w /= weights_total;
            sum_squared_w += w * w;
        }
    } else {
        sum_squared_w = 1.0 / this->num_particles;
    }

    // compute Effective Sample Size (N_eff) this gives us a measure of confidence
    double N_eff = 1.0 / sum_squared_w;

    // use N_eff to compute the new number of particles N_new
    double confidence_ratio = N_eff / this->max_particles; 
    
    int N_new = (int)(confidence_ratio * (this->max_particles - this->min_particles)) + this->min_particles;
    
    N_new = std::clamp(N_new, this->min_particles, this->max_particles);
    
    return N_new;
}

/*
* TODO
* This function resamples the set of particles by repopulating the particles using the weight as metric
*/
void ParticleFilter::resample() 
{
    int N_new = adaptive_resize_number_of_particles();
    int N_old = this->num_particles;

    #ifdef RESAMPLING_WHEEL
        resamplig_wheel(this->particles,N_old, N_new);
    #endif
    #ifdef RESAMPLING_STRATIFIED
        stratified_resampling(this->particles, N_old, N_new);
    #endif

    this->num_particles = N_new;
}


