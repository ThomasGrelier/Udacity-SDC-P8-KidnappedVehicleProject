/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    cout << "INITIALIZATION" << endl;

    default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

    // Standard deviations for x, y, and theta.
    std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// Create a normal (Gaussian) distribution for x, y, theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

    for (int i=0; i<num_particles_;++i ) {
        // generate noise. "gen" is the random engine initialized earlier.
        Particle particle;
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1;
        particles_.push_back(particle);
    };
    is_initialized_ = true;
    cout << "x_i:" << particles_[0].x << " y_i:" << particles_[0].y << " thteta_i:" << particles_[0].theta<<endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	cout << "PREDICTION" << endl;
    default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

    // Standard deviations for x, y, and theta.
    std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	// Create a normal (Gaussian) distribution for x, y, theta
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

    if (fabs(yaw_rate) < 0.001) {  //avoid division by zero
        for (int i=0; i<num_particles_;++i ) {
            // generate noise. "gen" is the random engine initialized earlier.
            float x = particles_[i].x;
            float y = particles_[i].y;
            float theta = particles_[i].theta;
            particles_[i].x = x + velocity*delta_t*cos(theta) + dist_x(gen);
            particles_[i].y = y + velocity*delta_t*sin(theta) + dist_y(gen);
            particles_[i].theta = theta + dist_theta(gen);
        }
    }
    else {
        for (int i=0; i<num_particles_;++i ) {
            // generate noise. "gen" is the random engine initialized earlier.
            float x = particles_[i].x;
            float y = particles_[i].y;
            float theta = particles_[i].theta;
            particles_[i].x = x + velocity/yaw_rate*(sin(theta+yaw_rate*delta_t)-sin(theta)) + dist_x(gen);
            particles_[i].y = y + velocity/yaw_rate*(-cos(theta+yaw_rate*delta_t)+cos(theta)) + dist_y(gen);
            particles_[i].theta = theta + yaw_rate*delta_t + dist_theta(gen);
        }
    };
    cout << "x:" << particles_[0].x << " y:" << particles_[0].y << " thteta:" << particles_[0].theta<<endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.

    for (int i = 0; i<observations.size(); ++i) {
        int id_landmark = 0;
        float distance_min = 100;
        int ind_lm;
        for (int j=0; j<predicted.size(); ++j){
            float distance;   // distance between predicted[j] and observations[i]
            distance = dist(predicted[j].x, predicted[j].y, observations[i].x,  observations[i].y);
            //cout << "Obs: (" <<observations[i].x<<","<<observations[i].y<<")- Landmark: (" <<predicted[j].x<<","<<predicted[j].y<<")-Id:"<< predicted[j].id << endl;
            //cout << "Distance: " << distance << endl;
            if (distance<distance_min){
                distance_min = distance;
                id_landmark = predicted[j].id;
                ind_lm = j;
            }
        }
        observations[i].id = id_landmark;
        cout << "Trans Obs: (" <<observations[i].x<<","<<observations[i].y<<")- Landmark: (" <<predicted[ind_lm].x<<","<<predicted[ind_lm].y<<")-Id:"<< id_landmark << endl;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    cout << "UPDATE" << endl;

    double weights_sum = 0; // for normalization

    for (int i=0; i<num_particles_; ++i){  // loop on particles

       vector<LandmarkObs> observations_m; // landmark observations in map frame

       float x_p, y_p, theta_p;  // particle state
       x_p = particles_[i].x;
       y_p = particles_[i].y;
       theta_p = particles_[i].theta;

       for (int j = 0; j<observations.size(); ++j){  // loop on observations
            // express observations in map coordinates system

            float x_obs, y_obs;  // coordinates of observations in car frame
            x_obs = observations[j].x;
            y_obs = observations[j].y;

            float x_m, y_m;  // coordinates of landmark in map frame
            x_m = cos(theta_p)*x_obs - sin(theta_p)*y_obs + x_p;
            y_m = sin(theta_p)*x_obs + cos(theta_p)*y_obs + y_p;

            LandmarkObs obs;
            obs.x = x_m;
            obs.y = y_m;
            observations_m.push_back(obs);
            cout << "Obs: (" <<x_obs<<","<<y_obs<<")-> Trans obs: (" <<obs.x<<","<<obs.y<<")"<< endl;
        }

        // landmark association
        vector<LandmarkObs> predicted;
        for (int i = 0; i<map_landmarks.landmark_list.size();++i){
            LandmarkObs obs;
            obs.id = map_landmarks.landmark_list[i].id_i;
            obs.x = map_landmarks.landmark_list[i].x_f;
            obs.y = map_landmarks.landmark_list[i].y_f;
            predicted.push_back(obs);
        }

        dataAssociation(predicted, observations_m);

        // update weight
        double proba = 1;
        for (int j = 0; j<observations_m.size(); ++j){
            int id_LM = observations_m[j].id;
            float x_obs = observations_m[j].x;
            float y_obs = observations_m[j].y;
            float x_LM = map_landmarks.landmark_list[id_LM].x_f;
            float y_LM = map_landmarks.landmark_list[id_LM].y_f;
            cout << "Obs: (" <<x_obs<<","<<y_obs<<") - LM: (" <<x_LM<<","<<y_LM<<")"<< endl;
            double proba_i = 1/(2*M_PI*std_landmark[0]*std_landmark[1])*exp(-((x_obs-x_LM)*(x_obs-x_LM)/(2*std_landmark[0])+(y_obs-y_LM)*(y_obs-y_LM)/(2*std_landmark[1])));
            cout << proba_i << endl;
            proba = proba*proba_i;
         }
        weights_.push_back(proba);
        weights_sum += proba;
        cout << weights_[i] << endl;
    }
    // normalization of weights
    for (int i=0; i<num_particles_; ++i){
        weights_[i] = weights_[i]/weights_sum;
        particles_[i].weight = weights_[i];
    };
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;
    // we need int values as input weights for discrete distribution
    // multiply by 10000 to get weights bigger than 1, then convert from float to int
    vector<double> weights;
    for (int i=0;i<weights.size();++i){
        double weight_i=weights_[i]*10000;
        weights_.push_back(weight_i);
    }
    vector<int> weights_int(weights.begin(), weights.end()); // conversion to int

	discrete_distribution<> d(weights_int.begin(), weights_int.end());

    vector<Particle> particles_new;
    for (int n = 0; n<num_particles_; n++) {
        int drawn_particle_id = d(gen);
        Particle particles_new_n = particles_[drawn_particle_id];
        particles_new.push_back(particles_new_n);
    }
    particles_ = particles_new;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
