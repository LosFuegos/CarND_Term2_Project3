/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <map>
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
    //std::cout << "Here 4" << std::endl;
    default_random_engine gen1;
    //cout << "here 1";
    num_particles = 23;
    double std_x = std[0];
    double std_y = std[1];
    double std_theta =std[2];

    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    for(int i = 0; i < num_particles; ++i){
        Particle p = {};
        p.id = i;
        p.x = dist_x(gen1);
        p.y = dist_y(gen1);
        p.theta = dist_theta(gen1);
        p.weight = 1.0;
        particles.push_back(p);
        weights.push_back(p.weight);
    }

    //std::cout << "----------------Intial State------------------" << endl;

    //std::cout << "X: " << particles[0].x << endl;
    //std::cout << "Y: " << particles[0].y << endl;
    //std::cout << "Theta: " << particles[0].theta << endl;
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen2;
    //cout << "here 2";
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_theta(0, std_theta);

    for(int i = 0; i < num_particles; ++i){

        double theta_f;
        double x_f;
        double y_f;


        double theta = particles[i].theta;

        if(fabs(yaw_rate) >= 1e-8){
            double velocity_yaw = velocity / yaw_rate;
            theta_f = theta + (yaw_rate * delta_t);
            x_f = particles[i].x + (velocity_yaw) * (sin(theta_f) - sin(theta));
            y_f = particles[i].y + (velocity_yaw) * (cos(theta) - cos(theta_f));

        }

        else{
            double velocityXdelta_t = velocity * delta_t;
            x_f = particles[i].x + (velocityXdelta_t * cos(theta));
            y_f = particles[i].y + (velocityXdelta_t * sin(theta));
            theta_f = theta;

        }


        particles[i].x = x_f + dist_x(gen2);
        particles[i].y = y_f + dist_y(gen2);
        particles[i].theta = theta_f + dist_theta(gen2);
    }
    //cout << "here 3";
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    double nearest;
    double distance;

    LandmarkObs landmark;
    vector<LandmarkObs> landmarks;

    for(int i = 0; i < observations.size(); ++i){
        nearest = 9999;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
        const std::vector<LandmarkObs> &observations, const Map &map_landmarks){
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

    //std::cout << "Here 7" << std::endl;
    double const gauss_norm = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
    double const sig_x_2 = 2.0 * std_landmark[0] * std_landmark[0];
    double const sig_y_2 = 2.0 * std_landmark[1] * std_landmark[1];
    //float sum = 0.0;
    for(int n = 0; n < num_particles; ++n){
        double theta = particles[n].theta;
        double x = particles[n].x;
        double y = particles[n].y;
        //std::std::vector<int> associations;
        //std::vector<double> sense_x;
        //std::vector<double> sense_y;
        std::vector<LandmarkObs> Transformed_Obs;
       // std::cout << endl;
       // std::cout << "---------------Transformations-----------------" << endl;

        for(int k = 0; k < observations.size(); ++k){
            LandmarkObs mark = {};
            double obs_x = observations[k].x;
            double obs_y = observations[k].y;
            mark.x = x + (cos(theta) * obs_x) - (sin(theta) * obs_y);
            mark.y = y + (sin(theta) * obs_x) + (cos(theta) * obs_y);
            Transformed_Obs.push_back(mark);

            //std::cout << "Obs(x,y)(" << obs_x << "," << obs_y << ")--->TObs(x,y)(" << mark.x << "," << mark.y << ")" << endl;
        }
        particles[n].weight = 1.0;
        //weights[n] = 1.0;
        int l_id;
        //std::cout << "----------------Associations-----------------" << endl;

        for(int i = 0; i < Transformed_Obs.size(); ++i){
            double diff_x = 0;
            double diff_y = 0;
            double nearest = 99999;

            for(int j = 0; j < map_landmarks.landmark_list.size(); ++j){
                double map_x_f = map_landmarks.landmark_list[j].x_f;
                double map_y_f = map_landmarks.landmark_list[j].y_f;

                if(dist(map_x_f, map_y_f, x, y) <= sensor_range){

                    double distance = dist(Transformed_Obs[i].x, Transformed_Obs[i].y, map_x_f, map_y_f);
                    if(distance < nearest){
                        nearest = distance;
                        l_id = j + 1;
                        diff_x = Transformed_Obs[i].x - map_x_f;
                        diff_y = Transformed_Obs[i].y - map_y_f;
                    }
                }
            }
            Transformed_Obs[i].id = l_id;
            //std::cout << "Index: " << l_id << "; Transformed(x,y): (" << Transformed_Obs[i].x << "," << Transformed_Obs[i].y << "); Landmark(x,y): (" << map_landmarks.landmark_list[l_id - 1].x_f << "," << map_landmarks.landmark_list[l_id - 1].y_f << ")" << endl;
            double weight = gauss_norm * exp(-( (pow(diff_x,2) / sig_x_2) + (pow(diff_y,2) / sig_y_2) ));
            //std::cout << "Weight: " << weight << endl;
            if(weight > 1e-8)
                particles[n].weight = particles[n].weight * weight;


        }
        Transformed_Obs.clear();
        weights[n] = particles[n].weight;
        //std::cout << "---------------Final Weight------------------" << endl;
        std::cout << weights[0] << endl;
        //weights.clear();
        //sum += particles[n].weight;

    }

    /*float normalized_sum= 0.0;
    for(int v = 0; v < num_particles; ++v){
        particles[v].weight /= sum;
        weights.push_back(particles[v].weight);
        normalized_sum += particles[v].weight;
        std::cout << particles[v].weight << endl;
    }
    std::cout << "Sum: " << sum << endl;
    std::cout << "Normalized Sum: " << normalized_sum << std::endl;
    */
    //std::cout << "Here 9" << std::endl;
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen3;
    //std::cout << "Here 11" << std::endl;
    //random_device seed;
    //mt19937 random_generator(seed());
    std::discrete_distribution<int> di(weights.begin(), weights.end());
    //std::cout << "Here 12" << std::endl;

    std::vector<Particle> resample_particles;
    //std::cout << "Here 13" << std::endl;

    for(int i = 0; i < num_particles; ++i)
        resample_particles.push_back(particles[di(gen3)]);
    //std::cout << "Here 14" << std::endl;

    particles = resample_particles;
    //weights.clear();
    //weights.assign(1.0, num_particles);
    //std::cout << "Here 15" << std::endl;



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
