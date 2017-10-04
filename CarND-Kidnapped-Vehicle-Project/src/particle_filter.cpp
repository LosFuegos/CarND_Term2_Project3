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
    std::cout << "Here 4" << std::endl;
    default_random_engine gen;
    //cout << "here 1";
    num_particles = 70;
    double std_x = std[0];
    double std_y = std[1];
    double std_theta =std[2];

    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    for(int i = 0; i < num_particles; ++i){
        Particle p = {};
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
        weights.push_back(p.weight);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    //cout << "here 2";
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    double theta_f = 0;
    double x_f;
    double y_f;
    double velocity_yaw;
    double velocityXdelta_t;
    double theta;

    for(int i = 0; i < num_particles; ++i){


        theta = particles[i].theta;

        if(yaw_rate == 0){
            velocityXdelta_t = velocity * delta_t;
            x_f = particles[i].x + (velocityXdelta_t * cos(theta));
            y_f = particles[i].y + (velocityXdelta_t * sin(theta));
            theta_f = theta;
        }

        else{
            velocity_yaw = velocity / yaw_rate;
            theta_f = theta + (yaw_rate * delta_t);
            x_f = particles[i].x + (velocity_yaw) * (sin(theta_f) - sin(theta));
            y_f = particles[i].y + (velocity_yaw) * (cos(theta) - cos(theta_f));
        }
        // add gaussian noise
        normal_distribution<double> dist_x(x_f, std_x);
        normal_distribution<double> dist_y(y_f, std_y);
        normal_distribution<double> dist_theta(theta_f, std_theta);

        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
    //cout << "here 3";
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    /*double nearest = 1000000;
    double distance;
    for(int i = 0; i < predicted.size(); ++i){
        for(int n = 0; n < observations.size(); ++i){
            distance = sqrt((predicted[i].x- observations[n].x) + (predicted[i].y - observations[i].y));

            if(distance < nearest){
                nearest = distance;

            }


        }
    }
    */
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
    vector<LandmarkObs> Transformed_Obs;
    //std::cout << "Here 7" << std::endl;
    double gauss_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
    double sig_x_2 = 2 * std_landmark[0] * std_landmark[0];
    double sig_y_2 = 2 * std_landmark[1] * std_landmark[1];
    for(int n = 0; n < num_particles; ++n){
        double theta = particles[n].theta;
        double x = particles[n].x;
        double y = particles[n].y;
        //std::cout << n << std::endl;
        //std::std::vector<int> associations;
        //std::vector<double> sense_x;
        //std::vector<double> sense_y;

        for(int k = 0; k < observations.size(); ++k){
            LandmarkObs mark;
            double obs_x = observations[k].x;
            double obs_y = observations[k].y;
            mark.x = x + (cos(theta) * obs_x) - (sin(theta) * obs_y);
            mark.y = y + (sin(theta) * obs_x) + (cos(theta) * obs_y);
            Transformed_Obs.push_back(mark);
        }
        particles[n].weight = 1.0;
        weights[n] = 1.0;

        double nearest;
        double distance;
        double weight;

        double sense_x;
        double sense_y;
        int l_id;
        //int O_association;
        //std::cout << "Here 8" << std::endl;
        for(int i = 0; i < Transformed_Obs.size(); ++i){
            double diff_x = 0;
            double diff_y = 0;
            for(int j = 0; j < map_landmarks.landmark_list.size(); ++j){
                diff_x = Transformed_Obs[j].x - map_landmarks.landmark_list[i].x_f;
                diff_y = Transformed_Obs[j].y - map_landmarks.landmark_list[i].y_f;
                distance = sqrt((diff_x) * (diff_x) + (diff_y) * (diff_y));
                if(j > 0){
                    if(distance < nearest){
                        //nearest = distance;
                        //l_id = j;
                        //O_association = n;
                        sense_x = diff_x;
                        sense_y = diff_y;
                    }
                }
                else{
                    nearest = distance;
                    //l_id = i;
                    //O_association = n;
                    sense_x = diff_x;
                    sense_y = diff_y;
                }
            }
            weight = gauss_norm * exp(-( (((sense_x) * (sense_x)) / (sig_x_2)) + (((sense_y) * (sense_y)) / (sig_y_2)) ));

            weights[i] *= weight;
            particles[i].weight = weights[i];
        }
        //Transformed_Obs.clear();
        //particles[p] = SetAssociations(particles[p], associations, sense_x, sense_y);
        //weights[p] = particles[p].weight;

    }

    //std::cout << "Here 9" << std::endl;
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;
    //std::random_device rd;
    //std::mt19937 gen(rd());
    std::cout << "Here 11" << std::endl;

    std::discrete_distribution<int> d = (weights.begin(), weights.end());
    std::cout << "Here 12" << std::endl;

    std::vector<Particle> resample_particles;
    std::cout << "Here 13" << std::endl;

    for(int i = 0; i < num_particles; ++i)
        resample_particles.push_back(particles[d(gen)]);
    std::cout << "Here 14" << std::endl;

    particles = resample_particles;
    std::cout << "Here 15" << std::endl;

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