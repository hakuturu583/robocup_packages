#include <particle_filter.h>
#include <robocup_msgs/RobotLocation.h>
#include <math.h>

//headers for ros
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

//headers for tf2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//headers for boost
#include <boost/math/constants/constants.hpp>

Particle::Particle(int dim)
{
  this->weight = 0.0;
  this->state = Eigen::Vector2d::Random(dim);
}

Particle::~Particle(){}

PFilter::PFilter(int num, Eigen::Vector2d& upper, Eigen::Vector2d& lower,double random_transition_size)
{
  this->random_transition_size = random_transition_size;
  this->num = num;
  this->dim = 2;
  this->upper = upper;
  this->lower = lower;
  this->control_input = Eigen::Vector2d::Zero();
  this->estimated_state = Eigen::Vector2d::Zero();
  this->rg = new RandomGenerator();
  this->covariance_matrix = Eigen::Matrix2d::Zero();
  maximum_weight = 0.0;

  // initialize particles
  for (int i=0; i<num; i++)
  {
    Particle* p = new Particle(dim);
    for (int j=0; j<dim;j++)
    {
      p->state(j) = rg->getRandomValue() * (upper(j)-lower(j)) + lower(j);
    }
    p->weight = 1.0 / (double)num;
    particles.push_back(p);
  }
  updateEstimatedState();
}

PFilter::~PFilter(){}

void PFilter::expansionReset(double radius)
{
  // expand particles
  for (int i=0; i<num; i++)
  {
    Particle* p = new Particle(dim);
    double rand_value_range = rg->getRandomValue() * radius;
    double rand_value_theta = rg->getRandomValue() * boost::math::constants::pi<double>();
    p->state(0) = particles[i]->state(0) + rand_value_range * sin(rand_value_theta);
    p->state(1) = particles[i]->state(1) + rand_value_range * cos(rand_value_theta);
    //p->state(2) = particles[i]->state(2) + rand_value_theta * 0.1;
    p->weight = 1.0 / (double)num;
    particles[i] = p;
  }
  updateEstimatedState();
}

void PFilter::predictionNextState()
{
  for (int i=0; i<num; i++)
  {
    // transition model
    // X_k = X_k-1 + u_k
    particles[i]->state(0) += control_input(0) + this->random_transition_size*((rg->getRandomValue()-0.5)*2);
    particles[i]->state(1) += control_input(1) + this->random_transition_size*((rg->getRandomValue()-0.5)*2);
    //particles[i]->state(2) += control_input(2) + 0.01* rg->getRandomValue() * control_input(2);
    for (int j=0; j<dim; j++)
    {
      if (particles[i]->state(j) > upper(j))  particles[i]->state(j) = upper(j);
      if (particles[i]->state(j) < lower(j))  particles[i]->state(j) = lower(j);
    }
  }
}

void PFilter::updateWeight(Eigen::VectorXd& weights)
{
  double sum = 0.0;
  for (int i=0; i<num; i++)
  {
    particles[i]->weight = weights[i];
    sum += particles[i]->weight;
  }
  maximum_weight = 0.0;
  if(sum == 0.0)
  {
    // normalize weight
    for (int i=0; i<num; i++)
    {
      particles[i]->weight = 1/(double)num;
    }
    maximum_weight = 1/(double)num;
  }
  else
  {
    // normalize weight
    for (int i=0; i<num; i++)
    {
      particles[i]->weight /= sum;
      if(particles[i]->weight > maximum_weight)
      {
        maximum_weight = particles[i]->weight;
      }
    }
  }
}

void PFilter::resampleParticles(){
  int index;
  for (int i=0; i<num; i++)
  {
    index = getParticleIndexBasedOnWeight();
    Particle* p = new Particle(dim);
    for (int j=0; j<dim;j++)
    {
      p->state(j) = particles[index]->state(j);
    }
    p->weight = 1.0 / (double)num;
    particles.push_back(p);
  }

  // delete old particles
  for (int i=0; i<num; i++)
  {
    particles.erase(particles.begin());
  }
}

int PFilter::getParticleIndexBasedOnWeight()
{
  double random_val = rg->getRandomValue();
  double sum = 0.0;

  for (int i=0; i<num; i++)
  {
    sum += particles[i]->weight;
    if (sum >= random_val) return i;
  }
  return num-1;
}

void PFilter::updateEstimatedState()
{
  estimated_state = Eigen::Vector2d::Zero();
  for (int i=0; i<num; i++)
  {

    estimated_state(0) += particles[i]->state(0)*particles[i]->weight;
    estimated_state(1) += particles[i]->state(1)*particles[i]->weight;
    //estimated_state(2) += particles[i]->state(2)*particles[i]->weight;
    /*
    estimated_state(0) += particles[i]->state(0)/double(num);
    estimated_state(1) += particles[i]->state(1)/double(num);
    estimated_state(2) += particles[i]->state(2)/double(num);
    */
  }
  update_covariance_matrix();
  location.x = estimated_state(0);
  location.y = estimated_state(1);
  //location.theta = estimated_state(2);
  location.stamp = ros::Time::now();
}

void PFilter::update_covariance_matrix()
{
  double var_x = 0;
  double var_y = 0;
  //double var_theta = 0;
  double cov_x_y = 0;
  //double cov_x_theta = 0;
  //double cov_y_theta = 0;
  for (int i=0; i<num; i++)
  {
    var_x += (estimated_state(0)-particles[i]->state(0))*(estimated_state(0)-particles[i]->state(0))/(double)num;
    var_y += (estimated_state(1)-particles[i]->state(1))*(estimated_state(1)-particles[i]->state(1))/(double)num;
    //var_theta += (estimated_state(2)-particles[i]->state(2))*(estimated_state(2)-particles[i]->state(2))/(double)num;
    cov_x_y += (estimated_state(0)-particles[i]->state(0))*(estimated_state(1)-particles[i]->state(1))/(double)num;
    //cov_x_theta += (estimated_state(0)-particles[i]->state(0))*(estimated_state(2)-particles[i]->state(2))/(double)num;
    //cov_y_theta += (estimated_state(1)-particles[i]->state(1))*(estimated_state(2)-particles[i]->state(2))/(double)num;
  }
  /*
  covariance_matrix << var_x,cov_x_y,cov_x_theta,
                       cov_x_y,var_y,cov_y_theta,
                       cov_x_theta,cov_y_theta,var_theta;
                       */
  covariance_matrix << var_x,cov_x_y,
                       cov_x_y,var_x;

}
