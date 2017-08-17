#include <robocup_msgs/RobotLocation.h>

#include <iostream>
#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

//headers for tf2
#include <tf2_ros/transform_broadcaster.h>

//headers for boost libraries
#include <boost/random.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sys/time.h>

class RandomGenerator
{
  public:
    RandomGenerator()
    {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      engine = boost::random::mt19937(tv.tv_usec);
      this->val = 0.0;
    };
    ~RandomGenerator();
    double getRandomValue(){ updateValue(); return val; };
  private:
    boost::random::mt19937 engine;
    boost::random::uniform_real_distribution<double> distribution;
    double val;
    void updateValue(){ val = distribution(engine); };
};

class Particle
{
  public:
    Particle(int dim);
    ~Particle();
    double weight;
    Eigen::Vector2d state;
};

class PFilter
{
  public:
    Eigen::Vector2d control_input;
    robocup_msgs::RobotLocation location;
    ros::Time last_updated_time;
    PFilter(int num, Eigen::Vector2d& upper, Eigen::Vector2d& lower, double random_transition_size);
    ~PFilter();
    void predictionNextState();
    void updateWeight(Eigen::VectorXd& weights);
    void resampleParticles();
    void updateEstimatedState();
    void expansionReset(double radius);
    double maximum_weight;
    std::vector<Particle*>& getParticles(){ return particles; };
    Eigen::Matrix2d covariance_matrix;

  private:
    double random_transition_size;
    int num; // the number of particles
    int dim; // the number of states
    Eigen::Vector2d upper; // the range of state
    Eigen::Vector2d lower;
    RandomGenerator* rg;
    std::vector<Particle*> particles;
    Eigen::Vector2d estimated_state;
    void update_covariance_matrix();
    int getParticleIndexBasedOnWeight();
};
