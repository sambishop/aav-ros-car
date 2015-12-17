#ifndef AAV_CONTROL_CTE_CALCULATOR_H
#define	AAV_CONTROL_CTE_CALCULATOR_H

#include <aav_msgs/QuinticPath.h>
#include <aav_quintic/path_calculator.h>
#include <gsl/gsl_min.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

namespace aav_control
{

class CteCalculator
{
public:
  CteCalculator(const aav_msgs::QuinticPath &path);
  ~CteCalculator();
  double calculate(const tf2::Vector3 &position);

private:
  static double calculateDistance(double t, void *that);
  double findT();
  const tf2::Vector3 *position_;
  unsigned int segment_index_;
  double prev_t_;
  aav_quintic::PathCalculator path_calculator_;
  gsl_min_fminimizer *minimizer_;
};

} // end namespace aav_control

#endif

