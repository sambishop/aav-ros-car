#include "cte_calculator.h"

#include <gsl/gsl_min.h>
#include <limits>

namespace aav_control
{

CteCalculator::CteCalculator(const aav_msgs::QuinticPath &path)
  : path_(path)
{
  for (unsigned int i = 0; i < path.segments.size(); ++i)
    calculators_.push_back(new PathSegmentCalculator(path.segments[i]));
}

CteCalculator::~CteCalculator()
{
  for (unsigned int i = 0; i < calculators_.size(); ++i)
    delete calculators_[i];
}

double CteCalculator::calculate(const tf2::Vector3 &position)
{
  position_ = &position;
  gsl_min_fminimizer *minimizer;
  return std::numeric_limits<double>::quiet_NaN();
}

double CteCalculator::calculateDistanceMeasure(double t, void *that)
{
  CteCalculator *c = static_cast<CteCalculator *>(that);
  tf2::Vector3 point = calculators_[c->segment_index_]->calculate(t);
  double x_delta = point.getX() - c->position_->getX(); 
  double y_delta = point.getY() - c->position_->getY(); 
  return x_delta * x_delta + y_delta + y_delta;
}

} // end namespace aav_control

