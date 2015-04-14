#include "aav_control/cte_calculator.h"

#include <cmath>
#include <limits>

namespace aav_control
{

CteCalculator::CteCalculator(const aav_msgs::QuinticPath &path)
  : path_(path)
{
  for (unsigned int i = 0; i < path.segments.size(); ++i)
    calculators_.push_back(new PathSegmentCalculator(path.segments[i]));
  minimizer_ = gsl_min_fminimizer_alloc(gsl_min_fminimizer_brent);
}

CteCalculator::~CteCalculator()
{
  for (unsigned int i = 0; i < calculators_.size(); ++i)
    delete calculators_[i];
  gsl_min_fminimizer_free(minimizer_);
}

double CteCalculator::calculate(const tf2::Vector3 &position)
{
  if (std::isnan(prev_t_))
    return std::numeric_limits<double>::quiet_NaN();
  position_ = &position;
  double t = findT();
  if (std::isnan(t))
    return t;
  tf2::Vector3 point = calculators_[segment_index_]->calculate(t);
  tf2::Vector3 tangent = calculators_[segment_index_]->calculate1stDerivative(t);
  tf2::Vector3 error = point - position;
  return error.length();
}

double CteCalculator::findT()
{
  gsl_function func;
  func.function = &CteCalculator::calculateDistanceMeasure;
  func.params = this;
  double t;
  while (true)
  {
    gsl_min_fminimizer_set(minimizer_, &func, prev_t_, -1, 2);
    for (int i = 0; i < 10; ++i)
      gsl_min_fminimizer_iterate(minimizer_);
    t = gsl_min_fminimizer_minimum(minimizer_);
    if (t < 0)
    {
      if (segment_index_ == 0)
      {
        t = 0;
        break;
      }
      else
      {
        --segment_index_;
        prev_t_ = 1;
      }
    }
    else if (t > 1)
    {
      if (segment_index_ == calculators_.size() - 1)
      {
        t = std::numeric_limits<double>::quiet_NaN();
        break;
      }
      else
      {
        ++segment_index_;
        prev_t_ = 0;
      }
    }
    else
      break;
  }
  prev_t_ = t;
  return t;
}

double CteCalculator::calculateDistanceMeasure(double t, void *that)
{
  CteCalculator *c = static_cast<CteCalculator *>(that);
  tf2::Vector3 point = c->calculators_[c->segment_index_]->calculate(t);
  return tf2::tf2Distance2(point, *c->position_);
}

} // end namespace aav_control

