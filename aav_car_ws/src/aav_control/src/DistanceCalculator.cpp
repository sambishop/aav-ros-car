#include <math.h>
#include <stdio.h>

#include "DistanceCalculator.h"

using namespace aav_msgs;
using namespace geometry_msgs;

DistanceCalculator::DistanceCalculator(
      const QuinticPathSegment *segment,
      const Point *point
    ) {
  this->segment = segment;
  this->point = point;
  this->minimizer = gsl_min_fminimizer_alloc(gsl_min_fminimizer_brent);
  this->func.function = &DistanceCalculator::calculateDistanceMeasure;
  this->func.params = this;
}

DistanceCalculator::~DistanceCalculator() {
  gsl_min_fminimizer_free(this->minimizer);
}

double DistanceCalculator::findT() {
  gsl_min_fminimizer_set(minimizer, &func, .5, -1, 2);
  for (int i = 0; i < 10; ++i) {
    gsl_min_fminimizer_iterate(minimizer);
  }
  return gsl_min_fminimizer_minimum(minimizer);
}

double DistanceCalculator::calculateCrossTrackError(double t) {
  double distanceMeasure = calculateDistanceMeasure(t, this);
  return sqrt(distanceMeasure);
}

double DistanceCalculator::calculateDistanceMeasure(double t, void *that) {
  DistanceCalculator *c = static_cast<DistanceCalculator *>(that);
  double x = c->calculateSegment(t, &c->segment->x_segment);
  double y = c->calculateSegment(t, &c->segment->y_segment);
  double x_delta = x - c->point->x;
  double y_delta = y - c->point->y;
  double val = x_delta * x_delta + y_delta * y_delta;
  return val;
}

double DistanceCalculator::calculateSegment(double t_1,
    const QuinticSplineSegment *segment) {
  double t_2 = t_1 * t_1;
  double t_3 = t_2 * t_1;
  double t_4 = t_2 * t_2;
  double t_5 = t_4 * t_1;

  double _t_1 = 1.0 - t_1;
  double _t_2 = _t_1 * _t_1;
  double _t_3 = _t_2 * _t_1;
  double _t_4 = _t_2 * _t_2;
  double _t_5 = _t_4 * _t_1;

  return     _t_5 *       segment->P0
      +  5 * _t_4 * t_1 * segment->P1
      + 10 * _t_3 * t_2 * segment->P2
      + 10 * _t_2 * t_3 * segment->P3
      +  5 * _t_1 * t_4 * segment->P4
      +             t_5 * segment->P5;
}

