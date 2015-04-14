#include <math.h>
#include <stdio.h>

#include "aav_control/DistanceCalculator.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

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

tf2::Quaternion DistanceCalculator::extractOrientation(const geometry_msgs::Pose &pose) {
  const geometry_msgs::Quaternion mq(pose.orientation);
  tf2::Quaternion tf2q(mq.x, mq.y, mq.z, mq.w);
  return tf2q;
}

tf2::Vector3 DistanceCalculator::extractPosition(const geometry_msgs::Pose &pose) {
  const geometry_msgs::Point p(pose.position);
  tf2::Vector3 v(p.x, p.y, p.z);
  return v;
}

tf2::Vector3 DistanceCalculator::calculatePosition(double t) {
  tf2::Vector3 position;
  position.setX(calculateSegment(t, &segment->x_segment));
  position.setY(calculateSegment(t, &segment->y_segment));
  return position;
}

double DistanceCalculator::calculateCrossTrackError(double t, const geometry_msgs::Pose &pose) {
  tf2::Quaternion orientation = extractOrientation(pose);
  tf2::Vector3 position = extractPosition(pose);
  tf2::Transform t1(orientation, position);
  tf2::Vector3 desiredPosition = calculatePosition(t);
  tf2::Vector3 offset = t1.inverse() * desiredPosition;
  offset.setZ(0);
  return (offset.getY() < 0 ? -1 : 1) * offset.length();
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

