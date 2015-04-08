#include "path_point.h"

namespace aav_control
{

PathPoint::PathPoint(unsigned int segment, double t) : segment(segment), t(t)
{}

unsigned int PathPoint::getSegment() const
{
  return segment;
}

double PathPoint::getT() const
{
  return t;
}

} // end namespace aav_control

