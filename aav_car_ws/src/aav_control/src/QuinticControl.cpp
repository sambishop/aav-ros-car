#include <stdio.h>

#include "QuinticControl.h"

using namespace aav_control;

void QuinticControl::callback(const DoQuinticPathGoalConstPtr &goal)
{ 
  fprintf(stderr, "# of points: %u\n", goal->path.segments.size());
}

