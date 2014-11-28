double clamp(double value, double lowerLimit, double upperLimit)
{
    if (value < lowerLimit) {
        return lowerLimit;
    } else if (value > upperLimit) {
        return upperLimit;
    } else {
        return value;
    }
}
 
class Vector3 {
    private:
        float x, y, z;

    public:
        // Returns the square of the length(magnitude) of the vector.
        float squaredLength();
};

class SimpleSpline {
    public:
        unsigned short getNumPoints();
        Vector3 interpolate(float r);
};

float getClosestPointOnSpline(SimpleSpline& spline, Vector3 testPoint, float s1, float s2, float s3, int maxIterations = 20)
{
    float s[3];  // The estimates
    s[0] = s1; s[1] = s2; s[2] = s3; 
 
    float Ds[3]; // The distances squared to the estimates

    float sk, skLast; // sk is the "hopefully" converging value generated, skLast is the previous one

    float Ps[4]; // The function P(s) evaluated for the 4 values

    // For gradient and curvature approximation
    float width = 1.0 / (spline.getNumPoints() * 1000); // step would be 1/1000 of a spline length 
 
    // The range of the parameter value for a spline segment * proportion of it is used to test for an exit condition 
    float termCond = 1.0 / (spline.getNumPoints() * 1000);
 
    for (int i = 0; i < maxIterations; ++i) { // its typically done in under 10
        Ds[0] = (spline.interpolate(s[0]) - testPoint).squaredLength();
        Ds[1] = (spline.interpolate(s[1]) - testPoint).squaredLength();
        Ds[2] = (spline.interpolate(s[2]) - testPoint).squaredLength();
     
        // Quadratic Minimization Bit
        sk = 0.5 * ((s[1]*s[1] - s[2]*s[2]) * Ds[0] + (s[2]*s[2] - s[0]*s[0]) * Ds[1] + (s[0]*s[0] - s[1]*s[1]) * Ds[2]) /
                   ((s[1]-s[2]) * Ds[0]         + (s[2] - s[0]) * Ds[1]       + (s[0] - s[1]) * Ds[2]);
 
        if (isnan(sk)) { // denominator = 0, how unfortunate
            //printf ("isnan %d %f\n", i, skLast);
            sk = skLast; // keep going?
            //return skLast;
            //return true;
        }
 
        // Newton Bit
        sk = clamp(sk, width, 1.0 - width); // so can interpolate points for Newtons method
 
        float grad, curv; // 1st 2nd derivatives
        float Ds_pt1 = (spline.interpolate(sk - width) - testPoint).squaredLength();
        float Ds_pt2 = (spline.interpolate(sk)         - testPoint).squaredLength();
        float Ds_pt3 = (spline.interpolate(sk + width) - testPoint).squaredLength();
     
        float g1 = (Ds_pt2 - Ds_pt1) / width;
        float g2 = (Ds_pt3 - Ds_pt2) / width;
     
        grad = (Ds_pt3 - Ds_pt1) / (2 * width);
 
        curv = (g2 - g1) / width;
 
        if (curv != 0.0) { 
            sk = sk - grad / curv;
            sk = clamp(sk, 0.0, 1.0);
        }
 
        // termination criteria
        // difference between skLast and sk <= range of s over the segment x small constant
        if (i > 0) {
            if (Math::Abs(sk - skLast) <= termCond) {
                //printf ("exit condition met %d %f %f\n", i, Math::Abs(sk - skLast), termCond);
                return sk;
                //return true;
            }
        }
        skLast = sk;
 
        // choose the best 3 from their Ps values (the closest ones we keep)
        // general Ps equation
        // Ps =    ((s-s2)*(s-s3))/((s1-s2)*(s1-s3)) * Ds1 + 
        //        ((s-s1)*(s-s3))/((s2-s1)*(s2-s3)) * Ds2 + 
        //        ((s-s1)*(s-s2))/((s3-s1)*(s3-s2)) * Ds3;
 
        Ps[0] = ((s[0]-s[1])*(s[0]-s[2]))/((s[0]-s[1])*(s[0]-s[2])) * Ds[0];
     
        Ps[1] = ((s[1]-s[0])*(s[1]-s[2]))/((s[1]-s[0])*(s[1]-s[2])) * Ds[1];
     
        Ps[2] = ((s[2]-s[0])*(s[2]-s[1]))/((s[2]-s[0])*(s[2]-s[1])) * Ds[2];
 
        Ps[3] = ((sk-s[1])*(sk-s[2]))/((s[0]-s[1])*(s[0]-s[2])) * Ds[0] + 
                ((sk-s[0])*(sk-s[2]))/((s[1]-s[0])*(s[1]-s[2])) * Ds[1] + 
                ((sk-s[0])*(sk-s[1]))/((s[2]-s[0])*(s[2]-s[1])) * Ds[2];
 
        // find the worst one
        int biggest = 0;
        for (int i = 1; i < 4; ++i) {
            if (Ps[i] > Ps[biggest]) {
                biggest = i;
            }
        }
 
        if (biggest <= 2) { // update one of the estimates
            // equations will blow up if any of the estimates are the same
            s[biggest] = sk;
 
            // make them unique values
            for (int i = 0; i < 3; ++i) {
                for (int j = i + 1; j < 3; ++j) {
                    if (s[i] == s[j]) {
                        if (s[j] < 0.5) {
                            s[j] = s[j] + 0.0001;
                        } else {
                            s[j] = s[j] - 0.0001;
                        }
                    }
                }
            }
        }
    }
 
    return sk;
    //return false;
}
