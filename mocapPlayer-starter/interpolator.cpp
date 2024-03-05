#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::DropFrames(Motion* pInputMotion, Motion* pOutputMotion, int N)
{

}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  double testAngles[3] = { 0.2, 3, 1.1 };
  Quaternion<double> testQuat;
  Euler2Quaternion(testAngles, testQuat);

  int i = 1;
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

// Converts assuming we're using right multiplication
void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    double xMatrix[9] = { 1.0,            0.0,             0.0,
                          0.0, cos(angles[0]), -sin(angles[0]),
                          0.0, sin(angles[0]), cos(angles[0]) };

    double yMatrix[9] = { cos(angles[1]), 0.0, sin(angles[1]),
                          0.0,            1.0,            0.0,
                         -sin(angles[1]), 0.0, cos(angles[1]) };

    double zMatrix[9] = { cos(angles[2]), -sin(angles[2]), 0.0,
                           sin(angles[2]), cos(angles[2]), 0.0,
                           0.0,            0.0,            1.0 };

    double intermediate[9];
    thirdDim_matrix_mult(yMatrix, xMatrix, intermediate);
    thirdDim_matrix_mult(zMatrix, intermediate, R);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    Quaternion<double> xQuat{ cos(angles[0] / 2.0), sin(angles[0] / 2.0), 0.0, 0.0 };
    Quaternion<double> yQuat{ cos(angles[1] / 2.0), 0.0, sin(angles[1] / 2.0), 0.0 };
    Quaternion<double> zQuat{ cos(angles[2] / 2.0), 0.0, 0.0, sin(angles[2] / 2.0) };

    xQuat.Normalize();
    yQuat.Normalize();
    zQuat.Normalize();

    q = xQuat * yQuat * zQuat;

    q.Normalize();
}


// Algorithm sourced from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9648712/
void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    // Going in k, j, i order
    // Not proper is true
    // i == 3 (Z), j == 2 (Y), k == 1 (X)
    // epsilon == -1

    double a = q.Gets() - q.Gety();
    double b = q.Getx() + -q.Getz();
    double c = q.Gety() + q.Gets();
    double d = -q.Getz() - q.Getx();

    double aSqr = pow(a, 2);
    double bSqr = pow(b, 2);
    double cSqr = pow(c, 2);
    double dSqr = pow(d, 2);

    double theta2 = acos((2 * (aSqr + bSqr) / (aSqr + bSqr + cSqr + dSqr)) - 1);
    double thetaPos = atan2(b, a);
    double thetaNeg = atan2(d, c);
    double theta1;
    double theta3;
    double halfPi = acos(0.0);

    if (fabs(theta2) < std::numeric_limits<double>::epsilon())
    {
        theta1 = 0.0;
        theta3 = 2 * thetaPos;
    }
    else if (fabs(theta2 - halfPi) < std::numeric_limits<double>::epsilon())
    {
        theta1 = 0.0;
        theta3 = 2 * thetaNeg;
    }
    else
    {
        theta1 = thetaPos - thetaNeg;
        theta3 = thetaPos + thetaNeg;
    }

    theta3 = -theta3;
    theta2 -= halfPi;

    angles[0] = theta1;
    angles[1] = theta2;
    angles[2] = theta3;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

