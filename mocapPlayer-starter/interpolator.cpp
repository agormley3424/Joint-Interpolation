#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits>
#include <stdexcept>
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

// Linearly interpolate angles
void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  // Iterate through every key frame
  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      /* Needs to be edited for each method */
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      /* Needs to be edited for each method */
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int numKeyFrames;
    if (inputLength > 0)
    {
        numKeyFrames = floor((inputLength - 1) / (N + 1)) + 1;
    }
    else
    {
        numKeyFrames = 0;
    }

    // This is only set up to work with 3 or more keyframes!
    if (numKeyFrames < 3)
    {
        //printf("Interpolator Error: BezierInterpolationEuler: Fewer than three key frames are present");
        throw std::exception("Interpolator Error: BezierInterpolationEuler: Fewer than three key frames are present");
    }

    // Iterate through every key frame
    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        int keyFrameIndex = startKeyframe / (N + 1);
        bool firstKeyFrame;
        bool lastKeyFrame;

        // q(n-1)
        Posture* lastStartPosture = nullptr;

        // q(n+2)
        Posture* nextEndPosture = nullptr;

        // Note if this is the first keyframe
        if (startKeyframe - N - 1 < 0)
        {
            firstKeyFrame = true;

            // q(n+2)
            nextEndPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
        }
        else
        {
            firstKeyFrame = false;

            // q(n-1)
            lastStartPosture = pInputMotion->GetPosture(startKeyframe - N - 1);
        }

        // q(n)
        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        // q(n+1)
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // Note if this is the penultimate keyframe
        if (endKeyframe + N + 1 > inputLength)
        {
            lastKeyFrame = true;
        }
        else
        {
            lastKeyFrame = false;
        }

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        /* Calculate all needed points, for the root position */
        vector qPrev;
        if (lastStartPosture != nullptr) qPrev = lastStartPosture->root_pos;
        vector qNow = startPosture->root_pos;
        vector qNext = endPosture->root_pos;
        vector qNextNext;
        if (nextEndPosture != nullptr) qNextNext = nextEndPosture->root_pos;

        vector a;
        vector b;

        CalculateSpline(firstKeyFrame, lastKeyFrame, qPrev, qNow, qNext, qNextNext, &a, &b);

        // interpolate in between frames
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            interpolatedPosture.root_pos = DeCasteljauEuler(t, qNow, a, b, qNext);

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        /* Calculate all needed points for this spline, for each bone */
        for (int bone = 1; bone <= MAX_BONES_IN_ASF_FILE; bone++)
        {
            if (lastStartPosture != nullptr)
            {
                qPrev = lastStartPosture->bone_rotation[bone];
            }
            else
            {
                qPrev = nullptr;
            }
            qNow = startPosture->bone_rotation[bone];
            qNext = endPosture->bone_rotation[bone];
            if (nextEndPosture != nullptr)
            {
                qNextNext = nextEndPosture->bone_rotation[bone];
            }
            else
            {
                qNextNext = nullptr;
            }

            CalculateSpline(firstKeyFrame, lastKeyFrame, qPrev, qNow, qNext, qNextNext, &a, &b);

            // interpolate in between frames
            for (int frame = 1; frame <= N; frame++)
            {
                Posture interpolatedPosture;
                double t = 1.0 * frame / (N + 1);

                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, qNow, a, b, qNext);

                pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
            }
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::CalculateSpline(bool firstKeyFrame, bool lastKeyFrame, vector& qPrev, vector& qNow, vector& qNext, vector& qNextNext, vector* aOut, vector* bOut)
{
    vector a = nullptr;
    if (firstKeyFrame)
    {
        a = Lerp(1.0 / 3.0, qNow, Lerp(2.0, qNextNext, qNext));
    }
    else
    {
        vector aBar = Lerp(0.5, Lerp(2.0, qPrev, qNow), qNext);

        a = Lerp(1.0 / 3.0, qNow, aBar);
    }

    vector b = nullptr;
    if (lastKeyFrame)
    {
        b = Lerp(1.0 / 3.0, qNext, Lerp(2.0, qPrev, qNow));
    }
    else
    {
        vector aBarNext = Lerp(0.5, Lerp(2.0, qNow, qNext), qNextNext);

        b = Lerp(-1.0 / 3.0, qNext, aBarNext);
    }

    *aOut = a;
    *bOut = b;
}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // students should implement this
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
  double angle = acos(qStart.dot(qEnd_));

  return (sin((1 - t) * angle) * qStart) / sin(angle) +
      (sin(t * angle) * qEnd_) / sin(angle);
}

vector Interpolator::Lerp(double t, vector& vStart, vector& vEnd)
{
    return (vEnd * t) + vStart * (1 - t);
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    return ((2 * p.dot(q)) * q) - p;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
    vector q0 = Lerp(t, p0, p1);
    vector q1 = Lerp(t, p1, p2);
    vector q2 = Lerp(t, p2, p3);

    vector r0 = Lerp(t, q0, q1);
    vector r1 = Lerp(t, q1, q2);

    return Lerp(t, r0, r1);
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    Quaternion<double> q0 = Slerp(t, p0, p1);
    Quaternion<double> q1 = Slerp(t, p1, p2);
    Quaternion<double> q2 = Slerp(t, p2, p3);

    Quaternion<double> r0 = Slerp(t, q0, q1);
    Quaternion<double> r1 = Slerp(t, q1, q2);

    return Slerp(t, r0, r1);
}

