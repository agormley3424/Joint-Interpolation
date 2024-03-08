#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>
#include <algorithm>
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

        // Note if this is the final keyframe
        if (endKeyframe + N + 1 >= inputLength)
        {
            lastKeyFrame = true;
        }
        else
        {
            lastKeyFrame = false;

            // q(n+2)
            nextEndPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
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

        vector a[MAX_BONES_IN_ASF_FILE + 1];
        vector b[MAX_BONES_IN_ASF_FILE + 1];

        // Calculate spline for root position
        CalculateSpline(firstKeyFrame, lastKeyFrame, qPrev, qNow, qNext, qNextNext, a[0], b[0]);

        /* Calculate splines for all the bones */
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            if (lastStartPosture != nullptr) qPrev = lastStartPosture->bone_rotation[bone];
            qNow = startPosture->bone_rotation[bone];
            qNext = endPosture->bone_rotation[bone];
            if (nextEndPosture != nullptr) qNextNext = nextEndPosture->bone_rotation[bone];

            //if (startKeyframe == 1080)
            //{
            //    int i = 0;
            //}

            CalculateSpline(firstKeyFrame, lastKeyFrame, qPrev, qNow, qNext, qNextNext, a[bone + 1], b[bone + 1]);
        }

        // interpolate in-between frames
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;

            double t = 1.0 * frame / (N + 1);

            qNow = startPosture->root_pos;
            qNext = endPosture->root_pos;
            interpolatedPosture.root_pos = DeCasteljauEuler(t, qNow, a[0], b[0], qNext);

            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                qNow = startPosture->bone_rotation[bone];
                qNext = endPosture->bone_rotation[bone];
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, qNow, a[bone + 1], b[bone + 1], qNext);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    CompareMotion(pInputMotion, pOutputMotion, "motionComparison");
}

void Interpolator::CalculateSpline(bool firstKeyFrame, bool lastKeyFrame, vector& qPrev, vector& qNow, vector& qNext, vector& qNextNext, vector& a, vector& b)
{
    if (firstKeyFrame)
    {
        a = Lerp(1.0 / 3.0, qNow, Lerp(2.0, qNextNext, qNext));
    }
    else
    {
        vector aBar = Lerp(0.5, Lerp(2.0, qPrev, qNow), qNext);

        a = Lerp(1.0 / 3.0, qNow, aBar);
    }

    if (lastKeyFrame)
    {
        b = Lerp(1.0 / 3.0, qNext, Lerp(2.0, qPrev, qNow));
    }
    else
    {
        vector aBarNext = Lerp(0.5, Lerp(2.0, qNow, qNext), qNextNext);

        b = Lerp(-1.0 / 3.0, qNext, aBarNext);
    }
}

void Interpolator::CompareMotion(Motion* motion1, Motion* motion2, std::string fileName)
{
    int inputLength1 = motion1->GetNumFrames();
    int inputLength2 = motion1->GetNumFrames();

    if (inputLength1 != inputLength2)
    {
        throw std::exception("Interpolator Error: CompareMotion: Two Motions have different number of frames");
    }

    std::ofstream outputFile;
    outputFile.open(fileName + ".csv");
    outputFile << "Frame,Type,x1,y1,z1,x2,y2,z2\n";

    for (int frame = 0; frame < inputLength1; frame++)
    {
        Posture* posture1 = motion1->GetPosture(frame);
        Posture* posture2 = motion2->GetPosture(frame);

        outputFile << frame << ",root_pos," << posture1->root_pos.x() << ',' <<
            posture1->root_pos.y() << ',' << posture1->root_pos.z() << ',' << posture2->root_pos.x() <<
            ',' << posture2->root_pos.y() << ',' << posture2->root_pos.z() << std::endl;

        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            outputFile << frame << ",bone_rotation_" << bone << "," << posture1->bone_rotation[bone].x() << ',' <<
                posture1->bone_rotation[bone].y() << ',' << posture1->bone_rotation[bone].z() << ',' << posture2->bone_rotation[bone].x() <<
                ',' << posture2->bone_rotation[bone].y() << ',' << posture2->bone_rotation[bone].z() << std::endl;
        }
    }

    outputFile.close();
}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
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
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            /* Needs to be edited for each method */
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            /* Needs to be edited for each method */
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // Convert euler angles to quaternions and interpolate
                Quaternion<double> startPos = Euler2QuaternionVector(startPosture->bone_rotation[bone]);

                Quaternion<double> endPos = Euler2QuaternionVector(endPosture->bone_rotation[bone]);

                vector startEuler = Quaternion2EulerVector(startPos);

                vector endEuler = Quaternion2EulerVector(endPos);

                Quaternion<double> result = Slerp(t, startPos, endPos);

                // Convert result back into euler angles
                vector resultEuler = Quaternion2EulerVector(result);

                interpolatedPosture.bone_rotation[bone] = resultEuler;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    CompareMotion(pInputMotion, pOutputMotion, "motionComparison");
}

void Interpolator::BezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
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

        // Note if this is the final keyframe
        if (endKeyframe + N + 1 >= inputLength)
        {
            lastKeyFrame = true;
        }
        else
        {
            lastKeyFrame = false;

            // q(n+2)
            nextEndPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
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

        vector aRoot;
        vector bRoot;

        // Calculate spline for root position
        CalculateSpline(firstKeyFrame, lastKeyFrame, qPrev, qNow, qNext, qNextNext, aRoot, bRoot);

        Quaternion<double> a[MAX_BONES_IN_ASF_FILE];
        Quaternion<double> b[MAX_BONES_IN_ASF_FILE];
        /* Calculate splines for all the bones */
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            Quaternion<double> qPrevQuat;
            if (lastStartPosture != nullptr) qPrevQuat = Euler2QuaternionVector(lastStartPosture->bone_rotation[bone]);
            Quaternion<double> qNowQuat = Euler2QuaternionVector(startPosture->bone_rotation[bone]);
            Quaternion<double> qNextQuat = Euler2QuaternionVector(endPosture->bone_rotation[bone]);
            Quaternion<double> qNextNextQuat;
            if (nextEndPosture != nullptr) qNextNextQuat = Euler2QuaternionVector(nextEndPosture->bone_rotation[bone]);

            //if (startKeyframe == 1080)
            //{
            //    int i = 0;
            //}

            CalculateSplineQuaternion(firstKeyFrame, lastKeyFrame, qPrevQuat, qNowQuat, qNextQuat, qNextNextQuat, a[bone], b[bone]);

            int i = 0;
        }

        // interpolate in-between frames
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;

            double t = 1.0 * frame / (N + 1);

            // For the root position
            qNow = startPosture->root_pos;
            qNext = endPosture->root_pos;
            interpolatedPosture.root_pos = DeCasteljauEuler(t, qNow, aRoot, bRoot, qNext);

            // For the bone angles
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> qNowQuat = Euler2QuaternionVector(startPosture->bone_rotation[bone]);
                Quaternion<double> qNextQuat = Euler2QuaternionVector(endPosture->bone_rotation[bone]);
                Quaternion<double> interpPos = DeCasteljauQuaternion(t, qNowQuat, a[bone], b[bone], qNextQuat);
                interpolatedPosture.bone_rotation[bone] = Quaternion2EulerVector(interpPos);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    CompareMotion(pInputMotion, pOutputMotion, "motionComparison");
}

void Interpolator::CalculateSplineQuaternion(bool firstKeyFrame, bool lastKeyFrame, Quaternion<double>& qPrev, Quaternion<double>& qNow, Quaternion<double>& qNext, Quaternion<double>& qNextNext, Quaternion<double>& a, Quaternion<double>& b)
{
    if (firstKeyFrame)
    {
        a = Slerp(1.0 / 3.0, qNow, Slerp(2.0, qNextNext, qNext));
    }
    else
    {
        Quaternion<double> aBar = Slerp(0.5, Slerp(2.0, qPrev, qNow), qNext);

        a = Slerp(1.0 / 3.0, qNow, aBar);
    }

    if (lastKeyFrame)
    {
        b = Slerp(1.0 / 3.0, qNext, Slerp(2.0, qPrev, qNow));
    }
    else
    {
        Quaternion<double> aBarNext = Slerp(0.5, Slerp(2.0, qNow, qNext), qNextNext);

        b = Slerp(-1.0 / 3.0, qNext, aBarNext);
    }
}

// pi rads = 180 deg
// 1 deg = pi / 180 rads
// 1 rad = 180 deg / pi
void Interpolator::DegreesToRadians(double degAngles[3], double radAngles[3])
{
    radAngles[0] = (acos(0.0) * 2 * degAngles[0]) / 180.0;
    radAngles[1] = (acos(0.0) * 2 * degAngles[1]) / 180.0;
    radAngles[2] = (acos(0.0) * 2 * degAngles[2]) / 180.0;
}

void Interpolator::RadiansToDegrees(double radAngles[3], double degAngles[3])
{
    radAngles[0] = (radAngles[0] * 180) / (acos(0.0) * 2);
    radAngles[1] = (radAngles[1] * 180) / (acos(0.0) * 2);
    radAngles[2] = (radAngles[2] * 180) / (acos(0.0) * 2);
}

// Output is in degrees, I think
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
    double radAngles[3];
    DegreesToRadians(angles, radAngles);

    double xMatrix[9] = { 1.0,            0.0,             0.0,
                          0.0, cos(radAngles[0]), -sin(radAngles[0]),
                          0.0, sin(radAngles[0]), cos(radAngles[0]) };

    double yMatrix[9] = { cos(radAngles[1]), 0.0, sin(radAngles[1]),
                          0.0,            1.0,            0.0,
                         -sin(radAngles[1]), 0.0, cos(radAngles[1]) };

    double zMatrix[9] = { cos(radAngles[2]), -sin(radAngles[2]), 0.0,
                           sin(radAngles[2]), cos(radAngles[2]), 0.0,
                           0.0,            0.0,            1.0 };

    double intermediate[9];
    thirdDim_matrix_mult(yMatrix, xMatrix, intermediate);
    thirdDim_matrix_mult(zMatrix, intermediate, R);

    int i = 1;
}

Quaternion<double> Interpolator::Euler2QuaternionVector(vector& angles)
{
    Quaternion<double> result;
    double angleArr[3];
    angleArr[0] = angles.x();
    angleArr[1] = angles.y();
    angleArr[2] = angles.z();
    Euler2Quaternion(angleArr, result);

    return result;
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    double radAngles[3];
    DegreesToRadians(angles, radAngles);
    Quaternion<double> xQuat{ cos(radAngles[0] / 2.0), sin(radAngles[0] / 2.0), 0.0, 0.0 };
    Quaternion<double> yQuat{ cos(radAngles[1] / 2.0), 0.0, sin(radAngles[1] / 2.0), 0.0 };
    Quaternion<double> zQuat{ cos(radAngles[2] / 2.0), 0.0, 0.0, sin(radAngles[2] / 2.0) };

    xQuat.Normalize();
    yQuat.Normalize();
    zQuat.Normalize();

    q = zQuat * yQuat * xQuat;

    q.Normalize();
}

vector Interpolator::Quaternion2EulerVector(Quaternion<double>& q)
{
    double angles[3];
    Quaternion2Euler(q, angles);
    vector result;
    result.set_x(angles[0]);
    result.set_y(angles[1]);
    result.set_z(angles[2]);

    return result;
}

void Interpolator::Quaternion2Euler(Quaternion<double>& q, double angles[3])
{
    double R[9];
    q.Quaternion2Matrix(R);
    Rotation2Euler(R, angles);
    // Output is already in degrees
}

void Interpolator::Quaternion2Rotation(Quaternion<double>& q, double R[9])
{
    R[0] = pow(q.Gets(), 2) + pow(q.Getx(), 2) - pow(q.Gety(), 2) - pow(q.Getz(), 2);
    R[1] = 2.0 * q.Getx() * q.Gety() - 2.0 * q.Gets() * q.Getz();
    R[2] = 2.0 * q.Getx() * q.Getz() + 2.0 * q.Gets() * q.Gety();

    R[3] = 2.0 * q.Getx() * q.Gety() + 2.0 * q.Gets() * q.Getz();
    R[4] = pow(q.Gets(), 2) - pow(q.Getx(), 2) + pow(q.Gety(), 2) - pow(q.Getz(), 2);
    R[5] = 2.0 * q.Gety() * q.Getz() - 2.0 * q.Gets() * q.Getx();
    
    R[6] = 2.0 * q.Getx() * q.Getz() - 2.0 * q.Gets() * q.Gety();
    R[7] = 2.0 * q.Gety() * q.Getz() + 2.0 * q.Gets() * q.Getx();
    R[8] = pow(q.Gets(), 2) - pow(q.Getx(), 2) - pow(q.Gety(), 2) + pow(q.Getz(), 2);
}

double Interpolator::Clamp(double value, double low, double high)
{
    if (value < low)
    {
        return low;
    }
    else if (value > high)
    {
        return high;
    }
    else
    {
        return value;
    }
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  double angle1 = acos(Clamp(qStart.dot(qEnd_), -1.0, 1.0));

  double angle2 = acos(Clamp(qStart.dot(-1 * qEnd_), -1.0, 1.0));

  double angle = std::min(angle1, angle2);

  Quaternion<double> result;

  if (abs(sin(angle)) < std::numeric_limits<double>::epsilon())
  {
      result = Lerp(t, qStart, qEnd_);
      result.Normalize();
  }
  else
  {
      result = ((sin((1 - t) * angle) * qStart) / sin(angle)) +
          ((sin(t * angle) * qEnd_) / sin(angle));
  }

  return result;
}

vector Interpolator::Lerp(double t, vector& vStart, vector& vEnd)
{
    return (vEnd * t) + vStart * (1 - t);
}

Quaternion<double> Interpolator::Lerp(double t, Quaternion<double>& vStart, Quaternion<double>& vEnd)
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

