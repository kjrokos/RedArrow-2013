//
//  Vision.h
//  First 2013
//
//  Created by Kyle Rokos on 2/11/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__Vision__
#define __First_2013__Vision__

#include <iostream>
#include "WPILib.h"
#include "Math.h"

//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640
#define VIEW_ANGLE 48		//Axis 206 camera
//#define VIEW_ANGLE 43.5  //Axis M1011 camera
#define PI 3.141592653

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double xMin[XMINSIZE] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
const double yMax[YMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double yMin[YMINSIZE] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
    .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
    .05, .05, .6, 0};



class VisionControl
{
    struct Scores {
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;
	};
    
public:
    VisionControl();
    ~VisionControl();
    
    void StartVisionTask();
    static int s_ProcessImage(VisionControl *closure);
    void StopVisionTask();
    
    int ProcessImage();
    
    float GetRelativeAzimuth(float currentRelativeAzimuth=0);
    float GetElevation();
    float GetDistance();
    
private: // new code
    double AspectRatio(BinaryImage *image, ParticleAnalysisReport *report);

private: // From sample code
    double ComputeDistance (BinaryImage *image, ParticleAnalysisReport *report, bool outer);
    
    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
     * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
     * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
     * and particle perimeter= 2x+2y
     *
     * @param image The image containing the particle to score, needed to perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
     * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
    double ScoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool outer);
    
    /**
     * Compares scores to defined limits and returns true if the particle appears to be a target
     *
     * @param scores The structure containing the scores to compare
     * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
     *
     * @return True if the particle meets all limits, false otherwise
     */
    bool ScoreCompare(Scores scores, bool outer);
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
     * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
     *
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    double ScoreRectangularity(ParticleAnalysisReport *report);
    /**
     * Computes a score based on the match between a template profile and the particle profile in the X direction. This method uses the
     * the column averages and the profile defined at the top of the sample to look for the solid vertical edges with
     * a hollow center.
     *
     * @param image The image to use, should be the image before the convex hull is performed
     * @param report The Particle Analysis Report for the particle
     *
     * @return The X Edge Score (0-100)
     */
    double ScoreXEdge(BinaryImage *image, ParticleAnalysisReport *report);
    
    /**
     * Computes a score based on the match between a template profile and the particle profile in the Y direction. This method uses the
     * the row averages and the profile defined at the top of the sample to look for the solid horizontal edges with
     * a hollow center
     *
     * @param image The image to use, should be the image before the convex hull is performed
     * @param report The Particle Analysis Report for the particle
     *
     * @return The Y Edge score (0-100)
     */
    double ScoreYEdge(BinaryImage *image, ParticleAnalysisReport *report);
    
private:
    AxisCamera *m_camera;
    float m_relativeAzimuth;
    float m_elevation;
    float m_distance;
    Task m_processTask;
};


#endif /* defined(__First_2013__Vision__) */
