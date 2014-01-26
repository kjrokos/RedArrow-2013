//
//  Vision.cpp
//  First 2013
//
//  Created by Kyle Rokos on 2/11/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#include "Vision.h"

VisionControl::VisionControl()
:m_camera(NULL),
m_relativeAzimuth(0),
m_elevation(0),
m_distance(0),
m_processTask("ProcessImageTask", (FUNCPTR) s_ProcessImage)
{
    m_camera->GetInstance("10.32.34.11");
    m_camera->WriteResolution(AxisCamera::kResolution_320x240);
    m_camera->WriteCompression(30);
    m_camera->WriteBrightness(0);
    m_camera->WriteColorLevel(50);
}

void VisionControl::StartVisionTask()
{
    m_processTask.Start((int)this);
}

int VisionControl::s_ProcessImage(VisionControl *closure)
{
    while (1)
    {
        if(!closure->ProcessImage())
            Wait(1/60.f);
    }
    return 0;
}

void VisionControl::StopVisionTask()
{
    m_processTask.Stop();
}

int VisionControl::ProcessImage()
{
    if(m_camera->IsFreshImage())
    {
        ColorImage *image = NULL;
        m_camera->GetImage(image);
        
        Threshold threshold(60,100,90,255,20,255);
        ParticleFilterCriteria2 criteria[] = {IMAQ_MT_AREA,AREA_MINIMUM,65535,false,false};
        
        BinaryImage *thresholdImage = image->ThresholdHSV(threshold);
        BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);
        BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1);
        
        vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();
        
        for (unsigned int i = 0; i < reports->size(); i++)
        {
            ParticleAnalysisReport *report = &(reports->at(i));
            
            // first, determine if this is a particle we are looking at.
            if(report->boundingRect.left > 320/2 || report->boundingRect.left + report->boundingRect.width < 320/2)
            {
                // particle is not lined up with center of vision
                // note: may not want to do this for autonomous!
                continue;
            }
            
            double aspectRatio = AspectRatio(filteredImage, report);

            double difference3ptGoal = fabs(1-(aspectRatio / ((54.f+4+4)/(12.f+4+4))));
            double difference2ptGoal = fabs(1-(aspectRatio / ((54.f+4+4)/(21.f+4+4))));
            
            if(difference2ptGoal < 0.25 && difference2ptGoal < difference3ptGoal)
            {
                m_elevation = 0;
                m_distance = ComputeDistance(thresholdImage, report, true);
                m_relativeAzimuth = 0;
            }
            else if(difference3ptGoal < 0.25 && difference3ptGoal < difference2ptGoal)
            {
                m_elevation = 0;
                m_distance = ComputeDistance(thresholdImage, report, false);
                m_relativeAzimuth = 0;
            }
            else
            {
                // didn't sufficiently match a target!
            }
        }
        /*
        Scores *scores = new Scores[reports->size()];
        
        //Iterate through each particle, scoring it and determining whether it is a target or not
        for (unsigned i = 0; i < reports->size(); i++)
        {
            ParticleAnalysisReport *report = &(reports->at(i));
            
            scores[i].rectangularity = ScoreRectangularity(report);
            scores[i].aspectRatioOuter = ScoreAspectRatio(filteredImage, report, true);
            scores[i].aspectRatioInner = ScoreAspectRatio(filteredImage, report, false);
            scores[i].xEdge = ScoreXEdge(thresholdImage, report);
            scores[i].yEdge = ScoreYEdge(thresholdImage, report);
            
            if(ScoreCompare(scores[i], false))
            {
                printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
                printf("Distance: %f \n", ComputeDistance(thresholdImage, report, false));
            }
            else if (ScoreCompare(scores[i], true))
            {
                printf("particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
                printf("Distance: %f \n", ComputeDistance(thresholdImage, report, true));
            }
            else
            {
                printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
            }
            printf("rect: %f  ARinner: %f \n", scores[i].rectangularity, scores[i].aspectRatioInner);
            printf("ARouter: %f  xEdge: %f  yEdge: %f  \n", scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);	
        }
        */
        
        delete image;
        delete thresholdImage;
        delete convexHullImage;
        delete filteredImage;
        delete reports;
        //delete scores;
        return 1;
    }
    
    return 0;
}

float VisionControl::GetRelativeAzimuth(float currentRelativeAzimuth)
{
    return m_relativeAzimuth + currentRelativeAzimuth;
}

float VisionControl::GetElevation()
{
    return m_elevation;
}

float VisionControl::GetDistance()
{
    return m_distance;
}

double VisionControl::ComputeDistance (BinaryImage *image, ParticleAnalysisReport *report, bool outer)
{
    double rectShort, height;
    int targetHeight;
    
    imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
    //using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
    //on skewed rectangles
    height = min(report->boundingRect.height, rectShort);
    targetHeight = outer ? 29 : 21;
    
    return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
}

double VisionControl::AspectRatio(BinaryImage *image, ParticleAnalysisReport *report)
{
    double rectLong, rectShort, aspectRatio;

    imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
    imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
    
    aspectRatio = rectLong/rectShort;

    return aspectRatio;
}

double VisionControl::ScoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool outer)
{
    double rectLong, rectShort, idealAspectRatio, aspectRatio;
    idealAspectRatio = outer ? (62/29) : (62/20);	//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
    
    imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
    imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
    
    //Divide width by height to measure aspect ratio
    if(report->boundingRect.width > report->boundingRect.height)
    {
        //particle is wider than it is tall, divide long by short
        aspectRatio = 100*(1-fabs((1-((rectLong/rectShort)/idealAspectRatio))));
    }
    else
    {
        //particle is taller than it is wide, divide short by long
        aspectRatio = 100*(1-fabs((1-((rectShort/rectLong)/idealAspectRatio))));
    }
    return (max(0, min(aspectRatio, 100)));		//force to be in range 0-100
}


bool VisionControl::ScoreCompare(Scores scores, bool outer)
{
    bool isTarget = true;
    
    isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
    if(outer)
    {
        isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
    }
    else
    {
        isTarget &= scores.aspectRatioInner > ASPECT_RATIO_LIMIT;
    }
    isTarget &= scores.xEdge > X_EDGE_LIMIT;
    isTarget &= scores.yEdge > Y_EDGE_LIMIT;
    
    return isTarget;
}


double VisionControl::ScoreRectangularity(ParticleAnalysisReport *report)
{
    if(report->boundingRect.width*report->boundingRect.height !=0)
    {
        return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
    }
    else
    {
        return 0;
    }
}


double VisionControl::ScoreXEdge(BinaryImage *image, ParticleAnalysisReport *report)
{
    double total = 0;
    LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(), IMAQ_COLUMN_AVERAGES, report->boundingRect);
    for(int i=0; i < (averages->columnCount); i++)
    {
        if(xMin[i*(XMINSIZE-1)/averages->columnCount] < averages->columnAverages[i]
           && averages->columnAverages[i] < xMax[i*(XMAXSIZE-1)/averages->columnCount])
        {
            total++;
        }
    }
    total = 100*total/(averages->columnCount);		//convert to score 0-100
    imaqDispose(averages);							//let IMAQ dispose of the averages struct
    return total;
}


double VisionControl::ScoreYEdge(BinaryImage *image, ParticleAnalysisReport *report)
{
    double total = 0;
    LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(), IMAQ_ROW_AVERAGES, report->boundingRect);
    for(int i=0; i < (averages->rowCount); i++)
    {
        if(yMin[i*(YMINSIZE-1)/averages->rowCount] < averages->rowAverages[i]
           && averages->rowAverages[i] < yMax[i*(YMAXSIZE-1)/averages->rowCount])
        {
            total++;
        }
    }
    total = 100*total/(averages->rowCount);		//convert to score 0-100
    imaqDispose(averages);						//let IMAQ dispose of the averages struct
    return total;
}