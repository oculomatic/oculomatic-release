/****************************************************************************
**
** Copyright (C) 2015 Jan Zimmermann.
** All rights reserved.
** Contact: Jan Zimmermann (maloman@gmail.com)
**
** This file is part of the Oculomatic software.
**
** The MIT License (MIT)
**
****************************************************************************/

#pragma once

#include "ofMain.h"
#include "ofxGui.h"
//#include "ofxDatGui.h"
#include "ofxOpenCv.h"
#include "FlyCapture2.h"
#include "NIDAQmx.h"
#include "boost/circular_buffer.hpp"

using namespace FlyCapture2;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void normalize_pixelvalues(float pixX, float pixY, float64* aout);
		void heuristic_filtering(float64* aout);

    
        #ifdef _USE_LIVE_VIDEO
        ofVideoGrabber 		vidGrabber;
        #else
        ofVideoPlayer 		vidPlayer;
        #endif
    
        bool bHide;
		bool pHide;
    
        ofxCvColorImage	colorImg;
        ofxCvGrayscaleImage threshImg_pupil, threshImg_cr, monoImg;
        ofxCvContourFinder blobfinder_pupil, blobfinder_cr;
        ofxFloatSlider circularity, convexity, inertia;
        ofxIntSlider circleResolution, pupil_threshold, cr_threshold, minarea_p, maxarea_p, minarea_cr,maxarea_cr;
		ofxIntSlider xGain, yGain;
		ofxToggle videorec, heuristic;
		ofxToggle invertX, invertY;
        ofxLabel screenSize, oculomatic;
        ofxPanel gui;
		//ofxDatGui* guiPlot;
		//ofxDatGuiValuePlotter* plotterX;
		//ofxDatGuiValuePlotter* plotterY;
		//ofxDatGuiSlider* plotterS;
		float centerOffsetX, centerOffsetY;
		

		// flycapture stuff
		Image					rawImage;
		Error					error;
		Camera					camera;
		CameraInfo				camInfo;
		Format7Info				fmt7info;
		Format7ImageSettings	fmt7ImageSettings;
		Format7PacketInfo		fmt7PacketInfo;
		Property				frmRate;

		// nidaqmx stuff
		int						err = 0;
		TaskHandle				taskHandle = 0;
		float64					data[1000];
		float64					aout[2];
		float					aout_min = -10.0;
		float					aout_max = 10.0;
		float					voltageRange = aout_max - aout_min;
		float					tmp1, tmp2;
		boost::circular_buffer<float> buffer_x;
		boost::circular_buffer<float> buffer_y;
};
/*
class Template : public ofxDatGuiTemplate
{
public:
	Template() {
		row.width = 300;
		row.height = 20;
		row.spacing = 2.0f;
		row.stripeWidth = 0.0f;
		row.label.maxAreaWidth = 50.0f;
		slider.color.fill = ofColor(128, 128, 128);
		slider.color.text = ofColor(255, 255, 255);
		graph.color.lines = ofColor(255, 255, 255);
		font.size = 12;
		font.highlightPadding = 0.1f;
		init();
	}
};
*/