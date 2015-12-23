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

#include "ofApp.h"
#include "FlyCapture2.h"
#include "NIDAQmx.h"
#include "boost/circular_buffer.hpp"

using namespace FlyCapture2;

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetVerticalSync(false);

	// NiDAQmx setup
	DAQmxCreateTask("", &taskHandle);
	DAQmxCreateAOVoltageChan(taskHandle, "Dev1/ao0:1", "", aout_min, aout_max, DAQmx_Val_Volts, NULL); // First two channels
	DAQmxStartTask(taskHandle);

	// Connect to the camera
	error = camera.Connect(0);
	if (error != PGRERROR_OK) {
		std::cout << "failed to connect to camera..." << std::endl;
		return;
	}

	error = camera.GetCameraInfo(&camInfo);
	if (error != PGRERROR_OK) {
		std::cout << "failed to get camera info from camera" << std::endl;
		return;
	}

	std::cout << camInfo.vendorName << " "
		<< camInfo.modelName << " "
		<< camInfo.serialNumber << std::endl;

	error = camera.StartCapture();
	if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
		std::cout << "bandwidth exceeded" << std::endl;
		return;
	}
	else if (error != PGRERROR_OK) {
		std::cout << "failed to start image capture" << std::endl;
		return;
	}

   // colorImg.allocate(640,480);
   // threshImg_pupil.allocate(640,480);
   // threshImg_cr.allocate(640,480);

	// include the buffer for heuritstic filtering
	buffer_x.set_capacity(4);
	buffer_y.set_capacity(4);

	// GUI for plotting
	/*guiPlot = new ofxDatGui(ofxDatGuiAnchor::TOP_RIGHT);
	guiPlot->setTemplate(new Template());
	guiPlot->setOpacity(0.5f);
	plotterX = guiPlot->addValuePlotter("X", -10, 10);
	plotterY = guiPlot->addValuePlotter("Y", -10, 10);
	plotterS = guiPlot->addSlider("f", 0, 3, 0.3);
	plotterX->setDrawMode(ofxDatGuiGraph::LINES);
	plotterY->setDrawMode(ofxDatGuiGraph::LINES);
	guiPlot->setAutoDraw(false);
	*/

	// GUI for rest of the stuff
	gui.setup();
	ofBackground(160, 160, 160);
    gui.add(oculomatic.setup("OCULOMATIC", "0.4"));
    gui.add(pupil_threshold.setup("Pupil Threshold", 15, 0, 100));
    gui.add(cr_threshold.setup("CR Threshold", 15, 0, 300));
    gui.add(minarea_p.setup("Min Area P", 14, 0, 20000));
    gui.add(maxarea_p.setup("Max Area P", 1000, 0, 40000));
    gui.add(minarea_cr.setup("Min Area CR", 14, 0, 1000));
    gui.add(maxarea_cr.setup("Max Area CR", 500, 0, 2000));
    gui.add(circularity.setup("Circularity",0.1,0,1));
    gui.add(convexity.setup("Convexity",0.87,0,1));
    gui.add(inertia.setup("Inertia",0.01,0,0.1));
	gui.add(heuristic.setup("Heuristic filter",true));
	gui.add(xGain.setup("X Gain", 100, 0, 500));
	gui.add(yGain.setup("Y Gain", 100, 0, 500));
	gui.add(invertX.setup("Invert X", false));
	gui.add(invertY.setup("Invert Y", false));
    gui.add(videorec.setup("Record Video",false));
    gui.add(screenSize.setup("screen size", ofToString(ofGetWidth())+"x"+ofToString(ofGetHeight())));
    bHide = false;
	pHide = true;
    

}

//----------------------------------------------fly----------------
void ofApp::update(){

    bool bNewFrame = false;

	//plotterX->setSpeed(guiPlot->getSlider("f")->getValue());
	//plotterY->setSpeed(guiPlot->getSlider("f")->getValue());

	error = camera.RetrieveBuffer(&rawImage);
	if (error != PGRERROR_OK) {
		std::cout << "capture error" << std::endl;
	}
	else {
		bNewFrame = true;
	}
	Image monoImage; 
	rawImage.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage);
    
    if (bNewFrame){
        
		monoImg.setFromPixels(monoImage.GetData(), monoImage.GetCols(), monoImage.GetRows());
        
        threshImg_pupil = monoImg;
        threshImg_cr = monoImg;
        threshImg_pupil.threshold(pupil_threshold,true);
        threshImg_cr.threshold(cr_threshold,false);
        blobfinder_pupil.findContours(threshImg_pupil, minarea_p, maxarea_p, 1, false);
        blobfinder_cr.findContours(threshImg_cr, minarea_cr, maxarea_cr, 1,false);
		if (blobfinder_pupil.nBlobs > 0) {
			normalize_pixelvalues(blobfinder_pupil.blobs.at(0).centroid.x, blobfinder_pupil.blobs.at(0).centroid.y, aout);
			if (heuristic) {
				heuristic_filtering(aout);
			}
			DAQmxWriteAnalogF64(taskHandle, 1, 1, 10.0, DAQmx_Val_GroupByChannel, aout, NULL, NULL);
			//plotterX->setValue(aout[0]);
			//plotterY->setValue(aout[1]);
		}
    }

    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255,255,255);
    ofDrawBitmapString(ofToString(ofGetFrameRate())+"fps", 10, monoImg.getHeight() + 20);
    ofDrawBitmapString("L: load settings", 10, monoImg.getHeight() + 35);
    ofDrawBitmapString("S: save settings", 10, monoImg.getHeight() + 50);
	ofDrawBitmapString("X: center voltage", 10, monoImg.getHeight() + 65);
	ofDrawBitmapString("P: voltage display (experimental!)", 10, monoImg.getHeight() + 80);
	monoImg.draw(0, 0);
    threshImg_pupil.draw(monoImg.getWidth(),0,monoImg.getWidth() / 2, monoImg.getHeight() / 2);
    threshImg_cr.draw(monoImg.getWidth(),monoImg.getHeight() /  2, monoImg.getWidth() / 2, monoImg.getHeight() / 2);
    blobfinder_pupil.draw(0,0);
    //blobfinder_cr.draw(0,0);
    if(blobfinder_pupil.nBlobs>0){
        ofNoFill();
        ofSetColor(0,0,255);
        ofDrawCircle(blobfinder_pupil.blobs.at(0).centroid,3);
        ofSetColor(0,255,255);
        //ofDrawCircle(blobfinder_pupil.blobs.at(0).centroid.x, blobfinder_pupil.blobs.at(0).centroid.y, blobfinder_pupil.blobs.at(0).boundingRect.width/2);
    }
    if(blobfinder_cr.nBlobs>0){
        ofSetColor(255,0,255);
        ofDrawCircle(blobfinder_cr.blobs.at(0).centroid.x,blobfinder_cr.blobs.at(0).centroid.y,blobfinder_cr.blobs.at(0).boundingRect.width/2);
    }
    
    if(!bHide){
        gui.draw();
		//gui.setPosition(monoImg.getWidth() * 1.5f, 0);
    }
	if (!pHide) {
		//guiPlot->update();
		//guiPlot->draw();
		//ofSetWindowShape(monoImg.getWidth() * 1.5f + gui.getWidth() + 300, monoImg.getHeight() + 95);

	}
	else {
		//ofSetWindowShape(monoImg.getWidth() * 1.5f + gui.getWidth(), monoImg.getHeight() + 95);
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'x') {
		centerOffsetX = 0;
		centerOffsetY = 0;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
    if(key == 'h'){
        bHide = !bHide;
    }
    else if(key == 's'){
        gui.saveToFile("settings.xml");
    }
    else if(key == 'l'){
        gui.loadFromFile("settings.xml");
    }
	else if (key == 'x') {
		centerOffsetX = aout[0];
		centerOffsetY = aout[1];
	}
	else if (key == 'p') {
		pHide = !pHide;
	}
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
	screenSize = ofToString(w) + "x" + ofToString(h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}


void ofApp::normalize_pixelvalues(float pixX, float pixY, float64* aout) {

	aout[0] = (((pixX - xGain) / (monoImg.getWidth() - xGain*2)) * voltageRange + aout_min) - centerOffsetX;
	aout[1] = (((pixY - yGain) / (monoImg.getHeight() - yGain*2)) * voltageRange + aout_min) - centerOffsetY;

	if(invertX && invertY) {
		aout[0] = (((pixX - xGain) / (monoImg.getWidth() - xGain * 2)) * voltageRange + aout_min) + centerOffsetX;
		aout[1] = (((pixY - yGain) / (monoImg.getHeight() - yGain * 2)) * voltageRange + aout_min) + centerOffsetY;
		aout[0] = aout[0] * (-1.0);
		aout[1] = aout[1] * (-1.0);
	}
	else if(invertX && !invertY) {
		aout[0] = (((pixX - xGain) / (monoImg.getWidth() - xGain * 2)) * voltageRange + aout_min) + centerOffsetX;
		aout[1] = (((pixY - yGain) / (monoImg.getHeight() - yGain * 2)) * voltageRange + aout_min) - centerOffsetY;
		aout[0] = aout[0] * (-1.0);
	}
	else if(!invertX && invertY) {
		aout[0] = (((pixX - xGain) / (monoImg.getWidth() - xGain * 2)) * voltageRange + aout_min) - centerOffsetX;
		aout[1] = (((pixY - yGain) / (monoImg.getHeight() - yGain * 2)) * voltageRange + aout_min) + centerOffsetY;
		aout[1] = aout[1] * (-1.0);
	}
	else {
		aout[0] = (((pixX - xGain) / (monoImg.getWidth() - xGain * 2)) * voltageRange + aout_min) - centerOffsetX;
		aout[1] = (((pixY - yGain) / (monoImg.getHeight() - yGain * 2)) * voltageRange + aout_min) - centerOffsetY;
	}

}

void ofApp::heuristic_filtering(float64* aout) {

	if (buffer_x.size() < 3) {
		buffer_x.push_front(aout[0]);
		buffer_y.push_front(aout[1]);
	}
	else {
		buffer_x.push_front(aout[0]);
		buffer_y.push_front(aout[1]);
		// filter level 1
		if (buffer_x[2] > buffer_x[1] && buffer_x[1] < buffer_x[0]) {
			tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
			tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

			if (tmp2 > tmp1) {
				buffer_x[1] = buffer_x[0];
			}
			else {
				buffer_x[1] = buffer_x[2];
			}
			buffer_x[3] = buffer_x[2];
			buffer_x[2] = buffer_x[1];
		}
		else if (buffer_x[2] < buffer_x[1] && buffer_x[1] > buffer_x[0]) {
			tmp1 = std::abs(buffer_x[1] - buffer_x[0]);
			tmp2 = std::abs(buffer_x[1] - buffer_x[2]);

			if (tmp2 > tmp1) {
				buffer_x[1] = buffer_x[0];
			}
			else {
				buffer_x[1] = buffer_x[2];
			}

			buffer_x[3] = buffer_x[2];
			buffer_x[2] = buffer_x[1];
		}
		else {
			buffer_x[3] = buffer_x[2];
			buffer_x[2] = buffer_x[1];
		}
		if (buffer_y[2] > buffer_y[1] && buffer_y[1] < buffer_y[0]) {
			tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
			tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

			if (tmp2 > tmp1) {
				buffer_y[1] = buffer_y[0];
			}
			else {
				buffer_y[1] = buffer_y[2];
			}
			buffer_y[3] = buffer_y[2];
			buffer_y[2] = buffer_y[1];
		}
		else if (buffer_y[2] < buffer_y[1] && buffer_y[1] > buffer_y[0]) {
			tmp1 = std::abs(buffer_y[1] - buffer_y[0]);
			tmp2 = std::abs(buffer_y[1] - buffer_y[2]);

			if (tmp2 > tmp1) {
				buffer_y[1] = buffer_y[0];
			}
			else {
				buffer_y[1] = buffer_y[2];
			}

			buffer_y[3] = buffer_y[2];
			buffer_y[2] = buffer_y[1];
		}
		else {
			buffer_y[3] = buffer_y[2];
			buffer_y[2] = buffer_y[1];
		}
	}

		aout[0] = buffer_x[2];
		aout[1] = buffer_y[2];
}
