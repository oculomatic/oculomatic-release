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

#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
	ofSetupOpenGL(1000,600,OF_WINDOW);			// <-------- setup the GL context

	ofRunApp(new ofApp());

}
