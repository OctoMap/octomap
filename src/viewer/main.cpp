/*
 * main.cpp
 *
 *  Created on: May 28, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#include <QtGui>
#include <QApplication>
#include "ViewerGui.h"

int main(int argc, char *argv[])
{
	std::string filename = "";
	if (argc == 2) {
		filename = std::string(argv[1]);
	}

	QApplication app(argc, argv);
	ViewerGui gui(filename);
	gui.show();
	return app.exec();
}
