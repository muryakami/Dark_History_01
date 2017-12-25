#ifndef _OUTPUT_H_
#define _OUTPUT_H_

#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

using namespace std;
namespace fs = boost::filesystem;


//ofstream processing_time(".\\outputfile\\time001.txt");
//ofstream accuracy_file(".\\outputfile\\accuracy001.txt");
//ofstream outputfile(".\\outputfile\\result001.txt");

ofstream processing_time("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\time000.txt");
ofstream accuracy_file("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\accuracy000.txt");
ofstream outputfile("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\result000.txt");

//ofstream source_keypointsNormal_IO("C:\\Users\\yuki\\Documents\\�i����\\06\\outputfile\\source_keypointsNormal_000.txt");
//ofstream target_keypointsNormal_IO("C:\\Users\\yuki\\Documents\\�i����\\06\\outputfile\\target_keypointsNormal_000.txt");
//ofstream source_keypointsXYZ_IO("C:\\Users\\yuki\\Documents\\�i����\\06\\outputfile\\source_keypointsXYZ_000.txt");
//ofstream target_keypointsXYZ_IO("C:\\Users\\yuki\\Documents\\�i����\\06\\outputfile\\target_keypointsXYZ_000.txt");

ofstream keypointsXYZ_IO("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\keypointsXYZ_000.txt", ios::app);
ofstream keypointsNormal_IO("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\keypointsNormal_000.txt", ios::app);

ofstream correspondencesOut("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\correspondences000.txt");

ofstream cylinderData("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\cylinderData000.txt");

ofstream cylinderDataRadius("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\cylinderDataRadius000.txt");
ofstream cylinderDataAxis("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\cylinderDataAxis000.txt");
ofstream cylinderDataPCA("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\cylinderDataPCA000.txt");

ofstream filenameIO("C:\\Users\\yuki\\Documents\\�i����\\07\\outputfile\\filenameIO.txt");

#endif // _OUTPUT_H_