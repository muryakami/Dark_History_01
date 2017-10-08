#ifndef _ReadSTL_HPP_
#define _ReadSTL_HPP_

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cassert>
#include <vector>
#include <string>

using namespace std;

// --------------------------------------------
// ---------------Open StL data---------------
// --------------------------------------------

typedef struct		// 法線ベクトル
{
	union {
		struct { float nx, ny, nz; };
		float np[3];
	};
	operator float*() { return np; };
} NORMAL;

typedef struct		// 頂点
{
	union {
		struct { float x, y, z; };
		float p[3];
	};
	operator float*() { return p; };
} VERTEX;

typedef struct _TRIANGLE	// 三角形情報
{
	NORMAL normal;				// 法線ベクトル
	VERTEX vertex[3];			// 頂点*3(面情報)
	inline void init(void) {
		normal.nx = normal.ny = normal.nz = 0.0f;
		for (int i = 0; i < 3; i++)
			vertex[i].x = vertex[i].y = vertex[i].z = 0.0f;
	};
	_TRIANGLE(void) {
		init();
	};
} TRIANGLE;

class STLDATA		//.stlファイル読み込み
{
	vector<TRIANGLE> data;
	vector<string> ParseStl(const string str);
	vector<TRIANGLE> ReadStl(const string filepath);
public:
	STLDATA(const string path) { RunReadProc(path); };
	~STLDATA(void) { data.clear(); };
	bool RunReadProc(const string path);
	int getDatanum(void) const { return data.size(); };
	bool IsEmpty(void) const { return data.empty(); };
	void getData(const unsigned int index, TRIANGLE* dst) const;
};

bool STLDATA::RunReadProc(const string path)
{
	this->data = this->ReadStl(path);
	if (data.empty())	return false;
	else				return true;
}

void STLDATA::getData(const unsigned int index, TRIANGLE* dst) const
{
	if (data.size() > index)
		*dst = data.at(index);
	else
		dst = NULL;

	return;
}

vector<string> STLDATA::ParseStl(const string str)
{
	vector<string> result;
	string buf;
	string::const_iterator bit, it;
	bool readchar = false;

	for (bit = str.begin(); bit != str.end(); bit++) {
		if (((*bit == 0x20) || (*bit == '\t')) && !readchar) {
			readchar = false;
			continue;
		}
		if (((*bit != 0x20) || (*bit != '\t')) && !readchar) {
			readchar = true;
			it = bit;
			continue;
		}
		if (((*bit == 0x20) || (*bit == '\t')) && readchar) {
			readchar = false;
			buf.assign("");
			buf.append(it, bit);
			result.push_back(buf);
			continue;
		}
	}

	return result;
}

vector<TRIANGLE> STLDATA::ReadStl(const string filepath)
{
	static vector<TRIANGLE> _data;
	if (!_data.empty())	_data.clear();
	vector<string> pstr;
	TRIANGLE tri;
	unsigned int vernum = 0;

	ifstream fs(filepath.c_str(), ios::in);
	assert(fs);
	if (!fs) {
		_data.clear();
		cerr << "The file can't open." << endl;
		return _data;
	}

	string str, buf;
	string::iterator bit, it1, it2;

	while (!fs.eof()) {
		getline(fs, str);
		str.append(" ");
		pstr = this->ParseStl(str);

		if (pstr.size() > 0) {
			if (pstr.at(0) == string("facet")) {
				tri.normal.nx = atof(pstr.at(2).c_str());
				tri.normal.ny = atof(pstr.at(3).c_str());
				tri.normal.nz = atof(pstr.at(4).c_str());
				vernum = 0;
			}
			if (pstr.at(0) == string("vertex")) {
				tri.vertex[vernum].x = atof(pstr.at(1).c_str());
				tri.vertex[vernum].y = atof(pstr.at(2).c_str());
				tri.vertex[vernum].z = atof(pstr.at(3).c_str());
				vernum++;
			}
			if (pstr.at(0) == string("endfacet")) {
				_data.push_back(tri);
				tri.init();
			}
		}
	}

	fs.close();

	return _data;
}

#endif // _ReadSTL_HPP_