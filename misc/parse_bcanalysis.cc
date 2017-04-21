#include <iostream>
#include <regex>
#include <list>

using namespace std;

int main(int argc, char **argv) {
  
  string str;

  regex HSV("HSV\\[([0-9]+) ([0-9]+) ([0-9]+)\\]");
  regex BGR("BGR\\[([0-9]+) ([0-9]+) ([0-9]+)\\]");
  smatch sm_hsv, sm_bgr;

  //TODO: typedef or c++11 equivalent for readability
  list<pair<tuple<int,int,int>,tuple<int,int,int>>> beacon_blue;
  list<pair<tuple<int,int,int>,tuple<int,int,int>>> beacon_green;
  list<pair<tuple<int,int,int>,tuple<int,int,int>>> beacon_red;
  list<pair<tuple<int,int,int>,tuple<int,int,int>>> floor_green;

  while (getline(cin,str)) {
    if (str[0] == '-') {
      int flag = 0;		//TODO: replace int with enum for readability
      while (getline(cin,str) && !str.empty()) {

	//cout << str << endl;
        if (str.find("beacon red") != string::npos) {
          //cout << "red values coming up!" << endl;
	  flag = 1;
	} else if (str.find("beacon green") != string::npos) {
          //cout << "green values coming up!" << endl;
	  flag = 2;
	} else if (str.find("beacon blue") != string::npos) {
	  //cout << "blue values coming up!" << endl;
	  flag = 3;
	} else if (str.find("floor green") != string::npos) {
	  //cout << "green floor values coming up!" << endl;
	  flag = 4;
	} else {
          //cout << "parse HSV and BGR values" << endl;
          if (regex_search(str,sm_hsv,HSV) && regex_search(str,sm_bgr,BGR)) {
	    int h = stoi(sm_hsv[1].str());
	    int s = stoi(sm_hsv[2].str());
	    int v = stoi(sm_hsv[3].str());
	    //cout << h << s << v << endl;
	    int b = stoi(sm_bgr[1].str());
	    int g = stoi(sm_bgr[2].str());
	    int r = stoi(sm_bgr[3].str());
	    //cout << b << g << r << endl;
	    if (flag == 1) {
	      beacon_red.push_back(make_pair(make_tuple(h,s,v),make_tuple(b,g,r)));
	    } else if (flag == 2) {
	      beacon_green.push_back(make_pair(make_tuple(h,s,v),make_tuple(b,g,r)));
	    } else if (flag == 3) {
	      beacon_blue.push_back(make_pair(make_tuple(h,s,v),make_tuple(b,g,r)));
	    } else if (flag == 4) {
	      floor_green.push_back(make_pair(make_tuple(h,s,v),make_tuple(b,g,r)));
	    }

	  } else {
	    cout << "Invalid line format! Skip line.." << endl;
	    continue;
	  }
	}

      }
    }
  }
  
  for (auto it : beacon_red) {
    cout << get<0>(it.first) << endl;
  }
  //process color information 
  //e.g.: normalized colorspace, boundaries, histogram equalization
  //autogenerated color boundaries?

  return 0;
}
