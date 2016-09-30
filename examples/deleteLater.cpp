/*  ---------------------------

* This is a simple example to process GNSS pseudorange measurements. 
	* Example constructs custome factor to generate 
*	  pseuodrange factors.

-------------------------------  */ 

#include <gtsam/base/Vector.h> 
#include <gtsam/navigation/gnssTools.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

	Vector vec1(3), vec2(3), H(5), e(3);
	vec1 << 12313, 12356, 42456;
	vec2 << 3435353, 2456033, 3567886;
	H << 1,1,1,1,1;
	for (int i=0;i<4;i++){
	e = -1*(((vec1-vec2))/((vec1-vec2).norm()));
	cout << e << endl;
	}
	H.block<3,1>(0,0)=-1*(((vec1-vec2))/((vec1-vec2).norm()));
	//cout << H << endl;
	// cout << vec1.block<2,1>(0,0) - vec2.block<2,1>(1,0) << endl;
	return 0;
}                       
