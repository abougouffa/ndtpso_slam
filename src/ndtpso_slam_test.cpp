//#include "ndtpso_slam/ndtcell.h"
//#include "ndtpso_slam/ndtframe.h"
#include <chrono>
#include <eigen3/Eigen/Core>
#include <iostream>

#define SAVE_DATA_TO_FILE true
#define CELL_SIZE .4
#define SIDE_M 200

using namespace Eigen;
using std::cout;
using std::endl;

static Vector3d global_trans, previous_pose, trans_estimate, initial_trans;

// static NDTFrame* current_frame;
// static NDTFrame ref_frame(Vector3d::Zero(), 50, 50, CELL_SIZE);
//

int main(int argc, char **argv) {

  // current_frame = new NDTFrame(initial_trans, SIDE_M, SIDE_M, SIDE_M, true);
  return 0;
}
