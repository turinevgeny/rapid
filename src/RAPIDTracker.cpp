#include "RAPIDTracker.hpp"

using std::cout;
using std::endl;
using namespace cv;

RAPIDTracker::RAPIDTracker(Model& _model, bool _isLogsEnabled)
    :Tracker(_model, _isLogsEnabled)
{}