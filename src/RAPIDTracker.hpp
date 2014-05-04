#pragma once

#include "Tracker.hpp"

class RAPIDTracker : public Tracker
{
public:
	RAPIDTracker(Model& model, bool isLogsEnabled);
};